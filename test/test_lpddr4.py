import re
import unittest
from collections import defaultdict
from typing import Mapping, Sequence

from migen import *

from litedram.phy import dfi
from litedram.phy.lpddr4phy import SimulationPHY

from litex.gen.sim import run_simulation

def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

class TestLPDDR4(unittest.TestCase):
    CMD_LATENCY = 2

    class PadsHistory(defaultdict):
        def __init__(self):
            super().__init__(str)

        def format(self):
            keys = list(self.keys())
            key_strw = max(len(k) for k in keys)
            keys = sorted(keys, key=lambda k: 2 if k == 'E' else 1)
            lines = []
            for k in keys:
                hist = ' '.join(chunk for chunk in chunks(self[k], 8))
                line = '{:{n}} {}'.format(k + ':' if k != 'E' else '', hist, n=key_strw+1)
                lines.append(line)
            return '\n'.join(lines)

    def pads_checker(self, pads, signals: Mapping[str, str], fail_fast=False):
        # signals: {sig: values}, values: a string of '0'/'1'/'x'
        lengths = [len(vals) for vals in signals.values()]
        n = lengths[0]
        assert all(l == n for l in lengths)

        errors = []
        history = self.PadsHistory()
        for i in range(n):
            for sig, vals in signals.items():
                m = re.match(r'(\S+)(\d+)', sig)
                if m:
                    pad = getattr(pads, m.group(1))[int(m.group(2))]
                else:
                    pad = getattr(pads, sig)
                val = vals[i]
                history[sig] += str((yield pad))
                if val != 'x':
                    msg = f'Cycle {i} Signal {sig}: {(yield pad)} vs {vals[i]}'
                    errors.append([i, (yield pad), int(vals[i]), msg])
                    if fail_fast:
                        msg += '\nHistory:\n{}'.format(history.format())
                        self.assertEqual((yield pad), int(vals[i]), msg=msg)
            yield

        for i, pad_val, ref_val, msg in errors:
            history['E'] = ''.join(['^' if j == i else ' ' for j in range(n)])
            msg += '\nHistory:\n{}'.format(history.format())
            self.assertEqual(pad_val, ref_val, msg=msg)

    def dfi_reset_value(self, sig):
        return 1 if sig.endswith('_n') else 0

    def dfi_reset_values(self):
        return {sig: self.dfi_reset_value(sig) for sig, _, _ in dfi.phase_description(1, 1, 1, 1)}

    DFIPhaseValues = Mapping[str, int]
    DFIPhase = int

    def dfi_generator(self, dfi, sequence: Sequence[Mapping[DFIPhase, DFIPhaseValues]]):
        # sequence: [{phase: {sig: value}}]
        for per_phase in sequence:
            # reset in case of any previous changes
            for phase in dfi.phases:
                for sig, val in self.dfi_reset_values().items():
                    yield getattr(phase, sig).eq(val)
            # set values
            for phase, values in per_phase.items():
                for sig, val in values.items():
                    # print(f'dfi.p{phase}.{sig} = {val}')
                    yield getattr(dfi.phases[phase], sig).eq(val)
            # print()
            yield

    def run_simulation(self, dut, generators, **kwargs):
        clocks = {  # sys and sys8x phase aligned
            "sys":      (32, 16),
            "sys8x":    ( 4,  2),
            "sys8x_90": ( 4,  1),
        }
        run_simulation(dut, generators, clocks, **kwargs)

    def run_test(self, dfi_sequence, pad_signals):
        dut = SimulationPHY()
        generators = {
            "sys":   [self.dfi_generator(dut.dfi, dfi_sequence)],
            "sys8x": [self.pads_checker(dut.pads, pad_signals)],
        }
        self.run_simulation(dut, generators)

    def test_cs_phase_0(self):
        latency = '00000000' * self.CMD_LATENCY
        self.run_test(
            dfi_sequence = [
                {p: dict(self.dfi_reset_values()) for p in range(8)},
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # p0: READ
            ],
            pad_signals = {
                'cs': latency + '00000000' + '10100000',
            }
        )

    def test_cs_multiple_phases(self):
        latency = '00000000' * self.CMD_LATENCY
        self.run_test(
            dfi_sequence = [
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},
                {3: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},
                {
                    1: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1),
                    4: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1),  # should be ignored
                },
                {
                    1: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1),
                    5: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1),  # should NOT be ignored
                },
                {6: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)}, # crosses cycle boundaries
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # should be ignored
            ],
            pad_signals = {
                'cs': latency + ''.join([
                    '10100000',  # p0
                    '00010100',  # p3
                    '01010000',  # p1, p4 ignored
                    '01010101',  # p1, p5
                    '00000010',  # p6 (cyc 0)
                    '10000000',  # p6 (cyc 1), p0 ignored
                ])
            }
        )

    def test_ca_sequencing(self):
        latency = '00000000' * self.CMD_LATENCY
        read = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)
        self.run_test(
            dfi_sequence = [
                {0: read, 3: read},  # p4 should be ignored
                {0: read, 4: read},
                {6: read},
                {0: read}, # ignored
            ],
            pad_signals = {
                'cs':  latency + '10100000' + '10101010' + '00000010' + '10000000',
                'ca0': latency + '00000000' + '00000000' + '00000000' + '00000000',
                'ca1': latency + '10100000' + '10101010' + '00000010' + '10000000',
                'ca2': latency + '00000000' + '00000000' + '00000000' + '00000000',
                'ca3': latency + '0x000000' + '0x000x00' + '0000000x' + '00000000',
                'ca4': latency + '00100000' + '00100010' + '00000000' + '10000000',
                'ca5': latency + '00000000' + '00000000' + '00000000' + '00000000',
            }
        )

    def test_ca_addressing(self):
        latency = '00000000' * self.CMD_LATENCY
        read       = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1, bank=0b101, address=0b1100110011)  # actually invalid because CA[1:0] should always be 0
        write_ap   = dict(cs_n=0, cas_n=0, ras_n=1, we_n=0, bank=0b111, address=0b10000000000)
        activate   = dict(cs_n=0, cas_n=1, ras_n=0, we_n=1, bank=0b010, address=0b11110000111100001)
        refresh_ab = dict(cs_n=0, cas_n=0, ras_n=0, we_n=1, bank=0b100, address=0b10000000000)
        precharge  = dict(cs_n=0, cas_n=1, ras_n=0, we_n=0, bank=0b011, address=0)
        mrw        = dict(cs_n=0, cas_n=0, ras_n=0, we_n=0, bank=0,     address=(0b110011 << 8) | 0b10101010)  # 6-bit address | 8-bit op code
        self.run_test(
            dfi_sequence = [
                {0: read, 4: write_ap},
                {0: activate, 4: refresh_ab},
                {0: precharge, 4: mrw},
            ],
            pad_signals = {
                # note that refresh and precharge have a single command so these go as cmd2
                #                              rd     wr       act    ref      pre    mrw
                'cs':  latency + '1010'+'1010' + '1010'+'0010' + '0010'+'1010',
                'ca0': latency + '0100'+'0100' + '1011'+'0000' + '0001'+'0100',
                'ca1': latency + '1010'+'0110' + '0110'+'0000' + '0001'+'1111',
                'ca2': latency + '0101'+'1100' + '0010'+'0001' + '0000'+'1010',
                'ca3': latency + '0x01'+'0x00' + '1110'+'001x' + '000x'+'0001',
                'ca4': latency + '0110'+'0010' + '1010'+'000x' + '001x'+'0110',
                'ca5': latency + '0010'+'0100' + '1001'+'001x' + '000x'+'1101',
            }
        )
