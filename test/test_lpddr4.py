import unittest
from collections import defaultdict

from migen import *

from litedram.phy import dfi
from litedram.phy.lpddr4phy import SimulationPHY

from litex.gen.sim import run_simulation

def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

class TestLPDDR4(unittest.TestCase):
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
                line = '{:{n}} {}'.format(k + ':', hist, n=key_strw+1)
                lines.append(line)
            return '\n'.join(lines)

    def pads_checker(self, pads, signals, fail_fast=False):
        lengths = [len(vals) for vals in signals.values()]
        n = lengths[0]
        assert all(l == n for l in lengths)

        errors = []
        history = self.PadsHistory()
        for i in range(n):
            for sig, vals in signals.items():
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

    def dfi_generator(self, dfi, sequence):
        for per_phase in sequence:
            for phase, values in per_phase.items():
                for sig, val in values.items():
                    yield getattr(dfi.phases[phase], sig).eq(val)
            yield
            for phase, values in per_phase.items():
                for sig, val in values.items():
                    yield getattr(dfi.phases[phase], sig).eq(self.dfi_reset_value(sig))

    def run_simulation(self, dut, generators, **kwargs):
        clocks = {  # sys and sys8x phase aligned
            "sys":      (32, 16),
            "sys8x":    ( 4,  2),
            "sys8x_90": ( 4,  1),
        }
        run_simulation(dut, generators, clocks, **kwargs)

    def test_cs_phase_0(self):
        dfi_reset = {sig: self.dfi_reset_value(sig) for sig, _, _ in dfi.phase_description(1, 1, 1, 1)}
        dfi_seq = [
            {p: dict(dfi_reset) for p in range(8)},
            {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # p0: READ
        ]

        latency = 2
        pads_signals = {
            'cs': '00000000' * latency + '00000000' + '10100000' + '00000000',
        }

        dut = SimulationPHY()
        generators = {
            "sys":   [self.dfi_generator(dut.dfi, dfi_seq)],
            "sys8x": [self.pads_checker(dut.pads, pads_signals)],
        }
        self.run_simulation(dut, generators, vcd_name='sim.vcd')

    def test_cs_multiple_phases(self):
        dfi_reset = {sig: self.dfi_reset_value(sig) for sig, _, _ in dfi.phase_description(1, 1, 1, 1)}
        dfi_seq = [
            {p: dict(dfi_reset) for p in range(8)},
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
        ]

        latency = 2
        pads_signals = {
            'cs': '00000000' * latency + ''.join([
                '00000000',
                '10100000',  # p0
                '00010100',  # p3
                '01010000',  # p1
                '01010101',  # p1, p5
                '00000010',  # p6 (cyc 0)
                '10000000',  # p6 (cyc 1)
            ])
        }

        dut = SimulationPHY()
        generators = {
            "sys":   [self.dfi_generator(dut.dfi, dfi_seq)],
            "sys8x": [self.pads_checker(dut.pads, pads_signals)],
        }
        self.run_simulation(dut, generators, vcd_name='sim.vcd')
