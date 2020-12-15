import re
import copy
import random
import unittest
import itertools
from collections import defaultdict
from typing import Mapping, Sequence

from migen import *

from litedram.phy import dfi
from litedram.phy.lpddr4phy import SimulationPHY, Serializer, Deserializer

from litex.gen.sim import run_simulation as _run_simulation


def bit(n, val):
    return (val & (1 << n)) >> n

def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def run_simulation(dut, generators, debug_clocks=False, **kwargs):
    # Migen simulator supports reset signals so we could add CRG to start all the signals
    # in the same time, however the clock signals will still be visible in the VCD dump
    # and the generators we assign to them will still work before reset. For this reason we
    # use clocks set up in such a way that we have all the phase aligned clocks start in tick
    # 1 (not zero), so that we avoid any issues with clock alignment.
    #
    # NOTE: On hardware proper reset must be ensured!
    #
    # The simulation should start like this:
    #   sys          |_--------------
    #   sys_11_25    |___------------
    #   sys8x        |_----____----__
    #   sys8x_ddr    |_--__--__--__--
    #   sys8x_90     |___----____----
    #   sys8x_90_ddr |-__--__--__--__
    #
    # sys8x_90_ddr does not trigger at the simulation start (not an edge),
    # BUT a generator starts before first edge, so a `yield` is needed to wait until the first
    # rising edge!
    clocks = {
        "sys":          (64, 31),
        "sys_11_25":    (64, 29),  # aligned to sys8x_90 (phase shift of 11.25)
        "sys8x":        ( 8,  3),
        "sys8x_ddr":    ( 4,  1),
        "sys8x_90":     ( 8,  1),
        "sys8x_90_ddr": ( 4,  3),
    }

    if debug_clocks:
        class DUT(Module):
            def __init__(self, dut):
                self.submodules.dut = dut
                for clk in clocks:
                    setattr(self.clock_domains, "cd_{}".format(clk), ClockDomain(clk))
                    cd = getattr(self, 'cd_{}'.format(clk))
                    self.comb += cd.rst.eq(0)

                    s = Signal(4, name='dbg_{}'.format(clk))
                    sd = getattr(self.sync, clk)
                    sd += s.eq(s + 1)
        dut = DUT(dut)

    _run_simulation(dut, generators, clocks, **kwargs)


class TestSimSerializers(unittest.TestCase):
    @staticmethod
    def data_generator(i, datas):
        for data in datas:
            yield i.eq(data)
            yield
        yield i.eq(0)
        yield

    @staticmethod
    def data_checker(o, datas, n, latency, yield1=False):
        if yield1:
            yield
        for _ in range(latency):
            yield
        for _ in range(n):
            datas.append((yield o))
            yield
        yield

    def serializer_test(self, *, data_width, datas, clk, clkdiv, latency, clkgen=None, clkcheck=None, **kwargs):
        clkgen = clkgen if clkgen is not None else clk
        clkcheck = clkcheck if clkcheck is not None else clkdiv

        received = []
        dut = Serializer(clk=clk, clkdiv=clkdiv, i_dw=data_width, o_dw=1)
        generators = {
            clkgen: self.data_generator(dut.i, datas),
            clkcheck: self.data_checker(dut.o, received, n=len(datas) * data_width, latency=latency * data_width, yield1=True),
        }
        run_simulation(dut, generators, **kwargs)

        received = list(chunks(received, data_width))
        datas  = [[bit(i, d) for i in range(data_width)] for d in datas]
        self.assertEqual(received, datas)

    def deserializer_test(self, *, data_width, datas, clk, clkdiv, latency, clkgen=None, clkcheck=None, **kwargs):
        clkgen = clkgen if clkgen is not None else clkdiv
        clkcheck = clkcheck if clkcheck is not None else clk

        datas = [[bit(i, d) for i in range(data_width)] for d in datas]

        received = []
        dut = Deserializer(clk=clk, clkdiv=clkdiv, i_dw=1, o_dw=data_width)
        generators = {
            clkgen: self.data_generator(dut.i, itertools.chain(*datas)),
            clkcheck: self.data_checker(dut.o, received, n=len(datas), latency=latency),
        }

        run_simulation(dut, generators, **kwargs)

        received = [[bit(i, d) for i in range(data_width)] for d in received]
        self.assertEqual(received, datas)

    DATA_8 = [0b11001100, 0b11001100, 0b00110011, 0b00110011, 0b10101010]
    DATA_16 = [0b1100110011001100, 0b0011001100110011, 0b0101010101010101]

    ARGS_8 = dict(
        data_width = 8,
        datas = DATA_8,
        clk = "sys",
        clkdiv = "sys8x",
        latency = Serializer.LATENCY,
    )

    ARGS_16 = dict(
        data_width = 16,
        datas = DATA_16,
        clk = "sys",
        clkdiv = "sys8x_ddr",
        latency = Serializer.LATENCY,
    )

    def _s(default, **kwargs):
        def test(self):
            new = default.copy()
            new.update(kwargs)
            self.serializer_test(**new)
        return test

    def _d(default, **kwargs):
        def test(self):
            new = default.copy()
            new["latency"] = Deserializer.LATENCY
            new.update(kwargs)
            self.deserializer_test(**new)
        return test

    test_sim_serializer_8 = _s(ARGS_8)
    test_sim_serializer_8_phase90 = _s(ARGS_8, clk="sys_11_25", clkdiv="sys8x_90")
    # when clkgen and clk are not phase aligned  (clk is delayed), there will be lower latency
    test_sim_serializer_8_phase90_gen0 = _s(ARGS_8, clk="sys_11_25", clkdiv="sys8x_90", clkgen="sys", latency=Serializer.LATENCY - 1)
    test_sim_serializer_8_phase90_check0 = _s(ARGS_8, clk="sys_11_25", clkdiv="sys8x_90", clkcheck="sys8x")

    test_sim_serializer_16 = _s(ARGS_16)
    test_sim_serializer_16_phase90 = _s(ARGS_16, clk="sys_11_25", clkdiv="sys8x_90_ddr")
    test_sim_serializer_16_phase90_gen0 = _s(ARGS_16, clk="sys_11_25", clkdiv="sys8x_90_ddr", clkgen="sys", latency=Serializer.LATENCY - 1)
    test_sim_serializer_16_phase90_check0 = _s(ARGS_16, clk="sys_11_25", clkdiv="sys8x_90_ddr", clkcheck="sys8x_ddr")

    # for phase aligned clocks the latency will be bigger (preferably avoid phase aligned reading?)
    test_sim_deserializer_8 = _d(ARGS_8, latency=Deserializer.LATENCY + 1)
    test_sim_deserializer_8_check90 = _d(ARGS_8, clkcheck="sys_11_25")
    test_sim_deserializer_8_gen90_check90 = _d(ARGS_8, clkcheck="sys_11_25", clkgen="sys8x_90")
    test_sim_deserializer_8_phase90 = _d(ARGS_8, clk="sys_11_25", clkdiv="sys8x_90", latency=Deserializer.LATENCY + 1)
    test_sim_deserializer_8_phase90_check0 = _d(ARGS_8, clk="sys_11_25", clkdiv="sys8x_90", clkcheck="sys", latency=Deserializer.LATENCY + 1)

    test_sim_deserializer_16 = _d(ARGS_16, latency=Deserializer.LATENCY + 1)
    test_sim_deserializer_16_check90 = _d(ARGS_16, clkcheck="sys_11_25")
    test_sim_deserializer_16_gen90_check90 = _d(ARGS_16, clkcheck="sys_11_25", clkgen="sys8x_90_ddr")
    test_sim_deserializer_16_phase90 = _d(ARGS_16, clk="sys_11_25", clkdiv="sys8x_90_ddr", latency=Deserializer.LATENCY + 1)
    test_sim_deserializer_16_phase90_check0 = _d(ARGS_16, clk="sys_11_25", clkdiv="sys8x_90_ddr", clkcheck="sys", latency=Deserializer.LATENCY + 1)


BOLD = '\033[1m'
HIGHLIGHT = '\033[91m'
CLEAR = '\033[0m'

def highlight(s, hl=True):
    return BOLD + (HIGHLIGHT if hl else '') + s + CLEAR


class PadsHistory(defaultdict):
    def __init__(self):
        super().__init__(str)

    def format(self, hl_cycle=None, hl_signal=None, underline_cycle=False, key_strw=None):
        if key_strw is None:
            key_strw = max(len(k) for k in self)
        lines = []
        for k in self:
            vals = list(self[k])
            if hl_cycle is not None and hl_signal is not None:
                vals = [highlight(val, hl=hl_signal == k) if i == hl_cycle else val
                        for i, val in enumerate(vals)]
            hist = ' '.join(''.join(chunk) for chunk in chunks(vals, 8))
            line = '{:{n}} {}'.format(k + ':', hist, n=key_strw+1)
            lines.append(line)
        if underline_cycle:
            assert hl_cycle is not None
            n = hl_cycle + hl_cycle//8
            line = ' ' * (key_strw+1) + ' ' + ' ' * n + '^'
            lines.append(line)
        if hl_signal is not None and hl_cycle is None:
            keys = list(self.keys())
            sig_i = keys.index(hl_signal)
            lines = ['{} {}'.format('>' if i == sig_i else ' ', line) for i, line in enumerate(lines)]
        return '\n'.join(lines)

    @staticmethod
    def width_for(histories):
        keys = itertools.chain.from_iterable(h.keys() for h in histories)
        return max(len(k) for k in keys)

class PadChecker:
    def __init__(self, pads, signals: Mapping[str, str]):
        # signals: {sig: values}, values: a string of '0'/'1'/'x'/' '
        self.pads = pads
        self.signals = signals
        self.history = PadsHistory()  # registered values
        self.ref_history = PadsHistory()  # expected values

        assert all(v in '01x' for values in signals.values() for v in values)

        lengths = [len(vals) for vals in signals.values()]
        assert all(l == lengths[0] for l in lengths)

    @property
    def length(self):
        return len(list(self.signals.values())[0])

    def run(self):
        for i in range(self.length):
            for sig, vals in self.signals.items():
                # transform numbered signal names to pad indicies (e.g. dq1 -> dq[1])
                m = re.match(r'([a-zA-Z_]+)(\d+)', sig)
                pad = getattr(self.pads, m.group(1))[int(m.group(2))] if m else getattr(self.pads, sig)

                # save the value at current cycle
                val = vals[i]
                self.history[sig] += str((yield pad))
                self.ref_history[sig] += val
            yield

    def find_error(self, start=0):
        for i in range(start, self.length):
            for sig in self.history:
                val = self.history[sig][i]
                ref = self.ref_history[sig][i]
                if ref != 'x' and val != ref:
                    return (i, sig, val, ref)
        return None

    def summary(self, **kwargs):
        error = self.find_error()
        cycle, sig = None, None
        if error is not None:
            cycle, sig, val, ref = error
        lines = []
        lines.append(self.history.format(hl_cycle=cycle, hl_signal=sig, **kwargs))
        lines.append('vs ref:')
        lines.append(self.ref_history.format(hl_cycle=cycle, hl_signal=sig, **kwargs))
        return '\n'.join(lines)

    @staticmethod
    def assert_ok(test_case, clock_checkers):
        # clock_checkers: {clock: PadChecker(...), ...}
        errors = list(filter(None, [c.find_error() for c in clock_checkers.values()]))
        if errors:
            all_histories = [c.history for c in clock_checkers.values()]
            all_histories += [c.ref_history for c in clock_checkers.values()]
            key_strw = PadsHistory.width_for(all_histories)
            summaries = ['{}\n{}'.format(highlight(clock, hl=False), checker.summary(key_strw=key_strw))
                         for clock, checker in clock_checkers.items()]
            first_error = min(errors, key=lambda e: e[0])  # first error
            i, sig, val, ref = first_error
            msg = f'Cycle {i} Signal `{sig}`: {val} vs {ref}\n'
            test_case.assertEqual(val, ref, msg=msg + '\n'.join(summaries))

class TestLPDDR4(unittest.TestCase):
    CMD_LATENCY = 2

    def dfi_reset_value(self, sig):
        return 1 if sig.endswith('_n') else 0

    def dfi_reset_values(self):
        desc = dfi.phase_description(addressbits=17, bankbits=3, nranks=1, databits=16)
        return {sig: self.dfi_reset_value(sig) for sig, _, _ in desc}

    DFIPhaseValues = Mapping[str, int]
    DFIPhase = int

    def dfi_generator(self, dfi, sequence: Sequence[Mapping[DFIPhase, DFIPhaseValues]]):
        # sequence: [{phase: {sig: value}}]
        def reset():
            for phase in dfi.phases:
                for sig, val in self.dfi_reset_values().items():
                    yield getattr(phase, sig).eq(val)

        for per_phase in sequence:
            # reset in case of any previous changes
            (yield from reset())
            # set values
            for phase, values in per_phase.items():
                for sig, val in values.items():
                    # print(f'dfi.p{phase}.{sig} = {val}')
                    yield getattr(dfi.phases[phase], sig).eq(val)
            # print()
            yield
        (yield from reset())
        yield

    def run_test(self, dut, dfi_sequence, pad_checkers: Mapping[str, Mapping[str, str]], **kwargs):
        # pad_checkers: {clock: {sig: values}}
        checkers = {clk: PadChecker(dut.pads, pad_signals) for clk, pad_signals in pad_checkers.items()}
        generators = defaultdict(list)
        generators["sys"].append(self.dfi_generator(dut.dfi, dfi_sequence))
        for clock, checker in checkers.items():
            generators[clock].append(checker.run())
        run_simulation(dut, generators, **kwargs)
        PadChecker.assert_ok(self, checkers)

    def test_lpddr4_cs_phase_0(self):
        latency = '00000000' * self.CMD_LATENCY
        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {0: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},  # p0: READ
            ],
            pad_checkers = {"sys8x_90": {
                'cs': latency + '10100000',
            }},
        )

    def test_lpddr4_clk(self):
        latency = 'xxxxxxxx' * self.CMD_LATENCY
        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {3: dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)},
            ],
            pad_checkers = {"sys8x_90_ddr": {
                'clk_p': latency + '10101010' * 3,
            }},
        )

    def test_lpddr4_cs_multiple_phases(self):
        latency = '00000000' * self.CMD_LATENCY
        self.run_test(SimulationPHY(),
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
                {2: dict(cs_n=1, cas_n=0, ras_n=1, we_n=1)},  # ignored due to cs_n=1
            ],
            pad_checkers = {"sys8x_90": {
                'cs': latency + ''.join([
                    '10100000',  # p0
                    '00010100',  # p3
                    '01010000',  # p1, p4 ignored
                    '01010101',  # p1, p5
                    '00000010',  # p6 (cyc 0)
                    '10000000',  # p6 (cyc 1), p0 ignored
                    '00000000',  # p2 ignored
                ])
            }},
        )

    def test_lpddr4_ca_sequencing(self):
        latency = '00000000' * self.CMD_LATENCY
        read = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)
        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {0: read, 3: read},  # p4 should be ignored
                {0: read, 4: read},
                {6: read},
                {0: read}, # ignored
            ],
            pad_checkers = {"sys8x_90": {
                'cs':  latency + '10100000' + '10101010' + '00000010' + '10000000',
                'ca0': latency + '00000000' + '00000000' + '00000000' + '00000000',
                'ca1': latency + '10100000' + '10101010' + '00000010' + '10000000',
                'ca2': latency + '00000000' + '00000000' + '00000000' + '00000000',
                'ca3': latency + '0x000000' + '0x000x00' + '0000000x' + '00000000',
                'ca4': latency + '00100000' + '00100010' + '00000000' + '10000000',
                'ca5': latency + '00000000' + '00000000' + '00000000' + '00000000',
            }},
        )

    def test_lpddr4_ca_addressing(self):
        latency = '00000000' * self.CMD_LATENCY
        read       = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1, bank=0b101, address=0b1100110011)  # actually invalid because CA[1:0] should always be 0
        write_ap   = dict(cs_n=0, cas_n=0, ras_n=1, we_n=0, bank=0b111, address=0b10000000000)
        activate   = dict(cs_n=0, cas_n=1, ras_n=0, we_n=1, bank=0b010, address=0b11110000111100001)
        refresh_ab = dict(cs_n=0, cas_n=0, ras_n=0, we_n=1, bank=0b100, address=0b10000000000)
        precharge  = dict(cs_n=0, cas_n=1, ras_n=0, we_n=0, bank=0b011, address=0)
        mrw        = dict(cs_n=0, cas_n=0, ras_n=0, we_n=0, bank=0,     address=(0b110011 << 8) | 0b10101010)  # 6-bit address | 8-bit op code
        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {0: read, 4: write_ap},
                {0: activate, 4: refresh_ab},
                {0: precharge, 4: mrw},
            ],
            pad_checkers = {"sys8x_90": {
                # note that refresh and precharge have a single command so these go as cmd2
                #                 rd     wr       act    ref      pre    mrw
                'cs':  latency + '1010'+'1010' + '1010'+'0010' + '0010'+'1010',
                'ca0': latency + '0100'+'0100' + '1011'+'0000' + '0001'+'0100',
                'ca1': latency + '1010'+'0110' + '0110'+'0000' + '0001'+'1111',
                'ca2': latency + '0101'+'1100' + '0010'+'0001' + '0000'+'1010',
                'ca3': latency + '0x01'+'0x00' + '1110'+'001x' + '000x'+'0001',
                'ca4': latency + '0110'+'0010' + '1010'+'000x' + '001x'+'0110',
                'ca5': latency + '0010'+'0100' + '1001'+'001x' + '000x'+'1101',
            }},
        )

    def test_lpddr4_command_pads(self):
        latency = '00000000' * self.CMD_LATENCY
        read = dict(cs_n=0, cas_n=0, ras_n=1, we_n=1)
        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {
                    0: dict(cke=1, odt=1, reset_n=1, **read),
                    2: dict(cke=0, odt=1, reset_n=0, **read),
                    3: dict(cke=1, odt=0, reset_n=0, **read),
                    5: dict(cke=0, odt=1, reset_n=1, **read),
                    7: dict(cke=0, odt=0, reset_n=0, **read),
                },
            ],
            pad_checkers = {"sys8x_90": {
                'cs':      latency + '10100101',  # p2, p3, p7 ignored
                'cke':     latency + '10010000',
                'odt':     latency + '10100100',
                'reset_n': latency + '11001110',
            }},
        )

    def wrdata_to_dq(self, dq_i, dfi_phases, nphases=8):
        # data on DQ should go in a pattern:
        # dq0: p0.wrdata[0], p0.wrdata[16], p1.wrdata[0], p1.wrdata[16], ...
        # dq1: p0.wrdata[1], p0.wrdata[17], p1.wrdata[1], p1.wrdata[17], ...
        for p in range(nphases):
            wrdata = dfi_phases[p]["wrdata"]
            yield bit(0  + dq_i, wrdata)
            yield bit(16 + dq_i, wrdata)

    def test_lpddr4_dq_out(self):
        dut = SimulationPHY()
        zero = '00000000' * 2  # zero for 1 sysclk clock in sys8x_ddr clock domain

        dfi_data = {
            0: dict(wrdata=0x11112222),
            1: dict(wrdata=0x33334444),
            2: dict(wrdata=0x55556666),
            3: dict(wrdata=0x77778888),
            4: dict(wrdata=0x9999aaaa),
            5: dict(wrdata=0xbbbbcccc),
            6: dict(wrdata=0xddddeeee),
            7: dict(wrdata=0xffff0000),
        }
        dfi_wrdata_en = {0: dict(wrdata_en=1)}  # wrdata_en=1 required on any single phase

        def dq_pattern(i):
            return ''.join(str(v) for v in self.wrdata_to_dq(i, dfi_data))

        self.run_test(dut,
            dfi_sequence = [dfi_wrdata_en, {}, dfi_data],
            pad_checkers = {"sys8x_90_ddr": {
                f'dq{i}': (self.CMD_LATENCY+1)*zero + zero + dq_pattern(i) + zero for i in range(16)
            }},
        )

    def test_lpddr4_dq_only_1cycle(self):
        dut = SimulationPHY()
        zero = '00000000' * 2

        dfi_data = {
            0: dict(wrdata=0x11112222),
            1: dict(wrdata=0x33334444),
            2: dict(wrdata=0x55556666),
            3: dict(wrdata=0x77778888),
            4: dict(wrdata=0x9999aaaa),
            5: dict(wrdata=0xbbbbcccc),
            6: dict(wrdata=0xddddeeee),
            7: dict(wrdata=0xffff0000),
        }
        dfi_wrdata_en = copy.deepcopy(dfi_data)
        dfi_wrdata_en[0].update(dict(wrdata_en=1))

        def dq_pattern(i):
            return ''.join(str(v) for v in self.wrdata_to_dq(i, dfi_data))

        self.run_test(dut,
            dfi_sequence = [dfi_wrdata_en, dfi_data, dfi_data],
            pad_checkers = {"sys8x_90_ddr": {
                f'dq{i}': (self.CMD_LATENCY+1)*zero + zero + dq_pattern(i) + zero for i in range(16)
            }},
        )

    def test_lpddr4_dqs(self):
        zero = '00000000' * 2

        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {0: dict(wrdata_en=1)},
                {},
                {  # to get 10101010... pattern on dq0 and only 1s on others
                    0: dict(wrdata=0xfffeffff),
                    1: dict(wrdata=0xfffeffff),
                    2: dict(wrdata=0xfffeffff),
                    3: dict(wrdata=0xfffeffff),
                    4: dict(wrdata=0xfffeffff),
                    5: dict(wrdata=0xfffeffff),
                    6: dict(wrdata=0xfffeffff),
                    7: dict(wrdata=0xfffeffff),
                },
            ],
            pad_checkers = {
                "sys8x_90_ddr": {
                    'dq0':  (self.CMD_LATENCY+1)*zero + '00000000'+'00000000' + '10101010'+'10101010' + '00000000'+'00000000' + zero,
                    'dq1':  (self.CMD_LATENCY+1)*zero + '00000000'+'00000000' + '11111111'+'11111111' + '00000000'+'00000000' + zero,
                },
                "sys8x_ddr": {  # preamble, pattern, preamble
                    'dqs0': (self.CMD_LATENCY+1)*zero + '01010101'+'01010100' + '01010101'+'01010101' + '00010101'+'01010101' + zero,
                    'dqs1': (self.CMD_LATENCY+1)*zero + '01010101'+'01010100' + '01010101'+'01010101' + '00010101'+'01010101' + zero,
                }
            },
        )

    def test_lpddr4_dmi_no_mask(self):
        # There should be no masking
        zero = '00000000' * 2

        self.run_test(SimulationPHY(),
            dfi_sequence = [
                {0: dict(wrdata_en=1)},
                {},
                {
                    0: dict(wrdata=0xffffffff),
                    1: dict(wrdata=0xffffffff),
                    2: dict(wrdata=0xffffffff),
                    3: dict(wrdata=0xffffffff),
                    4: dict(wrdata=0xffffffff),
                    5: dict(wrdata=0xffffffff),
                    6: dict(wrdata=0xffffffff),
                    7: dict(wrdata=0xffffffff),
                },
            ],
            pad_checkers = {
                "sys8x_90_ddr": {
                    'dq0':  (self.CMD_LATENCY+1)*zero + zero + '11111111'+'11111111' + 2*zero,
                },
                "sys8x_ddr": {
                    'dmi0': (self.CMD_LATENCY+1)*zero + (3 + 1)*zero,
                    'dmi1': (self.CMD_LATENCY+1)*zero + (3 + 1)*zero,
                }
            },
        )
