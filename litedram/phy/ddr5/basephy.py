#
# This file is part of LiteDRAM.
#
# Copyright (c) 2022 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

from operator import or_, and_, xor, add
from functools import reduce

from litedram.phy.sim_utils import SimLogger, log_level_getter

from migen import *
from migen.genlib.fifo import SyncFIFO

from litex.soc.interconnect.csr import *

from litedram.common import *
from litedram.phy.dfi import *

from litedram.phy.utils import (bitpattern, delayed, Serializer, Deserializer, Latency,
    CommandsPipeline)
from litedram.phy.ddr5.commands import DFIPhaseAdapter


class DDR5Output:
    """
        Unserialized output of DDR5PHY.
        Has to be serialized by concrete implementation.
    """
    def __init__(self, nphases, databits, nranks, nstrobes, with_sub_channels=False, name=None):
        self.ck_t   = Signal(2*nphases)
        self.ck_c   = Signal(2*nphases)
        self.reset_n = Signal(2*nphases)  # Serializer will work in ddr mode
        self.alert_n = Signal(2*nphases)  # Deserializer will work in ddr mode

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]

        for prefix in prefixes:
            setattr(self, prefix+'cs_n', [Signal(2*nphases, reset=2**(2*nphases)-1, name=name and f"{name}_{i}_cs_n") for i in range(nranks)])
            setattr(self, prefix+'ca',   [Signal(2*nphases, name=name and f"{name}_{i}_ca") for i in range(14)])
            # 2*nphases, as phy will run in ddr mode

            setattr(self, prefix+'par',  Signal(2*nphases, name=name and name+"par_n"))

            setattr(self, prefix+'dq_o',  [Signal(2*nphases, name=name and f"{name}_{i}_dq_o") for i in range(databits)])
            setattr(self, prefix+'dq_oe', [Signal(2*nphases, name=name and f"{name}_{i}_dq_oe") for i in range(nstrobes)])
            setattr(self, prefix+'dq_i',  [Signal(2*nphases, name=name and f"{name}_{i}_dq_i") for i in range(databits)])

            setattr(self, prefix+'dm_n_o',  [Signal(2*nphases, name=name and f"{name}_{i}_dm_n_o") for i in range(nstrobes)])
            setattr(self, prefix+'dm_n_i',  [Signal(2*nphases, name=name and f"{name}_{i}_dm_i_o") for i in range(nstrobes)])

            setattr(self, prefix+'dqs_t_o',  [Signal(2*nphases, name=name and f"{name}_{i}_dqs_t_o") for i in range(nstrobes)])
            setattr(self, prefix+'dqs_t_i',  [Signal(2*nphases, name=name and f"{name}_{i}_dqs_t_i") for i in range(nstrobes)])
            setattr(self, prefix+'dqs_oe',   [Signal(2*nphases, name=name and f"{name}_{i}_dqs_oe") for i in range(nstrobes)])
            setattr(self, prefix+'dqs_c_o',  [Signal(2*nphases, name=name and f"{name}_{i}_dqs_c_o") for i in range(nstrobes)])
            setattr(self, prefix+'dqs_c_i',  [Signal(2*nphases, name=name and f"{name}_{i}_dqs_c_i") for i in range(nstrobes)])


class DDR5DQOePattern(Module):
    def __init__(self, nphases):
        self.window = window = Signal(nphases+2)
        self.oe = Signal(2*nphases)
        self.comb += [
            self.oe.eq(Cat([Replicate(window[i], 2) for i in range(nphases)])),
        ]


class DDR5DQSPattern(Module):
    def __init__(self, nphases, wlevel_en: Signal()):
        self.window = window = Signal(nphases + 2)
        self.o  = Signal(2*nphases)
        self.oe = Signal(2*nphases)

        # # #

        # DQS Pattern transmitted as LSB-first.
        # Always enabled in write leveling mode, else during transfers
        # Preamble is 2 cycles and postamble is 0.5 cycle

        cases = []

        for i in range(nphases):
            cases.extend([
                If(reduce(or_, window[i:i+2]),
                    self.o[2*i:2*i+2].eq(0b01),
                ).Else(
                    self.o[2*i:2*i+2].eq(0),
                ),
                If(reduce(or_, window[i:i+3]) | wlevel_en,
                    self.oe[2*i:2*i+2].eq(0b11),
                ).Else(
                    self.o[2*i:2*i+2].eq(0),
                ),
            ])

        self.comb += [
            self.o.eq(0),
            self.oe.eq(0),
            *cases,
        ]


class DDR5PHY(Module, AutoCSR):
    """Core of DDR5 PHYs.

    This class implements all the logic required to convert DFI to/from pads.
    It works in a single clock domain. Signals for DRAM pads are stored in
    DDR5Output (self.out). Concrete implementations of DDR5 PHYs derive
    from this class and perform (de-)serialization of DDR5Output to pads.

    DFI commands
    ------------
    Not all DDR5 commands map directly to DFI commands. For this reason ZQC
    is treated specially in that DFI ZQC is translated into DDR5 MPC and has
    different interpretation depending on DFI.address.

    Due to the fact that DDR5 has 256-bit Mode Register space, the DFI MRS
    command encodes both register address *and* value in DFI.address (instead
    of the default in LiteDRAM to split them between DFI.address and DFI.bank).
    The MRS command is used both for Mode Register Write and Mode Register Read.
    The command is selected based on the value of DFI.bank.

    Refer to the documentation in `commands.py` for further information.

    Parameters
    ----------
    pads : object
        Object containing DDR5 pads.
    sys_clk_freq : float
        Frequency of memory controller's clock.
    ser_latency : Latency
        Additional latency introduced due to signal serialization.
    des_latency : Latency
        Additional latency introduced during signal deserialization.
    phytype : str
        Name of the PHY (concrete implementation).
    cmd_delay : int
        Used to force cmd delay during initialization in BIOS.
    masked_write : bool
        Use masked variant of WRITE command.
    """
    def __init__(self, pads, *,
                 sys_clk_freq, ser_latency, des_latency, phytype, with_sub_channels=False,
                 cmd_delay=None, masked_write=False, extended_overlaps_check=False,
                 with_odelay=False, with_clock_odelay=False, with_address_odelay=False,
                 with_idelay=False, with_per_dq_idelay=False, csr_cdc=None,
                 rd_extra_delay=Latency(sys=0), address_lines=13,
                 default_read_latency=0, default_write_latency=0):

        self.pads        = pads
        self.memtype     = memtype     = "DDR5"
        self.nranks      = nranks      = len(pads.cs_n) if hasattr(pads, "cs_n") else len(pads.A_cs_n) if hasattr(pads, "A_cs_n") else 1
        self.databits    = databits    = len(pads.dq) if hasattr(pads, "dq") else len(pads.A_dq)
        self.strobes     = strobes     = len(pads.dqs_t) if hasattr(pads, "dqs_t") else len(pads.A_dqs_t)
        self.addressbits = addressbits = 18 # for activate row address
        self.bankbits    = bankbits    = 8  # 5 bankbits, but we use 8 for Mode Register address in MRS
        self.nphases     = nphases     = 4
        self.with_sub_channels         = with_sub_channels
        self.tck         = tck         = 1 / (nphases*sys_clk_freq)
        assert databits % 4 == 0

        self.with_per_dq_idelay = with_per_dq_idelay

        # Parameters -------------------------------------------------------------------------------
        def get_cl_cw(memtype, tck):
            f_to_cl_cwl = OrderedDict()
            f_to_cl_cwl[3200e6] = 22
            f_to_cl_cwl[3600e6] = 28
            f_to_cl_cwl[4000e6] = 32
            f_to_cl_cwl[4400e6] = 36
            f_to_cl_cwl[4800e6] = 40
            f_to_cl_cwl[5200e6] = 42
            f_to_cl_cwl[5600e6] = 46
            f_to_cl_cwl[6000e6] = 50
            f_to_cl_cwl[6400e6] = 54
            f_to_cl_cwl[6800e6] = 56
            for f, cl in f_to_cl_cwl.items():
                if tck > 1/f:
                    return cl
            raise ValueError

        # Commands are sent over 2 DRAM clocks (sys4x) and we count cl/cwl from last bit
        cmd_latency     = 2

        cl              = get_cl_cw(memtype, tck)
        cwl = cl - 2
        # Read latency
        # This value should be the worst case delay between sending a read cmd and
        # getting data back. There will be exact delay may vary based on the training result.
        self.min_read_latency  = min_read_latency = (
            cmd_latency +        # CMD latency + extra clock cycle for 2N mode
            ser_latency.sys4x +  # CMD serialization latency
            des_latency.sys4x +  # Data deserialization latency
            rd_extra_delay.sys4x # Delays like CDCs
        ) # CL 0
        self.max_read_latency = max_read_latency = min_read_latency + 64 + 1 # CL 64 and 2N mode
        read_latency = (max_read_latency + nphases - 1) // nphases
        # Write latency
        # Set to 0, Training PHY will align DQS and DQ for write commands
        # See write leveling training in JESD79-5A
        # Max supported latency is 63 DRAm bus cycles
        self.min_write_latency = min_write_latency = (
            4 + # wrdata_en 0 tap delay
            4 + # Bitslip
            2   # We need to look 2 cycles "into the future" to properly generate write preable
        )

        # Registers --------------------------------------------------------------------------------

        def cdc(i):
            if csr_cdc is None:
                return i
            return csr_cdc(i)

        self._rst           = CSRStorage()
        self._rst_cdc       = cdc(self._rst.storage)
        self._rdimm_mode    = CSRStorage()

        self._rdphase = CSRStorage(log2_int(nphases), reset=0)
        self._wrphase = CSRStorage(log2_int(nphases), reset=0)

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]

        self._l = _l = dict()

        self.dq_dqs_ratio = dq_dqs_ratio = databits // strobes

        if with_odelay or with_clock_odelay:
            setattr(self, 'ckdly_rst' , CSR(name='ckdly_rst'))
            setattr(self, 'ckdly_stb' , CSR(name='ckdly_stb'))
            _l['ckdly_rst'] = cdc(getattr(self, 'ckdly_rst').re | self._rst.storage)
            _l['ckdly_stb'] = cdc(getattr(self, 'ckdly_stb').re)

        for prefix in prefixes:
            setattr(self, prefix+'wlevel_en', CSRStorage(name=prefix+'wlevel_en'))

            setattr(self, prefix+'dly_sel', CSRStorage(max(strobes, 14, nranks), name=prefix+'dly_sel'))
            getattr(self, prefix+'dly_sel').storage.attr.add("mr_ff")
            getattr(self, prefix+'dly_sel').storage.attr.add("keep")

            setattr(self, prefix+'ck_rdly_inc', CSR(name=prefix+'ck_rdly_inc'))
            setattr(self, prefix+'ck_rdly_dec', CSR(name=prefix+'ck_rdly_dec'))
            setattr(self, prefix+'ck_wdly_inc', CSR(name=prefix+'ck_wdly_inc'))
            setattr(self, prefix+'ck_wdly_dec', CSR(name=prefix+'ck_wdly_dec'))

            setattr(self, prefix+'dly_up', CSRStorage(1, name=prefix+'dly_up'))
            getattr(self, prefix+'dly_up').storage.attr.add("mr_ff")
            getattr(self, prefix+'dly_up').storage.attr.add("keep")
            if with_per_dq_idelay :
                setattr(self, prefix+'dq_dly_sel', CSRStorage(dq_dqs_ratio))
                getattr(self, prefix+'dq_dly_sel').storage.attr.add("mr_ff")
                getattr(self, prefix+'dq_dly_sel').storage.attr.add("keep")

            if with_odelay or with_address_odelay:
                setattr(self, prefix+'csdly_rst' , CSR(name=prefix+'csdly_rst'))
                setattr(self, prefix+'csdly_stb',  CSR(name=prefix+'csdly_stb'))
                setattr(self, prefix+'cadly_rst',  CSR(name=prefix+'cadly_rst'))
                setattr(self, prefix+'cadly_stb',  CSR(name=prefix+'cadly_stb'))
                setattr(self, prefix+'pardly_rst', CSR(name=prefix+'pardly_rst'))
                setattr(self, prefix+'pardly_stb', CSR(name=prefix+'pardly_stb'))

            if with_idelay:
                setattr(self, prefix+'rdly_dq_rst',  CSR(name=prefix+'rdly_dq_rst'))
                setattr(self, prefix+'rdly_dq_stb',  CSR(name=prefix+'rdly_dq_stb'))
                setattr(self, prefix+'rdly_dqs_rst', CSR(name=prefix+'rdly_dqs_rst'))
                setattr(self, prefix+'rdly_dqs_stb', CSR(name=prefix+'rdly_dqs_stb'))

            setattr(self, prefix+'rdly_dq_bitslip_rst', CSR(name=prefix+'rdly_dq_bitslip_rst'))
            setattr(self, prefix+'rdly_dq_bitslip'    , CSR(name=prefix+'rdly_dq_bitslip'))
            setattr(self, prefix+'rdly_dqs_bitslip_rst', CSR(name=prefix+'rdly_dqs_bitslip_rst'))
            setattr(self, prefix+'rdly_dqs_bitslip'    , CSR(name=prefix+'rdly_dqs_bitslip'))

            if with_odelay:
                setattr(self, prefix+'wdly_dq_rst',  CSR(name=prefix+'wdly_dq_rst'))
                setattr(self, prefix+'wdly_dq_stb',  CSR(name=prefix+'wdly_dq_stb'))
                setattr(self, prefix+'wdly_dm_rst',  CSR(name=prefix+'wdly_dm_rst'))
                setattr(self, prefix+'wdly_dm_stb',  CSR(name=prefix+'wdly_dm_stb'))
                setattr(self, prefix+'wdly_dqs_rst', CSR(name=prefix+'wdly_dqs_rst'))
                setattr(self, prefix+'wdly_dqs_stb', CSR(name=prefix+'wdly_dqs_stb'))

            setattr(self, prefix+'wdly_dq_bitslip_rst', CSR(name=prefix+'wdly_dq_bitslip_rst'))
            setattr(self, prefix+'wdly_dq_bitslip'    , CSR(name=prefix+'wdly_dq_bitslip'))
            setattr(self, prefix+'wdly_dqs_bitslip_rst', CSR(name=prefix+'wdly_dqs_bitslip_rst'))
            setattr(self, prefix+'wdly_dqs_bitslip'    , CSR(name=prefix+'wdly_dqs_bitslip'))

            if with_idelay:
                _l[prefix+'rdly_dq_rst']  = cdc(getattr(self, prefix+'rdly_dq_rst').re)
                _l[prefix+'rdly_dq_stb']  = cdc(getattr(self, prefix+'rdly_dq_stb').re)
                _l[prefix+'rdly_dqs_rst']  = cdc(getattr(self, prefix+'rdly_dqs_rst').re)
                _l[prefix+'rdly_dqs_stb']  = cdc(getattr(self, prefix+'rdly_dqs_stb').re)

            _l[prefix+'rdly_dq_bitslip_rst']  = cdc(getattr(self, prefix+'rdly_dq_bitslip_rst').re)
            _l[prefix+'rdly_dq_bitslip']  = cdc(getattr(self, prefix+'rdly_dq_bitslip').re)

            if with_odelay or with_address_odelay:
                _l[prefix+'csdly_rst']    = cdc(getattr(self, prefix+'csdly_rst').re | self._rst.storage)
                _l[prefix+'csdly_stb']    = cdc(getattr(self, prefix+'csdly_stb').re)
                _l[prefix+'cadly_rst']    = cdc(getattr(self, prefix+'cadly_rst').re | self._rst.storage)
                _l[prefix+'cadly_stb']    = cdc(getattr(self, prefix+'cadly_stb').re)
                _l[prefix+'pardly_rst']   = cdc(getattr(self, prefix+'pardly_rst').re | self._rst.storage)
                _l[prefix+'pardly_stb']   = cdc(getattr(self, prefix+'pardly_stb').re)

            if with_odelay:
                _l[prefix+'wdly_dq_rst']  = cdc(getattr(self, prefix+'wdly_dq_rst').re | self._rst.storage)
                _l[prefix+'wdly_dq_stb']  = cdc(getattr(self, prefix+'wdly_dq_stb').re)
                _l[prefix+'wdly_dm_rst']  = cdc(getattr(self, prefix+'wdly_dm_rst').re | self._rst.storage)
                _l[prefix+'wdly_dm_stb']  = cdc(getattr(self, prefix+'wdly_dm_stb').re)
                _l[prefix+'wdly_dqs_rst'] = cdc(getattr(self, prefix+'wdly_dqs_rst').re | self._rst.storage)
                _l[prefix+'wdly_dqs_stb'] = cdc(getattr(self, prefix+'wdly_dqs_stb').re)

            _l[prefix+'wdly_dq_bitslip_rst']  = cdc(getattr(self, prefix+'wdly_dq_bitslip_rst').re | self._rst.storage)
            _l[prefix+'wdly_dq_bitslip']  = cdc(getattr(self, prefix+'wdly_dq_bitslip').re)
            _l[prefix+'wdly_dqs_bitslip_rst']  = cdc(getattr(self, prefix+'wdly_dqs_bitslip_rst').re | self._rst.storage)
            _l[prefix+'wdly_dqs_bitslip']  = cdc(getattr(self, prefix+'wdly_dqs_bitslip').re)

        combined_data_bits = databits if not with_sub_channels else 2*databits
        combined_strobes = strobes if not with_sub_channels else 2*strobes

        # PHY settings -----------------------------------------------------------------------------
        self.settings = PhySettings(
            phytype       = phytype,
            memtype       = memtype,
            databits      = combined_data_bits,
            dfi_databits  = 2*combined_data_bits,
            nranks        = nranks,
            nphases       = nphases,
            rdphase       = self._rdphase.storage,
            wrphase       = self._wrphase.storage,
            cl            = cl,
            cwl           = cwl,
            read_latency  = read_latency + 2,
            write_latency = 0,
            cmd_latency   = cmd_latency,
            cmd_delay     = cmd_delay,
            bitslips      = 8,
            strobes       = combined_strobes,
            min_write_latency   = min_write_latency,
            min_read_latency    = 2,
            address_lines       = address_lines,
            min_write_latency   = min_write_latency,
            min_read_latency    = min_read_latency,
            with_sub_channels   = with_sub_channels,
            with_clock_odelay   = with_clock_odelay,
            with_address_odelay = with_address_odelay,
            with_odelay         = with_odelay,
            with_idelay         = with_idelay,
            with_per_dq_idelay  = with_per_dq_idelay,
        )

        # DFI Interface ----------------------------------------------------------------------------
        self.dfi = dfi = Interface(14, 1, nranks, 2*combined_data_bits, nphases=4, with_sub_channels=with_sub_channels)

        # Now prepare the data by converting the sequences on adapters into sequences on the pads.
        # We have to ignore overlapping commands, and module timings have to ensure that there are
        # no overlapping commands anyway.
        self.out = DDR5Output(nphases, databits, nranks, strobes, with_sub_channels, name="basephy")

        # Clocks -----------------------------------------------------------------------------------
        self.comb += self.out.ck_t.eq(bitpattern("-_-_-_-_"))
        self.comb += self.out.ck_c.eq(bitpattern("_-_-_-_-"))

        # Simple commands --------------------------------------------------------------------------
        self.comb += self.out.reset_n.eq(Cat((phase.reset_n, phase.reset_n) for phase in dfi.phases))
        self.comb += [phase.alert_n.eq(self.out.alert_n[i*2] & self.out.alert_n[i*2+1]) for i, phase in enumerate(self.dfi.phases)]

        for prefix in prefixes:
            # DDR5 Commands --------------------------------------------------------------------------

            for rank in range(nranks):
                carry_cs_n = Signal(reset=1)
                self.sync += [
                    If(dfi.phases[-1].mode_2n,
                        carry_cs_n.eq(getattr(dfi.phases[-1], prefix).cs_n[rank])
                    ).Else(
                        carry_cs_n.eq(1)
                    )
                ]

                for j, phase in enumerate(dfi.phases):
                    self.comb += [
                        If(~phase.mode_2n,
                            getattr(self.out, prefix + 'cs_n')[rank][2*j].eq(
                                getattr(phase, prefix).cs_n[rank] & (carry_cs_n if j == 0 else 1)
                            ),
                            getattr(self.out, prefix + 'cs_n')[rank][2*j+1].eq(
                                getattr(phase, prefix).cs_n[rank]
                            ),
                        ).Else(
                            getattr(self.out, prefix + 'cs_n')[rank][2*j].eq(
                                carry_cs_n if j == 0 else getattr(dfi.phases[j-1], prefix).cs_n[rank]
                            ),
                            getattr(self.out, prefix + 'cs_n')[rank][2*j+1].eq(
                                getattr(phase, prefix).cs_n[rank]
                            ),
                        ),
                    ]

            self.comb += getattr(self.out, prefix + 'par').eq(
                Cat([reduce(xor, getattr(phase, prefix).address[7*i:7+7*i])] for phase in dfi.phases for i in range(2)))

            stage_cnt = Signal()
            self.sync += [
                If(~reduce(and_, getattr(phase, prefix).cs_n),
                    stage_cnt.eq(0),
                ).Else(
                    stage_cnt.eq(~stage_cnt),
                )
            ]

            for bit in range(7):
                for j, phase in enumerate(dfi.phases):
                    self.comb += [
                        If(self._rdimm_mode.storage,
                            If(phase.mode_2n,
                                If(~reduce(and_, getattr(phase, prefix).cs_n) | stage_cnt == 0,
                                    getattr(self.out, prefix+'ca')[bit].eq(Replicate(getattr(phase, prefix).address[bit], 2))
                                ).Else(
                                    getattr(self.out, prefix+'ca')[bit].eq(Replicate(getattr(phase, prefix).address[bit + 7], 2))
                                )
                            ).Else(
                                getattr(self.out, prefix+'ca')[bit].eq(
                                    Cat([getattr(phase, prefix).address[bit + 7*i] for phase in dfi.phases for i in range (2)]))
                            ),
                        ).Else(
                            getattr(self.out, prefix+'ca')[bit].eq(
                                Cat([getattr(phase, prefix).address[bit] for phase in dfi.phases for _ in range (2)]))
                        ),
                    ]
            for bit in range(7, 14):
                self.comb += getattr(self.out, prefix+'ca')[bit].eq(
                    Cat([getattr(phase, prefix).address[bit] for phase in dfi.phases for _ in range (2)]))

            for strobe in range(strobes):
                # Read Control Path ------------------------------------------------------------------------
                # Creates a delay line of read commands coming from the DFI interface. The output is used to
                # signal a valid read data to the DFI interface.
                #
                # The read data valid is asserted for 1 sys_clk cycle when the data is available on the DFI
                # interface, the latency is the sum of the minimal PHY and user added delays.
                rddata_en = TappedDelayLine(
                    signal = Cat([getattr(dfi.phases[i], prefix).rddata_en for i in range(nphases)]),
                    ntaps  = read_latency + 2
                )
                self.submodules += rddata_en

                rd_window   = Signal(nphases)
                rd_delay    = Signal(max=4*read_latency, reset=min_read_latency + default_read_latency)
                rd_index    = Signal(max=read_latency)
                rd_offset   = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)
                nphases_log = nphases.bit_length() - 1

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                        getattr(self, prefix+'ck_rdly_inc').re,
                        rd_delay.eq(rd_delay + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                        getattr(self, prefix+'ck_rdly_dec').re,
                        rd_delay.eq(rd_delay - 1),
                    ),
                ]

                self.comb += [
                    rd_index.eq(rd_delay[nphases_log:]),
                    rd_offset.eq(rd_delay[:nphases_log]),
                ]

                rd_cases = {}
                rd_cases[0] = rd_window.eq(rddata_en.taps[rd_index])
                if nphases > 1:
                    for i in range(1, nphases):
                        rd_cases[i] = rd_window.eq(
                            Cat(rddata_en.taps[rd_index+1][:i], rddata_en.taps[rd_index][i:]))

                self.comb += [
                    Case(rd_offset,
                        rd_cases,
                    )
                ]

                # Read Data Path ----------------------------------------------------------------------------
                # The rd_window can present any arbitrary 1*0*1* pattern of length 4.
                # We detect where one full DFI phase of data finishes and where other starts
                # by counting how many valid bits are set in the rd_window, and how many
                # are set in range [0:i-1], for the i = {0, .., nphases-1}.
                # When data for full DFI phase are collected, they are stored in FIFO and await
                # for settings.read_latency-1 to pass before being presented on DFI bus.

                fifo = SyncFIFO(width=dq_dqs_ratio*nphases*2, depth=read_latency, fwft=False)
                self.submodules += fifo

                rddata_cnt          = Signal(max=nphases)
                rddata_intermediate = Array(Signal(2*dq_dqs_ratio) for _ in range(nphases))
                rddata_sel          = Array(Signal(2*dq_dqs_ratio) for _ in range(nphases))

                rddata_cnt_tmps      = [Signal(max=nphases) for _ in range(nphases)]
                rddata_cnt_and_tmp   = [Signal(max=2*nphases) for _ in range(nphases)]
                rddata_cnt_all_valid = Signal(max=2*nphases)

                self.comb += rddata_cnt_all_valid.eq(rddata_cnt + reduce(add, rd_window))

                for i in range(nphases):
                    dq_offset = strobe*dq_dqs_ratio
                    dq_start  = i*2
                    dq_end    = (i+1)*2
                    self.comb += [
                        rddata_cnt_tmps[i].eq(reduce(add, rd_window[:i], 0)),
                        rddata_cnt_and_tmp[i].eq(rddata_cnt + rddata_cnt_tmps[i]),
                        If(rd_window[i] & ~rddata_cnt_and_tmp[i][nphases_log] & rddata_cnt_all_valid[nphases_log],
                            rddata_sel[rddata_cnt_and_tmp[i][:nphases_log]].eq(
                                Cat([getattr(self.out, prefix+'dq_i')[dq_offset+dq][2*i] for dq in range(dq_dqs_ratio)] +
                                    [getattr(self.out, prefix+'dq_i')[dq_offset+dq][2*i+1] for dq in range(dq_dqs_ratio)])),
                        ),
                        If(i < rddata_cnt,
                            rddata_sel[i].eq(rddata_intermediate[i]),
                        ),
                    ]

                    self.sync += [
                        If(rd_window[i] & (rddata_cnt_and_tmp[i][nphases_log] | ~rddata_cnt_all_valid[nphases_log]),
                            rddata_intermediate[rddata_cnt_and_tmp[i][:nphases_log]].eq(
                                Cat([getattr(self.out, prefix+'dq_i')[dq_offset+dq][2*i] for dq in range(dq_dqs_ratio)] +
                                    [getattr(self.out, prefix+'dq_i')[dq_offset+dq][2*i+1] for dq in range(dq_dqs_ratio)])),
                        )
                    ]

                self.sync += [
                    fifo.din.eq(0),
                    fifo.we.eq(0),
                    If(reduce(or_, rd_window),
                        If(rddata_cnt_all_valid[nphases_log],
                            fifo.din.eq(Cat(rddata_sel)),
                            fifo.we.eq(1),
                        ),
                        rddata_cnt.eq(rddata_cnt_all_valid[:nphases_log]),
                    ),
                ]

                # Retime
                self.comb += [
                    getattr(phase, prefix).rddata_valid.eq( \
                        getattr(phase, prefix).rddata_valid | \
                        reduce(or_, rddata_en.output) | \
                        getattr(self, prefix+'wlevel_en').storage) \
                    for i, phase in enumerate(self.dfi.phases)
                ]

                rddata_start = strobe*2*dq_dqs_ratio
                rddata_end   = (strobe+1)*2*dq_dqs_ratio

                fifo_good = Signal()
                self.sync += [
                    fifo_good.eq(fifo.re & fifo.readable)
                ]

                self.comb += [
                    If(fifo_good, getattr(phase, prefix).rddata[rddata_start:rddata_end].eq(fifo.dout[i*2*dq_dqs_ratio:(i+1)*2*dq_dqs_ratio])) \
                    for i, phase in enumerate(self.dfi.phases)
                ] + [
                    fifo.re.eq(reduce(or_, rddata_en.taps[read_latency]))
                ]

                # Write Control Path -----------------------------------------------------------------------
                wrtap = 64//nphases - 1
                assert wrtap >= 0

                # Create a delay line of write commands coming from the DFI interface. This taps are used to
                # control DQ/DQS tristates.
                wrdata_en = TappedDelayLine(
                    signal = Cat([getattr(dfi.phases[i], prefix).wrdata_en for i in range(nphases)]),
                    ntaps  = wrtap
                )
                self.submodules += wrdata_en

                wr_window   = Signal(nphases + 2)
                wr_delay    = Signal(max=4*read_latency, reset=min_write_latency + default_write_latency)
                wr_index    = Signal(max=64//nphases)
                wr_offset   = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                wr_update   = Signal()

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                        getattr(self, prefix+'ck_wdly_inc').re,
                        wr_delay.eq(wr_delay + 1),
                        wr_update.eq(1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                        getattr(self, prefix+'ck_wdly_dec').re,
                        wr_delay.eq(wr_delay - 1),
                        wr_update.eq(1),
                    ),
                ]

                self.comb += [
                    wr_index.eq(wr_delay[nphases_log:]),
                    wr_offset.eq(wr_delay[:nphases_log]),
                ]

                wr_cases = {}
                if nphases == 1:
                    wr_cases[0] = wr_window.eq(Cat(wrdata_en.taps[wr_index+2], wrdata_en.taps[wr_index+1], wrdata_en.taps[wr_index]))
                else:
                    for i in range(nphases-2):
                        wr_cases[i] = wr_window.eq(Cat(wrdata_en.taps[wr_index+1][nphases-i-2:], wrdata_en.taps[wr_index][:nphases-i]))
                    wr_cases[nphases-1] = wr_window.eq(Cat(wrdata_en.taps[wr_index+2][nphases-1],
                                                       wrdata_en.taps[wr_index+1],
                                                       wrdata_en.taps[wr_index][0]))
                self.comb += [
                    Case(wr_offset,
                        wr_cases,
                    )
                ]

                dqs_oe        = Signal(2*nphases)
                dqs_pattern   = DDR5DQSPattern(
                    nphases       = nphases,
                    wlevel_en     = getattr(self, prefix+'wlevel_en').storage,
                )
                self.comb += dqs_pattern.window.eq(wr_window)
                self.submodules += dqs_pattern

                dq_oe        = Signal(2*nphases)
                dq_pattern   = DDR5DQOePattern(nphases=nphases)
                self.comb += dq_pattern.window.eq(wr_window)
                self.submodules += dq_pattern

                for byte in range(strobes):
                    # output
                    dqs_bitslip    = BitSlip(8,
                        i      = dqs_pattern.o,
                        rst    = self.get_rst(byte, getattr(self, prefix+'wdly_dqs_bitslip_rst').re, prefix),
                        slp    = self.get_stb(byte, getattr(self, prefix+'wdly_dqs_bitslip').re, prefix),
                        cycles = 1)
                    self.submodules += dqs_bitslip

                    dqs_oe_bitslip    = BitSlip(8,
                        i      = dqs_pattern.oe,
                        rst    = self.get_rst(byte, getattr(self, prefix+'wdly_dqs_bitslip_rst').re, prefix),
                        slp    = self.get_stb(byte, getattr(self, prefix+'wdly_dqs_bitslip').re, prefix),
                        cycles = 1)
                    self.submodules += dqs_oe_bitslip

                    dq_oe_bitslip    = BitSlip(8,
                        i      = dq_pattern.oe,
                        rst    = self.get_rst(byte, getattr(self, prefix+'wdly_dq_bitslip_rst').re, prefix),
                        slp    = self.get_stb(byte, getattr(self, prefix+'wdly_dq_bitslip').re, prefix),
                        cycles = 1)
                    self.submodules += dq_oe_bitslip

                    self.comb += [
                        getattr(self.out, prefix+'dqs_t_o')[byte].eq(dqs_bitslip.o),
                        getattr(self.out, prefix+'dqs_c_o')[byte].eq(~dqs_bitslip.o),
                        getattr(self.out, prefix+'dqs_oe')[byte].eq(dqs_oe_bitslip.o),
                        getattr(self.out, prefix+'dq_oe')[byte].eq(dq_oe_bitslip.o),
                    ]

                # DMI and DQ --------------------------------------------------------------------------

                write_head = Signal(5)
                wrdata     = []
                self.sync += write_head.eq(write_head + 1)
                output_memories = []
                for i in range(nphases):
                    phase = getattr(dfi.phases[i], prefix)
                    mem = Memory(2 * databits + 2 * strobes, 32, name=f"phase_{i}")
                    output_memories.append(mem)
                    self.specials += mem
                    write = mem.get_port(write_capable=True)
                    self.comb += [
                        write.adr.eq(write_head),
                        write.we.eq(phase.wrdata_en),
                        write.dat_w.eq(Cat(phase.wrdata, phase.wrdata_mask)),
                    ]
                    read_head = Signal(5, reset= -1 if i<nphases-2 else -2)
                    cases = {}
                    for j in range(nphases):
                        sub = 0
                        if i+j < nphases-2:
                            sub = 1
                        elif i+j < nphases+2:
                            sub = 2
                        else:
                            sub = 3
                        cases[j] = read_head.eq(write_head - wr_index - sub)
                    _offset = wr_offset if nphases > 1 else Signal(1, reset=0)
                    self.sync += [
                        If(wr_update,
                            Case(_offset,
                                cases
                            )
                        ).Else(
                            read_head.eq(read_head+1),
                        ),
                    ]
                    read = mem.get_port(async_read=True)
                    _wrdata = Signal(2 * databits + 2 * strobes)
                    self.comb += [
                        read.adr.eq(read_head),
                        _wrdata.eq(read.dat_r),
                    ]
                    wrdata.append(_wrdata)
                    self.specials += read, write

                # DMI --------------------------------------------------------------------------------------
                # DMI signal is used for Data Mask or Data Bus Invertion depending on Mode Registers values.
                # With DM and DBI disabled, this signal is a Don't Care.
                # With DM enabled, masking is performed only when the command used is WRITE-MASKED.
                # We don't support DBI, DM support is configured statically with `masked_write`.
                _cases = {}

                _wrdata_mem_mask = Signal(2*nphases*strobes)
                for i in range(nphases):
                    _cases[i] = _wrdata_mem_mask.eq(Cat([wrdata[(i+j+2)%4][2*databits:] for j in range(nphases)]))

                self.comb += [
                    Case(wr_offset,
                        _cases
                    ),
                ]
                for byte in range(strobes):
                    if isinstance(masked_write, Signal) or masked_write:
                        dm_i = [
                            _wrdata_mem_mask[i * strobes + byte] for i in range(2*nphases)
                        ]
                        dm_o_bitslip = BitSlip(8,
                            i      = dm_i,
                            rst    = self.get_rst(byte, getattr(self, prefix+'wdly_dq_bitslip_rst').re, prefix),
                            slp    = self.get_stb(byte, getattr(self, prefix+'wdly_dq_bitslip').re, prefix),
                            cycles = 1)

                        self.submodules += dm_o_bitslip
                        self.comb += getattr(self.out, prefix+'dm_n_o')[byte].eq(Cat(dm_o_bitslip.o))

                    else:
                        self.comb += getattr(self.out, prefix+'dm_n_o')[byte].eq(0)

                # DQ ---------------------------------------------------------------------------------------
                _cases = {}

                _wrdata_mem = Signal(2*nphases*databits)
                for i in range(nphases):
                    _cases[i] = _wrdata_mem.eq(Cat([wrdata[(i+j+2)%4][:2*databits] for j in range(nphases)]))

                self.comb += [
                    Case(wr_offset,
                        _cases
                    ),
                ]

                for bit in range(databits):
                    # output
                    _wrdata = [
                        _wrdata_mem[i * self.databits + bit] for i in range(2*nphases)
                    ]

                    dq_o_bitslip = BitSlip(8,
                        i      = Cat(*_wrdata),
                        rst    = self.get_rst(bit//dq_dqs_ratio, getattr(self, prefix+'wdly_dq_bitslip_rst').re, prefix),
                        slp    = self.get_stb(bit//dq_dqs_ratio, getattr(self, prefix+'wdly_dq_bitslip').re, prefix),
                        cycles = 1)

                    setattr(self.submodules, prefix+f'dq_o_bitslip_{bit}', dq_o_bitslip)
                    self.comb += getattr(self.out, prefix+'dq_o')[bit].eq(dq_o_bitslip.o)


    def get_rst(self, byte, rst, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        t = Signal()
        if not dq or not self.with_per_dq_idelay:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte] & rst) | self._rst.storage)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] & rst) | self._rst.storage)
        else:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] &
                            getattr(self, prefix+'dq_dly_sel').storage[byte%self.dq_dqs_ratio] & rst) |
                            self._rst.storage)
        return t

    def get_stb(self, byte, stb, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        t = Signal()
        if not dq or not self.with_per_dq_idelay:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte] & stb)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] & stb)
        else:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] &
                            getattr(self, prefix+'dq_dly_sel').storage[byte%self.dq_dqs_ratio] & stb)
        return t
