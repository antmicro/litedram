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

from litedram.phy.ddr5.BasePHYOutput import BasePHYOutput
from litedram.phy.ddr5.BasePHYPatternGenerators import DQOePattern, DQSPattern
from litedram.phy.ddr5.BasePHYAddressSlicer import PHYAddressSlicer


class DDR5PHY(Module, AutoCSR):
    """Core of DDR5 PHYs.

    This class implements all the logic required to convert DFI to/from pads.
    It works in a single clock domain. Signals for DRAM pads are stored in
    BasePHYOutput (self.out). Concrete implementations of DDR5 PHYs derive
    from this class and perform (de-)serialization of BasePHYOutput to pads.

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
                 sys_clk_freq, ser_latency, des_latency, phytype, direct_control,
                 with_sub_channels=False, cmd_delay=None, masked_write=False,
                 extended_overlaps_check=False, with_odelay=False,
                 with_clock_odelay=False, with_address_odelay=False,
                 with_idelay=False, with_per_dq_idelay=False, csr_cdc=None, csr_cdc_90=None,
                 rd_extra_delay=Latency(sys=0), address_lines=13,
                 i_domain=None, i_doman_ratio=1, o_doamin=None, o_domain_ratio=1,
                 default_read_latency=0, default_write_latency=0):

        self.pads        = pads
        self.memtype     = memtype     = "DDR5"
        self.nranks      = nranks      = 1 # no support for multiple ranks
        #self.nranks      = nranks      = len(pads.cs_n) if hasattr(pads, "cs_n") else len(pads.A_cs_n) if hasattr(pads, "A_cs_n") else 1
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

        # Address path delay before serialization
        addr_pre_ser_delay = PHYAddressSlicer.dfi_delay(nphases)
        assert addr_pre_ser_delay == 8

        self.des_latency          = des_latency
        self.ser_latency          = ser_latency
        self.ca_cdc_min_max_delay = (rd_extra_delay, rd_extra_delay)
        self.rd_cdc_min_max_delay = (Latency(sys=0), Latency(sys=0))
        self.wr_cdc_min_max_delay = (rd_extra_delay, rd_extra_delay)

        # Read latency
        # This value should be the worst case delay between sending a read cmd and
        # getting data back. The exact delay may vary based on the training result.
        self.min_read_latency  = min_read_latency = (
            cmd_latency - 1 +      # CMD latency
            addr_pre_ser_delay +   # PHY address buffering
            ser_latency.sys4x +    # CMD serialization latency
            rd_extra_delay.sys4x + # Delays like CDCs
            2 +                    # Minimal Preamble
            des_latency.sys4x      # Data deserialization latency
        ) # CL 0
        self.max_read_latency = max_read_latency = min_read_latency + 66 + 1 # CL 64 and 2N mode
        read_latency = (max_read_latency + nphases - 1) // nphases
        # Write latency
        # Set to 0, Training PHY will align DQS and DQ for write commands
        # See write leveling training in JESD79-5A
        # Max supported latency is 64 DRAM bus cycles + 1 for 2N mode
        write_addjust = -min(0, nphases + 2 - 1 - addr_pre_ser_delay)
        min_write_latency = nphases + 2 - 1 - addr_pre_ser_delay
        self.min_write_latency = min_write_latency + write_addjust
        self.max_write_latency = 64 + 1

        # Registers --------------------------------------------------------------------------------

        def cdc(i):
            if csr_cdc is None:
                return i
            return csr_cdc(i)

        def cdc_90(i):
            if csr_cdc_90 is None:
                return i
            return csr_cdc_90(i)

        self._rst           = CSRStorage()
        self._rst_cdc       = cdc(self._rst.storage)
        self._rst_cdc_90    = cdc_90(self._rst.storage)
        self._rdimm_mode    = CSRStorage()

        self._rdphase = CSRStorage(log2_int(nphases), reset=0)
        self._wrphase = CSRStorage(log2_int(nphases), reset=0)

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]

        self._l = _l = dict()

        self.dq_dqs_ratio = dq_dqs_ratio = databits // strobes

        self.alert = CSRStatus(1)
        self.alert_reduce = CSRStorage(1)

        if with_odelay or with_clock_odelay:
            setattr(self, 'ckdly_rst' , CSR(name='ckdly_rst'))
            setattr(self, 'ckdly_inc' , CSR(name='ckdly_inc'))
            _l['ckdly_rst'] = cdc(getattr(self, 'ckdly_rst').re | self._rst.storage)
            _l['ckdly_inc'] = cdc(getattr(self, 'ckdly_inc').re)

        for prefix in prefixes:
            setattr(self, prefix+'preamble', CSRStatus(2*2, name=prefix+'preamble'))

            setattr(self, prefix+'wlevel_en', CSRStorage(name=prefix+'wlevel_en'))

            setattr(self, prefix+'dly_sel', CSRStorage(max(strobes, 14, nranks), name=prefix+'dly_sel'))
            getattr(self, prefix+'dly_sel').storage.attr.add("mr_ff")
            getattr(self, prefix+'dly_sel').storage.attr.add("keep")

            setattr(self, prefix+'ck_rdly_inc', CSR(name=prefix+'ck_rdly_inc'))
            setattr(self, prefix+'ck_rdly_rst', CSR(name=prefix+'ck_rdly_rst'))
            setattr(self, prefix+'ck_wdly_inc', CSR(name=prefix+'ck_wdly_inc'))
            setattr(self, prefix+'ck_wdly_rst', CSR(name=prefix+'ck_wdly_rst'))
            setattr(self, prefix+'ck_wddly_inc', CSR(name=prefix+'ck_wddly_inc'))
            setattr(self, prefix+'ck_wddly_rst', CSR(name=prefix+'ck_wddly_rst'))


            if with_per_dq_idelay :
                setattr(self, prefix+'dq_dly_sel', CSRStorage(dq_dqs_ratio, name=prefix+'dq_dly_sel'))
                getattr(self, prefix+'dq_dly_sel').storage.attr.add("mr_ff")
                getattr(self, prefix+'dq_dly_sel').storage.attr.add("keep")

            if with_odelay or with_address_odelay:
                setattr(self, prefix+'csdly_rst',  CSR(name=prefix+'csdly_rst'))
                setattr(self, prefix+'csdly_inc',  CSR(name=prefix+'csdly_inc'))
                setattr(self, prefix+'cadly_rst',  CSR(name=prefix+'cadly_rst'))
                setattr(self, prefix+'cadly_inc',  CSR(name=prefix+'cadly_inc'))
                setattr(self, prefix+'pardly_rst', CSR(name=prefix+'pardly_rst'))
                setattr(self, prefix+'pardly_inc', CSR(name=prefix+'pardly_inc'))

                setattr(self, prefix+'cadly', CSRStatus(16, name=prefix+'cadly'))
                getattr(self, prefix+'cadly').status.attr.add("mr_ff")
                getattr(self, prefix+'cadly').status.attr.add("keep")

            if with_idelay:
                setattr(self, prefix+'rdly_dq_rst',  CSR(name=prefix+'rdly_dq_rst'))
                setattr(self, prefix+'rdly_dq_inc',  CSR(name=prefix+'rdly_dq_inc'))
                setattr(self, prefix+'rdly_dqs_rst', CSR(name=prefix+'rdly_dqs_rst'))
                setattr(self, prefix+'rdly_dqs_inc', CSR(name=prefix+'rdly_dqs_inc'))

                setattr(self, prefix+'rdly_dqs', CSRStatus(16, name=prefix+'rdly_dqs'))
                getattr(self, prefix+'rdly_dqs').status.attr.add("mr_ff")
                getattr(self, prefix+'rdly_dqs').status.attr.add("keep")
                setattr(self, prefix+'rdly_dq', CSRStatus(16, name=prefix+'rdly_dq'))
                getattr(self, prefix+'rdly_dq').status.attr.add("mr_ff")
                getattr(self, prefix+'rdly_dq').status.attr.add("keep")

            if with_odelay:
                setattr(self, prefix+'wdly_dq_rst',  CSR(name=prefix+'wdly_dq_rst'))
                setattr(self, prefix+'wdly_dq_inc',  CSR(name=prefix+'wdly_dq_inc'))
                setattr(self, prefix+'wdly_dm_rst',  CSR(name=prefix+'wdly_dm_rst'))
                setattr(self, prefix+'wdly_dm_inc',  CSR(name=prefix+'wdly_dm_inc'))
                setattr(self, prefix+'wdly_dqs_rst', CSR(name=prefix+'wdly_dqs_rst'))
                setattr(self, prefix+'wdly_dqs_inc', CSR(name=prefix+'wdly_dqs_inc'))

                setattr(self, prefix+'wdly_dqs', CSRStatus(16, name=prefix+'wdly_dqs'))
                getattr(self, prefix+'wdly_dqs').status.attr.add("mr_ff")
                getattr(self, prefix+'wdly_dqs').status.attr.add("keep")
                setattr(self, prefix+'wdly_dq', CSRStatus(16, name=prefix+'wdly_dq'))
                getattr(self, prefix+'wdly_dq').status.attr.add("mr_ff")
                getattr(self, prefix+'wdly_dq').status.attr.add("keep")
                setattr(self, prefix+'wdly_dm', CSRStatus(16, name=prefix+'wdly_dm'))
                getattr(self, prefix+'wdly_dm').status.attr.add("mr_ff")
                getattr(self, prefix+'wdly_dm').status.attr.add("keep")

            if with_idelay:
                _l[prefix+'rdly_dq_rst']  = cdc(getattr(self, prefix+'rdly_dq_rst').re)
                _l[prefix+'rdly_dq_inc']  = cdc(getattr(self, prefix+'rdly_dq_inc').re)
                _l[prefix+'rdly_dqs_rst']  = cdc(getattr(self, prefix+'rdly_dqs_rst').re)
                _l[prefix+'rdly_dqs_inc']  = cdc(getattr(self, prefix+'rdly_dqs_inc').re)

            if with_odelay or with_address_odelay:
                _l[prefix+'csdly_rst']    = cdc(getattr(self, prefix+'csdly_rst').re | self._rst.storage)
                _l[prefix+'csdly_inc']    = cdc(getattr(self, prefix+'csdly_inc').re)
                _l[prefix+'cadly_rst']    = cdc(getattr(self, prefix+'cadly_rst').re | self._rst.storage)
                _l[prefix+'cadly_inc']    = cdc(getattr(self, prefix+'cadly_inc').re)
                _l[prefix+'pardly_rst']   = cdc(getattr(self, prefix+'pardly_rst').re | self._rst.storage)
                _l[prefix+'pardly_inc']   = cdc(getattr(self, prefix+'pardly_inc').re)

            if with_odelay:
                _l[prefix+'wdly_dq_rst']  = cdc(getattr(self, prefix+'wdly_dq_rst').re | self._rst.storage)
                _l[prefix+'wdly_dq_inc']  = cdc(getattr(self, prefix+'wdly_dq_inc').re)
                _l[prefix+'wdly_dm_rst']  = cdc(getattr(self, prefix+'wdly_dm_rst').re | self._rst.storage)
                _l[prefix+'wdly_dm_inc']  = cdc(getattr(self, prefix+'wdly_dm_inc').re)
                _l[prefix+'wdly_dqs_rst'] = cdc(getattr(self, prefix+'wdly_dqs_rst').re | self._rst.storage)
                _l[prefix+'wdly_dqs_inc'] = cdc(getattr(self, prefix+'wdly_dqs_inc').re)

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
            read_latency  = read_latency + 3,
            write_latency = 0,
            cmd_latency   = cmd_latency,
            cmd_delay     = cmd_delay,
            strobes       = combined_strobes,
            address_lines       = address_lines,
            min_write_latency   = min_write_latency + write_addjust,
            min_read_latency    = 2,
            with_sub_channels   = with_sub_channels,
            with_clock_odelay   = with_clock_odelay,
            with_address_odelay = with_address_odelay,
            with_odelay         = with_odelay,
            with_idelay         = with_idelay,
            with_per_dq_idelay  = with_per_dq_idelay,
            direct_control      = direct_control,
            t_ctrl_delay        = addr_pre_ser_delay,
        )

        # DFI Interface ----------------------------------------------------------------------------
        self.dfi = dfi = Interface(14, 1, nranks, 2*combined_data_bits, nphases=4, with_sub_channels=with_sub_channels)

        # Now prepare the data by converting the sequences on adapters into sequences on the pads.
        # We have to ignore overlapping commands, and module timings have to ensure that there are
        # no overlapping commands anyway.
        self.out = BasePHYOutput(nphases, databits, nranks, strobes, with_sub_channels, name="basephy")

        # Clocks -----------------------------------------------------------------------------------
        self.comb += self.out.ck_t.eq(bitpattern("-_-_-_-_"))
        self.comb += self.out.ck_c.eq(bitpattern("_-_-_-_-"))

        # Simple commands --------------------------------------------------------------------------
        self.comb += self.out.reset_n.eq(Cat((phase.reset_n, phase.reset_n) for phase in dfi.phases))
        self.comb += [phase.alert_n.eq(self.out.alert_n[i*2] & self.out.alert_n[i*2+1]) for i, phase in enumerate(self.dfi.phases)]

        _alert_reduce = Signal()
        self.sync += [
            If(self.alert_reduce.storage,
                _alert_reduce.eq(reduce(and_, self.out.alert_n))
            ).Else(
                _alert_reduce.eq(reduce(or_, self.out.alert_n))
            )
        ]
        self.comb += self.alert.status.eq(_alert_reduce)

        for prefix in prefixes:
            self.submodules += PHYAddressSlicer(self.out, dfi, self._rdimm_mode.storage, prefix)

            for strobe in range(strobes):
                # Read Control Path ------------------------------------------------------------------------
                # Creates a delay line of read commands coming from the DFI interface. The output is used to
                # signal a valid read data to the DFI interface.
                #
                # The read data valid is asserted for 1 sys_clk cycle when the data is available on the DFI
                # interface, the latency is the sum of the minimal PHY and user added delays.
                rddata_en_input = Signal(nphases)

                for i in range(nphases):
                    self.comb += rddata_en_input[i].eq(getattr(dfi.phases[i], prefix).rddata_en | getattr(self, prefix+'wlevel_en').storage)

                default_read_latency = default_read_latency - 2 if default_read_latency > 2 else 0
                rd_reset_value = min_read_latency + default_read_latency

                nphases_log = nphases.bit_length() - 1

                # Read window ----------------------------------------------------------------------
                rddata_ens = [
                    ShiftRegister(
                        signal = rddata_en_input[i],
                        ntaps  = read_latency + 1
                    ) for i in range(nphases)
                ]
                for i, rs in enumerate(rddata_ens):
                    setattr(self.submodules, f"{prefix}{strobe}_Read_SR_{i}", rs)

                rddata_out_en = ShiftRegister(
                    signal = reduce(or_, rddata_en_input),
                    ntaps  = read_latency + 3
                )
                setattr(self.submodules, f"{prefix}{strobe}_Read_FIFO_SR_{i}", rddata_out_en)


                rd_window = Signal(nphases)
                rd_delay  = Signal(max=4*read_latency, reset=rd_reset_value)
                rd_index  = Signal(max=read_latency)
                rd_offset = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_rdly_inc').re & \
                       (rd_delay < (min_read_latency + 66)),
                        rd_delay.eq(rd_delay + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                           getattr(self, prefix+'ck_rdly_rst').re,
                        rd_delay.eq(rd_reset_value),
                    ),
                ]

                self.comb += [
                    rd_index.eq(rd_delay[nphases_log:]),
                    rd_offset.eq(rd_delay[:nphases_log]),
                ]

                rd_index_p = [Signal(max=read_latency) for _ in range(nphases)]
                rd_cases = {}
                for i in range(nphases):
                    first_part  = [rd_index_p[j].eq(rd_index + 1) for j in range(i)]
                    second_part = [rd_index_p[j].eq(rd_index) for j in range(i, nphases)]
                    rd_cases[i] = first_part + second_part

                self.comb += [
                    Case(rd_offset,
                        rd_cases,
                    ),
                    rd_window.eq(Cat([rddata_ens[i].taps[rd_index_p[i]] for i in range(nphases)])),
                ]

                # Read Preamble window -------------------------------------------------------------
                rddata_preamble_ens = [
                    ShiftRegister(
                        signal = rddata_en_input[i],
                        ntaps  = read_latency
                    ) for i in range(nphases)
                ]
                for i, rs in enumerate(rddata_preamble_ens):
                    setattr(self.submodules, f"{prefix}{strobe}_Preamble_SR_{i}", rs)

                rd_preamble_window      = Signal(nphases)
                rd_last_preamble_window = Signal(nphases)
                rd_preamble        = Signal(max=4*read_latency, reset=rd_reset_value - 2)
                rd_preamble_index  = Signal(max=read_latency)
                rd_preamble_offset = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_rdly_inc').re & \
                       (rd_delay < (min_read_latency + 66)),
                        rd_preamble.eq(rd_preamble + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                           getattr(self, prefix+'ck_rdly_rst').re,
                        rd_preamble.eq(rd_reset_value - 2),
                    ),
                ]

                self.comb += [
                    rd_preamble_index.eq(rd_preamble[nphases_log:]),
                    rd_preamble_offset.eq(rd_preamble[:nphases_log]),
                ]

                rd_preamble_index_p = [Signal(max=read_latency) for _ in range(nphases)]
                rd_preamble_cases = {}
                for i in range(nphases):
                    first_part  = [rd_preamble_index_p[j].eq(rd_preamble_index + 1) for j in range(i)]
                    second_part = [rd_preamble_index_p[j].eq(rd_preamble_index) for j in range(i, nphases)]
                    rd_preamble_cases[i] = first_part + second_part

                self.comb += [
                    Case(rd_preamble_offset,
                        rd_preamble_cases,
                    ),
                    rd_preamble_window.eq(
                        Cat([rddata_preamble_ens[i].taps[rd_preamble_index_p[i]] for i in range(nphases)])
                    ),
                ]
                self.sync += [
                    rd_last_preamble_window.eq(rd_preamble_window),
                ]

                # Read Preamble Path ---------------------------------------------------------------
                rd_preamble_rdy = Signal(max=2*nphases)
                self.comb += [
                    If(~rd_last_preamble_window[-1] & rd_preamble_window[0],
                        rd_preamble_rdy.eq(1)
                    ),
                ]
                for i in range(1, nphases):
                    self.comb += [
                        If(~rd_preamble_window[i-1] & rd_preamble_window[i],
                            rd_preamble_rdy.eq(2*i | 1)
                        ),
                    ]

                rd_sampled_preamble = Signal(2*2)
                rd_preamble_cnt     = Signal()

                rd_preamble_cases_sync = {}
                for i in range(nphases):
                    if i+1 < nphases:
                        rd_preamble_cases_sync[i] = [
                            rd_sampled_preamble.eq(getattr(self.out, prefix+'dqs_t_i')[strobe][i*2:i*2+4]),
                            rd_preamble_cnt.eq(0),
                        ]
                    else:
                        rd_preamble_cases_sync[i] = [
                            rd_sampled_preamble[0:2].eq(getattr(self.out, prefix+'dqs_t_i')[strobe][i*2:i*2+2]),
                            rd_preamble_cnt.eq(1),
                        ]

                self.sync += [
                    If(rd_preamble_rdy[0],
                        Case(rd_preamble_rdy[1:],
                            rd_preamble_cases_sync
                        ),
                    ),
                    If(rd_preamble_cnt == 1,
                        rd_sampled_preamble[2:4].eq(getattr(self.out, prefix+'dqs_t_i')[strobe][0:2]),
                        rd_preamble_cnt.eq(0),
                    ),
                ]

                self.comb += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe],
                        getattr(self, prefix+'preamble').status.eq(rd_sampled_preamble),
                    ),
                ]

                # Read Data Path ----------------------------------------------------------------------------
                # The rd_window can present any arbitrary (1*0*)* pattern of length nphases.
                # We detect where one full DFI phase of data finishes and where other starts
                # by counting how many valid bits are set in the rd_window, and how many
                # are set in range [0:i-1], for the i = {0, .., nphases-1}.
                # When data for full DFI phase are collected, they are stored in FIFO and await
                # for settings.read_latency-1 to pass before being presented on DFI bus.

                rd_fifo = SyncFIFO(width=dq_dqs_ratio*nphases*2, depth=read_latency, fwft=False)
                self.submodules += rd_fifo

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
                    If(reduce(or_, rd_window),
                        rddata_cnt.eq(rddata_cnt_all_valid[:nphases_log]),
                    ),
                ]

                self.comb += [
                    rd_fifo.din.eq(0),
                    rd_fifo.we.eq(0),
                    If(reduce(or_, rd_window),
                        If(rddata_cnt_all_valid[nphases_log],
                            rd_fifo.din.eq(Cat(rddata_sel)),
                            rd_fifo.we.eq(1),
                        ),
                    ),
                ]

                # Retime
                self.comb += [
                    getattr(phase, prefix).rddata_valid.eq( \
                        reduce(or_, rddata_out_en.output)) \
                    for phase in self.dfi.phases
                ]

                rddata_start = strobe*2*dq_dqs_ratio
                rddata_end   = (strobe+1)*2*dq_dqs_ratio

                rd_fifo_good = Signal()
                self.sync += [
                    rd_fifo_good.eq(rd_fifo.re & rd_fifo.readable)
                ]

                self.comb += [
                    If(rd_fifo_good,
                        getattr(phase, prefix).rddata[rddata_start:rddata_end].eq(rd_fifo.dout[i*2*dq_dqs_ratio:(i+1)*2*dq_dqs_ratio])
                    ) for i, phase in enumerate(self.dfi.phases)
                ] + [
                    rd_fifo.re.eq(rddata_out_en.taps[-2])
                ]

                # Write Control Path -----------------------------------------------------------------------
                wrtap = (self.min_write_latency + write_addjust + 66 + nphases + 1 + nphases - 1)//nphases
                assert wrtap >= 0

                # Create a delay line of write commands coming from the DFI interface. This taps are used to
                # control DQ/DQS tristates.

                wrdata_en_comb = Signal(nphases)
                self.comb += wrdata_en_comb.eq(Cat([getattr(dfi.phases[i], prefix).wrdata_en for i in range(nphases)]))

                wrdata_en = TappedDelayLine(
                    signal = wrdata_en_comb,
                    ntaps  = wrtap
                )
                self.submodules += wrdata_en

                assert default_write_latency >= min_write_latency or default_write_latency == 0, f"default_write_latency={default_write_latency} is to small, min_write_latency={min_write_latency}"

                wr_reset_value = 0 if default_write_latency < min_write_latency else default_write_latency - min_write_latency

                wr_window       = Signal(nphases + 3)
                wr_delay        = Signal(max=66 + write_addjust, reset=wr_reset_value)
                wr_index        = Signal(max=(66 + write_addjust)//nphases+1)
                wr_offset       = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_wdly_inc').re & \
                       (wr_delay < 65),
                        wr_delay.eq(wr_delay + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                           getattr(self, prefix+'ck_wdly_rst').re,
                        wr_delay.eq(wr_reset_value),
                    ),
                ]

                self.comb += [
                    wr_index.eq(wr_delay[nphases_log:]),
                    wr_offset.eq(wr_delay[:nphases_log]),
                ]

                wr_cases = {}
                if nphases > 1:
                    for i in range(nphases):
                        if 3+i <= nphases:
                            wr_cases[i] = wr_window.eq(Cat(wrdata_en.taps[wr_index+1][nphases-(3+i):], wrdata_en.taps[wr_index][:nphases-i]))
                        else:
                            wr_cases[i] = wr_window.eq(Cat(wrdata_en.taps[wr_index+2][2*nphases-(3+i):], wrdata_en.taps[wr_index+1], wrdata_en.taps[wr_index][:nphases-i]))
                else:
                    wr_cases[0] = wr_window.eq(Cat(wrdata_en.taps[wr_index+3], wrdata_en.taps[wr_index+2], wrdata_en.taps[wr_index+1], wrdata_en.taps[wr_index]))

                self.comb += [
                    Case(wr_offset,
                        wr_cases,
                    )
                ]

                dqs_oe        = Signal(2*nphases)
                dqs_pattern   = DQSPattern(
                    nphases   = nphases,
                    wlevel_en = getattr(self, prefix+'wlevel_en').storage,
                )
                self.comb += dqs_pattern.window.eq(wr_window)
                self.submodules += dqs_pattern

                self.comb += [
                    getattr(self.out, prefix+'dqs_t_o')[strobe].eq(dqs_pattern.o,),
                    getattr(self.out, prefix+'dqs_c_o')[strobe].eq(~dqs_pattern.o,),
                    getattr(self.out, prefix+'dqs_oe')[strobe].eq(dqs_pattern.oe),
                ]

                wr_data_window  = Signal(nphases+1)
                wr_data_delay   = Signal(max=68 + write_addjust, reset=wr_reset_value + 2)
                wr_data_index   = Signal(max=(68 + write_addjust)//nphases+1)
                wr_data_offset  = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_wddly_inc').re & \
                       (wr_data_delay < 67),
                        wr_data_delay.eq(wr_data_delay + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                           getattr(self, prefix+'ck_wddly_rst').re,
                        wr_data_delay.eq(wr_reset_value + 2),
                    ),
                ]

                self.comb += [
                    wr_data_index.eq(wr_data_delay[nphases_log:]),
                    wr_data_offset.eq(wr_data_delay[:nphases_log]),
                ]

                wr_data_cases = {}
                for i in range(nphases):
                    if 1+i <= nphases: # only false for last i = nphases -1
                        wr_data_cases[i] = wr_data_window.eq(Cat(wrdata_en.taps[wr_data_index+1][nphases-(1+i):], wrdata_en.taps[wr_data_index][:nphases-i]))

                self.comb += [
                    Case(wr_data_offset,
                        wr_data_cases,
                    )
                ]

                dq_oe        = Signal(2*nphases)
                dq_pattern   = DQOePattern(
                    nphases   = nphases,
                    wlevel_en = getattr(self, prefix+'wlevel_en').storage,
                )
                self.comb += dq_pattern.window.eq(wr_data_window)
                self.submodules += dq_pattern

                self.comb += [
                    getattr(self.out, prefix+'dq_oe')[strobe].eq(dq_pattern.oe),
                ]

                # Write Data Path ----------------------------------------------------------------------------

                wr_fifo = SyncFIFO(width=(dq_dqs_ratio+1)*nphases*2, depth=wrtap, fwft=False)
                self.submodules += wr_fifo

                self.comb += [
                    wr_fifo.din.eq(Cat([Cat([getattr(phase, prefix).wrdata[2*strobe*dq_dqs_ratio:2*(strobe+1)*dq_dqs_ratio],
                                             getattr(phase, prefix).wrdata_mask[strobe*2:(strobe+1)*2] if dq_dqs_ratio > 4 else Replicate(getattr(phase, prefix).wrdata_mask[strobe], 2)]) for phase in self.dfi.phases])),
                    If(wr_data_index > 0,
                        wr_fifo.we.eq(reduce(or_, [getattr(phase, prefix).wrdata_en for phase in self.dfi.phases])),
                    ),
                ]

                wr_data             = Signal(2*nphases*dq_dqs_ratio)
                wr_fifo_data        = Signal(2*nphases*dq_dqs_ratio)
                wr_input_data       = Signal(2*nphases*dq_dqs_ratio)
                wr_register_data    = Signal(2*nphases*dq_dqs_ratio)
                wr_dmi              = Signal(2*nphases)
                wr_fifo_dmi         = Signal(2*nphases)
                wr_input_dmi        = Signal(2*nphases)
                wr_register_dmi     = Signal(2*nphases)
                wr_fifo_data_valid  = Signal()

                self.sync += wr_fifo_data_valid.eq(wr_fifo.re & wr_fifo.readable)
                self.sync += [
                    wr_input_data.eq(Cat([getattr(phase, prefix).wrdata[2*strobe*dq_dqs_ratio:2*(strobe+1)*dq_dqs_ratio] for phase in self.dfi.phases])),
                    wr_input_dmi.eq(Cat([getattr(phase, prefix).wrdata_mask[strobe*2:(strobe+1)*2] if dq_dqs_ratio > 4 else Replicate(getattr(phase, prefix).wrdata_mask[strobe], 2) for phase in self.dfi.phases])),
                ]

                self.comb += [
                    If(wr_data_index > 0,
                        wr_fifo.re.eq(reduce(or_, wrdata_en.taps[wr_data_index-1])),
                        If(wr_fifo_data_valid,
                            wr_fifo_data.eq(Cat([wr_fifo.dout[(2*i)*(dq_dqs_ratio+1): (2*i)*(dq_dqs_ratio+1)+2*dq_dqs_ratio] for i in range(nphases)])),
                            wr_fifo_dmi.eq(Cat([wr_fifo.dout[(2*i)*(dq_dqs_ratio+1)+2*dq_dqs_ratio: (2*i+2)*(dq_dqs_ratio+1)] for i in range(nphases)])),
                        ),
                    ).Else(
                        wr_fifo_data.eq(wr_input_data),
                        wr_fifo_dmi.eq(wr_input_dmi),
                    ),
                ]

                dq_dmi_wr_cases_comb = {}
                dq_dmi_wr_cases_sync = {}

                dq_dmi_wr_cases_comb[0] = [
                    wr_data.eq(Cat(wr_fifo_data[:nphases*2*dq_dqs_ratio])),
                    wr_dmi.eq(Cat(wr_fifo_dmi[:nphases*2])),
                ]
                dq_dmi_wr_cases_sync[0] = [
                    wr_register_data.eq(0),
                    wr_register_dmi.eq(0),
                ]

                for i in range(1, nphases):
                    dq_dmi_wr_cases_comb[i] = [
                        wr_data.eq(Cat(wr_register_data[:i*2*dq_dqs_ratio], wr_fifo_data[:(nphases-i)*2*dq_dqs_ratio])),
                        wr_dmi.eq(Cat(wr_register_dmi[:i*2], wr_fifo_dmi[:(nphases-i)*2])),
                    ]
                    dq_dmi_wr_cases_sync[i] = [
                        wr_register_data.eq(wr_fifo_data[(nphases-i)*2*dq_dqs_ratio:]),
                        wr_register_dmi.eq(wr_fifo_dmi[(nphases-i)*2:]),
                    ]

                self.comb += [
                    Case(wr_data_offset,
                        dq_dmi_wr_cases_comb
                    ),
                ]

                self.sync += [
                    Case(wr_data_offset,
                        dq_dmi_wr_cases_sync
                    ),
                ]

                # DMI --------------------------------------------------------------------------------------
                # DMI signal is used for Data Mask or Data Bus Invertion depending on Mode Registers values.
                # With DM and DBI disabled, this signal is a Don't Care.
                # With DM enabled, masking is performed only when the command used is WRITE-MASKED.
                # We don't support DBI, DM support is configured statically with `masked_write`.
                self.comb += getattr(self.out, prefix+'dm_n_o')[strobe].eq(~wr_dmi)

                # DQ ---------------------------------------------------------------------------------------
                for bit in range(dq_dqs_ratio):
                    # output
                    _wrdata = [
                        wr_data[i * dq_dqs_ratio + bit] for i in range(2*nphases)
                    ]

                    self.comb += getattr(self.out, prefix+'dq_o')[bit + strobe*dq_dqs_ratio].eq(Cat(_wrdata))


    def get_rst(self, byte, rst, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        t = Signal()
        if not dq:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte] & rst) | self._rst.storage)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] & rst) | self._rst.storage)
        else:
            cd_clk += t.eq((getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] &
                            getattr(self, prefix+'dq_dly_sel').storage[byte%self.dq_dqs_ratio] & rst) |
                            self._rst.storage)
        return t

    def get_inc(self, byte, stb, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        t = Signal()
        if not dq:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte] & stb)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] & stb)
        else:
            cd_clk += t.eq(getattr(self, prefix+'dly_sel').storage[byte//self.dq_dqs_ratio] &
                            getattr(self, prefix+'dq_dly_sel').storage[byte%self.dq_dqs_ratio] & stb)
        return t
