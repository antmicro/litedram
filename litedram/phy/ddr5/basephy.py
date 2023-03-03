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
        self.reset_n = Signal(2*nphases, reset=~0) # Serializer will work in ddr mode
        self.alert_n = Signal(2*nphases)           # Deserializer will work in ddr mode

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
    def __init__(self, nphases, wlevel_en):
        self.window = window = Signal(nphases + 1)
        self.oe = Signal(2*nphases)
        for i in range(nphases):
            self.comb += [
                If(~wlevel_en,
                    self.oe[2*i:2*i+2].eq(Cat(Replicate(reduce(or_, [window[i], window[i+1]]), 2))),
                ),
            ]


class DDR5DQSPattern(Module):
    def __init__(self, nphases, wlevel_en: Signal()):
        self.window = window = Signal(nphases + 3)
        self.o  = Signal(2*nphases)
        self.oe = Signal(2*nphases)

        # # #

        # DQS Pattern transmitted as LSB-first.
        # Always enabled in write leveling mode, else during transfers
        # Preamble is 2 cycles and postamble is 0.5 cycle

        cases = []

        for i in range(1, nphases+1):
            cases.extend([
                If(reduce(or_, window[i:i+2]),
                    self.o[2*(i-1):2*i].eq(0b01),
                ).Else(
                    self.o[2*(i-1):2*i].eq(0),
                ),
                If(reduce(or_, window[i-1:i+3]) | wlevel_en,
                    self.oe[2*(i-1):2*i].eq(0b11),
                ).Else(
                    self.o[2*(i-1):2*i].eq(0),
                ),
            ])

        self.comb += [
            self.o.eq(0),
            self.oe.eq(0),
            *cases,
        ]


class DDR5PHYAddress(Module):
    def __init__(self, out, dfi, rdimm_mode, prefix):
        # DDR5 CS/CA/PAR PATH ----------------------------------------------------------------------

        nranks = len(getattr(dfi.phases[0], prefix).cs_n)
        assert nranks > 0
        nphases = len(dfi.phases)
        assert nphases > 0 and (nphases & (nphases-1)) == 0

        # DDR5 CS ----------------------------------------------------------------------------------
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
                        getattr(out, prefix + 'cs_n')[rank][2*j].eq(
                            getattr(phase, prefix).cs_n[rank] & (carry_cs_n if j == 0 else 1)
                        ),
                        getattr(out, prefix + 'cs_n')[rank][2*j+1].eq(
                            getattr(phase, prefix).cs_n[rank]
                        ),
                    ).Else(
                        getattr(out, prefix + 'cs_n')[rank][2*j].eq(
                            carry_cs_n if j == 0 else getattr(dfi.phases[j-1], prefix).cs_n[rank]
                        ),
                        getattr(out, prefix + 'cs_n')[rank][2*j+1].eq(
                            getattr(phase, prefix).cs_n[rank]
                        ),
                    ),
                ]

        # DDR5 PAR -------------------------------------------------------------------------------------
        self.comb += getattr(out, prefix + 'par').eq(
            Cat([reduce(xor, getattr(phase, prefix).address[7*i:7+7*i])] for phase in dfi.phases for i in range(2)))

        # DDR5 CA --------------------------------------------------------------------------------------
            # RDIMM 2N mode ----------------------------------------------------------------------------
        mem   = Signal(max(3, nphases))

        take_lower_bits   = Signal(nphases)
        take_lower_bits_m = Signal(nphases)
        take_lower_bits_1 = Signal(nphases)
        take_lower_bits_2 = Signal(nphases)
        for i in range(1, len(take_lower_bits)):
            self.comb += take_lower_bits_1[i].eq(~reduce(and_, getattr(dfi.phases[i-1], prefix).cs_n))
        for i in range(3, len(take_lower_bits)):
            self.comb += take_lower_bits_2[i].eq(
                ~reduce(and_, getattr(dfi.phases[i-3], prefix).cs_n) & ~getattr(dfi.phases[i-3], prefix).address[1]
            )

        self.comb += take_lower_bits_m.eq(Cat([phase.mode_2n for phase in dfi.phases]))
        for i in range(0, 3, nphases):
            for j in range(nphases):
                if i+j >= 3:
                    break
                arr = []
                if i+j+nphases < 3:
                    arr.append(mem[i+j+nphases])
                if i + j < 1:
                    phase = getattr(dfi.phases[nphases-1+i+j], prefix)
                    arr.append(~reduce(and_, phase.cs_n))
                if 0 <= nphases-3 + i+j:
                    phase = getattr(dfi.phases[nphases-3+i+j], prefix)
                    arr.append(~reduce(and_, phase.cs_n) & ~phase.address[1])
                self.sync += mem[i+j].eq(reduce(or_, arr))

        for i in range(nphases):
            self.comb += take_lower_bits[i].eq(
                (take_lower_bits_1[i] | take_lower_bits_2[i] | mem[i]) & take_lower_bits_m[i]
            )

            # CA Slicer ----------------------------------------------------------------------------
        for bit in range(7):
            for j, phase in enumerate(dfi.phases):
                sig = getattr(out, prefix+'ca')[bit][j*2:j*2+2]
                ca = getattr(phase, prefix).address
                self.comb += [
                    If(rdimm_mode,
                        If(phase.mode_2n,
                            If(~take_lower_bits[j],
                                sig.eq(Replicate(ca[bit], 2)),
                            ).Else(
                                sig.eq(Replicate(ca[bit + 7], 2)),
                            )
                        ).Else(
                            sig.eq(Cat([ca[bit + 7*i] for i in range (2)])),
                        ),
                    ).Else(
                        sig.eq(Cat([ca[bit] for _ in range (2)])),
                    ),
                ]

        for bit in range(7, 14):
            _ca = getattr(out, prefix+'ca')[bit]
            for j, phase in enumerate(dfi.phases):
                self.comb += [
                    If(~rdimm_mode,
                        _ca[j*2:j*2+2].eq(Replicate(getattr(phase, prefix).address[bit], 2)),
                    ).Else(
                        _ca[j*2:j*2+2].eq(Replicate(0, 2)),
                    ),
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

        self.des_latency          = des_latency
        self.ser_latency          = ser_latency
        self.ca_cdc_min_max_delay = (rd_extra_delay, rd_extra_delay)
        self.rd_cdc_min_max_delay = (Latency(sys=0), Latency(sys=0))
        self.wr_cdc_min_max_delay = (rd_extra_delay, rd_extra_delay)

        # Read latency
        # This value should be the worst case delay between sending a read cmd and
        # getting data back. There will be exact delay may vary based on the training result.
        self.min_read_latency  = min_read_latency = (
            cmd_latency - 1 +      # CMD latency + extra clock cycle for 2N mode
            ser_latency.sys4x +    # CMD serialization latency
            rd_extra_delay.sys4x + # Delays like CDCs
            2 +                    # Minimal Preamble
            des_latency.sys4x      # Data deserialization latency
        ) # CL 0
        self.max_read_latency = max_read_latency = min_read_latency + 64 + 2 # CL 64 and 2N mode
        read_latency = (max_read_latency + nphases - 1) // nphases
        # Write latency
        # Set to 0, Training PHY will align DQS and DQ for write commands
        # See write leveling training in JESD79-5A
        # Max supported latency is 64 DRAM bus cycles
        self.min_write_latency = min_write_latency = (
            nphases +           # wrdata_en 0 tap delay
            2 -                 # We need to look 2 cycles "into the future" to properly generate write preable
            1                   # Reduce by 1 as cmd has 2 beats
        )
        self.max_write_latency = min_write_latency + 64

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
            min_write_latency   = min_write_latency,
            min_read_latency    = 2,
            with_sub_channels   = with_sub_channels,
            with_clock_odelay   = with_clock_odelay,
            with_address_odelay = with_address_odelay,
            with_odelay         = with_odelay,
            with_idelay         = with_idelay,
            with_per_dq_idelay  = with_per_dq_idelay,
            direct_control      = direct_control,
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
            self.submodules += DDR5PHYAddress(self.out, dfi, self._rdimm_mode.storage, prefix)

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

                rddata_en = TappedDelayLine(
                    signal = rddata_en_input,
                    ntaps  = read_latency + 3
                )
                self.submodules += rddata_en

                default_read_latency = default_read_latency - 2 if default_read_latency > 2 else 0
                rd_reset_value = min_read_latency + default_read_latency

                rd_window   = Signal(nphases)
                rd_delay    = Signal(max=4*read_latency, reset=rd_reset_value)
                rd_index    = Signal(max=read_latency)
                rd_offset   = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                rd_preamble_window  = Signal(nphases)
                rd_last_preamble_window  = Signal(nphases)
                rd_preamble         = Signal(max=4*read_latency, reset=rd_reset_value - 2)
                rd_preamble_index   = Signal(max=read_latency)
                rd_preamble_offset  = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                nphases_log = nphases.bit_length() - 1

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_rdly_inc').re & \
                       (rd_delay < (min_read_latency + 66)),
                        rd_delay.eq(rd_delay + 1),
                        rd_preamble.eq(rd_preamble + 1),
                    ).Elif(getattr(self, prefix+'dly_sel').storage[strobe] & \
                           getattr(self, prefix+'ck_rdly_rst').re,
                        rd_delay.eq(rd_reset_value),
                        rd_preamble.eq(rd_reset_value - 2),
                    ),
                ]

                self.comb += [
                    rd_index.eq(rd_delay[nphases_log:]),
                    rd_offset.eq(rd_delay[:nphases_log]),
                    rd_preamble_index.eq(rd_preamble[nphases_log:]),
                    rd_preamble_offset.eq(rd_preamble[:nphases_log]),
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

                rd_preamble_cases = {}
                rd_preamble_cases[0] = rd_preamble_window.eq(rddata_en.taps[rd_preamble_index])
                if nphases > 1:
                    for i in range(1, nphases):
                        rd_preamble_cases[i] = rd_preamble_window.eq(
                            Cat(rddata_en.taps[rd_preamble_index+1][:i], rddata_en.taps[rd_preamble_index][i:]))

                self.comb += [
                    Case(rd_preamble_offset,
                        rd_preamble_cases,
                    )
                ]

                self.sync += [
                    rd_last_preamble_window.eq(rd_preamble_window),
                ]

                rd_preamble_rdy = Signal(max=2*nphases)
                self.comb += [If(~rd_last_preamble_window[-1] & rd_preamble_window[0], rd_preamble_rdy.eq(1))]
                for i in range(1, nphases):
                    self.comb += [If(~rd_preamble_window[i-1] & rd_preamble_window[i], rd_preamble_rdy.eq(2*i | 1))]

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
                        reduce(or_, rddata_en.output)) \
                    for i, phase in enumerate(self.dfi.phases)
                ]

                rddata_start = strobe*2*dq_dqs_ratio
                rddata_end   = (strobe+1)*2*dq_dqs_ratio

                rd_fifo_good = Signal()
                self.sync += [
                    rd_fifo_good.eq(rd_fifo.re & rd_fifo.readable)
                ]

                self.comb += [
                    If(rd_fifo_good, getattr(phase, prefix).rddata[rddata_start:rddata_end].eq(rd_fifo.dout[i*2*dq_dqs_ratio:(i+1)*2*dq_dqs_ratio])) \
                    for i, phase in enumerate(self.dfi.phases)
                ] + [
                    rd_fifo.re.eq(reduce(or_, rddata_en.taps[-2]))
                ]

                # Write Control Path -----------------------------------------------------------------------
                wrtap = (self.min_write_latency + 64 + nphases - 1)//nphases
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
                wr_delay        = Signal(max=65, reset=wr_reset_value)
                wr_index        = Signal(max=64//nphases+1)
                wr_offset       = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_wdly_inc').re & \
                       (wr_delay < 64),
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
                dqs_pattern   = DDR5DQSPattern(
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
                wr_data_delay   = Signal(max=67, reset=wr_reset_value + 2)
                wr_data_index   = Signal(max=67//nphases+1)
                wr_data_offset  = Signal(max=nphases) if nphases > 1 else Signal(1, reset=0)

                self.sync += [
                    If(getattr(self, prefix+'dly_sel').storage[strobe] & \
                       getattr(self, prefix+'ck_wddly_inc').re & \
                       (wr_data_delay < 66),
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
                #for i in range(nphases):
                #    if i == 0:
                #        wr_data_cases[i] = wr_data_window.eq(Cat(wrdata_en.taps[wr_data_index]))
                #    else:
                #        wr_data_cases[i] = wr_data_window.eq(Cat(wrdata_en.taps[wr_data_index+1][nphases-i:], wrdata_en.taps[wr_data_index][:nphases-i]))

                self.comb += [
                    Case(wr_data_offset,
                        wr_data_cases,
                    )
                ]

                dq_oe        = Signal(2*nphases)
                dq_pattern   = DDR5DQOePattern(
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
