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
from litedram.phy.ddr5.BasePHYCSR import BasePHYCSR
from litedram.phy.ddr5.BasePHYWritePath import BasePHYWritePath, BasePHYWritePathInput, BasePHYWritePathOutput
from litedram.phy.ddr5.BasePHYReadPath import BasePHYReadPath, BasePHYReadPathInput, BasePHYReadPathOutput


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
                 i_domain=None, i_domain_ratio=1, o_doamin=None, o_domain_ratio=1,
                 SyncFIFO_cls=SyncFIFO,
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
        self.dq_dqs_ratio = dq_dqs_ratio = databits // strobes

        prefixes = [""] if not with_sub_channels else ["A_", "B_"]
        # Registers --------------------------------------------------------------------------------
        self.submodules.CSRModule = BasePHYCSR(
            prefixes,
            nphases,
            nranks,
            strobes,
            with_clock_odelay,
            with_address_odelay,
            with_idelay,
            with_odelay,
            with_per_dq_idelay,
            databits,
            dq_dqs_ratio,
        )
        self.CSRs = CSRs = self.CSRModule.CSR_to_dict()

        def cdc(i):
            if csr_cdc is None:
                return i
            return csr_cdc(i)

        def cdc_90(i):
            if csr_cdc_90 is None:
                return i
            return csr_cdc_90(i)

        self.CDCCSRs = CDCCSRs = dict()

        self._rst_cdc       = cdc(CSRs['_rst'].storage)
        self._rst_cdc_90    = cdc_90(CSRs['_rst'].storage)

        for key, CSR in CSRs.items():
            if "ck_" not in key and "dly" in key and "_inc" in key:
                CDCCSRs[key] = cdc(CSR.re)
            elif "ck_" not in key and "dly" in key and "_rst" in key:
                CDCCSRs[key] = cdc(CSR.re | CSRs['_rst'].storage)

        # PHY settings -----------------------------------------------------------------------------

        combined_data_bits = databits if not with_sub_channels else 2*databits
        combined_strobes = strobes if not with_sub_channels else 2*strobes

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
        self.min_read_latency, self.max_read_latency, extra_delay = BasePHYReadPath.get_min_max_supported_latencies(
            nphases, addr_pre_ser_delay, ser_latency, rd_extra_delay,des_latency)
        read_latency = (self.max_read_latency + extra_delay + nphases - 1) // nphases

        # Write latency
        # Set to 0, Training PHY will align DQS and DQ for write commands
        # See write leveling training in JESD79-5A
        # Max supported latency is 64 DRAM bus cycles + 1 for 2N mode
        min_write_latency, max_write_latency, write_addjust = \
            BasePHYWritePath.get_min_max_supported_latencies(nphases, addr_pre_ser_delay)

        self.settings = PhySettings(
            phytype       = phytype,
            memtype       = memtype,
            databits      = combined_data_bits,
            dfi_databits  = 2*combined_data_bits,
            nranks        = nranks,
            nphases       = nphases,
            rdphase       = CSRs['_rdphase'].storage,
            wrphase       = CSRs['_wrphase'].storage,
            cl            = cl,
            cwl           = cwl,
            masked_write  = masked_write,
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
        _alert = Signal.like(self.out.alert_n)
        self.sync += _alert.eq(self.out.alert_n)
        self.sync += [
            If(CSRs['alert_reduce'].storage,
                _alert_reduce.eq(reduce(and_, _alert))
            ).Else(
                _alert_reduce.eq(reduce(or_, _alert))
            )
        ]
        self.comb += CSRs['alert'].status.eq(_alert_reduce)

        for prefix in prefixes:
            self.submodules += PHYAddressSlicer(self.out, dfi, CSRs['_rdimm_mode'].storage, prefix)

            for strobe in range(strobes):
                # Read Control Path ------------------------------------------------------------------------
                _csr = {}
                _csr['dly_sel'] = CSRs[prefix+'dly_sel'].storage[strobe]
                _csr['ck_rdly_inc'] = CSRs[prefix+'ck_rdly_inc'].re
                _csr['ck_rdly_rst'] = CSRs[prefix+'ck_rdly_rst'].re
                _csr['preamble'] = CSRs[prefix+'preamble'].status
                _csr['wlevel_en'] = CSRs[prefix+'wlevel_en'].storage

                dq_offset = strobe*dq_dqs_ratio
                phy     = BasePHYReadPathInput(nphases, dq_dqs_ratio)
                self.comb += [t_phase.rddata_en.eq(getattr(s_phase, prefix).rddata_en)
                    for t_phase, s_phase in zip(phy.phases, dfi.phases)]
                self.comb += phy.dqs_t_i.eq(getattr(self.out, prefix+'dqs_t_i')[strobe])
                self.comb += [getattr(phy, f"dq{i}_i").eq(
                    getattr(self.out, prefix+'dq_i')[dq_offset+i]) for i in range(dq_dqs_ratio)]

                dfi_out = BasePHYReadPathOutput(nphases, dq_dqs_ratio)

                self.submodules += BasePHYReadPath(
                    dfi_out,
                    phy,
                    CSRs=_csr,
                    default_read_latency=default_read_latency
                )

                self.comb += [
                    getattr(t_phase, prefix).rddata_valid.eq( \
                        reduce(or_, s_phase.rddata_valid)) \
                    for t_phase, s_phase in zip(self.dfi.phases, dfi_out.phases)
                ]

                rddata_start = strobe*2*dq_dqs_ratio
                rddata_end   = (strobe+1)*2*dq_dqs_ratio
                self.comb += [
                    getattr(t_phase, prefix).rddata[rddata_start:rddata_end].eq(s_phase.rddata)
                    for t_phase, s_phase in zip(self.dfi.phases, dfi_out.phases)
                ]

                # Write Control Path -----------------------------------------------------------------------
                _csr = {}
                _csr['dly_sel'] = CSRs[prefix+'dly_sel'].storage[strobe]
                _csr['ck_wdly_inc'] = CSRs[prefix+'ck_wdly_inc'].re
                _csr['ck_wdly_rst'] = CSRs[prefix+'ck_wdly_rst'].re
                _csr['ck_wddly_inc'] = CSRs[prefix+'ck_wddly_inc'].re
                _csr['ck_wddly_rst'] = CSRs[prefix+'ck_wddly_rst'].re
                _csr['wlevel_en'] = CSRs[prefix+'wlevel_en'].storage
                wrdata_start = strobe*2*dq_dqs_ratio
                wrdata_end   = (strobe+1)*2*dq_dqs_ratio
                wrdata_mask_bits = dq_dqs_ratio // 4
                wrdata_m_start = strobe*wrdata_mask_bits
                wrdata_m_end   = (strobe+1)*wrdata_mask_bits
                dfi_in = BasePHYWritePathInput(nphases, dq_dqs_ratio)
                self.comb += [t_phase.wrdata_en.eq(getattr(s_phase, prefix).wrdata_en)
                    for t_phase, s_phase in zip(dfi_in.phases, dfi.phases)]
                self.comb += [t_phase.wrdata.eq(
                    getattr(s_phase, prefix).wrdata[wrdata_start:wrdata_end])
                    for t_phase, s_phase in zip(dfi_in.phases, dfi.phases)]
                def rep(sig, cnt):
                    return sig
                if dq_dqs_ratio == 4:
                    rep = Replicate
                self.comb += [t_phase.wrdata_mask.eq(
                    rep(getattr(s_phase, prefix).wrdata_mask[wrdata_m_start:wrdata_m_end], 2))
                    for t_phase, s_phase in zip(dfi_in.phases, dfi.phases)]

                out = BasePHYWritePathOutput(nphases, dq_dqs_ratio)
                self.submodules += BasePHYWritePath(
                    dfi=dfi_in, out=out, CSRs=_csr,
                    default_write_latency=default_write_latency,
                    SyncFIFO_cls=SyncFIFO_cls,
                    with_data_mask=masked_write,
                )

                self.comb += [
                    getattr(self.out, prefix+'dqs_t_o')[strobe].eq(out.dqs_t_o),
                    getattr(self.out, prefix+'dqs_c_o')[strobe].eq(out.dqs_c_o),
                    getattr(self.out, prefix+'dqs_oe')[strobe].eq(out.dqs_oe),
                    getattr(self.out, prefix+'dq_oe')[strobe].eq(out.dq_oe),
                    getattr(self.out, prefix+'dm_n_o')[strobe].eq(out.dm_n_o)
                ]
                for bit in range(dq_dqs_ratio):
                    self.comb += getattr(self.out, prefix+'dq_o')[bit + strobe*dq_dqs_ratio].eq(getattr(out, f"dq{bit}_o"))


    def get_rst(self, byte, rst, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        CSRs = self.CSRs
        t = Signal()
        if not dq:
            cd_clk += t.eq((CSRs[prefix+'dly_sel'].storage[byte] & rst) | CSRs['_rst'].storage)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq((CSRs[prefix+'dly_sel'].storage[byte//self.dq_dqs_ratio] & rst) | CSRs['_rst'].storage)
        else:
            cd_clk += t.eq((CSRs[prefix+'dly_sel'].storage[byte//self.dq_dqs_ratio] &
                            CSRs[prefix+'dq_dly_sel'].storage[byte%self.dq_dqs_ratio] & rst) |
                            CSRs['_rst'].storage)
        return t

    def get_inc(self, byte, stb, prefix="", clk="sys", dq=False):
        cd_clk = getattr(self.sync, clk)
        CSRs = self.CSRs
        t = Signal()
        if not dq:
            cd_clk += t.eq(CSRs[prefix+'dly_sel'].storage[byte] & stb)
        elif not self.with_per_dq_idelay:
            cd_clk += t.eq(CSRs[prefix+'dly_sel'].storage[byte//self.dq_dqs_ratio] & stb)
        else:
            cd_clk += t.eq(CSRs[prefix+'dly_sel'].storage[byte//self.dq_dqs_ratio] &
                            CSRs[prefix+'dq_dly_sel'].storage[byte%self.dq_dqs_ratio] & stb)
        return t
