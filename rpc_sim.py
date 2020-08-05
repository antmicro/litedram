#!/usr/bin/env python3

import argparse
from collections import defaultdict

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.common import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import *
from litex.soc.cores.bitbang import *
from litex.soc.cores.cpu import CPUS

from litedram import modules as litedram_modules
from litedram.modules import parse_spd_hexdump
from litedram.common import *
from litedram.phy.model import SDRAMPHYModel

from liteeth.phy.model import LiteEthPHYModel
from liteeth.mac import LiteEthMAC
from liteeth.core.arp import LiteEthARP
from liteeth.core.ip import LiteEthIP
from liteeth.core.udp import LiteEthUDP
from liteeth.core.icmp import LiteEthICMP
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone
from liteeth.common import *

from litescope import LiteScopeAnalyzer

from litedram.phy.etronrpcphy import EM6GA16L, DFIAdapter

# Platform -----------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys2x_clk", 0, Pins(1)),
    ("sys4x_clk", 0, Pins(1)),
    ("sys4x_ddr_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),
        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
    ("sim_trigger", 0, Pins(1))
]

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# DFI PHY model settings ---------------------------------------------------------------------------

sdram_module_nphases = {
    "SDR":   1,
    "DDR":   2,
    "LPDDR": 2,
    "DDR2":  2,
    "DDR3":  4,
    "DDR4":  4,
}

def get_sdram_phy_settings(memtype, data_width, clk_freq):
    nphases = sdram_module_nphases[memtype]
    assert memtype == "DDR3"

    # Settings from s7ddrphy
    tck                 = 2/(2*nphases*clk_freq)
    cmd_latency         = 0
    cl, cwl             = get_cl_cw(memtype, tck)
    cl_sys_latency      = get_sys_latency(nphases, cl)
    cwl                 = cwl + cmd_latency
    cwl_sys_latency     = get_sys_latency(nphases, cwl)
    rdcmdphase, rdphase = get_sys_phases(nphases, cl_sys_latency, cl)
    wrcmdphase, wrphase = get_sys_phases(nphases, cwl_sys_latency, cwl)
    read_latency        = 2 + cl_sys_latency + 2 + 3
    write_latency       = cwl_sys_latency

    sdram_phy_settings = {
        "nphases":       nphases,
        "rdphase":       rdphase,
        "wrphase":       wrphase,
        "rdcmdphase":    rdcmdphase,
        "wrcmdphase":    wrcmdphase,
        "cl":            cl,
        "cwl":           cwl,
        "read_latency":  read_latency,
        "write_latency": write_latency,
    }

    return PhySettings(
        phytype      = "SDRAMPHYModel",
        memtype      = memtype,
        databits     = data_width,
        dfi_databits = data_width if memtype == "SDR" else 2*data_width,
        **sdram_phy_settings,
    )

# Simulation SoC -----------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, domains=None):
        if domains is None:
            domains = ["sys"]
        # request() before clreating domains to avoid signal renaming problem
        domains = {name: platform.request(name + "_clk") for name in domains}

        self.clock_domains.cd_por = ClockDomain(reset_less=True)
        for name in domains.keys():
            setattr(self.clock_domains, "cd_" + name, ClockDomain(name=name))

        int_rst = Signal(reset=1)
        self.sync.por += int_rst.eq(0)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)

        for name, clk in domains.items():
            cd = getattr(self, "cd_" + name)
            self.comb += cd.clk.eq(clk)
            self.comb += cd.rst.eq(int_rst)

class SimSoC(SoCCore):
    def __init__(self, sys_clk_freq, **kwargs):
        platform     = Platform()

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident         = "LiteX Simulation",
            ident_version = True,
            cpu_variant   = "lite",
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        domains = ["sys", "sys2x", "sys4x", "sys4x_ddr"]
        self.submodules.crg = _CRG(platform, domains)

        for name in domains + ["por"]:
            s = Signal(name="test_" + name)
            self.comb += s.eq(getattr(self.crg, "cd_" + name).clk)

        # Simulation trigger -----------------------------------------------------------------------
        class SimTrigger(Module, AutoCSR):
            def __init__(self, trigger_pin):
                self.trigger = CSR()
                self.comb += trigger_pin.eq(self.trigger.re)
        self.submodules.sim_trigger = SimTrigger(self.platform.request("sim_trigger"))
        self.add_csr("sim_trigger")

        return

        # RPC DRAM ---------------------------------------------------------------------------------
        sdram_module   = EM6GA16L(sys_clk_freq, "1:4")
        phy_settings   = get_sdram_phy_settings(
            #  memtype    = sdram_module.memtype,
            memtype    = "DDR3",
            data_width = 16,
            clk_freq   = sdram_clk_freq)
        self.submodules.sdrphy = SDRAMPHYModel(
            module    = sdram_module,
            settings  = phy_settings,
            clk_freq  = sdram_clk_freq,
            verbosity = 0,
            init      = [])
        self.add_sdram("sdram",
            phy                     = self.sdrphy,
            module                  = sdram_module,
            origin                  = self.mem_map["main_ram"],
            size                    = kwargs.get("max_sdram_size", 0x40000000),
            l2_cache_size           = kwargs.get("l2_size", 8192),
            l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
            l2_cache_reverse        = False
        )
        # Reduce memtest size for simulation speedup
        self.add_constant("MEMTEST_DATA_SIZE", 8*1024)
        self.add_constant("MEMTEST_ADDR_SIZE", 8*1024)

        self.submodules.dfi_adapter_p0 = DFIAdapter(self.sdrphy.dfi.p0)
        self.submodules.dfi_adapter_p1 = DFIAdapter(self.sdrphy.dfi.p1)
        self.submodules.dfi_adapter_p2 = DFIAdapter(self.sdrphy.dfi.p2)
        self.submodules.dfi_adapter_p3 = DFIAdapter(self.sdrphy.dfi.p3)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generic LiteX SoC Simulation")
    builder_args(parser)
    soc_sdram_args(parser)
    parser.add_argument("--threads",              default=1,               help="Set number of threads (default=1)")
    parser.add_argument("--rom-init",             default=None,            help="rom_init file")
    parser.add_argument("--sdram-init",           default=None,            help="SDRAM init file")
    parser.add_argument("--sdram-verbosity",      default=0,               help="Set SDRAM checker verbosity")
    parser.add_argument("--with-analyzer",        action="store_true",     help="Enable Analyzer support")
    parser.add_argument("--trace",                action="store_true",     help="Enable Tracing")
    parser.add_argument("--trace-fst",            action="store_true",     help="Enable FST tracing (default=VCD)")
    parser.add_argument("--trace-start",          default=0,               help="Cycle to start tracing")
    parser.add_argument("--trace-end",            default=-1,              help="Cycle to end tracing")
    parser.add_argument("--opt-level",            default="O3",            help="Compilation optimization level")
    parser.add_argument("--sys-clk-freq",         default="100e6",         help="Core clock frequency")
    args = parser.parse_args()

    soc_kwargs     = soc_sdram_argdict(args)
    builder_kwargs = builder_argdict(args)

    sys_clk_freq = int(float(args.sys_clk_freq))
    sim_config = SimConfig()
    sim_config.add_clocker("sys_clk",       freq_hz=sys_clk_freq)
    sim_config.add_clocker("sys2x_clk",     freq_hz=2*sys_clk_freq)
    sim_config.add_clocker("sys4x_clk",     freq_hz=4*sys_clk_freq)
    sim_config.add_clocker("sys4x_ddr_clk", freq_hz=2*4*sys_clk_freq)

    # Configuration --------------------------------------------------------------------------------

    cpu = CPUS[soc_kwargs.get("cpu_type", "vexriscv")]
    if soc_kwargs["uart_name"] == "serial":
        soc_kwargs["uart_name"] = "sim"
        sim_config.add_module("serial2console", "serial")
    if args.rom_init:
        soc_kwargs["integrated_rom_init"] = get_mem_data(args.rom_init, cpu.endianness)
    args.with_sdram = True
    soc_kwargs["integrated_main_ram_size"] = 0x0
    soc_kwargs["sdram_verbosity"]          = int(args.sdram_verbosity)

    # SoC ------------------------------------------------------------------------------------------
    soc = SimSoC(
        with_analyzer = args.with_analyzer,
        sys_clk_freq  = sys_clk_freq,
        sdram_init    = [] if args.sdram_init is None else get_mem_data(args.sdram_init, cpu.endianness),
        **soc_kwargs)

    # Build/Run ------------------------------------------------------------------------------------
    builder_kwargs["csr_csv"] = "csr.csv"
    builder = Builder(soc, **builder_kwargs)
    vns = builder.build(run=False, threads=args.threads, sim_config=sim_config,
        opt_level   = args.opt_level,
        trace       = args.trace,
        trace_fst   = args.trace_fst,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))
    if args.with_analyzer:
        soc.analyzer.export_csv(vns, "analyzer.csv")
    builder.build(build=False, threads=args.threads, sim_config=sim_config,
        opt_level   = args.opt_level,
        trace       = args.trace,
        trace_fst   = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end)
    )

if __name__ == "__main__":
    main()
