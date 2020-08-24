#!/usr/bin/env python3
# This file is Copyright (c) 2020 Antmicro <www.antmicro.com>

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
from litedram.core.controller import ControllerSettings
from litedram.phy.model import SDRAMPHYModel
from litedram.phy import dfi

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

from litedram.phy.etronrpcphy import EM6GA16L, SimulationPHY as RPCPHY

# Platform -----------------------------------------------------------------------------------------

_io = [
    # clocks added later
    ("sys_rst", 0, Pins(1)),

    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),
        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),

    # DDR3 pads
    ("ddram", 0,
        Subsignal("a",       Pins(14)),
        Subsignal("ba",      Pins(3)),
        Subsignal("ras_n",   Pins(1)),
        Subsignal("cas_n",   Pins(1)),
        Subsignal("we_n",    Pins(1)),
        Subsignal("cs_n",    Pins(1)),
        Subsignal("dm",      Pins(2)),
        Subsignal("dq",      Pins(16)),
        Subsignal("dqs_p",   Pins(2)),
        Subsignal("dqs_n",   Pins(2)),
        Subsignal("clk_p",   Pins(1)),
        Subsignal("clk_n",   Pins(1)),
        Subsignal("cke",     Pins(1)),
        Subsignal("odt",     Pins(1)),
        Subsignal("reset_n", Pins(1)),
    ),
]

class Platform(SimPlatform):
    def __init__(self):
        print('_io', end=' = '); __import__('pprint').pprint(_io)
        SimPlatform.__init__(self, "SIM", _io)

# DFI PHY model settings ---------------------------------------------------------------------------

sdram_module_nphases = {
    "SDR":   1,
    "DDR":   2,
    "LPDDR": 2,
    "DDR2":  2,
    "DDR3":  4,
    "RPC":   4,
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

# Clocks -------------------------------------------------------------------------------------------

class Clocks(dict):
    # FORMAT: {name: {"freq_hz": _, "phase_deg": _}, ...}
    def names(self):
        return list(self.keys())

    def add_io(self, io):
        for name in self.names():
            print((name + "_clk", 0, Pins(1)))
            io.append((name + "_clk", 0, Pins(1)))

    def add_clockers(self, sim_config):
        for name, desc in self.items():
            sim_config.add_clocker(name + "_clk", **desc)

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

# Simulation SoC -----------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self, clocks, **kwargs):
        platform     = Platform()
        sys_clk_freq = clocks["sys"]["freq_hz"]

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident         = "LiteX Simulation",
            ident_version = True,
            cpu_variant   = "lite",
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, clocks.names())

        # Debugging --------------------------------------------------------------------------------
        platform.add_debug(self)

        # RPC DRAM ---------------------------------------------------------------------------------
        class PhyDuplicator(Module):
            def __init__(self, main_phy, *phys, **dfi_kwargs):
                self.submodules += main_phy
                self.settings = main_phy.settings
                self.dfi = dfi.Interface(**dfi_kwargs)
                self.comb += self.dfi.connect(main_phy.dfi)
                for phy in phys:
                    self.submodules += phy
                    omit = {"rddata", "rddata_valid"}
                    self.comb += self.dfi.connect(phy.dfi, omit=omit)

        sdram_module = EM6GA16L(sys_clk_freq, "1:4")
        rpc_phy = RPCPHY(platform.request("ddram"), sys_clk_freq=sys_clk_freq)

        #  phy_settings = get_sdram_phy_settings(
        #      memtype    = "DDR3",
        #      data_width = 16,
        #      clk_freq   = sys_clk_freq)
        #  phy_settings.dfi_databits = rpc_phy.dfi_databits
        import copy
        phy_settings = copy.deepcopy(rpc_phy.settings)
        phy_settings.memtype = "RPC"
        phy_settings.phytype = "SDRAMPHYModel"

        model_phy = SDRAMPHYModel(
            module    = sdram_module,
            settings  = phy_settings,
            clk_freq  = sys_clk_freq,
            verbosity = 0,
            init      = [])

        self.submodules.sdrphy = PhyDuplicator(
            model_phy, rpc_phy,
            addressbits = sdram_module.geom_settings.addressbits,
            bankbits    = sdram_module.geom_settings.bankbits,
            nranks      = phy_settings.nranks,
            databits    = phy_settings.dfi_databits,
            nphases     = phy_settings.nphases,
        )

        controller_settings = ControllerSettings()
        controller_settings.auto_precharge = False

        self.add_sdram("sdram",
            phy                     = self.sdrphy,
            module                  = sdram_module,
            origin                  = self.mem_map["main_ram"],
            size                    = kwargs.get("max_sdram_size", 0x40000000),
            l2_cache_size           = kwargs.get("l2_size", 8192),
            l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
            l2_cache_reverse        = False,
            controller_settings     = controller_settings
        )
        # Reduce memtest size for simulation speedup
        self.add_constant("MEMTEST_DATA_SIZE", 8*1024)
        self.add_constant("MEMTEST_ADDR_SIZE", 8*1024)

        # Print info
        def dump(obj):
            print()
            print(" " + obj.__class__.__name__)
            print(" " + "-" * len(obj.__class__.__name__))
            d = obj if isinstance(obj, dict) else vars(obj)
            for var, val in d.items():
                if var == "self":
                    continue
                print("  {}: {}".format(var, val))
        print("=" * 80)
        dump(clocks)
        dump(phy_settings)
        dump(sdram_module.geom_settings)
        dump(sdram_module.timing_settings)
        print()
        print("=" * 80)

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
    clocks = Clocks({
        "sys":       dict(freq_hz=sys_clk_freq),
        "sys2x":     dict(freq_hz=2*sys_clk_freq),
        "sys4x":     dict(freq_hz=4*sys_clk_freq),
        "sys4x_90":  dict(freq_hz=4*sys_clk_freq, phase_deg=90),
        "sys4x_ddr": dict(freq_hz=2*4*sys_clk_freq),
        "sys4x_90_ddr": dict(freq_hz=2*4*sys_clk_freq, phase_deg=180),
    })

    clocks.add_io(_io)

    sim_config = SimConfig()
    clocks.add_clockers(sim_config)

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
        clocks        = clocks,
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
