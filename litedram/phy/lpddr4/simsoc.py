import argparse

from migen import *

from litex.build.generic_platform import Pins, Subsignal
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc_sdram import soc_sdram_args, soc_sdram_argdict
from litex.soc.integration.builder import builder_args, builder_argdict, Builder
from litex.soc.cores.cpu import CPUS

from litedram import modules as litedram_modules
from litedram.core.controller import ControllerSettings

from litedram.phy.lpddr4.simphy import LPDDR4SimPHY
from litedram.phy.lpddr4.sim import LPDDR4Sim

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

    ("lpddr4", 0,
        Subsignal("clk_p",   Pins(1)),
        Subsignal("clk_n",   Pins(1)),
        Subsignal("cke",     Pins(1)),
        Subsignal("odt",     Pins(1)),
        Subsignal("reset_n", Pins(1)),
        Subsignal("cs",      Pins(1)),
        Subsignal("ca",      Pins(6)),
        Subsignal("dqs",     Pins(2)),
        # Subsignal("dqs_n",   Pins(2)),
        Subsignal("dmi",     Pins(2)),
        Subsignal("dq",      Pins(16)),
    ),
]

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Clocks -------------------------------------------------------------------------------------------

class Clocks(dict):  # FORMAT: {name: {"freq_hz": _, "phase_deg": _}, ...}
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
        # request() before creating domains to avoid signal renaming problem
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

def get_clocks(sys_clk_freq):
    return Clocks({
        "sys":           dict(freq_hz=sys_clk_freq),
        "sys_11_25":     dict(freq_hz=sys_clk_freq, phase_deg=11.25),
        "sys8x":         dict(freq_hz=8*sys_clk_freq),
        "sys8x_ddr":     dict(freq_hz=2*8*sys_clk_freq),
        "sys8x_90":      dict(freq_hz=8*sys_clk_freq, phase_deg=90),
        "sys8x_90_ddr":  dict(freq_hz=2*8*sys_clk_freq, phase_deg=2*90),
    })

# SoC ----------------------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self, clocks, auto_precharge=False, with_refresh=True, trace_reset=0,
            log_level="INFO", disable_delay=False, **kwargs):
        platform     = Platform()
        sys_clk_freq = clocks["sys"]["freq_hz"]

        # SoCCore ----------------------------------------------------------------------------------
        super().__init__(platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX Simulation",
            ident_version = True,
            cpu_variant   = "minimal",
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, clocks.names())

        # Debugging --------------------------------------------------------------------------------
        platform.add_debug(self, reset=trace_reset)

        # LPDDR4 -----------------------------------------------------------------------------------
        sdram_module = litedram_modules.MT53E256M16D1(sys_clk_freq, "1:8")
        pads = platform.request("lpddr4")
        self.submodules.sdrphy = LPDDR4SimPHY(sys_clk_freq=sys_clk_freq, aligned_reset_zero=True)
        self.add_csr("sdrphy")

        for p in ["clk_p", "clk_n", "cke", "odt", "reset_n", "cs", "ca", "dq", "dqs", "dmi"]:
            self.comb += getattr(pads, p).eq(getattr(self.sdrphy.pads, p))

        controller_settings = ControllerSettings()
        controller_settings.auto_precharge = auto_precharge
        controller_settings.with_refresh = with_refresh

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

        # LPDDR4 Sim -------------------------------------------------------------------------------
        self.submodules.lpddr4sim = LPDDR4Sim(
            pads          = self.sdrphy.pads,
            settings      = self.sdram.controller.settings,
            sys_clk_freq  = sys_clk_freq,
            log_level     = log_level,
            disable_delay = disable_delay,
        )

        self.add_constant("CONFIG_SIM_DISABLE_BIOS_PROMPT")
        if disable_delay:
            self.add_constant("CONFIG_SIM_DISABLE_DELAYS")

        # Debug info -------------------------------------------------------------------------------
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
        dump(self.sdrphy.settings)
        dump(sdram_module.geom_settings)
        dump(sdram_module.timing_settings)
        print()
        print("=" * 80)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generic LiteX SoC Simulation")
    builder_args(parser)
    soc_sdram_args(parser)
    parser.add_argument("--sdram-verbosity",      default=0,               help="Set SDRAM checker verbosity")
    parser.add_argument("--trace",                action="store_true",     help="Enable Tracing")
    parser.add_argument("--trace-fst",            action="store_true",     help="Enable FST tracing (default=VCD)")
    parser.add_argument("--trace-start",          default=0,               help="Cycle to start tracing")
    parser.add_argument("--trace-end",            default=-1,              help="Cycle to end tracing")
    parser.add_argument("--trace-reset",          default=0,               help="Initial traceing state")
    parser.add_argument("--sys-clk-freq",         default="50e6",          help="Core clock frequency")
    parser.add_argument("--auto-precharge",       action="store_true",     help="Use DRAM auto precharge")
    parser.add_argument("--no-refresh",           action="store_true",     help="Disable DRAM refresher")
    parser.add_argument("--log-level",            default="INFO",          help="Set simulation logging level")
    parser.add_argument("--disable-delay",        action="store_true",     help="Disable CPU delays")
    args = parser.parse_args()

    soc_kwargs     = soc_sdram_argdict(args)
    builder_kwargs = builder_argdict(args)

    sim_config = SimConfig()
    sys_clk_freq = int(float(args.sys_clk_freq))
    clocks = get_clocks(sys_clk_freq)
    clocks.add_io(_io)
    clocks.add_clockers(sim_config)

    # Configuration --------------------------------------------------------------------------------
    cpu = CPUS[soc_kwargs.get("cpu_type", "vexriscv")]
    if soc_kwargs["uart_name"] == "serial":
        soc_kwargs["uart_name"] = "sim"
        sim_config.add_module("serial2console", "serial")
    args.with_sdram = True
    soc_kwargs["integrated_main_ram_size"] = 0x0
    soc_kwargs["sdram_verbosity"]          = int(args.sdram_verbosity)

    # SoC ------------------------------------------------------------------------------------------
    soc = SimSoC(
        clocks         = clocks,
        auto_precharge = args.auto_precharge,
        with_refresh   = not args.no_refresh,
        trace_reset    = int(args.trace_reset),
        log_level      = args.log_level,
        disable_delay  = args.disable_delay,
        **soc_kwargs)

    # Build/Run ------------------------------------------------------------------------------------
    builder_kwargs["csr_csv"] = "csr.csv"
    builder = Builder(soc, **builder_kwargs)
    build_kwargs = dict(
        sim_config  = sim_config,
        trace       = args.trace,
        trace_fst   = args.trace_fst,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end)
    )
    vns = builder.build(run=False, **build_kwargs)
    builder.build(build=False, **build_kwargs)

if __name__ == "__main__":
    main()
