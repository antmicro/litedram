#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# python
import logging
# migen
from migen import *
# RCD
from litedram.DDR5RCD01.DDR5RCD01ChannelIngressSimulationPads import DDR5RCD01ChannelIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01CommonIngressSimulationPads import DDR5RCD01CommonIngressSimulationPads
from litedram.DDR5RCD01.DDR5RCD01DataBufferSimulationPads import DDR5RCD01DataBufferSimulationPads
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *


class RCDGlueDDR5Channel(Module):
    """
    DRCGlueDDR5Common,
    -------------
    This module provides glue logic for channel A and B common signals
    between RCD Simulation Pads and DDR5 Simulation Pads.
    This is required only to translate names between blocks,
    which used different naming conventions(UDIMM from host, RDIMM in RCD).

    Module
    ------

    Parameters
    ----------
        - pads_RCD
        - pads_SDRAM
        - quarter (default = "A")
    """
    def __init__(self, pads_RCD, pads_SDRAM, quarter="A"):
        quarter_to_signals = {
            #     ck_t  cd_c   cs        ca    rst
            "A": ["qa", "qa", ("qa", 0), "qa", ""],
            "B": ["qb", "qb", ("qb", 0), "qb", ""],
            "C": ["qc", "qc", ("qa", 1), "qa", ""],
            "D": ["qd", "qd", ("qb", 1), "qb", ""],
        }
        # Connect simPHY to RCD
        connection_matrix_sc = [
            ('ck_t', "ck_t"),
            ('ck_c', "ck_c"),
            ('cs_n', "cs_n"),
            ('ca',   "ca"),
            ('qrst_n', "reset_n"),
        ]
        src, dst = pads_RCD, pads_SDRAM

        for prefix, (src_name, dst_name) in zip(quarter_to_signals[quarter], connection_matrix_sc):
            if isinstance(prefix, str):
                logging.info(f'Connect : {prefix+src_name} to {dst_name}')
                self.comb += getattr(dst, dst_name).eq(getattr(src, prefix+src_name))
            else:
                prefix, idx = prefix
                logging.info(f'Connect : {prefix+src_name}[{idx}] to {dst_name}')
                self.comb += getattr(dst, dst_name).eq(getattr(src, prefix+src_name)[idx])


class RCDGlueDDR5DataBuffer(Module):
    """
    RCDGlueDDR5DataBuffer
    -------------
    This module provides glue logic for channels data signals
    between DDR5 Simulation Pads and RCD Simulation Pads.
    This is required only to translate names between blocks,
    which used different naming conventions(UDIMM from host, RDIMM in RCD).

    Module
    ------
    Parameters
    ----------
        - pads_RCD
        - pads_SDRAM
    """
    def __init__(self, pads_RCD, pads_SDRAM, prefix=""):
        # Connect simPHY to RCD
        connection_matrix_sc = [
            ('dq', 'dq'),
            # ECC not yet supported
            # ('cb', 'cb'),
            ('dqs_t', 'dqs_t'),
            ('dqs_c', 'dqs_c'),
        ]
        src, dst = pads_RCD, pads_SDRAM

        for src_name, dst_name in connection_matrix_sc:
            for suffix in ["", "_o", "_oe", "_i"]:
                logging.info(f'Connect : {src_name+suffix} to {dst_name+suffix}')
                if suffix != "_i":
                    self.comb += getattr(dst, dst_name+suffix).eq(getattr(src, src_name+suffix))
                else:
                    self.comb += getattr(src, src_name+suffix).eq(getattr(dst, dst_name+suffix) | getattr(src, src_name+suffix))


if __name__ == "__main__":
    raise NotSupportedException
