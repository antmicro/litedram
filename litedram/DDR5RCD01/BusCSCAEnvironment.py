#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
import random
from collections import namedtuple
from operator import xor
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.BusCSCAAgent import BusCSCAAgent

Payload = namedtuple('payload', ['mra', 'op', 'cw'])


class BusCSCAEnvironment(Module):
    """
        Scenarios:
            Single MRW/MRR
            Pattern MRWs/MRRs
            Random Valid
            Random Random

        Write options:
            Randomized
            Range (i,i+i,...)
            Fixed

        queue_pattern_MRWs = {
            ["MRW", MRA, OP, CW]
            ["MRW", MRA, OP+1, CW]
            ["MRW", MRA, OP+2, CW]
            ["MRW", MRA, OP+3, CW]
        }
        ["MRW_2_RCD_PATTERN_ITER", range=[23,46], payload="random|fixed|range"] # Perform MRWs from addr 23,46
        ["MRW_2_RCD_PATTERN_RND", n=20 writes, payload="random|fixed|range"] # Perform MRWs at random addresses
        ["MRW_2_RCD_SINGLE", dest_register=0x5E, dest_val=0xFF]
        ["MRW_2_DRAM_SINGLE", 3]
    """

    def __init__(self,
                 if_ibuf_o
                 ):

        self.queue = []
        xBusCSCAAgent = BusCSCAAgent(
            if_ibuf_o=if_ibuf_o,
        )
        self.submodules.agent = xBusCSCAAgent

    def run_env(self, scenario="None"):
        self.create_queue(scenario)
        yield from self.agent.run_agent(self.queue)

    def create_queue(self,scenario):
        self.queue = [
            {
                "opcode": 0b00101,
                "payload": Payload(0x10, 0x20, 0x1),
                "randomize_payload": False,
                "destination_rank": "AB",
                "datarate": "DDR",
                "ui": 2,
                "is_padded": True,
                "padding_len": 1
            },
            {
                "opcode": 0b00101,
                "payload": Payload(0x10, 0x20, 0x0),
                "randomize_payload": False,
                "destination_rank": "A",
                "datarate": "DDR",
                "ui": 2,
                "is_padded": True,
                "padding_len": 1
            },
            {
                "opcode": 0b11111,
                "payload": Payload(0x10, 0x20, 0x0),
                "randomize_payload": True,
                "destination_rank": "B",
                "datarate": "DDR",
                "ui": 1,
                "is_padded": False,
                "padding_len": 1
            },
        ]


class TestBed(Module):
    def __init__(self):
        if_ibuf_o = If_ibuf()
        self.submodules.dut = BusCSCAEnvironment(
            if_ibuf_o=if_ibuf_o,
        )


def run_test(tb):
    logging.debug('Write test')
    yield from tb.dut.run_env()
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
