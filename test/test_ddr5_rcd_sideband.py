#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import unittest
import logging
# migen
from migen import *
from migen.fhdl import verilog
# RCD
from litedram.DDR5RCD01.RCD_utils import *
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
# Submodules
from litedram.DDR5RCD01.DDR5RCD01Registers import DDR5RCD01Registers
from litedram.DDR5RCD01.I2CMockMaster import I2CMockMaster
from litedram.DDR5RCD01.I2CMockSlave import I2CMockSlave
from litedram.DDR5RCD01.CRG import CRG


class TestBed(Module):
    def __init__(self, is_dual_channel=False):
        RESET_TIME = 1
        self.clocks = {
            "sys":      (128, 63),
            "sysx2":    (64, 31),
            "sys_rst":  (128, 63+4),
        }
        self.submodules.xcrg = CRG(
            clocks=self.clocks,
            reset_cnt=RESET_TIME
        )
        self.generators = {}

        """
            I2C master
            Master is responsible for generating a WR/RD pattern
        """

        xmock_master = I2CMockMaster()
        self.submodules.xmock_master = xmock_master

        """
            Slave is responsible for receiving commands from master
            and translating them into RCD Regfile reads/writes
        """
        xmock_slave = I2CMockSlave()
        self.submodules.xmock_slave = xmock_slave

        """
            RCD register file
            If write is to address 0x0 to 0x5F, the write is to register file
            If write is to address 0x60 to 0xFF, the write is to pages
            Reads are always through reg_q (must set pointers before reading)
        """
        self.d = Signal(CW_REG_BIT_SIZE)
        self.addr = Signal(CW_REG_BIT_SIZE)
        self.we = Signal()
        self.q = Signal(CW_REG_BIT_SIZE)
        # cw_page_num = CW_PAGE_NUM
        cw_page_num = 6
        xregisters = DDR5RCD01Registers(
            d=self.d,
            addr=self.addr,
            we=self.we,
            q=self.q,
            cw_page_num=cw_page_num
        )
        self.submodules.xregisters = xregisters

        """
            Generators
        """
        self.add_generators(
            self.generators_dict()
        )

    def generators_dict(self):
        return {
            "sys":
            [
                self.seq()
            ]
        }

    def add_generators(self, generators):
        for key, value in generators.items():
            if key not in self.generators:
                self.generators[key] = list()
            if not isinstance(value, list):
                value = list(value)
            self.generators[key].extend(value)

    def run_test(self):
        return self.generators

    def seq(self):
        while (yield ResetSignal("sys")):
            yield
        for i in range(5):
            yield from self.reg_write(w_addr=i, w_data=32+i)
        # ADDR_CW_READ_POINTER
        # Write 0 to ADDR_CW_READ_POINTER means: "q show register 0"
        yield from self.reg_write(w_addr=ADDR_CW_READ_POINTER, w_data=1)
        # ADDR_CW_PAGE
        # Write 0 to ADDR_CW_READ_POINTER means: "registers 0x60-0xFF are from page 0"
        yield from self.reg_write(w_addr=0x61, w_data=0xFF)
        yield from self.reg_write(w_addr=ADDR_CW_PAGE, w_data=0)
        yield
        yield from self.reg_write(w_addr=ADDR_CW_PAGE, w_data=1)
        yield
        yield

    def reg_init(self):
        yield self.d.eq(0)
        yield self.addr.eq(0)
        yield self.we.eq(0)
        yield

    def reg_write(self, w_addr, w_data):
        yield from self.reg_init()
        yield self.d.eq(w_data)
        yield self.addr.eq(w_addr)
        yield self.we.eq(1)
        yield
        yield from self.reg_init()


class DDR5RCD01DecoderTests(unittest.TestCase):

    def setUp(self):
        self.tb = TestBed(is_dual_channel=False)
        """
            Waveform file
        """
        dir_name = "./wave_ut"
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)
        file_name = self._testMethodName
        self.wave_file_name = dir_name + '/' + file_name + ".vcd"
        """
            Logging
        """
        LOG_FILE_NAME = dir_name + '/' + file_name + ".log"
        FORMAT = "[%(module)s.%(funcName)s] %(message)s"
        fileHandler = logging.FileHandler(filename=LOG_FILE_NAME, mode='w')
        fileHandler.formatter = logging.Formatter(FORMAT)
        streamHandler = logging.StreamHandler()

        logger = logging.getLogger('root')
        logger.addHandler(fileHandler)
        logger.addHandler(streamHandler)
        logger.setLevel(logging.DEBUG)

    def tearDown(self):
        del self.tb

    def test_cw_rd_wr(self):
        logger = logging.getLogger('root')
        logger.debug("-"*80)
        run_simulation(
            self.tb,
            generators=self.tb.run_test(),
            clocks=self.tb.xcrg.clocks,
            vcd_name=self.wave_file_name
        )
        """
            Use cases:
            TODO write to a register and read from it
            TODO write to a register in bank and read from it
        """


if __name__ == '__main__':
    unittest.main()
