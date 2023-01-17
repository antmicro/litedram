#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
# migen
from migen import *
# RCD
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
# Submodules
from litedram.DDR5RCD01.DDR5RCD01Loopback import DDR5RCD01Loopback
from litedram.DDR5RCD01.DDR5RCD01Alert import DDR5RCD01Alert
from litedram.DDR5RCD01.DDR5RCD01PLL import DDR5RCD01PLL


class DDR5RCD01Common(Module):
    """DDR5 RCD01 Common
    TODO
    The common:
        - PLL
        - Loopback
        - Error/alert
        - QRST

    Module
    ------
    <interface> : CS,CA,etc.
    dck, dck_pll 
    """

    def __init__(self,
                 if_ck_rst,
                 if_alert_n,
                 if_lb,
                 if_pll,
                 if_common,
                 if_ctrl_common,
                 if_config_common,
                 ):

        # xlb = DDR5RCD01Loopback(
        #     if_ck_rst=if_ck_rst,
        #     if_lb=if_lb,
        #     if_common=if_common,
        #     if_ctrl_common=if_ctrl_common,
        #     if_config_common=if_config_common,
        # )
        # self.submodules += xlb

        # xalert = DDR5RCD01Alert(
        #     if_alert_n=if_alert_n,
        #     if_common=if_common,
        #     if_ctrl_common=if_ctrl_common,
        #     if_config_common=if_config_common,
        # )
        # self.submodules += xalert

        xpll = DDR5RCD01PLL(
            if_ck_rst=if_ck_rst,
            if_pll=if_pll,
            if_common=if_common,
            if_ctrl_common=if_ctrl_common,
            if_config_common=if_config_common,
        )
        self.submodules += xpll

        # Reset distribution
        # self.comb += if_channel_A_rst_n.rst_n.eq(if_host_rst_n.rst_n)
        # self.comb += if_channel_B_rst_n.rst_n.eq(if_host_rst_n.rst_n)
        # self.comb += if_sdram_A_rst_n.rst_n.eq(if_host_rst_n.rst_n)
        # self.comb += if_sdram_B_rst_n.rst_n.eq(if_host_rst_n.rst_n)


class TestBed(Module):
    def __init__(self):

        self.submodules.dut = DDR5RCD01Common()


def run_test(tb):
    logging.debug('Write test')
    for i in range(5):
        yield
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
