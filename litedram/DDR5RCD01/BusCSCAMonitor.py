#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import logging
from operator import xor
from dataclasses import dataclass
# migen
from migen import *
from migen.fhdl import verilog
# Litex
from litedram.DDR5RCD01.RCD_definitions import *
from litedram.DDR5RCD01.RCD_interfaces import *
from litedram.DDR5RCD01.RCD_interfaces_external import *
from litedram.DDR5RCD01.RCD_utils import *
#
from litedram.DDR5RCD01.BusCSCAEnvironment import BusCSCAEnvironment
from litedram.DDR5RCD01.BusCSCAEnvironment import EnvironmentScenarios


@enum.unique
class MonitorCmdTypes(enum.IntEnum):
    INACTIVE = 0x1
    SINGLE_UI = 0x2
    DOUBLE_UI = 0x3
    FOLLOW_UP = 0xF


class MonitorQueue:
    def __init__(self):
        self.q = []
        self.q_filter_inactive = []

    def set_statistics(self):
        self.pattern_num = len(self.q)
        self.received_cmds = len(self.q_filter_inactive)

    def filter_inactive(self):
        filter_ids = []
        for q_id, q_item in enumerate(self.q):
            for cmd_item in q_item:
                if cmd_item.cmd[0].cmd_type == MonitorCmdTypes.INACTIVE:
                    is_filtered = True
                else:
                    is_filtered = False
            if not is_filtered:
                self.q_filter_inactive.append(q_item)
        self.set_statistics()

    def __eq__(self, other):
        pass

    def __str__(self, is_filtered=False):
        if is_filtered:
            q = self.q_filter_inactive
        else:
            q = self.q

        s = ""

        for q_id, q_item in enumerate(q):
            s += f" --- Q[{q_id}]\r\n"
            for cmd_item in q_item:
                for id, cmd in enumerate(cmd_item.cmd):
                    s += f"UI[{id}]: "
                    s += str(cmd)
                    s += "\r\n"
        s += "Monitor statistics\r\n"
        s += "------------------\r\n"
        s += f"Received [{self.received_cmds}] commands in [{self.pattern_num}] patterns\r\n"
        return s


class MonitorCmd:
    def __init__(self, monitor_ctrl_list):
        self.cmd = monitor_ctrl_list

    def __eq__(self, other):
        pass


@dataclass
class MonitorCtrl:
    """
    Track monitor signals
    """
    is_active: int
    is_follow_up: int
    cmd_len: int
    cmd_type: int
    dcs_n: int
    dca: int
    dpar: int

    def __init__(self, tuple):
        attrs = ["is_active", "is_follow_up", "cmd_len",
                 "cmd_type", "dcs_n", "dca", "dpar"]
        for id, attr in enumerate(attrs):
            setattr(self, attr, tuple[id])

    def __str__(self, debug=False):
        if debug:
            s = ""
            if self.is_active:
                s += "is_active = " + str(self.is_active) + " "
                s += "cmd_len = " + str(self.cmd_len) + " "
                if self.is_follow_up:
                    s += "follow_up_UI"
                if self.cmd_type != MonitorCmdTypes.FOLLOW_UP:
                    if self.cmd_type == MonitorCmdTypes.INACTIVE:
                        s += "cmd_type = " + "INACTIVE" + " "
                    elif self.cmd_type == MonitorCmdTypes.SINGLE_UI:
                        s += "cmd_type = " + "1 UI CMD" + " "
                    elif self.cmd_type == MonitorCmdTypes.DOUBLE_UI:
                        s += "cmd_type = " + "2 UI CMD" + " "
                    s += "dcs_n = " + str(self.dcs_n) + " "
                    s += "dca = " + str(self.dca) + " "
                    s += "dpar = " + str(self.dpar) + " "
            else:
                s += "inactive"
        else:
            s = ""
            if self.is_active:
                # if self.is_follow_up:
                #     s += "NUI"
                # else:
                #     s += "1UI"
                s += "dcs_n = " + str(self.dcs_n) + " "
                s += "dca = " + str(self.dca) + " "
                s += "dpar = " + str(self.dpar) + " "
            else:
                s += "inactive"
        return s


class BusCSCAMonitor(Module):
    """
        DDR5 RCD01 Monitor

        cmd_len : bit width of this signal should be large enough
        to count all clock cycles of the simulation.
        Hard to assess how much array depth is required, this
        is related to environment setup. The longer the simulation,
        the longer the array.

        Module
        ------

        Parameters
        ------

        Sample output:
            - inactive for n clocks
            - 1 ui message
            - inactive for 1 clock
            - 2 ui message
            - 1 ui message
            - inactive

        Data structure to hold this:
        List would be best. migen style list is an Array

        I think that squashing inactives together is useful and would free some memory,
        array width {CSCA  bus capture, metadata}
        single entry
        is_active, cmd_len (cmd id),ui_n,{command_payload}
        []

        if cmd is inactive:
            increase inactive counter

        if cmd is active:
            save(inactive counter state)
            check if cmd is 1 ui or 2 ui
            UI length is based on CA1:
                CA1==HIGH => CMD is 1 UI
                CA1==LOW  => CMD is 2 UI
            save(next 1/2ui)

        reset(counters)

        post_process():
            read():
            analyze():

        cmd_type = {INACTIVE, SINGLE_UI, DOUBLE_UI}
    """

    def __init__(self,
                 if_ibuf_i,
                 dcs_n_w=2,
                 dca_w=7,
                 monit_arr_d=128):

        dcs_n = Signal(dcs_n_w)
        dca = Signal(dca_w)
        dpar = Signal()
        dpar_w = len(dpar)

        self.comb += dcs_n.eq(if_ibuf_i.dcs_n)
        self.comb += dca.eq(if_ibuf_i.dca)
        self.comb += dpar.eq(if_ibuf_i.dpar)

        CSCABus_w = dcs_n_w + dca_w + dpar_w

        """
            Create the Array
            Signals that come into the array are meant for post-processing

        """
        cmd_len = Signal(16)
        cmd_len_w = len(cmd_len)

        xarr_ptr = Signal(cmd_len_w)

        counter_invalid = Signal(cmd_len_w)
        counter_en = Signal()
        counter_rst = Signal(reset=0)

        """
            XOR edge detection
        """
        del_dcs_n = Signal(dcs_n_w, reset=~0)
        self.sync += del_dcs_n.eq(dcs_n)

        del_dca = Signal(dca_w, reset=0)
        self.sync += del_dca.eq(dca)

        del_dpar = Signal(reset=0)
        self.sync += del_dpar.eq(dpar)

        det_edge = Signal(2)
        self.comb += det_edge.eq(dcs_n ^ del_dcs_n)

        det_posedge = Signal(2)
        self.comb += det_posedge.eq(det_edge & dcs_n)

        det_negedge = Signal(2)
        self.comb += det_negedge.eq(det_edge & ~dcs_n)

        cmd_active = Signal()

        is_1_ui_command = Signal()
        self.comb += is_1_ui_command.eq(dca[1])

        ui_counter = Signal(8)
        self.sync += If(
            det_negedge,
            If(
                is_1_ui_command,
                ui_counter.eq(2)
            ).Else(
                ui_counter.eq(4)
            )
        ).Else(
            If(
                ui_counter > 0,
                ui_counter.eq(ui_counter-1)
            ).Else(
                ui_counter.eq(ui_counter)
            )
        )

        self.comb += If(
            ui_counter > 0,
            cmd_active.eq(1)
        )

        """
            Invalid counter
            Multiple invalid cycles (cs kept high) are counted and kept in the

        """
        counter_save = Signal()
        self.comb += If(
            det_negedge,
            counter_save.eq(1),
        )
        self.comb += If(
            cmd_active == 1,
            counter_en.eq(0),
            counter_rst.eq(1),
        ).Else(
            counter_en.eq(1),
            counter_rst.eq(0),
        )

        self.sync += If(
            counter_rst,
            counter_invalid.eq(0),
        ).Else(
            If(
                counter_en,
                counter_invalid.eq(counter_invalid + 1)
            )
        )

        """
            Assemble the signals and write to Array
        """

        self.xarr_is_active = Array(Signal() for _ in range(monit_arr_d))
        self.xarr_is_follow_up = Array(Signal() for _ in range(monit_arr_d))
        self.xarr_cmd_len = Array(Signal(16) for _ in range(monit_arr_d))
        self.xarr_cmd_type = Array(Signal(4) for _ in range(monit_arr_d))
        self.xarr_dcs_n = Array(Signal(dcs_n_w) for _ in range(monit_arr_d))
        self.xarr_dca = Array(Signal(dca_w) for _ in range(monit_arr_d))
        self.xarr_dpar = Array(Signal() for _ in range(monit_arr_d))

        xarr_we = Signal()

        self.comb += xarr_we.eq(
            counter_save | cmd_active
        )

        cmd_len = Signal(16)
        self.comb += If(
            counter_save,
            cmd_len.eq(counter_invalid)
        ).Else(
            cmd_len.eq(1)
        )

        del_cmd_active = Signal()
        self.sync += del_cmd_active.eq(cmd_active)
        is_follow_up = Signal()
        self.comb += is_follow_up.eq(cmd_active & del_cmd_active)

        cmd_type = Signal(4)

        self.sync += If(
            det_negedge,
            If(
                is_1_ui_command,
                cmd_type.eq(MonitorCmdTypes.SINGLE_UI)
            ).Else(
                cmd_type.eq(MonitorCmdTypes.DOUBLE_UI)
            )
        ).Else(
            If(
                cmd_active,
                cmd_type.eq(MonitorCmdTypes.FOLLOW_UP)
            ).Else(
                cmd_type.eq(MonitorCmdTypes.INACTIVE)
            )
        )

        self.sync += If(
            xarr_we,
            self.xarr_is_active[xarr_ptr].eq(cmd_active),
            self.xarr_is_follow_up[xarr_ptr].eq(is_follow_up),
            self.xarr_cmd_len[xarr_ptr].eq(cmd_len),
            self.xarr_cmd_type[xarr_ptr].eq(cmd_type),
            self.xarr_dcs_n[xarr_ptr].eq(del_dcs_n),
            self.xarr_dca[xarr_ptr].eq(del_dca),
            self.xarr_dpar[xarr_ptr].eq(del_dpar),
        )

        self.sync += If(
            xarr_we,
            xarr_ptr.eq(xarr_ptr+1)
        )

        self.xarr_overflow = Signal()
        self.comb += If(
            xarr_ptr >= monit_arr_d,
            self.xarr_overflow.eq(1)
        )

    def post_process(self):
        xarr_is_active = yield self.xarr_is_active
        xarr_is_follow_up = yield self.xarr_is_follow_up
        xarr_cmd_len = yield self.xarr_cmd_len
        xarr_cmd_type = yield self.xarr_cmd_type
        xarr_dcs_n = yield self.xarr_dcs_n
        xarr_dca = yield self.xarr_dca
        xarr_dpar = yield self.xarr_dpar

        xarr_post_sim = list(zip(xarr_is_active, xarr_is_follow_up, xarr_cmd_len,
                                 xarr_cmd_type, xarr_dcs_n, xarr_dca, xarr_dpar))

        xarr_monitor_ctrls = []
        for id, item in enumerate(xarr_post_sim):
            xarr_monitor_ctrls.append(MonitorCtrl(item))

        self.squash_follow_ups(xarr=xarr_monitor_ctrls)
        self.monit_q.filter_inactive()

    def squash_follow_ups(self, xarr):
        self.monit_q = MonitorQueue()
        bus_cs_ca_cmd = []
        for id, item in enumerate(xarr):
            if item.cmd_type == MonitorCmdTypes.SINGLE_UI:
                """ Expect that the next entry is a follow-up"""
                if xarr[id+1].cmd_type == MonitorCmdTypes.FOLLOW_UP:
                    bus_cs_ca_cmd.append(MonitorCmd(
                        [xarr[id+_] for _ in range(2)]))

            if item.cmd_type == MonitorCmdTypes.DOUBLE_UI:
                """ Expect that the next 3 entries are follow-ups"""
                follow_up_ids = [1, 2, 3]
                follow_ups_types = [xarr[id+m].cmd_type for m in follow_up_ids]
                are_follow_ups = [(MonitorCmdTypes.FOLLOW_UP == follow_up_type)
                                  for follow_up_type in follow_ups_types]
                if all(are_follow_ups):
                    bus_cs_ca_cmd.append(MonitorCmd(
                        [xarr[id+_] for _ in range(4)]))

                    pass

            if item.cmd_type == MonitorCmdTypes.INACTIVE:
                bus_cs_ca_cmd.append(MonitorCmd([xarr[id]]))

            if bus_cs_ca_cmd != []:
                self.monit_q.q.append(bus_cs_ca_cmd)

            bus_cs_ca_cmd = []


class TestBed(Module):
    def __init__(self):
        if_ibuf = If_ibuf()
        self.submodules.env = BusCSCAEnvironment(
            if_ibuf_o=if_ibuf,
        )
        self.submodules.monitor = BusCSCAMonitor(
            if_ibuf_i=if_ibuf
        )


def run_test(tb):
    logging.debug('Write test')
    scenario_select = EnvironmentScenarios.SIMPLE_GENERIC
    yield from tb.env.run_env(scenario_select=scenario_select)
    yield from tb.monitor.post_process()
    logging.debug(str(tb.monitor.monit_q))
    logging.debug('Yield from write test.')


if __name__ == "__main__":
    eT = EngTest()
    logging.info("<- Module called")
    tb = TestBed()
    logging.info("<- Module ready")
    run_simulation(tb, run_test(tb), vcd_name=eT.wave_file_name)
    logging.info("<- Simulation done")
    logging.info(str(eT))
