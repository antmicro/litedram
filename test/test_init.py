#
# This file is part of LiteDRAM.
#
# Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import difflib
import unittest

from litedram.init import get_sdram_phy_c_header, get_sdram_phy_py_header


def compare_with_reference(test_case, content, filename):
    ref_filename = os.path.join("test", "reference", filename)
    with open(ref_filename, "r") as f:
        reference = f.read().split("\n")
    content = content.split("\n")
    diff = list(difflib.unified_diff(content, reference, fromfile=filename, tofile=ref_filename))
    msg = "Unified diff:\n" + "\n".join(diff)
    test_case.assertEqual(len(diff), 0, msg=msg)


class TestInit(unittest.TestCase):
    def test_sdr(self):
        from litex.boards.targets.minispartan6 import BaseSoC
        soc       = BaseSoC()
        c_header  = get_sdram_phy_c_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        py_header = get_sdram_phy_py_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        compare_with_reference(self, c_header, "sdr_init.h")
        compare_with_reference(self, py_header, "sdr_init.py")

    def test_ddr3(self):
        from litex.boards.targets.kc705 import BaseSoC
        soc       = BaseSoC()
        c_header  = get_sdram_phy_c_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        py_header = get_sdram_phy_py_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        compare_with_reference(self, c_header, "ddr3_init.h")
        compare_with_reference(self, py_header, "ddr3_init.py")

    def test_ddr4(self):
        from litex.boards.targets.kcu105 import BaseSoC
        soc       = BaseSoC(max_sdram_size=0x4000000)
        c_header  = get_sdram_phy_c_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        py_header = get_sdram_phy_py_header(soc.sdram.controller.settings.phy, soc.sdram.controller.settings.timing)
        compare_with_reference(self, c_header, "ddr4_init.h")
        compare_with_reference(self, py_header, "ddr4_init.py")
