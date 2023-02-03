#
# This file is part of LiteDRAM.
#
# Copyright (c) 2023 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

# Python
import subprocess
import os
import pytest
# migen
from migen import *

RCDSubmodulesDirectory = os.path.dirname(os.path.dirname(__file__))+"/litedram/DDR5RCD01"

pytestmark = pytest.mark.parametrize("file", [file for file in os.listdir(RCDSubmodulesDirectory) if file.endswith('.py')])

class Test_DDR5RCD01System:
    class TestBed(Module):
        def __init__(self):
            pass

        def scenario(self, py):
            try:
                subprocess.check_output(
                    "python "+RCDSubmodulesDirectory+"/"+py, shell=True, stderr=subprocess.STDOUT)
            except subprocess.CalledProcessError as e:
                if "UnderConstruction" in str(e.output):
                    raise UnderConstruction
                if "NotSupportedException" in str(e.output):
                    return
                raise ValueError

    def setup_method(self):
        self.tb = Test_DDR5RCD01System.TestBed()

    def teardown_method(self):
        del self.tb

    def test_main(self, file):
        self.tb.scenario(file)
