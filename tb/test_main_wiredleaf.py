import cocotb
from cocotb.triggers import Timer

@cocotb.test()
def test_main_wiredleaf(dut):
    """
    Try accessing the design
    """
    dut.log.info("Running test!")
    for cycle in range(10):
        dut.clock_50mhz = 0
        yield Timer(1000)
        dut.clock_50mhz = 1
        yield Timer(1000)
    dut.log.info("Running test!")
