
TOPLEVEL_LANG ?= verilog
PWD=$(shell pwd)
TOPDIR=$(PWD)/..

EXTRA_ARGS += -y $(XILINX)/verilog/src/unisims -y $(XILINX)/verilog/src/simprims -y $(XILINX)/verilog/src/XilinxCoreLib
### List all Verilog (.v) files for this project explicitly below.
### Leave blank if you only have VHDL files.
### Use one file per line; do not use wildcards (eg, hdl/*.v).
VERILOG_SOURCES = $(XILINX)/verilog/src/glbl.v
VERILOG_SOURCES += $(TOPDIR)/hdl/main_wiredleaf.v
VERILOG_SOURCES += $(TOPDIR)/hdl/clk_wiz_50_44mhz.v
VERILOG_SOURCES += $(TOPDIR)/hdl/clk_wiz_50_125mhz.v
VERILOG_SOURCES += $(TOPDIR)/hdl/rhd2000_driver.v
VERILOG_SOURCES += $(TOPDIR)/hdl/rhd2000_spi.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sngdaq.v
VERILOG_SOURCES += $(TOPDIR)/hdl/central_core.v
VERILOG_SOURCES += $(TOPDIR)/hdl/cfg_master.v
VERILOG_SOURCES += $(TOPDIR)/hdl/spi_slave.v
VERILOG_SOURCES += $(TOPDIR)/hdl/debounce.v
VERILOG_SOURCES += $(TOPDIR)/hdl/board_identifier.v
VERILOG_SOURCES += $(TOPDIR)/hdl/gpio_core.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sine_table.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sata_core.v
VERILOG_SOURCES += $(TOPDIR)/hdl/udp_core.v
VERILOG_SOURCES += $(TOPDIR)/hdl/daq_core.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sngdaq_dummy.v
VERILOG_SOURCES += $(TOPDIR)/hdl/daq_interfaces.v
VERILOG_SOURCES += $(TOPDIR)/hdl/udp.v
VERILOG_SOURCES += $(TOPDIR)/hdl/crc.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sata_top.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sata_io.v
VERILOG_SOURCES += $(TOPDIR)/hdl/sata_control.v

# DDR3 Autogen Core
VERILOG_SOURCES += $(TOPDIR)/hdl/infrastructure.v
VERILOG_SOURCES += $(TOPDIR)/hdl/memc_wrapper.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/mcb_ui_top.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/mcb_raw_wrapper.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/mcb_soft_calibration_top.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/mcb_soft_calibration.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/iodrp_controller.v
VERILOG_SOURCES += $(TOPDIR)/hdl/mcb_controller/iodrp_mcb_controller.v

# DDR3 fifo wrapper files
VERILOG_SOURCES += $(TOPDIR)/hdl/cross_clock_data.v
VERILOG_SOURCES += $(TOPDIR)/hdl/cross_clock_enable.v
VERILOG_SOURCES += $(TOPDIR)/hdl/ddr3_fifo.v
VERILOG_SOURCES += $(TOPDIR)/hdl/ddr3_fifo_wrapper.v

# Other Autogen Cores
VERILOG_SOURCES += $(TOPDIR)/cores/daq2udp_fifo.v
VERILOG_SOURCES += $(TOPDIR)/cores/daq_bram.v
VERILOG_SOURCES += $(TOPDIR)/cores/sata2udp_fifo.v
VERILOG_SOURCES += $(TOPDIR)/cores/sata_read_fifo.v

TOPLEVEL=main_wiredleaf

GPI_IMPL := vpi

export TOPLEVEL_LANG
MODULE=test_main_wiredleaf

include $(COCOTB)/makefiles/Makefile.inc
include $(COCOTB)/makefiles/Makefile.sim
