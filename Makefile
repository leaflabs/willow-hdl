#
# Makefile
#
# Copyright: (C) 2015 LeafLabs, LLC
# License: MIT License (See LICENSE file)
# Author: Jess Barber (jessb@leaflabs.com)
#
# This module contains a top level test bench to drive all tests and
# simulations for the high-level Willow system.
#

# Edit project-specific variables in this file.

project := willow-hdl
top_module := main_wiredleaf
vendor := xilinx

# list all .v files explicitly with verilog_files (no hdl/*.v business)

EXTRA_ARGS := -y $(XILINX)/verilog/src/unisims -y $(XILINX)/verilog/src/simprims -y $(XILINX)/verilog/src/XilinxCoreLib
### List all Verilog (.v) files for this project explicitly below.
### Leave blank if you only have VHDL files.
### Use one file per line; do not use wildcards (eg, hdl/*.v).
verilog_files := $(XILINX)/verilog/src/glbl.v
verilog_files += hdl/main_wiredleaf.v
verilog_files += hdl/clk_wiz_50_44mhz.v
verilog_files += hdl/clk_wiz_50_125mhz.v
verilog_files += hdl/rhd2000_driver.v
verilog_files += hdl/rhd2000_spi.v
verilog_files += hdl/sngdaq.v
verilog_files += hdl/central_core.v
verilog_files += hdl/cfg_master.v
verilog_files += hdl/spi_slave.v
verilog_files += hdl/debounce.v
verilog_files += hdl/board_identifier.v
verilog_files += hdl/gpio_core.v
verilog_files += hdl/sine_table.v
verilog_files += hdl/sata_core.v
verilog_files += hdl/udp_core.v
verilog_files += hdl/daq_core.v
verilog_files += hdl/sngdaq_dummy.v
verilog_files += hdl/daq_interfaces.v
verilog_files += hdl/udp.v
verilog_files += hdl/crc.v
verilog_files += hdl/sata_top.v
verilog_files += hdl/sata_io.v
verilog_files += hdl/sata_control.v

# DDR3 Autogen Core
verilog_files += hdl/infrastructure.v
verilog_files += hdl/memc_wrapper.v
verilog_files += hdl/mcb_controller/mcb_ui_top.v
verilog_files += hdl/mcb_controller/mcb_raw_wrapper.v
verilog_files += hdl/mcb_controller/mcb_soft_calibration_top.v
verilog_files += hdl/mcb_controller/mcb_soft_calibration.v
verilog_files += hdl/mcb_controller/iodrp_controller.v
verilog_files += hdl/mcb_controller/iodrp_mcb_controller.v

# DDR3 fifo wrapper files
verilog_files += hdl/cross_clock_data.v
verilog_files += hdl/cross_clock_enable.v
verilog_files += hdl/ddr3_fifo.v
verilog_files += hdl/ddr3_fifo_wrapper.v

# Other Autogen Cores
verilog_files += cores/daq2udp_fifo.v
verilog_files += cores/daq_bram.v
verilog_files += cores/sata2udp_fifo.v
verilog_files += cores/sata_read_fifo.v

# Testbench files
tbfiles := tb/top_level_tb.v
tbfiles += tb/spi_master.v

# what gets run by "make tests"
alltests := test/top_level_tb

# Pass verilog parameters to the top-level module
vgenerics += "INTERFACE_BOARD_VERSION=2"
vgenerics += "NUM_CHIPS=32"
vgenerics += "DUMMY_DAQ=0"
vgenerics += "FORCE_ACQUISITION=1"
vgenerics += "MOSI_ON_FALLING=1"
vgenerics += "MOSI_ON_NEXT_RISING=1" # overrides MOSI_ON_FALLING
vgenerics += "SIGNED_ADC_VALUES=0"
# vgeneric usage which passes the git commit and current unix time as
# parameters. See page 339 of the "Xilinx XST User Guide" for details
gitcommit = $(shell (git log --abbrev=8 --oneline -n 1 2> /dev/null || echo "00000000") | head -c 8)
build_unixtime = $(shell date +%s || echo "0")
vgenerics += "GIT_COMMIT=h$(gitcommit)"
vgenerics += "BUILD_UNIX_TIME=d$(build_unixtime)"

all: sim

sim: $(tbfiles) $(verilog_files)
	iverilog $(EXTRA_ARGS) -v -o compiler.out $(tbfiles) $(verilog_files)
	vvp -v compiler.out

clean: 
	rm compiler.out
