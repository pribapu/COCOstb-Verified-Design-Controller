# Classic cocotb Makefile -- runs the SPI master regression under Icarus Verilog.
#
#   make            # run with Icarus (SIM=icarus)
#   make SIM=verilator
#   make waves      # open the VCD in GTKWave after a run
#   make apb        # run the APB wrapper regression instead
#
# Coverage is aggregated into spi_cov.json across widths when using runner.py;
# this single-width Makefile flow is the quickest way to reproduce a pass.

TOPLEVEL_LANG ?= verilog
SIM           ?= icarus
DATA_WIDTH    ?= 8

PWD    := $(shell pwd)
RTLDIR := $(PWD)/../rtl

ifndef APB
VERILOG_SOURCES := $(RTLDIR)/spi_master.sv
TOPLEVEL        := spi_master
MODULE          := test_spi_master

# pass the parameter through to the chosen simulator
ifeq ($(SIM),icarus)
    COMPILE_ARGS += -Pspi_master.DATA_WIDTH=$(DATA_WIDTH) -g2012
    PLUSARGS     += -fst
endif
ifeq ($(SIM),verilator)
    EXTRA_ARGS   += --trace -GDATA_WIDTH=$(DATA_WIDTH)
endif
else
VERILOG_SOURCES := $(RTLDIR)/spi_master.sv $(RTLDIR)/spi_apb_wrapper.sv
TOPLEVEL        := spi_apb_wrapper
MODULE          := test_spi_apb

ifeq ($(SIM),icarus)
    COMPILE_ARGS += -g2012
    PLUSARGS     += -fst
endif
ifeq ($(SIM),verilator)
    EXTRA_ARGS   += --trace
endif
endif

export SPI_COV_FILE := $(PWD)/spi_cov.json

include $(shell cocotb-config --makefiles)/Makefile.sim

apb:
	$(MAKE) APB=1

waves:
	gtkwave dump.vcd &
