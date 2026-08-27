# test/Makefile
SIM          ?= icarus
TOPLEVEL_LANG ?= verilog

# All RTL sources (adjust path if src/ is named differently)
VERILOG_SOURCES += $(PWD)/../src/Memory.sv
VERILOG_SOURCES += $(PWD)/../src/mac_unit.sv
VERILOG_SOURCES += $(PWD)/../src/convergence_compare.sv
VERILOG_SOURCES += $(PWD)/../src/parallel_matvec_core.sv
VERILOG_SOURCES += $(PWD)/../src/matmul_top.sv
VERILOG_SOURCES += $(PWD)/../src/markov_core_tt.v
VERILOG_SOURCES += $(PWD)/../src/tt_um_OwenAReich_markov_chain.v

TOPLEVEL = tt_um_OwenAReich_markov_chain
MODULE   = test

# Icarus needs SystemVerilog flag for .sv files
COMPILE_ARGS += -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
