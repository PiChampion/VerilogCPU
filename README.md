# VerilogCPU

This project was completed during the ECE 310 course at NC State. The project involved implementing a simple CPU from scratch using Verilog. Characteristics of the CPU include separate control and data path with multiple modules, utilizing a pipelined multiplier to handle multiplication opcodes, and a synchronous reset. Assumptions made to simplify design include there being no off-chip memory, the program consists of valid instructions only, and no overflow detection is required. 

The testbench used to test the implementation is included in the proj1.v file. 

test.txt and memory.txt are memories which can be used to make sure that outputs match expected values.
