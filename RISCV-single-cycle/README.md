Demo Video  
[Watch the project demo here](https://drive.google.com/file/d/1g8GcoGoaPczfH6TVS4Lg4pZ73KVb_Kzs/view?usp=drive_link) 

RV32I Single‑Cycle CPU (Basys‑3)
A compact RV32I single‑cycle processor with self‑checking tests and an FPGA demo on Basys‑3. Focuses on clear datapath/control, ISA correctness, and quick bring‑up.

Overview
Implements the RV32I base ISA in a single cycle per instruction for simplicity and clarity.

Clean separation of datapath (ALU, regfile, imem/dmem, PC) and control (decoder, PC select).

Hardware demo: Fibonacci sequence on Basys‑3 7‑segment/LEDs.

Repository layout
src/: RTL (alu, regfile, control, pc, imem, dmem, top_single_cycle).

Features
ALU ops: add, sub, and, or, xor, slt/u, sll, srl, sra; byte loads/stores.

Branch/jump support with PC+4 vs target mux.

Self‑checking TBs compare register/memory state against expected.

Quick start — Simulation
Prereqs: <Icarus/Verilator/Questa>, <Make>.

Example (Icarus):

iverilog -g2012 -o build/alu_tb tb/alu_tb.sv src/alu.sv

vvp build/alu_tb # expect “TEST PASS”

make test # run regression, logs in logs/*.log

Quick start — FPGA (Basys‑3)
Tools: Vivado <20XX.X>, Board: Basys‑3, Top: <top_single_cycle.sv>, XDC: fpga/basys3.xdc.
Design notes
Single‑cycle timing keeps control simple at the cost of fmax; pipeline variant is in the other repo.

Instruction memory initialized from sw/asm/*.mem for quick demos.

Results
All unit/ISA tests pass on commit <hash>; baseline timing met on Basys‑3.

Hardware demo verified on LEDs/7‑seg.

Roadmap
Add CSR/interrupt stubs; formal checks for ALU/branches.

License
<MIT/BSD/Apache‑2.0>

Maintainer
Aakarsh Singh — contact in profile.

vivado -mode tcl -source fpga/build.tcl # generate bitstream

Program board; 7‑seg shows Fibonacci; LEDs map in XDC.
