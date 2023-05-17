# Processor Architecture Course Project
# Y86-64 Instruction Set Architecture
- Tanish Taneja
- 2021112011
---
- We have developed a processor architecture design based on the Y86 ISA using Verilog.
- Processor Frequency = 1 GHz 
- Instruction Memory = 1 Kb
- Data Memory = 16 Kb
```
project-tanish
│  ├── ALU
│  ├── PIPE
│  ├── SEQ
│  ├── ALUReport.pdf
│  ├── ProjectReport.pdf
│  └── README.md
```
---
## Instructions Implemented
```
1) halt
2) nop
3) cmovXX
4) irmovq
5) rmmovq
6) mrmovq
7) OPq
8) jXX
9) call
10) ret
11) pushq
12) popq
```
---
## ALU
1) The ALU is implemented to carry out all the required Arithmetic and Logic operations during the execute stage of the processor.
2) The file `ALU/ALU-wrapper/alu.v` contains the code for the final ALU and all the other operations are coded as modules in their respective folders.
```
 ALU
│  ├── ADD
│  │  ├── add
│  │  ├── add-1.v
│  │  ├── add-64.v
│  │  ├── add-dump.vcd
│  │  └── add-testbench.v
│  ├── ALU-wrapper
│  │  ├── alu
│  │  ├── alu-dump.vcd
│  │  ├── alu-testbench.v
│  │  ├── alu.v
│  │  └── helper.v
│  ├── AND
│  │  ├── and
│  │  ├── and-64.v
│  │  ├── and-dump.vcd
│  │  └── and-testbench.v
│  ├── SUB
│  │  ├── helper.v
│  │  ├── not-64.v
│  │  ├── sub
│  │  ├── sub-64.v
│  │  ├── sub-dump.vcd
│  │  └── sub-testbench.v
│  └── XOR
│     ├── xor
│     ├── xor-64.v
│     ├── xor-dump.vcd
│     └── xor-testbench.v
```
---
## SEQ Implementation
1) The 6 stages (fetch, decode, execute, memory, writeback, PC Update) are separately written as modules in their respective directories.
2) The file `SEQ/processor.v` contains the code for the final SEQ Processor.
3) Run `iverilog -o processor processor.v` followed by `./processor` to get the processor output for the instructions specified in `processor.v`.
4) The command `gtkwave processor.vcd` can be executed to obtain the gtk wave plot for our processor. 
```
SEQ
│  ├── 1-fetch
│  │  ├── 1-fetch
│  │  ├── 1-fetch-gtkwave.png
│  │  ├── 1-fetch.v
│  │  ├── 1-fetch.vcd
│  │  └── 1-fetch_tb.v
│  ├── 2-decode
│  │  ├── 2-decode
│  │  ├── 2-decode-gtkwave.png
│  │  ├── 2-decode.v
│  │  ├── 2-decode.vcd
│  │  └── 2-decode_tb.v
│  ├── 3-execute
│  │  ├── 3-execute
│  │  ├── 3-execute-gtkwave.png
│  │  ├── 3-execute.v
│  │  ├── 3-execute.vcd
│  │  ├── 3-execute_tb.v
│  │  └── alu.v
│  ├── 4-memory
│  │  ├── 4-memory
│  │  ├── 4-memory-gtkwave.png
│  │  ├── 4-memory.v
│  │  ├── 4-memory.vcd
│  │  └── 4-memory_tb.v
│  ├── 5-writeback
│  │  ├── 5-writeback
│  │  ├── 5-writeback-gtkwave.png
│  │  ├── 5-writeback.v
│  │  ├── 5-writeback.vcd
│  │  └── 5-writeback_tb.v
│  ├── 6-PCupdate
│  │  ├── 6-PCupdate
│  │  ├── 6-PCupdate-gtkwave.png
│  │  ├── 6-PCupdate.v
│  │  ├── 6-PCupdate.vcd
│  │  └── 6-PCupdate_tb.v
│  ├── processor
│  ├── processor-gtkwave.png
│  ├── processor.v
│  └── processor.vcd
```
---
## PIPE Implementation
1) All the required pipelining registers are in `pipelineRegisters.v`.
2) The file `PIPE/processor.v` contains the code for the final PIPE Processor.
3) Run `iverilog -o processor processor.v` followed by `./processor` to get the processor output for the instructions specified in `processor.v`.
4) The command `gtkwave processor.vcd` can be executed to obtain the gtk wave plot for our processor. 