# RISC-V 5-Stage Pipelined CPU (RV32I) in Verilog

This project implements a 32-bit RISC-V RV32I CPU core from scratch in **Verilog**, featuring a fully functional **5-stage pipeline** architecture. It includes a **SystemVerilog testbench** with **random instruction generation** and a **self-checking scoreboard** for functional verification.

---

##  Features

- **5-stage pipeline:** IF, ID, EX, MEM, WB
- **Supported instructions:**
  - `ADDI` (I-type)
  - `ADD`, `SUB` (R-type)
- **Hazard resolution:**
  - Implements **data forwarding** for RAW hazards
- **Scoreboard-based verification**
- **Random instruction generation**
- **Self-contained simulation environment**
- **Shared memory (Von Neumann architecture)**

---

##  Pipeline Diagram
IF ──► ID ──► EX ──► MEM ──► WB


| Stage | Description                         |
|-------|-------------------------------------|
| IF    | Instruction fetch from memory       |
| ID    | Decode + Register read              |
| EX    | ALU operation, forwarding applied   |
| MEM   | Placeholder for load/store support  |
| WB    | Write result back to register file  |

---

##  Project Structure
riscv-pipelined-cpu/
- ├── core.sv # RISC-V pipelined CPU (Verilog)
- ├── tb_core.sv # SystemVerilog testbench

##  Testbench Highlights

- Generates 10 random `ADD`, `SUB`, or `ADDI` instructions
- Preloads them into instruction memory
- Applies a **shadow model** to track expected register values
- At the end, compares DUT register file with the scoreboard

## This project can be extended to include:
- [ ] `LW`, `SW` (load/store)
- [ ] `BEQ`, `JAL`, `BNE` (branching)
- [ ] Stall logic for load-use hazards
- [ ] Instruction/data memory separation (Harvard arch)
- [ ] Support for more RV32I instructions
- [ ] Integration with real FPGA (e.g., Arty A7)


