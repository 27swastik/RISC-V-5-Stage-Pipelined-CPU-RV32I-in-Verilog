// 5-stage RV32I pipelined processor (simulation only)

`timescale 1ns / 1ps

module riscv_core (
    input logic clk,
    input logic rst
);
      // --------------------------
    // Initialization for Simulation
    // --------------------------
    initial begin
        integer i;
        for (i = 0; i < 32; i = i + 1)
            regfile[i] = 0;

        for (i = 0; i < MEM_SIZE; i = i + 1)
            memory[i] = 32'h00000013; // NOP
    end


  
// Constants
localparam XLEN = 32;
localparam REG_ADDR_W = 5;
localparam MEM_SIZE = 1024;

// =====================
// Instruction & Data Memory (Shared)
// =====================
logic [XLEN-1:0] memory [0:MEM_SIZE-1];
logic [XLEN-1:0] instr;

// =====================
// Register File
// =====================
logic [XLEN-1:0] regfile [0:31];
logic [XLEN-1:0] rs1_val, rs2_val;
logic [REG_ADDR_W-1:0] rs1, rs2, rd;

// =====================
// Pipeline Registers
// =====================
typedef struct packed {
    logic [XLEN-1:0] pc;
} IF_ID_t;

typedef struct packed {
    logic [XLEN-1:0] pc;
    logic [XLEN-1:0] instr;
    logic [XLEN-1:0] rs1_val, rs2_val;
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
} ID_EX_t;

typedef struct packed {
    logic [XLEN-1:0] alu_result;
    logic [XLEN-1:0] rs2_val;
    logic [4:0] rd;
    logic mem_read, mem_write, wb_en, mem_to_reg;
} EX_MEM_t;

typedef struct packed {
    logic [XLEN-1:0] mem_out;
    logic [XLEN-1:0] alu_result;
    logic [4:0] rd;
    logic wb_en, mem_to_reg;
} MEM_WB_t;

IF_ID_t IF_ID;
ID_EX_t ID_EX;
EX_MEM_t EX_MEM;
MEM_WB_t MEM_WB;

// =====================
// Program Counter
// =====================
logic [XLEN-1:0] pc, pc_next;
assign pc_next = pc + 4;

// =====================
// Fetch Stage
// =====================
always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        pc <= 0;
    end else begin
        pc <= pc_next;
        IF_ID.pc <= pc;
    end
end

assign instr = memory[pc >> 2];

// =====================
// Decode Stage
// =====================
logic [6:0] opcode;
logic [2:0] funct3;
logic [6:0] funct7;

always_ff @(posedge clk) begin
    IF_ID.pc <= pc;
    ID_EX.pc <= IF_ID.pc;
    ID_EX.instr <= instr;
    
    ID_EX.opcode <= instr[6:0];
    ID_EX.rd <= instr[11:7];
    ID_EX.funct3 <= instr[14:12];
    ID_EX.rs1 <= instr[19:15];
    ID_EX.rs2 <= instr[24:20];
    ID_EX.funct7 <= instr[31:25];
    
    rs1 = instr[19:15];
    rs2 = instr[24:20];
    rs1_val = (rs1 != 0) ? regfile[rs1] : 0;
    rs2_val = (rs2 != 0) ? regfile[rs2] : 0;
    
    ID_EX.rs1_val <= rs1_val;
    ID_EX.rs2_val <= rs2_val;
    
    // Immediate decoding (I-type only for now)
    ID_EX.imm <= {{20{instr[31]}}, instr[31:20]};
end

// =====================
// Forwarding Logic
// =====================
logic [XLEN-1:0] fwd_rs1, fwd_rs2;

always @(*) begin
    fwd_rs1 = ID_EX.rs1_val;
    fwd_rs2 = ID_EX.rs2_val;

    // From EX_MEM
    if (EX_MEM.wb_en && EX_MEM.rd != 0) begin
        if (EX_MEM.rd == ID_EX.rs1)
            fwd_rs1 = EX_MEM.alu_result;
        if (EX_MEM.rd == ID_EX.rs2)
            fwd_rs2 = EX_MEM.alu_result;
    end

    // From MEM_WB
    if (MEM_WB.wb_en && MEM_WB.rd != 0) begin
        if (MEM_WB.rd == ID_EX.rs1)
            fwd_rs1 = MEM_WB.mem_to_reg ? MEM_WB.mem_out : MEM_WB.alu_result;
        if (MEM_WB.rd == ID_EX.rs2)
            fwd_rs2 = MEM_WB.mem_to_reg ? MEM_WB.mem_out : MEM_WB.alu_result;
    end
end

// =====================
// Execute Stage
// =====================
logic [XLEN-1:0] alu_result;

always @(*) begin
    case (ID_EX.opcode)
        7'b0010011: begin // ADDI
            alu_result = fwd_rs1 + ID_EX.imm;
        end
        7'b0110011: begin // ADD, SUB
            case ({ID_EX.funct7, ID_EX.funct3})
                10'b0000000000: alu_result = fwd_rs1 + fwd_rs2; // ADD
                10'b0100000000: alu_result = fwd_rs1 - fwd_rs2; // SUB
                default: alu_result = 0;
            endcase
        end
        default: alu_result = 0;
    endcase
end

always_ff @(posedge clk) begin
    EX_MEM.alu_result <= alu_result;
    EX_MEM.rs2_val <= ID_EX.rs2_val;
    EX_MEM.rd <= ID_EX.rd;
    EX_MEM.wb_en <= 1;
    EX_MEM.mem_read <= 0;
    EX_MEM.mem_write <= 0;
    EX_MEM.mem_to_reg <= 0;
end

// =====================
// Memory Stage
// =====================
always_ff @(posedge clk) begin
    MEM_WB.alu_result <= EX_MEM.alu_result;
    MEM_WB.rd <= EX_MEM.rd;
    MEM_WB.wb_en <= EX_MEM.wb_en;
    MEM_WB.mem_to_reg <= EX_MEM.mem_to_reg;
    MEM_WB.mem_out <= 0; // no memory op yet
end

// =====================
// Writeback Stage
// =====================
always_ff @(posedge clk) begin
    if (MEM_WB.wb_en && MEM_WB.rd != 0) begin
        regfile[MEM_WB.rd] <= MEM_WB.mem_to_reg ? MEM_WB.mem_out : MEM_WB.alu_result;
    end
end

endmodule
