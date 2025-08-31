`timescale 1ns/1ps

module tb_core;

    logic clk = 0;
    logic rst = 1;

    // Instantiate DUT
    riscv_core uut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Scoreboard
    logic [31:0] expected_regs[0:31];

    // Instruction type enum (instead of string)
    typedef enum logic [1:0] {
        ITYPE_ADDI = 2'b00,
        ITYPE_ADD  = 2'b01,
        ITYPE_SUB  = 2'b10,
        ITYPE_NOP  = 2'b11
    } instr_type_t;

    instr_type_t instr_type;
    int rd, rs1, rs2, imm;

    // Generate random instruction
    function [31:0] gen_random_instr(input int index);
        int opcode_sel;
        reg [31:0] instr;
        begin
            rd  = $urandom_range(1, 31);
            rs1 = $urandom_range(1, 31);
            rs2 = $urandom_range(1, 31);
            imm = $urandom_range(0, 15);
            opcode_sel = $urandom_range(0, 2); // 0 = ADDI, 1 = ADD, 2 = SUB

            case (opcode_sel)
                0: begin // ADDI
                    instr = {imm[11:0], rs1[4:0], 3'b000, rd[4:0], 7'b0010011};
                    instr_type = ITYPE_ADDI;
                    $display("[%0d] ADDI x%0d, x%0d, %0d -> 0x%08h", index, rd, rs1, imm, instr);
                end
                1: begin // ADD
                    instr = {7'b0000000, rs2[4:0], rs1[4:0], 3'b000, rd[4:0], 7'b0110011};
                    instr_type = ITYPE_ADD;
                    $display("[%0d] ADD  x%0d, x%0d, x%0d -> 0x%08h", index, rd, rs1, rs2, instr);
                end
                2: begin // SUB
                    instr = {7'b0100000, rs2[4:0], rs1[4:0], 3'b000, rd[4:0], 7'b0110011};
                    instr_type = ITYPE_SUB;
                    $display("[%0d] SUB  x%0d, x%0d, x%0d -> 0x%08h", index, rd, rs1, rs2, instr);
                end
                default: begin
                    instr = 32'h00000013; // NOP
                    instr_type = ITYPE_NOP;
                end
            endcase
            return instr;
        end
    endfunction

    // Apply instruction to scoreboard
    task apply_instruction();
        case (instr_type)
            ITYPE_ADDI: expected_regs[rd] = expected_regs[rs1] + imm;
            ITYPE_ADD:  expected_regs[rd] = expected_regs[rs1] + expected_regs[rs2];
            ITYPE_SUB:  expected_regs[rd] = expected_regs[rs1] - expected_regs[rs2];
            default: ; // do nothing
        endcase
    endtask

    initial begin
        $display("=== RISC-V Pipelined Core Testbench (with Scoreboard) ===");

        // Init scoreboard
        foreach (expected_regs[i])
            expected_regs[i] = 0;

        // Generate and load instructions
        for (int i = 0; i < 10; i++) begin
            uut.memory[i] = gen_random_instr(i);
            apply_instruction();
        end

        // Reset sequence
        repeat (2) @(posedge clk);
        rst <= 0;

        // Run for some cycles
        repeat (30) @(posedge clk);

        // Check register file
        $display("=== Register File Check ===");
        for (int i = 1; i < 32; i++) begin
            if (uut.regfile[i] !== expected_regs[i]) begin
                $display(" MISMATCH: x%0d => DUT = %0d, expected = %0d", i, uut.regfile[i], expected_regs[i]);
            end else begin
                $display(" x%0d OK: %0d", i, uut.regfile[i]);
            end
        end

        $display(" Scoreboard Check Complete.");
        $finish;
    end

endmodule
