
module top(
    input clk, reset
);
    wire [31:0] PC, Instr, ReadData;
    wire MemWrite;
    wire [31:0] ALUResult, WriteData;
    // Processor core
    riscv riscvcore(clk, reset, PC, Instr, MemWrite, ALUResult, WriteData, ReadData);
    
    // Instruction memory
    imem imemory(PC, Instr);
    
    // Data memory
    dmem dmemory(clk, MemWrite, ALUResult, WriteData, ReadData);
    
   
    
endmodule
// riscv.v - Main RISC-V processor
module riscv(
    input clk, reset,
    output [31:0] PC,
    input [31:0] Instr,
    output MemWrite,
    output [31:0] ALUResult, WriteData,
    input [31:0] ReadData
);
    wire ALUSrc, RegWrite, Jump, Zero, PCSrc, lt, ltu;
    wire carry, overflow;  
    wire [1:0] ResultSrc, ImmSrc;
    wire [3:0] ALUControl;
    
    
    controller c(Instr[6:0], Instr[14:12], Instr[30], Zero, lt, ltu, carry, overflow,
                ResultSrc, MemWrite, PCSrc, ALUSrc, RegWrite, Jump, ImmSrc, ALUControl);
    
    
    datapath dp(clk, reset, ResultSrc, PCSrc, ALUSrc, RegWrite, ImmSrc, ALUControl,
               Zero, lt, ltu, carry, overflow, PC, Instr, ALUResult, WriteData, ReadData);
endmodule

// controller.v - Control unit
module controller(
    input [6:0] op,
    input [2:0] funct3,
    input funct7b5,
    input Zero, lt, ltu,
    input carry, overflow,     
    output [1:0] ResultSrc,
    output MemWrite,
    output PCSrc, ALUSrc, RegWrite, Jump,
    output [1:0] ImmSrc,
    output [3:0] ALUControl
);
    wire [1:0] ALUOp;
    wire Branch;
    
    maindec md(op, ResultSrc, MemWrite, Branch, ALUSrc, RegWrite, Jump, ImmSrc, ALUOp);
    aludec ad(op[5], funct3, funct7b5, ALUOp, ALUControl);
    
    // Branch logic for all branch types including potential flag-based branches
    wire BranchTaken;
    assign BranchTaken = (funct3 == 3'b000) ? Zero     :  // beq
                        (funct3 == 3'b001) ? ~Zero    :  // bne
                        (funct3 == 3'b100) ? lt       :  // blt
                        (funct3 == 3'b101) ? ~lt      :  // bge
                        (funct3 == 3'b110) ? ltu      :  // bltu
                        (funct3 == 3'b111) ? ~ltu     :  // bgeu
                         1'b0;
    
    assign PCSrc = (Branch & BranchTaken) | Jump;
endmodule

// maindec.v - Main decoder

module maindec(
    input [6:0] op,
    output reg [1:0] ResultSrc,
    output reg MemWrite,
    output reg Branch, ALUSrc, RegWrite, Jump,
    output reg [1:0] ImmSrc, ALUOp
);
    always @* begin
        case(op)
            // R-type
            7'b0110011: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_00_0_0_00_0_10_0;
            // I-type (immediate)
            7'b0010011: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_00_1_0_00_0_10_0;
            // Load
            7'b0000011: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_00_1_0_01_0_00_0;
            // Store
            7'b0100011: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b0_01_1_1_00_0_00_0;
            // B-type
            7'b1100011: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b0_10_0_0_00_1_01_0;
            // LUI (U-type) - Load Upper Immediate
            7'b0110111: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_11_1_0_00_0_11_0;
            // AUIPC (U-type) - Add Upper Immediate to PC  
            7'b0010111: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_11_1_0_00_0_11_0;
            // JAL (J-type) - Jump and Link - FIXED ImmSrc
            7'b1101111: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_11_0_0_10_0_00_1;
            // JALR (I-type) - Jump and Link Register
            7'b1100111: {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b1_00_1_0_10_0_00_1;
            default:    {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = 11'b0_00_0_0_00_0_00_0;
        endcase
    end
endmodule


module aludec(
    input opb5,
    input [2:0] funct3,
    input funct7b5,
    input [1:0] ALUOp,
    output reg [3:0] ALUControl
);
    wire RtypeSub;
    assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract
    
    always @* begin
        case(ALUOp)
            2'b00: ALUControl = 4'b0000; // addition (loads/stores)
            // Add SLTU support
            2'b01: begin
            case(funct3)
            3'b110: ALUControl = 4'b0111; // bltu - use sltu
            3'b111: ALUControl = 4'b0111; // bgeu - use sltu  
            default: ALUControl = 4'b0001; // subtraction for other branches
            endcase
            end
            2'b11: ALUControl = 4'b0000; // addition (LUI/AUIPC)
            default: case(funct3) // R-type or I-type ALU
                3'b000: ALUControl = RtypeSub ? 4'b0001 : 4'b0000; // sub or add
                3'b001: ALUControl = 4'b0101; // sll
                3'b010: ALUControl = 4'b0110; // slt
                3'b011: ALUControl = 4'b0111; // sltu
                3'b100: ALUControl = 4'b0100; // xor
                3'b101: 
                        ALUControl = funct7b5 ? 4'b1000 : 4'b1001; // sra or srl
                3'b110: ALUControl = 4'b0011; // or
                3'b111: ALUControl = 4'b0010; // and
                default: ALUControl = 4'bxxxx;
            endcase
        endcase
    end
endmodule

// datapath.v - Datapath

module datapath(
    input clk, reset,
    input [1:0] ResultSrc,
    input PCSrc, ALUSrc,
    input RegWrite,
    input [1:0] ImmSrc,
    input [3:0] ALUControl,
    output Zero, lt, ltu,
    output carry, overflow,    
    output [31:0] PC,
    input [31:0] Instr,
    output [31:0] ALUResult, WriteData,
    input [31:0] ReadData
);
    wire [31:0] PCNext, PCPlus4, PCTarget;
    wire [31:0] ImmExt;
    wire [31:0] SrcA, SrcB;
    wire [31:0] Result;
    
    // PC logic
    flopr #(32) pcreg(clk, reset, PCNext, PC);
    adder pcadd4(PC, 32'd4, PCPlus4);
    adder pcaddbranch(PC, ImmExt, PCTarget);
    
    // PC selection: JALR uses ALUResult, others use PCTarget
    wire isJALR;
    assign isJALR = (Instr[6:0] == 7'b1100111);
    wire [31:0] JumpTarget;
    assign JumpTarget = isJALR ? {ALUResult[31:1], 1'b0} : PCTarget;
    mux2 #(32) pcmux(PCPlus4, JumpTarget, PCSrc, PCNext);
    
    // Register file
    regfile rf(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, SrcA, WriteData);
    
    // Immediate extension
    extend ext(Instr[31:0], ImmSrc, ImmExt);
    
    // ALU source selection
    wire [31:0] SrcAFinal;
    wire isAUIPC, isLUI;
    assign isAUIPC = (Instr[6:0] == 7'b0010111);
    assign isLUI = (Instr[6:0] == 7'b0110111);
    
    assign SrcAFinal = isAUIPC ? PC :      // AUIPC uses PC
                       isLUI ? 32'b0 :     // LUI uses 0
                       SrcA;               // Normal operation
    
    mux2 #(32) srcbmux(WriteData, ImmExt, ALUSrc, SrcB);
    
    // ALU with carry and overflow flags
    alu alu(SrcAFinal, SrcB, ALUControl, ALUResult, Zero, lt, ltu, carry, overflow);
    
    // Result selection
    mux4 #(32) resultmux(ALUResult, ReadData, PCPlus4, ALUResult, ResultSrc, Result);
endmodule




// alu.v - Arithmetic Logic Unit
module alu(
    input [31:0] a, b,
    input [3:0] alucontrol,
    output reg [31:0] result,
    output zero,
    output lt, ltu,
    output reg carry,     
    output reg overflow 
);
    wire [31:0] sum, diff;
    wire cout_add, cout_sub;
    wire ovf_add, ovf_sub;
    
    // Addition with carry detection
    assign {cout_add, sum} = {1'b0, a} + {1'b0, b};
    
    // Subtraction with carry detection
    assign {cout_sub, diff} = {1'b0, a} - {1'b0, b};
    
    assign zero = (result == 32'd0);
    assign lt = (a[31] != b[31]) ? a[31] : diff[31]; // signed comparison
    assign ltu = (a < b); // unsigned comparison
    
    // Overflow detection for addition
    assign ovf_add = (a[31] == b[31]) && (sum[31] != a[31]);
    
    // Overflow detection for subtraction  
    assign ovf_sub = (a[31] != b[31]) && (diff[31] != a[31]);
    
    always @* begin
        carry = 1'b0;
        overflow = 1'b0;
        case(alucontrol)
            4'b0000: begin // add
                result = sum;
                carry = cout_add;
                overflow = ovf_add;
            end
            4'b0001: begin // subtract
                result = diff;
                carry = ~cout_sub; // borrow flag (inverted carry)
                overflow = ovf_sub;
            end
            4'b0010: result = a & b;                  // and
            4'b0011: result = a | b;                  // or
            4'b0100: result = a ^ b;                  // xor
            4'b0101: result = a << b[4:0];            // sll
            4'b0110: result = {31'b0, lt};            // slt
            4'b0111: result = {31'b0, ltu};           // sltu
            4'b1000: result = a >>> b[4:0];           // sra
            4'b1001: result = a >> b[4:0];            // srl
            default: result = 32'bx;
        endcase
    end
endmodule


module regfile(
    input clk,
    input we3,
    input [4:0] a1, a2, a3,
    input [31:0] wd3,
    output [31:0] rd1, rd2
);
    reg [31:0] rf[31:0];
    integer i;
    
    // Initialize ALL registers to zero
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            rf[i] = 32'b0;
        end
    end
    
    // Write port
    always @(posedge clk) begin
        if (we3 && a3 != 5'b0) begin
            rf[a3] <= wd3;
            $display("Writing %h to register x%d at time %t", wd3, a3, $time);
        end
    end
    
    // Read ports
    assign rd1 = (a1 != 5'b0) ? rf[a1] : 32'b0;
    assign rd2 = (a2 != 5'b0) ? rf[a2] : 32'b0;
endmodule


module extend(
    input [31:0] instr,
    input [1:0] immsrc,
    output reg [31:0] immext
);
    always @* begin
        case(immsrc)
            // I-type (ADDI, JALR, loads)
            2'b00: immext = {{20{instr[31]}}, instr[31:20]};
            // S-type (stores)
            2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            // B-type (branches)
            2'b10: immext = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            // U-type and J-type
            2'b11: begin
                // Check if it's JAL (J-type) by looking at the opcode
                if (instr[6:0] == 7'b1101111) begin // JAL
                    // J-type immediate: [20|10:1|11|19:12] with bit 0 = 0
                    immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
                end else begin // LUI, AUIPC (U-type)
                    immext = {instr[31:12], 12'b0};
                end
            end
            default: immext = 32'bx;
        endcase
    end
endmodule


// imem.v - Instruction memory

module imem(
    input [31:0] a,
    output [31:0] rd
);
    reg [31:0] RAM[250:0];
    
    initial begin
        // Corrected instructions with proper register encoding
       /* RAM[0] = 32'h00500213;  // addi x4, x0, 5    (correct encoding)
        RAM[1] = 32'h00300313;  // addi x6, x0, 3    (correct encoding)
        RAM[2] = 32'h00700413;  // addi x8, x0, 7    (correct encoding)
        RAM[3] = 32'h00100513;  // addi x10, x0, 1   (correct encoding)
        RAM[4] = 32'h0ff00613;  // addi x12, x0, 255 (correct encoding)
        RAM[5] = 32'h00000713;  // addi x14, x0, 0   (correct encoding)
        
        // Branch tests with corrected targets
        RAM[6] = 32'h00a30463;  // beq x6, x10, 8    (branch if x6 == x10)
        RAM[7] = 32'h00000013;  // nop
        RAM[8] = 32'h00000013;  // nop
        RAM[9] = 32'h00650463;  // beq x10, x6, 8    (branch if x10 == x6)
        RAM[10] = 32'h00000013; // nop
        RAM[11] = 32'h00000013; // nop
        RAM[12] = 32'h00641463; // bne x8, x6, 8     (branch if x8 != x6)
        RAM[13] = 32'h00000013; // nop (should be skipped)
        RAM[14] = 32'h00000013; // nop (should be skipped)
        RAM[15] = 32'h00651463; // bne x10, x6, 8    (branch if x10 != x6)
        RAM[16] = 32'h00000013; // nop (should be skipped)
        RAM[17] = 32'h00000013; // nop (should be skipped)
        RAM[18] = 32'h00e64463; // blt x12, x14, 8   (255 < 0 signed - false)
        RAM[19] = 32'h00000013; // nop
        RAM[20] = 32'h00000013; // nop
        RAM[21] = 32'h00e66463; // bltu x12, x14, 8  (255 < 0 unsigned - false)
        RAM[22] = 32'h00000013; // nop
        RAM[23] = 32'h00000013; // nop
        RAM[24] = 32'h00e45463; // bge x8, x14, 8    (7 >= 0 - true)
        RAM[25] = 32'h00000013; // nop (should be skipped)
        RAM[26] = 32'h00000013; // nop (should be skipped)
        RAM[27] = 32'h00e47463; // bgeu x8, x14, 8   (7 >= 0 unsigned - true)
        RAM[28] = 32'h00000013; // nop (should be skipped)
        RAM[29] = 32'h00000013; // nop (should be skipped)
        RAM[30] = 32'h00000013; // nop (end)*/
        // Updated imem.v with JAL/JALR/AUIPC/LUI tests
 
        // imem.v - Fixed test program
   


   /* RAM[0]  = 32'h00900513;  // addi a0, zero, 9
    RAM[1]  = 32'h004000ef;  // jal ra, 16
    RAM[2]  = 32'hff410113;  // addi sp, sp, -12
    RAM[3]  = 32'h00812423;  // sw s0, 8(sp)
    RAM[4]  = 32'h00912223;  // sw s1, 4(sp)
    RAM[5]  = 32'h01212023;  // sw s2, 0(sp)
    RAM[6]  = 32'h00000413;  // addi s0, zero, 0
    RAM[7]  = 32'h00100493;  // addi s1, zero, 1
    RAM[8]  = 32'h00100913;  // addi s2, zero, 1
    RAM[9]  = 32'h01254a63;  // blt s2, a0, 12
    RAM[10] = 32'h00940433;  // add s0, s0, s1
    RAM[11] = 32'h409404b3;  // sub s1, s0, s1
    RAM[12] = 32'h00190913;  // addi s2, s2, 1
    RAM[13] = 32'hff1ff06f;  // jal zero, -16
    RAM[14] = 32'h00800533;  // add a0, zero, s0
    RAM[15] = 32'h00812403;  // lw s0, 8(sp)
    RAM[16] = 32'h00412483;  // lw s1, 4(sp)
    RAM[17] = 32'h00012903;  // lw s2, 0(sp)
    RAM[18] = 32'h00c10113;  // addi sp, sp, 12
    RAM[19] = 32'h00008067;  // jal*/

    // Setup: x4=5, x6=3, x7=-1, x8=10
   /*RAM[0] = 32'h00500213;  // addi x4, x0, 5
    RAM[1] = 32'h00300313;  // addi x6, x0, 3  
    RAM[2] = 32'hfff00393;  // addi x7, x0, -1
    RAM[3] = 32'h00a00413;  // addi x8, x0, 10
    
    // BLT test: 3 < 10? YES
    RAM[4] = 32'h00834463;  // blt x6, x8, 8
    RAM[5] = 32'h00000013;  // nop (skipped)
    RAM[6] = 32'h00100513;  // addi x10, x0, 1
    
    // BGE test: 3 >= -1? YES  
    RAM[7] = 32'h00735463;  // bge x6, x7, 8
    RAM[8] = 32'h00000013;  // nop (skipped)
    RAM[9] = 32'h00200513;  // addi x10, x0, 2
    
    // BLTU test: 3 < 0xFFFFFFFF? YES
    RAM[10] = 32'h00736463; // bltu x6, x7, 8
    RAM[11] = 32'h00000013; // nop (skipped)
    RAM[12] = 32'h00300513; // addi x10, x0, 3
    
    // BGEU test: 3 >= 10? NO
    RAM[13] = 32'h00837463; // bgeu x6, x8, 8
    RAM[14] = 32'h00400513; // addi x10, x0, 4 (should execute)
    RAM[15] = 32'h00000013; // nop*/


     
    // Test LUI and AUIPC
    RAM[0] = 32'h12345237;  // lui x4, 0x12345     
    RAM[1] = 32'h00001297;  // auipc x5, 1         
    
    // Test basic arithmetic
    RAM[2] = 32'h00500313;  // addi x6, x0, 5      
    RAM[3] = 32'h00300393;  // addi x7, x0, 3      
    
    // Test JAL - jump exactly to RAM[6]
    RAM[4] = 32'h008000ef;  // jal x1, 8           (PC=0x10, target=0x18=RAM[6])
    RAM[5] = 32'h00100413;  // addi x8, x0, 1      (skipped, executed later via JALR)
    
    // JAL target
    RAM[6] = 32'h00300513;  // addi x10, x0, 3     (x10 = 3)
    
    // JALR back to RAM[5] 
    RAM[7] = 32'h00008067;  // jalr x0, x1, 0      (jump to x1=0x14=RAM[5])
    
    RAM[8] = 32'h00000013;  // nop
     /* RAM[0] = 32'h00000093; // addi x1, x0, 0
    RAM[1] = 32'h00100113; // addi x2, x0, 1

    // Store first two numbers to memory 0(x0) and 4(x0)
    RAM[2] = 32'h00102023; // sw x1, 0(x0) = 0
    RAM[3] = 32'h00202223; // sw x2, 4(x0) = 1

    // Generate Fibonacci numbers (fully unrolled, storing each result to memory)
    RAM[4]  = 32'h002081B3; // add x3, x1, x2 => 0+1=1
    RAM[5]  = 32'h00302423; // sw x3, 8(x0) => 1
    RAM[6]  = 32'h002000B3; // add x1, x0, x2 => x1=1
    RAM[7]  = 32'h00300133; // add x2, x0, x3 => x2=1

    RAM[8]  = 32'h002081B3; // add x3, x1, x2 => 1+1=2
    RAM[9]  = 32'h00302623; // sw x3, 12(x0) => 2
    RAM[10] = 32'h002000B3; // add x1, x0, x2 => x1=1
    RAM[11] = 32'h00300133; // add x2, x0, x3 => x2=2

    RAM[12] = 32'h002081B3; // add x3, x1, x2 => 1+2=3
    RAM[13] = 32'h00302823; // sw x3, 16(x0) => 3
    RAM[14] = 32'h002000B3; // add x1, x0, x2 => x1=2
    RAM[15] = 32'h00300133; // add x2, x0, x3 => x2=3

    RAM[16] = 32'h002081B3; // add x3, x1, x2 => 2+3=5
    RAM[17] = 32'h00302A23; // sw x3, 20(x0) => 5
    RAM[18] = 32'h002000B3; // add x1, x0, x2 => x1=3
    RAM[19] = 32'h00300133; // add x2, x0, x3 => x2=5

    RAM[20] = 32'h002081B3; // add x3, x1, x2 => 3+5=8
    RAM[21] = 32'h00302C23; // sw x3, 24(x0) => 8
    RAM[22] = 32'h002000B3; // add x1, x0, x2 => x1=5
    RAM[23] = 32'h00300133; // add x2, x0, x3 => x2=8

    RAM[24] = 32'h002081B3; // add x3, x1, x2 => 5+8=13
    RAM[25] = 32'h00302E23; // sw x3, 28(x0) => 13
    RAM[26] = 32'h002000B3; // add x1, x0, x2 => x1=8
    RAM[27] = 32'h00300133; // add x2, x0, x3 => x2=13

    RAM[28] = 32'h002081B3; // add x3, x1, x2 => 8+13=21
    RAM[29] = 32'h003020A3; // sw x3, 32(x0) => 21
    RAM[30] = 32'h002000B3; // add x1, x0, x2 => x1=13
    RAM[31] = 32'h00300133; // add x2, x0, x3 => x2=21

    RAM[32] = 32'h002081B3; // add x3, x1, x2 => 13+21=34
    RAM[33] = 32'h003022A3; // sw x3, 36(x0) => 34
    RAM[34] = 32'h002000B3; // add x1, x0, x2 => x1=21
    RAM[35] = 32'h00300133; // add x2, x0, x3 => x2=34

    RAM[36] = 32'h002081B3; // add x3, x1, x2 => 21+34=55
    RAM[37] = 32'h003024A3; // sw x3, 40(x0) => 55
    RAM[38] = 32'h002000B3; // add x1, x0, x2 => x1=34
    RAM[39] = 32'h00300133; // add x2, x0, x3 => x2=55

    RAM[40] = 32'h002081B3; // add x3, x1, x2 => 34+55=89
    RAM[41] = 32'h003026A3; // sw x3, 44(x0) => 89
    RAM[42] = 32'h002000B3; // add x1, x0, x2 => x1=55
    RAM[43] = 32'h00300133; // add x2, x0, x3 => x2=89

    RAM[44] = 32'h002081B3; // add x3, x1, x2 => 55+89=144
    RAM[45] = 32'h003028A3; // sw x3, 48(x0) => 144
    RAM[46] = 32'h002000B3; // add x1, x0, x2 => x1=89
    RAM[47] = 32'h00300133; // add x2, x0, x3 => x2=144

    RAM[48] = 32'h002081B3; // add x3, x1, x2 => 89+144=233
    RAM[49] = 32'h00302AA3; // sw x3, 52(x0) => 233
    RAM[50] = 32'h002000B3; // add x1, x0, x2 => x1=144
    RAM[51] = 32'h00300133; // add x2, x0, x3 => x2=233

    RAM[52] = 32'h002081B3; // add x3, x1, x2 => 144+233=377
    RAM[53] = 32'h00302CA3; // sw x3, 56(x0) => 377
    RAM[54] = 32'h002000B3; // add x1, x0, x2 => x1=233
    RAM[55] = 32'h00300133; // add x2, x0, x3 => x2=377

    RAM[56] = 32'h002081B3; // add x3, x1, x2 => 233+377=610
    RAM[57] = 32'h00302EA3; // sw x3, 60(x0) => 610
    RAM[58] = 32'h002000B3; // add x1, x0, x2 => x1=377
    RAM[59] = 32'h00300133; // add x2, x0, x3 => x2=610

    RAM[60] = 32'h002081B3; // add x3, x1, x2 => 377+610=987
    RAM[61] = 32'h00302023; // sw x3, 64(x0) => 987
    RAM[62] = 32'h002000B3; // add x1, x0, x2 => x1=610
    RAM[63] = 32'h00300133; // add x2, x0, x3 => x2=987

    RAM[64] = 32'h002081B3; // add x3, x1, x2 => 610+987=1597
    RAM[65] = 32'h00302223; // sw x3, 68(x0) => 1597
    RAM[66] = 32'h002000B3; // add x1, x0, x2 => x1=987
    RAM[67] = 32'h00300133; // add x2, x0, x3 => x2=1597

    RAM[68] = 32'h002081B3; // add x3, x1, x2 => 987+1597=2584
    RAM[69] = 32'h00302423; // sw x3, 72(x0) => 2584
    RAM[70] = 32'h002000B3; // add x1, x0, x2 => x1=1597
    RAM[71] = 32'h00300133; // add x2, x0, x3 => x2=2584

    RAM[72] = 32'h002081B3; // add x3, x1, x2 => 1597+2584=4181
    RAM[73] = 32'h00302623; // sw x3, 76(x0) => 4181
    RAM[74] = 32'h002000B3; // add x1, x0, x2 => x1=2584
    RAM[75] = 32'h00300133; // add x2, x0, x3 => x2=4181
    
    RAM[76] = 32'h002081B3; // add x3, x1, x2 => 2584+4181=6765
    RAM[77] = 32'h00302423; // sw x3, 80(x0) => 6765
    RAM[78] = 32'h00000093; // addi x1, x0, 0
    RAM[79] = 32'h00100113; // addi x2, x0, 1
    RAM[80] = 32'hEC1FF0EF; // jal x0, -320 (go back to RAM[0])*/

end
  
 
    assign rd = RAM[a[31:2]]; // word aligned
endmodule



module dmem(
    input clk, we,
    input [31:0] a, wd,
    output [31:0] rd
);
    reg [31:0] RAM[250:0];
    
    assign rd = RAM[a[31:2]]; // word aligned
    
    always @(posedge clk) begin
        if (we) RAM[a[31:2]] <= wd;
    end
endmodule
// utils.v - Utility modules
module flopr #(parameter WIDTH = 8)(
    input clk, reset,
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q
);
    always @(posedge clk, posedge reset) begin
        if (reset) q <= 0;
        else q <= d;
    end
endmodule

module mux2 #(parameter WIDTH = 8)(
    input [WIDTH-1:0] d0, d1,
    input s,
    output [WIDTH-1:0] y
);
    assign y = s ? d1 : d0;
endmodule


module adder(
    input [31:0] a, b,
    output [31:0] y
);
    assign y = a + b;
endmodule

module mux4 #(parameter WIDTH = 8)(
    input [WIDTH-1:0] d0, d1, d2, d3,
    input [1:0] s,
    output [WIDTH-1:0] y
);
    assign y = (s == 2'b00) ? d0 :
               (s == 2'b01) ? d1 :
               (s == 2'b10) ? d2 : d3;
endmodule
