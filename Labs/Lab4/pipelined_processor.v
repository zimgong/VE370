module PC (PC_next, PC_curr, clk);
    parameter width = 32;

    output reg [width-1:0] PC_curr;
    input [width-1:0] PC_next;
    input clk;

    initial begin // initialize PC
        PC_curr = 32'b0;
    end

    always @(negedge clk) begin // get next PC
        PC_curr = PC_next;
    end
endmodule

module instructionMemory (readAddress, instruction);
    parameter width = 32;
    parameter addr_width = 7;
    parameter number = 2**addr_width; // instruction memory with 128x32bit 
    
    output [width-1:0] instruction;
    input [width-1:0] readAddress;

    reg [width-1:0] memory [number-1:0];

    initial begin // initialize instructions from file
        $readmemb("E:/Semesters/FA22/VE370/Labs/Lab4/Lab4_testcase.txt", memory);
    end

    assign instruction = memory[readAddress]; // read instruction
endmodule

module registers (readRegister1, readRegister2, writeRegister, writeData, regWrite, readData1, readData2);
    parameter width = 32;
    parameter addr_width = 5;
    parameter number = 2**addr_width;

    output [width-1:0]      readData1, readData2;
    input  [width-1:0]      writeData;
    input  [addr_width-1:0] readRegister1, readRegister2, writeRegister;
    input                   regWrite;

    reg [width-1:0] memory [number-1:0]; // 32 x registers

    assign readData1 = memory[readRegister1];
    assign readData2 = memory[readRegister2];      
    
    integer i;
    initial begin // initialize registers
        for (i = 0; i < 32; i = i + 1) memory[i] = 32'b0;
    end
    
    always @(*) begin // write value without clock control
        if (regWrite) memory[writeRegister] = writeData;    
    end
endmodule

module immGen (instruction, immediate);
    parameter width = 32;

    output reg [width-1:0] immediate;
    input  [width-1:0] instruction;
    
    always @(instruction) begin
       case (instruction[6:0])
        7'b0000011: // I
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b0001111: // I
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b0010011: // I
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b0010111: // U
            immediate = {{12{instruction[31]}}, instruction[31:12]};
        7'b0100011: // S
            immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        7'b0110111: // U
            immediate = {{12{instruction[31]}}, instruction[31:12]};
        7'b1100011: // B
            immediate = {{20{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
        7'b1100111: // I
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b1101111: // J
            immediate = {{12{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21]};
        7'b1110011: // I
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        default: immediate = 'bz;
       endcase 
    end
endmodule

module mux_32b_2to1 (inData0, inData1, outData, ALU_src);
    parameter width = 32;

    output [width-1:0] outData;
    input  [width-1:0] inData0, inData1;
    input              ALU_src;

    assign outData = (ALU_src)? inData1: inData0;
endmodule

module PC_mux_32b_3to1 (inData0, inData1, inData2, outData, ALU_src);
    parameter width = 32;

    output reg [width-1:0] outData;
    input      [width-1:0] inData0, inData1, inData2;
    input       [1:0]       ALU_src;

    always @(*) begin // special mux for PC, with initialize and support jump with an additional data input
        case (ALU_src)
            2'b00: outData = inData0;
            2'b01: outData = inData1;
            2'b10: outData = inData2;
            default: outData = 32'b00000000000000000000000000000001;
        endcase
    end
endmodule

module ALU (inData0, inData1, branchOK, ALU_result, ALU_control);
    parameter width = 32;

    output reg [width-1:0] ALU_result;
    output reg             branchOK;
    input      [width-1:0] inData0, inData1;
    input      [3:0]       ALU_control;

    always @(*) begin
        case (ALU_control)
            4'b0000: begin
                ALU_result = inData0 & inData1;
                branchOK = 'b0;
            end
            4'b0001: begin
                ALU_result = inData0 | inData1;
                branchOK = 'b0;
            end
            4'b0010: begin
                ALU_result = inData0 + inData1;
                branchOK = 'b1;
            end
            4'b0110: begin // branch for beq
                ALU_result = inData0 - inData1;
                if (ALU_result==32'b0) branchOK = 'b1;
                else branchOK = 'b0;
            end
            4'b0111: begin // branch for bne
                ALU_result = inData0 - inData1;
                if (ALU_result==32'b0) branchOK = 'b0;
                else branchOK = 'b1;
            end
            4'b1000: begin // branch for blt
                ALU_result = inData0 - inData1;
                if (ALU_result[31]=='b1) branchOK = 'b1;
                else branchOK = 'b0;
            end
            4'b1001: begin // branch for bge
                ALU_result = inData0 - inData1;
                if (ALU_result[31]=='b0) branchOK = 'b1;
                else branchOK = 'b0;
            end
            4'b1110: begin // sl
                ALU_result = inData0 << inData1;
                branchOK = 'b0;
            end
            4'b1101: begin // sr
                ALU_result = inData0 >> inData1;
                branchOK = 'b0;
            end
            4'b1111: begin // sra
                ALU_result = $signed($signed(inData0) >>> inData1);
                branchOK = 'b0;
            end
            default: begin
                ALU_result = 'bz;
                branchOK = 'bz;
            end
        endcase
    end
endmodule

module dataMemory (address, writeData, readData, memWrite, memRead);
    parameter width = 32;
    parameter addr_width = 7;
    parameter number = 2**addr_width;

    output [width-1:0] readData;
    input [width-1:0] address;
    input [width-1:0] writeData;
    input [1:0] memWrite;
    input [1:0] memRead;

    reg [width-1:0] readData;
    reg [(width/4)-1:0] memory [number-1:0];
    // ==zyl==
    // memory: 32bit 32rows
    // memory: 8*128 =-8*4 * 32

    always @(*) begin
        // ==zyl==
        // little Endian
        // 小的地址写 data 最右端的
        if (memWrite==2'b01) begin // sw
            memory[address]   = writeData[ 7: 0];
            memory[address+1] = writeData[15: 8];
            memory[address+2] = writeData[23:16];
            memory[address+3] = writeData[31:24];
        end
        else if (memWrite==2'b10) begin // sb
            memory[address]   = writeData[ 7: 0];
        end
        case (memRead)
            2'b01: begin // lw
                readData[ 7: 0] = memory[address];
                readData[15: 8] = memory[address+1];
                readData[23:16] = memory[address+2];
                readData[31:24] = memory[address+3];
            end
            2'b10: begin // lb
                readData[7:0] = memory[address];
                readData[31:8] = (readData[7] == 1'b1)? 24'hffffff : 24'h000000;
            end
            2'b11: begin // lbu
                readData[7:0] = memory[address];
                readData[31:8] = 24'h000000;
            end
            default: readData = 'bz;
        endcase
    end
endmodule

module add (inData0, inData1, sum);
    parameter width = 32;

    output [width-1:0] sum;
    input [width-1:0] inData0, inData1;

    assign sum = inData0 + inData1;
endmodule

module shiftRight1 (inData, outData);
    parameter width = 32;

    output [width-1:0] outData;
    input [width-1:0] inData;

    reg [width-1:0] outData;

    always @(inData) begin
        outData = inData/2;
    end
endmodule

module branchControl (branch, branchOK, branchOp);
    input branchOK;
    input branch;
    output reg branchOp;

    initial branchOp = 'b0;

    always @(*) begin
        if (branch && branchOK) branchOp = 'b1;
        else branchOp = 'b0;
    end
endmodule

module ALU_ct (instruction, ALU_control, ALU_op);
    output reg [3:0] ALU_control;
    input      [3:0] instruction;
    input      [1:0] ALU_op;

    always @(*) begin
        case (ALU_op)
            2'b00:
                ALU_control = 4'b0010;
            2'b01:
                case (instruction[2:0])
                    3'b000: ALU_control = 4'b0110;
                    3'b001: ALU_control = 4'b0111;
                    3'b100: ALU_control = 4'b1000;
                    3'b101: ALU_control = 4'b1001;
                    default: ALU_control = 'bz;
                endcase
            2'b10:
                case (instruction[2:0])
                    3'b000: 
                        if (instruction[3]) ALU_control = 4'b0110;
                        else ALU_control = 4'b0010;
                    3'b001:
                        ALU_control = 4'b1110;
                    3'b101:
                        if (instruction[3]) ALU_control = 4'b1111;
                        else ALU_control = 4'b1101;
                    3'b111:
                        ALU_control = 4'b0000;
                    3'b110:
                        ALU_control = 4'b0001;
                    3'b010:
                        ALU_control = 4'b0010;
                    default: ALU_control = 'bz;
                endcase
            2'b11:
                case (instruction[2:0])
                    3'b000:
                        ALU_control = 4'b0010;
                    3'b001:
                        ALU_control = 4'b1110;
                    3'b101:
                        if (instruction[3]) ALU_control = 4'b1111;
                        else ALU_control = 4'b1101;
                    3'b111:
                        ALU_control = 4'b0000;
                    default: ALU_control = 'bz;
                endcase
            default: ALU_control = 'bz; 
        endcase
    end
endmodule

module control (branch, memRead, memToReg, ALU_op, memWrite, ALU_src, regWrite, jump, instruction);
    parameter width = 32;

    output reg branch, memToReg, ALU_src, regWrite, jump;
    output reg [1:0] ALU_op, memRead, memWrite;
    input [width-1:0] instruction;
    
    always @(instruction) begin
        case (instruction[6:0])
            7'b0110011: begin // R-type
                jump = 'b0;
                branch = 'b0;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b10;
                memWrite = 2'b00;
                ALU_src = 'b0;
                regWrite = 'b1;
                end
            7'b0010011: begin // I-type
                jump = 'b0;
                branch = 'b0;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b11;
                memWrite = 2'b00;
                ALU_src = 'b1;
                regWrite = 'b1;
                end
            7'b0000011: begin // load
                case (instruction[14:12])
                    3'b000: memRead = 2'b10; // lb
                    3'b010: memRead = 2'b01; // lw
                    3'b100: memRead = 2'b11; // lbu
                    default: memRead = 'bz; 
                endcase
                jump = 'b0;
                branch = 'b0;
                memToReg = 'b1;
                ALU_op = 2'b00;
                memWrite = 2'b00;
                ALU_src = 'b1;
                regWrite = 'b1;
                end
            7'b0100011: begin // store
                case (instruction[14:12])
                    3'b000: memWrite = 2'b10; // sb
                    3'b010: memWrite = 2'b01; // sw
                    default: memWrite = 'bz; 
                endcase
                jump = 'b0;
                branch = 'b0;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b00;
                ALU_src = 'b1;
                regWrite = 'b0;
                end
            7'b1100011: begin // B-type
                jump = 'b0;
                branch = 'b1;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b01;
                memWrite = 2'b00;
                ALU_src = 'b0;
                regWrite = 'b0;
                end
            7'b1100111: begin // jalr
                jump = 'b1;
                branch = 'b0;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b00;
                memWrite = 2'b00;
                ALU_src = 'b1;
                regWrite = 'b0;
                end
            7'b1101111: begin // J-type
                jump = 'b0;
                branch = 'b1;
                memRead = 2'b00;
                memToReg = 'b0;
                ALU_op = 2'b00;
                memWrite = 2'b00;
                ALU_src = 'b0;
                regWrite = 'b1;
                end
            default: begin
                jump = 'bz;
                branch = 'bz;
                memRead = 'bz;
                memToReg = 'bz;
                ALU_op = 'bz;
                memWrite = 'bz;
                ALU_src = 'bz;
                regWrite = 'bz;
                end 
        endcase
    end
endmodule

module IF_ID (IF_PC_add_4, IF_PC, IF_instruction, clk, ID_PC_add_4, ID_PC, ID_instruction);
    parameter width = 32;

    input clk;
    input [width-1:0] IF_PC_add_4, IF_PC, IF_instruction;
    output reg [width-1:0] ID_PC_add_4, ID_PC, ID_instruction;

    always @(posedge clk) begin
        ID_PC_add_4 = IF_PC_add_4;
        ID_PC = IF_PC;
        ID_instruction = IF_instruction;
    end
endmodule

module ID_EX (
    input [1:0] ID_WB, 
    input [5:0] ID_M, 
    input [2:0] ID_EX, 
    input [31:0] ID_PC_add_4, ID_PC, ID_readData1, ID_readData2, ID_immediate, 
    input [3:0] ID_ALU_control, 
    input [4:0] ID_writeRegister,
    input clk, 
    output reg[1:0] EX_WB, 
    output reg [5:0] EX_M, 
    output reg [2:0] EX_EX, 
    output reg [31:0] EX_PC_add_4, EX_PC, EX_readData1, EX_readData2, EX_immediate, 
    output reg [3:0] EX_ALU_control, 
    output reg [4:0] EX_writeRegister);
    parameter width = 32;

    always @(posedge clk) begin
        EX_WB =ID_WB;
        EX_M = ID_M;
        EX_EX = ID_EX;
        EX_PC_add_4 = ID_PC_add_4;
        EX_PC = ID_PC;
        EX_readData1 = ID_readData1;
        EX_readData2 = ID_readData2;
        EX_immediate = ID_immediate;
        EX_ALU_control = ID_ALU_control;
        EX_writeRegister = ID_writeRegister;
    end
endmodule

module EX_MEM (
    input [1:0] EX_WB, 
    input [5:0] EX_M, 
    input [31:0] EX_PC_branch, EX_ALU_result, EX_readData2, 
    input EX_branchOK,
    input [4:0] EX_writeRegister,
    input clk, 
    output reg[1:0] MEM_WB, 
    output reg [5:0] MEM_M, 
    output reg [31:0] MEM_PC_branch, MEM_ALU_result, MEM_readData2, 
    output reg MEM_branchOK, 
    output reg [4:0] MEM_writeRegister);
    parameter width = 32;

    always @(posedge clk) begin
        MEM_WB = EX_WB;
        MEM_M = EX_M;
        MEM_PC_branch = EX_PC_branch;
        MEM_ALU_result = EX_ALU_result;
        MEM_readData2 = EX_readData2;
        MEM_branchOK = EX_branchOK;
        MEM_writeRegister = EX_writeRegister;
    end
endmodule

module MEM_WB (
    input [1:0] MEM_WB, 
    input [31:0] MEM_readData, MEM_ALU_result, 
    input [4:0] MEM_writeRegister, 
    input clk, 
    output reg[1:0] WB_WB, 
    output reg [31:0] WB_readData, WB_ALU_result,
    output reg [4:0] WB_writeRegister);
    parameter width = 32;

    always @(posedge clk) begin
        WB_WB = MEM_WB;
        WB_readData = MEM_readData;
        WB_ALU_result = MEM_ALU_result;
        WB_writeRegister = MEM_writeRegister;
    end
endmodule

module top (clk);
    parameter width = 32;
    parameter addr_width = 5;
    
    input clk;
    
    wire [width-1:0] IF_PC_add_4, PC_next;

    wire [width-1:0] IF_PC, IF_instruction;

    wire [width-1:0] ID_PC_add_4, ID_PC, ID_instruction, ID_immediate, ID_readData1, ID_readData2;
    wire [3:0] ID_ALU_control_input;
    wire [4:0] ID_writeRegister;
    wire [1:0] ID_ALU_op, ID_memRead, ID_memWrite;

    wire [width-1:0] EX_PC_add_4, EX_PC, EX_immediate, EX_readData1, EX_readData2, EX_ALU_input2, EX_branch_immediate, EX_PC_branch, EX_ALU_result, EX_writeData;
    wire [3:0] EX_ALU_control_input;
    wire [4:0] EX_writeRegister;
    wire [3:0] EX_ALU_control;
    wire [1:0] EX_ALU_op, EX_memRead, EX_memWrite;

    wire [width-1:0] MEM_readData2, MEM_PC_branch, MEM_ALU_result, MEM_readData;
    wire [4:0] MEM_writeRegister;
    wire [1:0] MEM_memRead, MEM_memWrite;

    wire [width-1:0] WB_ALU_result, WB_readData, WB_writeData;
    wire [4:0] WB_writeRegister;

    PC pc (PC_next, IF_PC, clk);
    instructionMemory im (IF_PC, IF_instruction);
    add add_4 (IF_PC, 1, IF_PC_add_4);
    PC_mux_32b_3to1 mux_pc (IF_PC_add_4, MEM_PC_branch, MEM_ALU_result, PC_next, {MEM_jump, Branch_Src});

    IF_ID if_id (IF_PC_add_4, IF_PC, IF_instruction, clk, ID_PC_add_4, ID_PC, ID_instruction);

    registers r (ID_instruction[19:15], ID_instruction[24:20], WB_writeRegister, WB_writeData, WB_regWrite, ID_readData1, ID_readData2);
    immGen ig (ID_instruction, ID_immediate);
    control c (ID_branch, ID_memRead, ID_memToReg, ID_ALU_op, ID_memWrite, ID_ALU_src, ID_regWrite, ID_jump, ID_instruction);
    assign ID_ALU_control_input = {ID_instruction[30], ID_instruction[14:12]};
    assign ID_writeRegister = ID_instruction[11:7];

    ID_EX id_ex ({ID_regWrite, ID_memToReg}, {ID_jump, ID_branch, ID_memRead, ID_memWrite}, {ID_ALU_op, ID_ALU_src}, ID_PC_add_4, ID_PC, ID_readData1, ID_readData2, ID_immediate, ID_ALU_control_input, ID_writeRegister, clk, {EX_regWrite, EX_memToReg}, {EX_jump, EX_branch, EX_memRead, EX_memWrite}, {EX_ALU_op, EX_ALU_src}, EX_PC_add_4, EX_PC, EX_readData1, EX_readData2, EX_immediate, EX_ALU_control_input, EX_writeRegister);

    mux_32b_2to1 mux_ALU (EX_readData2, EX_immediate, EX_ALU_input2, EX_ALU_src);
    ALU alu (EX_readData1, EX_ALU_input2, EX_branchOK, EX_ALU_result, EX_ALU_control);
    ALU_ct alu_ct (EX_ALU_control_input, EX_ALU_control, EX_ALU_op);
    mux_32b_2to1 mux_wd (EX_ALU_result, EX_PC_add_4, EX_writeData, EX_branch);
    shiftRight1 SL (EX_immediate, EX_branch_immediate);
    add add_immediate (EX_PC, EX_branch_immediate, EX_PC_branch);

    EX_MEM ex_mem ({EX_regWrite, EX_memToReg}, {EX_jump, EX_branch, EX_memRead, EX_memWrite}, EX_PC_branch, EX_writeData, EX_readData2, EX_branchOK, EX_writeRegister, clk, {MEM_regWrite, MEM_memToReg}, {MEM_jump, MEM_branch, MEM_memRead, MEM_memWrite}, MEM_PC_branch, MEM_ALU_result, MEM_readData2, MEM_branchOK, MEM_writeRegister);

    dataMemory dm (MEM_ALU_result, MEM_readData2, MEM_readData, MEM_memWrite, MEM_memRead);
    branchControl bc (MEM_branch, MEM_branchOK, Branch_Src);

    MEM_WB mem_wb ({MEM_regWrite, MEM_memToReg}, MEM_readData, MEM_ALU_result, MEM_writeRegister, clk, {WB_regWrite, WB_memToReg}, WB_readData, WB_ALU_result, WB_writeRegister);
    
    mux_32b_2to1 mux_reg (WB_ALU_result, WB_readData, WB_writeData, WB_memToReg);
endmodule
