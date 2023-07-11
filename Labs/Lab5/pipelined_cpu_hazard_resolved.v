module PC (PC_next, IF_PC, clk, PCWrite);
    parameter width = 32;

    output reg [width-1:0] IF_PC;
    input [width-1:0] PC_next;
    input clk, PCWrite;

    initial begin // initialize PC
        IF_PC = 32'b0;
    end

    always @(posedge clk) begin // get next PC
        if (PCWrite) IF_PC = PC_next;
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
        $readmemb("E:/Semesters/FA22/VE370/Lab Files/Lab5_testcase.txt", memory);
    end

    assign instruction = memory[readAddress]; // read instruction
endmodule

module registerFile (readRegister1, readRegister2, writeRegister, writeData, regWrite, readData1, readData2);
    parameter width = 32;
    parameter addr_width = 5;
    parameter number = 2**addr_width;

    output [width-1:0]      readData1, readData2;
    input  [width-1:0]      writeData;
    input  [addr_width-1:0] readRegister1, readRegister2, writeRegister;
    input                   regWrite;

    reg [width-1:0] registers [number-1:0]; // 32 x registers

    assign readData1 = registers[readRegister1];
    assign readData2 = registers[readRegister2];      
    
    integer i;
    initial begin // initialize registers
        for (i = 0; i < 32; i = i + 1) registers[i] = 32'b0;
    end
    
    always @(*) begin // write value without clock control
        if (regWrite) registers[writeRegister] = writeData;    
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

module mux_32b_2to1 (inData0, inData1, outData, mux_src);
    parameter width = 32;

    output [width-1:0] outData;
    input  [width-1:0] inData0, inData1;
    input              mux_src;

    assign outData = (mux_src)? inData1: inData0;
endmodule

module PC_mux_32b_2to1 (inData0, inData1, outData, mux_src);
    parameter width = 32;

    output reg [width-1:0] outData;
    input      [width-1:0] inData0, inData1;
    input                  mux_src;

    always @(*) begin // special mux for PC, with initialize and support jalr with an additional data input
        case (mux_src)
            'b0: outData = inData0;
            'b1: outData = inData1;
            default: outData = inData0;
        endcase
    end
endmodule

module mux_32b_3to1 (inData0, inData1, inData2, outData, mux_src);
    parameter width = 32;

    output reg [width-1:0] outData;
    input      [width-1:0] inData0, inData1, inData2;
    input      [1:0]       mux_src;

    always @(*) begin
        case (mux_src)
            2'b00: outData = inData0;
            2'b01: outData = inData1;
            2'b10: outData = inData2;
            default: outData = 'bz;
        endcase
    end
endmodule

module ALU (inData0, inData1, ALU_result, ALU_control);
    parameter width = 32;

    output reg [width-1:0] ALU_result;
    input      [width-1:0] inData0, inData1;
    input      [3:0]       ALU_control;

    always @(*) begin
        case (ALU_control)
            4'b0000: begin
                ALU_result = inData0 & inData1;
            end
            4'b0001: begin
                ALU_result = inData0 | inData1;
            end
            4'b0010: begin
                ALU_result = inData0 + inData1;
            end
            4'b0110: begin // branch for beq
                ALU_result = inData0 - inData1;
            end
            4'b0111: begin // branch for bne
                ALU_result = inData0 - inData1;
            end
            4'b1000: begin // branch for blt
                ALU_result = inData0 - inData1;
            end
            4'b1001: begin // branch for bge
                ALU_result = inData0 - inData1;
            end
            4'b1110: begin // sl
                ALU_result = inData0 << inData1;
            end
            4'b1101: begin // sr
                ALU_result = inData0 >> inData1;
            end
            4'b1111: begin // sra
                ALU_result = $signed($signed(inData0) >>> inData1);
            end
            default: begin
                ALU_result = 'bz;
            end
        endcase
    end
endmodule

module dataMemory (address, writeData, readData, memWrite, memRead);
    parameter width = 32;
    parameter addr_width = 6;
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
        // 小的地址�??? data �???右端�???
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

module branchControl (jal, jalr, branch, branchOK, branchOp);
    input branchOK, branch, jal, jalr;
    output reg branchOp;

    initial branchOp = 'b0;

    always @(*) begin
        if ((branch && branchOK) || jal || jalr) branchOp = 'b1;
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

module control (ct_sig, instruction);
    parameter width = 32;

    output reg [12:0] ct_sig;
    input [width-1:0] instruction;
    
    always @(instruction) begin
        case (instruction[6:0])
            7'b0110011: ct_sig = 13'b0000001000010; // R-type
            7'b0010011: ct_sig = 13'b0000001100110; // I-type
            7'b0000011: begin // load
                    case (instruction[14:12])
                        3'b000: ct_sig = 13'b0001010000110; // lb
                        3'b010: ct_sig = 13'b0000110000110; // lw
                        3'b100: ct_sig = 13'b0001110000110; // lbu
                        default: ct_sig = 'bz; 
                    endcase
                    end
            7'b0100011: begin // store
                    case (instruction[14:12])
                        3'b000: ct_sig = 13'b0000000010100; // sb
                        3'b010: ct_sig = 13'b0000000001100;  // sw
                        default: ct_sig = 'bz; 
                    endcase
                    end
            7'b1100011: ct_sig = 13'b0010000100000; // B-type
            7'b1100111: ct_sig = 13'b0100000000100; // jalr
            7'b1101111: ct_sig = 13'b1010000000011; // J-type
            default:    ct_sig = 13'b0000000000000;
        endcase 
    end
endmodule

// module control (PCToReg, branch, memRead, memToReg, ALU_op, memWrite, ALU_src, regWrite, jalr, jal, instruction, controlFlush);
//     parameter width = 32;

//     output reg PCToReg, branch, memToReg, ALU_src, regWrite, jalr, jal;
//     output reg [1:0] ALU_op, memRead, memWrite;
//     input [width-1:0] instruction;
//     input controlFlush;
    
//     always @(*) begin
//         if (controlFlush=='b1) begin
//             jal = 'b0;
//             jalr = 'b0;
//             branch = 'b0;
//             memRead = 2'b00;
//             memToReg = 'b0;
//             ALU_op = 2'b00;
//             memWrite = 2'b00;
//             ALU_src = 'b0;
//             regWrite = 'b0;
//             PCToReg = 'b0;
//         end
//         else begin
//             case (instruction[6:0])
//                 7'b0110011: begin // R-type
//                     jal = 'b0;
//                     jalr = 'b0;
//                     branch = 'b0;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b10;
//                     memWrite = 2'b00;
//                     ALU_src = 'b0;
//                     regWrite = 'b1;
//                     PCToReg = 'b0;
//                     end
//                 7'b0010011: begin // I-type
//                     jal = 'b0;
//                     jalr = 'b0;
//                     branch = 'b0;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b11;
//                     memWrite = 2'b00;
//                     ALU_src = 'b1;
//                     regWrite = 'b1;
//                     PCToReg = 'b0;
//                     end
//                 7'b0000011: begin // load
//                     case (instruction[14:12])
//                         3'b000: memRead = 2'b10; // lb
//                         3'b010: memRead = 2'b01; // lw
//                         3'b100: memRead = 2'b11; // lbu
//                         default: memRead = 'bz; 
//                     endcase
//                     jal = 'b0;
//                     jalr = 'b0;
//                     branch = 'b0;
//                     memToReg = 'b1;
//                     ALU_op = 2'b00;
//                     memWrite = 2'b00;
//                     ALU_src = 'b1;
//                     regWrite = 'b1;
//                     PCToReg = 'b0;
//                     end
//                 7'b0100011: begin // store
//                     case (instruction[14:12])
//                         3'b000: memWrite = 2'b10; // sb
//                         3'b010: memWrite = 2'b01; // sw
//                         default: memWrite = 'bz; 
//                     endcase
//                     jal = 'b0;
//                     jalr = 'b0;
//                     branch = 'b0;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b00;
//                     memWrite
//                     ALU_src = 'b1;
//                     regWrite = 'b0;
//                     PCToReg = 'b0;
//                     end
//                 7'b1100011: begin // B-type
//                     jal = 'b0;
//                     jalr = 'b0;
//                     branch = 'b1;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b01;
//                     memWrite = 2'b00;
//                     ALU_src = 'b0;
//                     regWrite = 'b0;
//                     PCToReg = 'b0;
//                     end
//                 7'b1100111: begin // jalr
//                     jal = 'b0;
//                     jalr = 'b1;
//                     branch = 'b0;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b00;
//                     memWrite = 2'b00;
//                     ALU_src = 'b1;
//                     regWrite = 'b0;
//                     PCToReg = 'b0;
//                     end
//                 7'b1101111: begin // J-type
//                     jal = 'b1;
//                     jalr = 'b0;
//                     branch = 'b1;
//                     memRead = 2'b00;
//                     memToReg = 'b0;
//                     ALU_op = 2'b00;
//                     memWrite = 2'b00;
//                     ALU_src = 'b0;
//                     regWrite = 'b1;
//                     PCToReg = 'b1;
//                     end
//                 default: begin
//                     jal = 'bz;
//                     jalr = 'bz;
//                     branch = 'bz;
//                     memRead = 'bz;
//                     memToReg = 'bz;
//                     ALU_op = 'bz;
//                     memWrite = 'bz;
//                     ALU_src = 'bz;
//                     regWrite = 'bz;
//                     PCToReg = 'bz;
//                     end 
//             endcase
//         end
//     end
// endmodule

module ID_forwardingUnit (ID_branch, ID_Rs1, ID_Rs2, MEM_Rd, MEM_regWrite, Forward1, Forward2);
    parameter addr_width = 5;
    input MEM_regWrite, ID_branch;
    input [addr_width-1:0] ID_Rs1, ID_Rs2, MEM_Rd;
    output reg Forward1, Forward2;

    initial begin
        Forward1 = 'b0;
        Forward2 = 'b0;
    end

    always @(*) begin
        if (ID_branch && MEM_regWrite && MEM_Rd && MEM_Rd==ID_Rs1) Forward1 = 'b1;
        else Forward1 = 'b0;
        if (ID_branch && MEM_regWrite && MEM_Rd && MEM_Rd==ID_Rs2) Forward2 = 'b1;
        else Forward2 = 'b0;
    end
endmodule

module EX_forwardingUnit (EX_Rs1, EX_Rs2, EX_memRead, EX_memWrite, MEM_Rd, WB_Rd, MEM_regWrite, MEM_memRead, WB_regWrite, ForwardA, ForwardB);
    parameter addr_width = 5;
    input MEM_regWrite, WB_regWrite;
    input [addr_width-1:0] EX_Rs1, EX_Rs2, MEM_Rd, WB_Rd;
    input [1:0] MEM_memRead, EX_memRead, EX_memWrite;
    output reg [1:0] ForwardA, ForwardB;

    initial begin
        ForwardA = 2'b00;
        ForwardB = 2'b00;
    end

    always @(*) begin
        if (MEM_regWrite && MEM_Rd && MEM_Rd == EX_Rs1) ForwardA = 2'b10;
        else if (WB_regWrite && WB_Rd && WB_Rd==EX_Rs1) ForwardA = 2'b01;
        else ForwardA = 2'b00;
        if (MEM_regWrite && MEM_Rd && MEM_Rd == EX_Rs2) ForwardB = 2'b10;
        else if (WB_regWrite && WB_Rd && WB_Rd==EX_Rs2) ForwardB = 2'b01;
        else ForwardB = 2'b00;
    end
endmodule

module hazardDetectionUnit (ID_branch, Forward1, Forward2, ID_Rs1, ID_Rs2, EX_Rd, MEM_Rd, ID_memWrite, EX_memRead, MEM_memRead, EX_regWrite, PCWrite, ID_write, controlFlush);
    parameter addr_width = 5;
    input [addr_width-1:0] ID_Rs1, ID_Rs2, EX_Rd, MEM_Rd;
    input [1:0] EX_memRead, ID_memWrite, MEM_memRead;
    input Forward1, Forward2, ID_branch, EX_regWrite;
    output reg PCWrite, ID_write, controlFlush;

    initial begin // initialize PC
        PCWrite = 'b1;
        ID_write = 'b1;
        controlFlush = 'b0;
    end

    always @(*) begin
        if ((EX_memRead && !ID_memWrite && (EX_Rd==ID_Rs1 || EX_Rd==ID_Rs2))) begin
            PCWrite = 'b0;
            ID_write = 'b0;
            controlFlush = 'b1;
        end
        else if (ID_branch && EX_regWrite && EX_Rd && (EX_Rd==ID_Rs1 || EX_Rd==ID_Rs2)) begin
            PCWrite = 'b0;
            ID_write = 'b0;
            controlFlush = 'b1;
        end
        else if (ID_branch && MEM_memRead && MEM_Rd && (MEM_Rd==ID_Rs1 || MEM_Rd==ID_Rs2)) begin
            PCWrite = 'b0;
            ID_write = 'b0;
            controlFlush = 'b1;
        end
        else begin
            PCWrite = 'b1;
            ID_write = 'b1;
            controlFlush = 'b0;
        end
    end
endmodule

module memSrcUnit (WB_Rd, MEM_Rs2, WB_memRead, MEM_memWrite, memSrc);
    parameter addr_width = 5;
    input [addr_width-1:0] WB_Rd, MEM_Rs2;
    input [1:0] WB_memRead, MEM_memWrite;
    output reg memSrc = 'b0;

    always @(*) begin
        if (WB_Rd==MEM_Rs2 && MEM_memWrite && WB_memRead) memSrc = 'b1;
        else memSrc = 'b0;
    end
endmodule

module branchDetection (readData1, readData2, instruction, branchOK);
    // instr: func3 part of instruction
    parameter width = 32;
    input [width-1:0] readData1, readData2, instruction;
    output reg branchOK;

    always @(*) begin
        if (instruction[6:0]==7'b1100011) begin
            case (instruction[14:12])
                3'b000: branchOK = (readData1 == readData2); // beq
                3'b001: branchOK = (readData1 != readData2); // bne
                3'b100: branchOK = (readData1  < readData2); // blt
                3'b101: branchOK = (readData1 >= readData2); // bge
                default: branchOK = 'b1;
            endcase
        end
    end
endmodule

module IF_ID (IF_PC_add_4, IF_PC, IF_instruction, clk, ID_PC_add_4, ID_PC, ID_instruction, ID_write, IF_flush);
    parameter width = 32;

    input clk, ID_write, IF_flush;
    input [width-1:0] IF_PC_add_4, IF_PC, IF_instruction;
    output reg [width-1:0] ID_PC_add_4, ID_PC, ID_instruction;

    always @(posedge clk) begin
        if (IF_flush) ID_instruction = 32'b0;
        if (ID_write) begin
            ID_PC_add_4 = IF_PC_add_4;
            ID_PC = IF_PC;
            ID_instruction = IF_instruction;
        end
    end
endmodule

module ID_EX (
    input [2:0] ID_WB, 
    input [3:0] ID_M, 
    input [2:0] ID_EX, 
    input [31:0] ID_instruction, ID_PC_add_4, ID_readData1, ID_readData2, ID_immediate, 
    input [3:0] ID_ALU_control, 
    input [4:0] ID_Rs1, ID_Rs2, ID_Rd,
    input clk, 
    output reg [2:0] EX_WB, 
    output reg [3:0] EX_M, 
    output reg [2:0] EX_EX, 
    output reg [31:0] EX_instruction, EX_PC_add_4, EX_readData1, EX_readData2, EX_immediate, 
    output reg [3:0] EX_ALU_control, 
    output reg [4:0] EX_Rs1, EX_Rs2, EX_Rd);
    parameter width = 32;

    always @(posedge clk) begin
        EX_WB =ID_WB;
        EX_M = ID_M;
        EX_EX = ID_EX;
        EX_instruction = ID_instruction;
        EX_PC_add_4 = ID_PC_add_4;
        EX_readData1 = ID_readData1;
        EX_readData2 = ID_readData2;
        EX_immediate = ID_immediate;
        EX_ALU_control = ID_ALU_control;
        EX_Rs1 = ID_Rs1;
        EX_Rs2 = ID_Rs2;
        EX_Rd = ID_Rd;
    end
endmodule

module EX_MEM (
    input [2:0] EX_WB, 
    input [3:0] EX_M, 
    input [31:0] EX_PC_add_4, EX_ALU_result, EX_readData2, 
    input [4:0] EX_Rd, EX_Rs2,
    input clk, 
    output reg[2:0] MEM_WB, 
    output reg [3:0] MEM_M, 
    output reg [31:0] MEM_PC_add_4, MEM_ALU_result, MEM_readData2,  
    output reg [4:0] MEM_Rd, MEM_Rs2);
    parameter width = 32;

    always @(posedge clk) begin
        MEM_WB = EX_WB;
        MEM_M = EX_M;
        MEM_PC_add_4 = EX_PC_add_4;
        MEM_ALU_result = EX_ALU_result;
        MEM_readData2 = EX_readData2;
        MEM_Rd = EX_Rd;
        MEM_Rs2 = EX_Rs2;
    end
endmodule

module MEM_WB (
    input [2:0] MEM_WB, 
    input [31:0] MEM_PC_add_4, MEM_readData, MEM_ALU_result, 
    input [4:0] MEM_Rd, 
    input [1:0] MEM_memRead,
    input clk, 
    output reg[2:0] WB_WB, 
    output reg [31:0] WB_PC_add_4, WB_readData, WB_ALU_result,
    output reg [4:0] WB_Rd,
    output reg [1:0] WB_memRead);
    parameter width = 32;

    always @(posedge clk) begin
        WB_WB = MEM_WB;
        WB_readData = MEM_readData;
        WB_PC_add_4 = MEM_PC_add_4;
        WB_ALU_result = MEM_ALU_result;
        WB_Rd = MEM_Rd;
        WB_memRead = MEM_memRead;
    end
endmodule

module top (clk);
    parameter width = 32;
    parameter addr_width = 5;
    
    input clk;
    
    wire [width-1:0] IF_PC_add_4, PC_next;

    wire [width-1:0] IF_PC, IF_instruction;

    wire [width-1:0] ID_PC_add_4, ID_PC, ID_instruction, ID_immediate, ID_readData1, ID_readData2, ID_branch_immediate, ID_PC_branch, ID_forward1_result, ID_forward2_result, ID_addfirst, ID_addsecond;
    wire [3:0] ID_ALU_control_input;
    wire [4:0] ID_Rs1, ID_Rs2, ID_Rd;
    wire [1:0] ID_ALU_op, ID_memRead, ID_memWrite;
    wire [12:0] ct_sig, ct_sig_new;

    wire [width-1:0] EX_PC_add_4, EX_instruction, EX_immediate, EX_readData1, EX_readData2, EX_ALU_input2, EX_ALU_result, EX_writeData, EX_forwardA_result, EX_forwardB_result;
    wire [3:0] EX_ALU_control_input;
    wire [4:0] EX_Rs1, EX_Rs2, EX_Rd;
    wire [3:0] EX_ALU_control;
    wire [1:0] EX_ALU_op, EX_memRead, EX_memWrite, ForwardA, ForwardB;

    wire [width-1:0] MEM_PC_add_4, MEM_readData2, MEM_ALU_result, MEM_readData, MEM_writeData;
    wire [4:0] MEM_Rd, MEM_Rs2;
    wire [1:0] MEM_memRead, MEM_memWrite;

    wire [width-1:0] WB_PC_add_4, WB_ALU_result, WB_readData, WB_writeData;
    wire [4:0] WB_Rd;
    wire [1:0] WB_memRead;

    PC pc (PC_next, IF_PC, clk, PCWrite);
    instructionMemory im (IF_PC, IF_instruction);
    add add_4 (IF_PC, 1, IF_PC_add_4);
    PC_mux_32b_2to1 mux_pc (IF_PC_add_4, ID_PC_branch, PC_next, Branch_Src);

    IF_ID if_id (IF_PC_add_4, IF_PC, IF_instruction, clk, ID_PC_add_4, ID_PC, ID_instruction, ID_write, IF_flush);

    registerFile registerfile (ID_Rs1, ID_Rs2, WB_Rd, WB_writeData, WB_regWrite, ID_readData1, ID_readData2);
    immGen ig (ID_instruction, ID_immediate);
    control c (ct_sig, ID_instruction);
    mux_32b_2to1 #(13) mux_ct (ct_sig, 13'b0, ct_sig_new, controlFlush);
    assign {ID_jal, ID_jalr, ID_branch, ID_memRead, ID_memToReg, ID_ALU_op, ID_memWrite, ID_ALU_src, ID_regWrite, ID_PCToReg} = ct_sig_new;
    assign ID_ALU_control_input = {ID_instruction[30], ID_instruction[14:12]};
    assign ID_Rs1 = ID_instruction[19:15], ID_Rs2 = ID_instruction[24:20], ID_Rd = ID_instruction[11:7];
    hazardDetectionUnit hdu (ID_branch, Forward1, Forward2, ID_Rs1, ID_Rs2, EX_Rd, MEM_Rd, ID_memWrite, EX_memRead, MEM_memRead, EX_regWrite, PCWrite, ID_write, controlFlush);
    branchDetection bd (ID_forward1_result, ID_forward2_result, ID_instruction, ID_branchOK);
    branchControl bc (ID_jal, ID_jalr, ID_branch, ID_branchOK, Branch_Src);
    shiftRight1 SL (ID_immediate, ID_branch_immediate);
    mux_32b_2to1 mux_add1 (ID_PC, ID_forward1_result, ID_addfirst, ID_jalr);
    mux_32b_2to1 mux_add2 (ID_branch_immediate, ID_immediate, ID_addsecond, ID_jalr);
    add add_immediate (ID_addfirst, ID_addsecond, ID_PC_branch);
    assign IF_flush = Branch_Src;
    mux_32b_2to1 mux_f1 (ID_readData1, MEM_ALU_result, ID_forward1_result, Forward1);
    mux_32b_2to1 mux_f2 (ID_readData2, MEM_ALU_result, ID_forward2_result, Forward2);
    ID_forwardingUnit idfu (ID_branch, ID_Rs1, ID_Rs2, MEM_Rd, MEM_regWrite, Forward1, Forward2);

    ID_EX id_ex ({ID_PCToReg, ID_regWrite, ID_memToReg}, {ID_memRead, ID_memWrite}, {ID_ALU_op, ID_ALU_src}, ID_instruction, ID_PC_add_4, ID_forward1_result, ID_forward2_result, ID_immediate, ID_ALU_control_input, ID_Rs1, ID_Rs2, ID_Rd, clk, {EX_PCToReg, EX_regWrite, EX_memToReg}, {EX_memRead, EX_memWrite}, {EX_ALU_op, EX_ALU_src}, EX_instruction, EX_PC_add_4, EX_readData1, EX_readData2, EX_immediate, EX_ALU_control_input, EX_Rs1, EX_Rs2, EX_Rd);

    mux_32b_3to1 mux_fa (EX_readData1, WB_readData, MEM_ALU_result, EX_forwardA_result, ForwardA);
    mux_32b_3to1 mux_fb (EX_readData2, WB_readData, MEM_ALU_result, EX_forwardB_result, ForwardB);
    mux_32b_2to1 mux_ALU (EX_forwardB_result, EX_immediate, EX_ALU_input2, EX_ALU_src);
    ALU alu (EX_forwardA_result, EX_ALU_input2, EX_ALU_result, EX_ALU_control);
    ALU_ct alu_ct (EX_ALU_control_input, EX_ALU_control, EX_ALU_op);
    EX_forwardingUnit exfu (EX_Rs1, EX_Rs2, EX_memRead, EX_memWrite, MEM_Rd, WB_Rd, MEM_regWrite, MEM_memRead, WB_regWrite, ForwardA, ForwardB);

    EX_MEM ex_mem ({EX_PCToReg, EX_regWrite, EX_memToReg}, {EX_memRead, EX_memWrite}, EX_PC_add_4, EX_ALU_result, EX_forwardB_result, EX_Rd, EX_Rs2, clk, {MEM_PCToReg, MEM_regWrite, MEM_memToReg}, {MEM_memRead, MEM_memWrite}, MEM_PC_add_4, MEM_ALU_result, MEM_readData2, MEM_Rd, MEM_Rs2);

    memSrcUnit msu (WB_Rd, MEM_Rs2, WB_memRead, MEM_memWrite, memSrc);
    mux_32b_2to1 mux_dm (MEM_readData2, WB_writeData, MEM_writeData, memSrc);
    dataMemory dm (MEM_ALU_result, MEM_writeData, MEM_readData, MEM_memWrite, MEM_memRead);

    MEM_WB mem_wb ({MEM_PCToReg, MEM_regWrite, MEM_memToReg}, MEM_PC_add_4, MEM_readData, MEM_ALU_result, MEM_Rd, MEM_memRead, clk, {WB_PCToReg, WB_regWrite, WB_memToReg}, WB_PC_add_4, WB_readData, WB_ALU_result, WB_Rd, WB_memRead);
    
    mux_32b_3to1 mux_reg (WB_ALU_result, WB_readData, WB_PC_add_4, WB_writeData, {WB_PCToReg, WB_memToReg});
endmodule
