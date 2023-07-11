`timescale 1ns / 1ps

module PC (
    input clk,
    // input PCSrc,
    // input [31:0] PC_jump,
    input [31:0] PC_next,
    output reg [31:0] PC_cur
    // output [31:0] PC_4
);
initial begin
    PC_cur <= 32'b0;
end

always @(negedge clk) begin
    PC_cur = PC_next;
    // PC_cur = (PCSrc)? (PC_jump) : (PC_cur+1);
end

// assign PC_4 = PC_cur + 1;

endmodule

module instr_mem (
    input [31:0] PC,
    output [31:0] instruction
);
    reg [31:0] mem [80:0];
    initial begin
    mem[0]  = 32'b00011001001100000000001010010011;
        mem[1]  = 32'b00000000000000000000000000010011;
        mem[2]  = 32'b00000000000000000000000000010011;
        mem[3]  = 32'b00000000010100101000001100110011;
        mem[4]  = 32'b00000000000000000000000000010011;
        mem[5]  = 32'b00000000000000000000000000010011;
        mem[6]  = 32'b01000000011000000000001110110011;
        mem[7]  = 32'b00000000011000101111111000110011;
        mem[8]  = 32'b00000000001000000000111000010011;
        mem[9]  = 32'b00000000000000000000000000010011;
        mem[10] = 32'b00000000000000000000000000010011;
        mem[11] = 32'b00000001110000110001001100110011;
        mem[12] = 32'b00000000000000000000000000010011;
        mem[13] = 32'b00000000000000000000000000010011;
        mem[14] = 32'b00000000011000111110001110110011;
        mem[15] = 32'b00000000000000000000000000010011;
        mem[16] = 32'b00000000000000000000000000010011;
        mem[17] = 32'b01110011001000111111111010010011;
        mem[18] = 32'b00000000000000000000000000010011;
        mem[19] = 32'b00000000000000000000000000010011;
        mem[20] = 32'b00000000010111101101111010010011;
        mem[21] = 32'b00000000000000000000000000010011;
        mem[22] = 32'b00000000000000000000000000010011;
        mem[23] = 32'b00000001110000111101001110110011;
        mem[24] = 32'b00000000000000000000000000010011;
        mem[25] = 32'b00000000000000000000000000010011;
        mem[26] = 32'b00000001000000111001001110010011;
        mem[27] = 32'b00000000000000000000000000010011;
        mem[28] = 32'b00000000000000000000000000010011;
        mem[29] = 32'b01000001110000111101001110110011;
        mem[30] = 32'b00000000011000101001100001100011;
        mem[31] = 32'b00000000000000000000000000010011;
        mem[32] = 32'b00000000000000000000000000010011;
        mem[33] = 32'b00000000000000000000001110110011;
        mem[34] = 32'b00000010000000111000100001100011;
        mem[35] = 32'b00000000000000000000000000010011;
        mem[36] = 32'b00000000000000000000000000010011;
        mem[37] = 32'b00000011110100111101001001100011;
        mem[38] = 32'b00000000000000000000000000010011;
        mem[39] = 32'b00000000000000000000000000010011;
        mem[40] = 32'b00000000000000111000001010110011;
        mem[41] = 32'b00000000000000000000000000010011;
        mem[42] = 32'b00000000000000000000000000010011;
        mem[43] = 32'b00000001110100111100100001100011;
        mem[44] = 32'b00000000000000000000000000010011;
        mem[45] = 32'b00000000000000000000000000010011;
        mem[46] = 32'b00000000000000000000001010110011;
        mem[47] = 32'b00000000010100111001110001100011;
        mem[48] = 32'b00000000000000000000000000010011;
        mem[49] = 32'b00000000000000000000000000010011;
        mem[50] = 32'b00000000010100111000100001100011;
        mem[51] = 32'b00000000000000000000000000010011;
        mem[52] = 32'b00000000000000000000000000010011;
        mem[53] = 32'b00000000000000000000001100110011;
        mem[54] = 32'b00000001110100110100110001100011;
        mem[55] = 32'b00000000000000000000000000010011;
        mem[56] = 32'b00000000000000000000000000010011;
        mem[57] = 32'b00000001110000110101100001100011;
        mem[58] = 32'b00000000000000000000000000010011;
        mem[59] = 32'b00000000000000000000000000010011;
        mem[60] = 32'b00000000000000000000111000110011;
        mem[61] = 32'b00000000000000000000111010110011;
        mem[62] = 32'b00000001100000000000000011101111;
        mem[63] = 32'b00000000000000000000000000010011;
        mem[64] = 32'b00000000000000000000000000010011;
        mem[65] = 32'b00000010011100101000100001100011;
        mem[66] = 32'b00000000000000000000000000010011;
        mem[67] = 32'b00000000000000000000000000010011;
        mem[68] = 32'b00000000010100010010000000100011;
        mem[69] = 32'b00000000011000010000001000100011;
        mem[70] = 32'b00000000000000010010111010000011;
        mem[71] = 32'b00000000010000010000111010000011;
        mem[72] = 32'b00000000010000010100111010000011;
        mem[73] = 32'b00000000000000001000000001100111;
        mem[74] = 32'b00000000000000000000000000010011;
        mem[75] = 32'b00000000000000000000000000010011;
        mem[76] = 32'b00000000000000000000000010110011;
        mem[77] = 32'b00000000000000000000001110110011;

    end
    assign instruction = mem[PC];
endmodule


module Control_unit (
    input [6:0] instr_6_0,
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUop,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite
);

always @(instr_6_0) begin
    case (instr_6_0)
        7'b0000011 : begin
           //lw
        Branch = 0;
        MemRead = 1;
        MemtoReg = 1;
        ALUop = 2'b00;
        MemWrite = 0;
        ALUSrc = 1;
        RegWrite = 1; 
        end
        7'b0010011 : begin
        //addi andi slli srli
        Branch = 0;
        MemRead = 0;
        MemtoReg = 0;
        ALUop = 2'b00; //not sure
        MemWrite = 0;
        ALUSrc = 1;
        RegWrite = 1;    
        end

        7'b0100011: begin
        //sw sb
        Branch = 0;
        MemRead = 0;
        MemtoReg = 0; //don't care
        ALUop = 2'b00;
        MemWrite = 1;
        ALUSrc = 1;
        RegWrite = 0;    
        end

        7'b0110011 : begin
        //add,sub,and,or,sll,srl,sra
        Branch = 0;
        MemRead = 0;
        MemtoReg = 0;
        ALUop = 2'b10;
        MemWrite = 0;
        ALUSrc = 0;
        RegWrite = 1;    
        end

        7'b1100011 : begin
        //beq bne bge blt
        Branch = 1;
        MemRead = 0;
        MemtoReg = 0; 
        ALUop = 2'b01;
        MemWrite = 0;
        ALUSrc = 0;
        RegWrite = 0;
        end

        7'b1100111 : begin
        //jalr
        Branch = 1;
        MemRead = 0;
        MemtoReg = 0; 
        ALUop = 2'b00;
        MemWrite = 0;
        ALUSrc = 1;
        RegWrite = 1;
        end
        
        7'b1101111 : begin
        //jal
        Branch = 1;
        MemRead = 0;
        MemtoReg = 0; 
        ALUop = 2'b11;
        MemWrite = 0;
        ALUSrc = 1;
        RegWrite = 1;
        end
        
        default: begin
        Branch = 0;
        MemRead = 0;
        MemtoReg = 0; 
        ALUop = 2'b00;
        MemWrite = 0;
        ALUSrc = 0;
        RegWrite = 0;
        end
        
    endcase

end
endmodule


module Control_unit2
(
    input [6:0] instr_6_0,
    input i_30,
    input [2:0] funct3,
    output reg IsJalr,
    output reg JumpSrc,
    output reg AsByte,
    output reg AsUnsigned
);
always @(*) begin

    if (instr_6_0 == 7'b1100111 && funct3 == 3'b000) begin
        //jalr 
        IsJalr = 1;
        JumpSrc = 1;
        AsByte = 0;
        AsUnsigned =0;
    end else if (instr_6_0 == 7'b1101111) begin
        IsJalr = 0;
        JumpSrc = 1;
        AsByte = 0;
        AsUnsigned =0;
    end else if ((instr_6_0 == 7'b0000011 && funct3 == 3'b000) || (instr_6_0 == 7'b0100011 && funct3 == 3'b000)) begin
        // lb sb
        IsJalr = 0;
        JumpSrc = 0;
        AsByte = 1;
        AsUnsigned =0;
    end else if (instr_6_0 == 7'b0000011 && funct3 == 3'b100) begin
        //lbu
        IsJalr = 0;
        JumpSrc = 0;
        AsByte = 1;
        AsUnsigned =1;
    end else begin 
        //others
        IsJalr = 0;
        JumpSrc = 0;
        AsByte = 0;
        AsUnsigned =0;
    end
end
endmodule


module Reg_file (
    input clk,
    input [4:0] R_reg1,R_reg2,W_reg,
    input [31:0] W_data,
    input Reg_write,
    output [31:0] R_data1,R_data2
);

reg [31:0] mem [31:0]; // memory

// Initial
integer i;
initial begin
    for (i = 0; i < 32; i = i + 1) begin
        mem[i] <= 32'b0;
    end
end

always @(negedge clk) begin
    if ( Reg_write && (W_reg > 0))
        mem[W_reg] = W_data;
end

assign R_data1 = mem[R_reg1];
assign R_data2 = mem[R_reg2];
    
endmodule

   

module Immediate_generator
 (
    input [31:0] instruction,
    output reg [31:0] immediate

);
    always @(*)
        case (instruction[6:0])
            // R-type
            7'b0110011: immediate = 'bz;
            // I-type
            7'b0000011: immediate = {{20{instruction[31]}}, instruction[31:20]}; 
            7'b0001111: immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b0010011: immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b1100111: immediate = {{20{instruction[31]}}, instruction[31:20]};
            7'b1110011: immediate = {{20{instruction[31]}}, instruction[31:20]};
            // S-type
            7'b0100011: immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            // B-type
            7'b1100011: immediate = {{20{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
            // U-type
            7'b0010111: immediate = {{12{instruction[31]}}, instruction[31:12]};
            7'b0110111: immediate = {{12{instruction[31]}}, instruction[31:12]};
            // J-type
            7'b1101111: immediate = {{12{instruction[31]}}, instruction[31], instruction[19:12], 
                instruction[20], 
                instruction[30:21]};
            default: immediate = 'bz;
        endcase
endmodule

module Mux_32bit (
    input [31:0] in1,
    input [31:0] in0,
    input select,
    output [31:0] result
);
assign result = (select)? in1 : in0;

endmodule



module ALU(
    input [3:0] ALU_control,
    input [31:0] R_data1,
    input [31:0] R_data2_or_imme,
    output reg Zero,
    output reg [31:0] Results
);

always @(*)begin
    Zero = 0; // initialize the zero
    case (ALU_control)
        4'b0010 : begin //add or lw or sw or addi
            Results =  R_data1 + R_data2_or_imme;
            if(Results == 0) Zero = 1;
        end
        4'b0110 : begin //beq or sub
            Results =  R_data1 - R_data2_or_imme;
            if(Results == 0) Zero = 1;
        end
        4'b0000 :begin //AND
            Results = R_data1 & R_data2_or_imme;
        end
        4'b0001 :begin //OR
            Results = R_data1 | R_data2_or_imme;
        end
        4'b0011: begin //<<
            Results = R_data1 << R_data2_or_imme;
        end
        4'b0111: begin //>>
            Results = R_data1 >> R_data2_or_imme;
        end
        4'b1000: begin
            Results = $signed(R_data1) >>> R_data2_or_imme;
        end
        4'b1001 :begin //bne
            if(R_data1 != R_data2_or_imme) Zero = 1;
        end
        4'b1010 :begin //bge
            if($signed(R_data1) >= $signed(R_data2_or_imme)) Zero = 1;
        end
        4'b1011 :begin //blt
            if($signed(R_data1) < $signed(R_data2_or_imme)) Zero = 1;
        end
        4'b1111 :begin //jal or jalr
            Results =  R_data1 + R_data2_or_imme;
            Zero = 1;
        end
        default: begin //other types
            Results =  R_data1 + R_data2_or_imme;
            if(Results == 0) Zero = 1;
        end
    endcase
end
    
endmodule

module ALU_control (
    input [1:0] ALUop,
    input i_30,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] ALU_control
);

always @(*) begin
    case (ALUop)
        2'b00 : begin
            if (i_30 == 0 && funct3 == 3'b001) ALU_control = 4'b0011; //slli
            else if (i_30 == 0 && funct3 == 3'b101) ALU_control = 4'b0111; //srli
            else if (funct3 == 3'b111) ALU_control = 4'b0000; //andi
            else if (funct3 == 3'b000) ALU_control = 4'b1111; //jalr
            else ALU_control = 4'b0010; //load & store type & addi
        end
        2'b01 : begin
            if (funct3 == 3'b000) ALU_control = 4'b0110; //beq
            else if (funct3 == 3'b001) ALU_control = 4'b1001; //bne
            else if (funct3 == 3'b101) ALU_control = 4'b1010; //bge
            else if (funct3 == 3'b100) ALU_control = 4'b1011; //blt
        end
        2'b10 : begin
            if(i_30 == 0 && funct3 == 3'b000) ALU_control = 4'b0010; //add
            else if (i_30 == 1 && funct3 == 3'b000) ALU_control = 4'b0110; //sub
            else if (i_30 == 0 && funct3 == 3'b111) ALU_control = 4'b0000; //and
            else if (i_30 == 0 && funct3 == 3'b110) ALU_control = 4'b0001; //or
            else if (i_30 == 0 && funct3 == 3'b001) ALU_control = 4'b0011; //sll
            else if (funct7 == 7'b0000000 && funct3 == 3'b101) ALU_control = 4'b0111; //srl
            else if (funct7 == 7'b0100000 && funct3 == 3'b101) ALU_control = 4'b1000; //sra
        end
        2'b11: begin
            ALU_control = 4'b1111; //jal
        end
        default: ALU_control =  ALU_control;
    endcase
end
endmodule

module PCSrc_generator (
    input MEM_Branch,
    input MEM_Zero,
    output reg MEM_PCSrc
);
initial begin
    MEM_PCSrc = 1'b0;
end
always @(*) begin
    MEM_PCSrc = MEM_Branch && MEM_Zero;
end
    
endmodule


module Data_mem (
input clk,
    input MemWrite,
    input [31:0] Data_mem_addr,
    input [31:0] Write_data,
    input AsByte,
    input AsUnsigned,
    output [31:0] Read_data
);

reg [7:0] mem [214748:0];
wire [31:0] signed_value, unsigned_value, byte_value, word_value;

always @(posedge clk) begin
    if(MemWrite && (~ AsByte)) begin
        mem[Data_mem_addr]     = Write_data[7:0];
        mem[Data_mem_addr + 1] = Write_data[15:8];
        mem[Data_mem_addr + 2] = Write_data[23:16];
        mem[Data_mem_addr + 3] = Write_data[31:24];
    end else if (MemWrite) begin
        mem[Data_mem_addr] = Write_data[7:0];
    end
end

assign unsigned_value = {24'b0, mem[Data_mem_addr]};
assign signed_value = {{24{mem[Data_mem_addr][7]}}, mem[Data_mem_addr]};
assign word_value = {mem[Data_mem_addr + 3], mem[Data_mem_addr + 2], mem[Data_mem_addr + 1], mem[Data_mem_addr]};

assign byte_value = (AsUnsigned)? unsigned_value : signed_value;
assign Read_data = (AsByte)? byte_value : word_value;

// Mux_32bit uut (PC_imme, PC_4, select, PC_next);
// assign PC_next = (select)? PC_imme:PC_4 ;

endmodule




// Pipeline Reg

// IF/ID Reg

module IF_ID_Reg (
    input clk,
    input [31:0] IF_PC_4,
    input [31:0] IF_PC_cur,
    input [31:0] IF_instruction,
    output reg [31:0] ID_PC_4,
    output reg [31:0] ID_PC_cur,
    output reg [31:0] ID_instruction
);
initial begin
    ID_PC_4 = 0;
    ID_PC_cur = 0;
    ID_instruction = 0;
end
always @(posedge clk) begin
    ID_PC_4 = IF_PC_4;
    ID_PC_cur = IF_PC_cur;
    ID_instruction = IF_instruction;
end
endmodule




module ID_EX_Reg (
    input clk,
    // Control unit1
    input ID_Branch, 
    input ID_MemRead, 
    input ID_MemtoReg, 
    input ID_MemWrite, 
    input ID_ALUSrc, 
    input ID_RegWrite,
    input [1:0] ID_ALUop,
    output reg EX_Branch, 
    output reg EX_MemRead, 
    output reg EX_MemtoReg, 
    output reg EX_MemWrite, 
    output reg EX_ALUSrc, 
    output reg EX_RegWrite,
    output reg [1:0] EX_ALUop,
    // Control unit2
    input ID_IsJalr, 
    input ID_JumpSrc, 
    input ID_AsByte, 
    input ID_AsUnsigned,
    output reg EX_IsJalr, 
    output reg EX_JumpSrc, 
    output reg EX_AsByte, 
    output reg EX_AsUnsigned,
    //PC & PC_4
    input [31:0] ID_PC_4,
    input [31:0] ID_PC_cur,
    output reg [31:0] EX_PC_4,
    output reg [31:0] EX_PC_cur,
    //Reg file
    input [31:0] ID_R_data1, 
    input [31:0] ID_R_data2,
    output reg [31:0] EX_R_data1, 
    output reg [31:0] EX_R_data2,
    //Imme Gen
    input [31:0] ID_imme,
    output reg [31:0] EX_imme,
    //instruction
    input ID_instruction_30,
    input [2:0] ID_instruction_func3,
    input [6:0] ID_instruction_funct7,
    input [4:0] ID_W_reg,
    output reg EX_instruction_30,
    output reg [2:0] EX_instruction_func3,
    output reg [6:0] EX_instruction_funct7,
    output reg [4:0] EX_W_reg
);
initial begin
    EX_Branch = 0;
    EX_MemRead = 0;
    EX_MemtoReg = 0;
    EX_MemWrite = 0;
    EX_ALUSrc = 0;
    EX_RegWrite = 0;
    EX_ALUop = 0;
    EX_IsJalr = 0;
    EX_JumpSrc = 0;
    EX_AsByte = 0;
    EX_AsUnsigned = 0;
    EX_PC_4 = 0;
    EX_PC_cur = 0;
    EX_R_data1 = 0;
    EX_R_data2 = 0;
    EX_imme = 0;
    EX_instruction_30 = 0;
    EX_instruction_func3 = 0;
    EX_instruction_funct7 =0;
    EX_W_reg = 0;
end
always @(posedge clk) begin
    EX_Branch = ID_Branch;
    EX_MemRead = ID_MemRead;
    EX_MemtoReg = ID_MemtoReg;
    EX_MemWrite = ID_MemWrite;
    EX_ALUSrc = ID_ALUSrc;
    EX_RegWrite = ID_RegWrite;
    EX_ALUop = ID_ALUop;
    EX_IsJalr = ID_IsJalr;
    EX_JumpSrc = ID_JumpSrc;
    EX_AsByte = ID_AsByte;
    EX_AsUnsigned = ID_AsUnsigned;
    EX_PC_4 =ID_PC_4;
    EX_PC_cur = ID_PC_cur;
    EX_R_data1 = ID_R_data1;
    EX_R_data2 = ID_R_data2;
    EX_imme = ID_imme;
    EX_instruction_30 = ID_instruction_30;
    EX_instruction_func3 = ID_instruction_func3;
    EX_instruction_funct7 = ID_instruction_funct7;
    EX_W_reg = ID_W_reg;
end
endmodule



module EX_MEM_Reg (
    input clk,
    //Control unit1
    input EX_Branch, 
    input EX_MemRead, 
    input EX_MemtoReg, 
    input EX_MemWrite, 
    input EX_RegWrite,
    output reg MEM_Branch, 
    output reg MEM_MemRead, 
    output reg MEM_MemtoReg, 
    output reg MEM_MemWrite, 
    output reg MEM_RegWrite,
    //Control unit2
    input EX_IsJalr, 
    input EX_JumpSrc, 
    input EX_AsByte, 
    input EX_AsUnsigned,
    output reg MEM_IsJalr, 
    output reg MEM_JumpSrc, 
    output reg MEM_AsByte, 
    output reg MEM_AsUnsigned,
    //PC
    input [31:0] EX_PC_4,
    input [31:0] EX_PC_cur,
    output reg [31:0] MEM_PC_4,
    output reg [31:0] MEM_PC_cur,
    //ADD_SUM_PC
    input [31:0] EX_PC_Imme,
    output reg [31:0] MEM_PC_Imme,
    //ALU
    input EX_Zero, 
    input [31:0] EX_ALU_results,
    output reg MEM_Zero, 
    output reg [31:0] MEM_ALU_results,
    //Data for Data mem
    input [31:0] EX_R_data2,
    output reg [31:0] MEM_R_data2,
    //Instruction write reg
    input [4:0] EX_W_reg,
    output reg [4:0] MEM_W_reg
);

always @(posedge clk) begin
    MEM_Branch   = EX_Branch;
    MEM_MemRead  = EX_MemRead;
    MEM_MemtoReg = EX_MemtoReg;
    MEM_MemWrite = EX_MemWrite;
    MEM_RegWrite = EX_RegWrite;
    MEM_IsJalr   = EX_IsJalr;
    MEM_JumpSrc  = EX_JumpSrc;  
    MEM_AsByte   = EX_AsByte;    
    MEM_AsUnsigned = EX_AsUnsigned; 
    MEM_PC_4 = EX_PC_4;
    MEM_PC_cur = EX_PC_cur;
    MEM_PC_Imme = EX_PC_Imme;
    MEM_Zero = EX_Zero;
    MEM_ALU_results = EX_ALU_results;
    MEM_R_data2 = EX_R_data2;
    MEM_W_reg = EX_W_reg;
end
endmodule


module MEM_WB_Reg (
    input clk,
    input MEM_MemtoReg,
    input MEM_RegWrite,
    output reg WB_MemtoReg,
    output reg WB_RegWrite,
    // JumpSrc 
    input MEM_JumpSrc,
    output reg WB_JumpSrc,
    //PC 
    input [31:0] MEM_PC_4,
    output reg [31:0] WB_PC_4,
    //Data mem
    input [31:0] MEM_Data_mem_data,
    output reg [31:0] WB_Data_mem_data,
    //ALU results
    input [31:0] MEM_ALU_results,
    output reg [31:0] WB_ALU_results,
    //Instruction write reg
    input [4:0] MEM_W_reg,
    output reg [4:0] WB_W_reg
);
always @(posedge clk) begin
    WB_MemtoReg = MEM_MemtoReg;
    WB_RegWrite = MEM_RegWrite;
    WB_JumpSrc = MEM_JumpSrc;
    WB_PC_4 = MEM_PC_4;
    WB_Data_mem_data = MEM_Data_mem_data;
    WB_ALU_results = MEM_ALU_results;
    WB_W_reg = MEM_W_reg;
end
endmodule

module Adder_32bit (
input [31:0] In1,
input [31:0] In2,
output [31:0] out
);
assign out = In1 + In2;
endmodule


module Top (
    input clk
);


//
// IF stage
//


wire MEM_PCSrc;
wire [31:0] MEM_PC_jump;
wire [31:0] IF_PC_4;
wire [31:0] IF_PC_cur;
wire [31:0] IF_instruction;

// PC PC(clk, MEM_PCSrc, MEM_PC_jump, IF_PC_cur, IF_PC_4);

wire [31:0] IF_PC_next;
Mux_32bit IF_PC_MUX(MEM_PC_jump, IF_PC_4, MEM_PCSrc, IF_PC_next);
PC PC(clk, IF_PC_next, IF_PC_cur);
assign IF_PC_4 = IF_PC_cur+1;
instr_mem instr_mem(IF_PC_cur, IF_instruction);

// IF/ID pipeline register
IF_ID_Reg IF_ID_Reg(clk, IF_PC_4, IF_PC_cur, IF_instruction, ID_PC_4, ID_PC_cur, ID_instruction);


//
// ID stage
// 


wire [31:0] ID_PC_4;
wire [31:0] ID_PC_cur;
wire [31:0] ID_instruction;

wire ID_Branch, ID_MemRead, ID_MemtoReg, ID_MemWrite, ID_ALUSrc, ID_RegWrite;
wire [1:0] ID_ALUop;
Control_unit Control_unit(ID_instruction[6:0], ID_Branch, ID_MemRead, ID_MemtoReg, ID_ALUop, ID_MemWrite, ID_ALUSrc, ID_RegWrite);

wire ID_IsJalr, ID_JumpSrc, ID_AsByte, ID_AsUnsigned;
Control_unit2 Control_unit2(ID_instruction[6:0], ID_instruction[30], ID_instruction[14:12], ID_IsJalr, ID_JumpSrc, ID_AsByte, ID_AsUnsigned);

wire [4:0] WB_W_reg;
wire [31:0] WB_W_data;
wire WB_RegWrite;
wire [31:0] ID_R_data1, ID_R_data2;
Reg_file Reg_file(clk, ID_instruction[19:15], ID_instruction[24:20], WB_W_reg, WB_W_data, WB_RegWrite, ID_R_data1, ID_R_data2);

wire [31:0] ID_imme;
Immediate_generator Immediate_generator(ID_instruction, ID_imme);

// ID_EX_Reg
ID_EX_Reg ID_EX_Reg(clk, ID_Branch, ID_MemRead, ID_MemtoReg, ID_MemWrite, ID_ALUSrc, ID_RegWrite, ID_ALUop, 
EX_Branch, EX_MemRead, EX_MemtoReg, EX_MemWrite, EX_ALUSrc, EX_RegWrite, EX_ALUop, 
ID_IsJalr, ID_JumpSrc, ID_AsByte, ID_AsUnsigned, EX_IsJalr,  EX_JumpSrc,  EX_AsByte,  EX_AsUnsigned,
ID_PC_4, ID_PC_cur, EX_PC_4, EX_PC_cur,
ID_R_data1, ID_R_data2, EX_R_data1, EX_R_data2,
ID_imme, EX_imme,
ID_instruction[30], ID_instruction[14:12], ID_instruction[31:25], ID_instruction[11:7],  EX_instruction_30, EX_instruction_func3,  EX_instruction_funct7, EX_W_reg);


//
// EX  stage
// 


wire EX_Branch, EX_MemRead, EX_MemtoReg, EX_MemWrite, EX_ALUSrc, EX_RegWrite; wire [1:0] EX_ALUop;
wire EX_IsJalr, EX_JumpSrc, EX_AsByte, EX_AsUnsigned;
wire [31:0] EX_PC_4, EX_PC_cur;
wire [31:0] EX_R_data1, EX_R_data2;
wire [31:0] EX_imme;
wire EX_instruction_30; wire [2:0] EX_instruction_func3; wire [6:0] EX_instruction_funct7; wire [4:0] EX_W_reg;

//PC jump
wire [31:0] EX_PC_Imme;
//assign EX_PC_Imme = EX_PC_cur + ($signed(EX_imme) >> 1);
Adder_32bit Adder1(EX_PC_cur, $signed(EX_imme) >> 1,  EX_PC_Imme);
//ALU Control
wire [3:0] EX_ALU_control;
ALU_control ALU_control(EX_ALUop, EX_instruction_30, EX_instruction_func3, EX_instruction_funct7, EX_ALU_control);

//ALU & ALU_MUX
wire [31:0] EX_R_data2_or_imme;
wire EX_Zero;
wire [31:0] EX_ALU_results;
Mux_32bit ALU_MUX (EX_imme, EX_R_data2, EX_ALUSrc, EX_R_data2_or_imme);
ALU ALU(EX_ALU_control, EX_R_data1, EX_R_data2_or_imme, EX_Zero, EX_ALU_results);

//EX_MEM_Reg
EX_MEM_Reg EX_MEM_Reg(clk, EX_Branch, EX_MemRead, EX_MemtoReg, EX_MemWrite, EX_RegWrite, MEM_Branch, MEM_MemRead, MEM_MemtoReg, MEM_MemWrite, MEM_RegWrite,
EX_IsJalr,  EX_JumpSrc,  EX_AsByte,  EX_AsUnsigned, MEM_IsJalr, MEM_JumpSrc, MEM_AsByte, MEM_AsUnsigned,
EX_PC_4, EX_PC_cur, MEM_PC_4, MEM_PC_cur,
EX_PC_Imme, MEM_PC_Imme,
EX_Zero, EX_ALU_results, MEM_Zero, MEM_ALU_results,
EX_R_data2, MEM_R_data2,
EX_W_reg, MEM_W_reg);


//
// MEM  stage
// 

wire MEM_Branch, MEM_MemRead, MEM_MemtoReg, MEM_MemWrite, MEM_RegWrite;
wire MEM_IsJalr, MEM_JumpSrc, MEM_AsByte, MEM_AsUnsigned;
wire [31:0] MEM_PC_4,MEM_PC_cur;
wire [31:0] MEM_PC_Imme;
wire MEM_Zero; wire [31:0] MEM_ALU_results;
wire [31:0] MEM_R_data2;
wire [4:0] MEM_W_reg;

//Is jalr mux
Mux_32bit Is_Jalr_MUX(MEM_ALU_results, MEM_PC_Imme, MEM_IsJalr, MEM_PC_jump);

// assign Mem_PCSrc = MEM_Branch & MEM_Zero;
PCSrc_generator PCSrc_generator(MEM_Branch, MEM_Zero, MEM_PCSrc);

//data mem
wire [31:0] MEM_Data_mem_data;
Data_mem Data_mem(clk, MEM_MemWrite, MEM_ALU_results, MEM_R_data2, MEM_AsByte, MEM_AsUnsigned, MEM_Data_mem_data);

//MEM_WB_Reg
MEM_WB_Reg MEM_WB_Reg(clk, MEM_MemtoReg, MEM_RegWrite, WB_MemtoReg, WB_RegWrite, 
MEM_JumpSrc, WB_JumpSrc,
MEM_PC_4, WB_PC_4,
MEM_Data_mem_data, WB_Data_mem_data,
MEM_ALU_results, WB_ALU_results,
MEM_W_reg, WB_W_reg
);


//
// WB  stage
// 

wire WB_JumpSrc; wire [31:0] WB_PC_4;
wire [31:0] WB_Data_mem_data;
wire [31:0] WB_ALU_results;
wire [31:0] Temp_Data1;
wire WB_MemtoReg;

Mux_32bit MEM_to_Reg_MUX (WB_Data_mem_data, WB_ALU_results, WB_MemtoReg, Temp_Data1);
Mux_32bit PC_4_MUX (WB_PC_4, Temp_Data1, WB_JumpSrc, WB_W_data);

endmodule


module tb_Top;
reg clk = 0;

Top uut(
    .clk (clk)
    );

localparam CLK_PERIOD = 10;
always #(CLK_PERIOD/2) clk=~clk;

initial begin
    $dumpfile("tb_Top.vcd");
    $dumpvars(0, tb_Top);
end

initial begin
    //#0 clk<=1'b1;
    #5000 $finish;
end
                   
endmodule
`default_nettype wire






// module Mux_32bit (
//     input [31:0] in1,
//     input [31:0] in0,
//     input select,
//     output [31:0] result
// );
// assign result = (select)? in1 : in0;

// endmodule

