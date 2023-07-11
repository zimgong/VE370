`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/11/12 15:39:53
// Design Name: 
// Module Name: pipe
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module adder(a,b,s);
input [31:0] a,b;
output [31:0] s;
reg [31:0] s=32'b0;
always @ (a or b) s<=a+b;
endmodule

module mux21(a,b,s1,out);
parameter N=32;
input [N-1:0] a,b;
output [N-1:0] out;
input s1;
reg [N-1:0] out;
always @ (a or b or s1) begin
if (s1==1'b0) out<=a;
else out<=b;
end
endmodule

module mux42(a,b,c,d,s1,out);
parameter N=32;
input [1:0] s1;
input [N-1:0] a,b,c,d;
output reg [N-1:0] out;
always @(*) begin
case (s1)
    2'b00: out=a;
    2'b01: out=b;
    2'b10: out=c;
    2'b11: out=d;
    default: out=0;
endcase
end
endmodule

module hazard_detection(
input [4:0] IF_ID_rs1,IF_ID_rs2,ID_EX_rd,EX_MEM_rd,
input ID_EX_MemRead,EX_MEM_MemRead,IF_ID_branch,IF_ID_MemWrite,ID_EX_RegWrite,
output reg PC_write=1'b1, IF_ID_write=1'b1, mux_sel=1'b1 
);
always @(*) begin 
    if (ID_EX_MemRead && !IF_ID_MemWrite && (ID_EX_rd == IF_ID_rs1 || ID_EX_rd == IF_ID_rs2)) begin //load use hazard
            PC_write=1'b0; 
            IF_ID_write=1'b0; 
            mux_sel=1'b0;
    end
    else if (IF_ID_branch && ID_EX_RegWrite && ID_EX_rd!=0 && (ID_EX_rd==IF_ID_rs1 || ID_EX_rd==IF_ID_rs2)) begin  // r + branch
            PC_write=1'b0; 
            IF_ID_write=1'b0; 
            mux_sel=1'b0;
    end    
    else if (IF_ID_branch && EX_MEM_MemRead && EX_MEM_rd!=0 && (EX_MEM_rd==IF_ID_rs1 || EX_MEM_rd==IF_ID_rs2)) begin  // lw + branch
            PC_write=1'b0; 
            IF_ID_write=1'b0; 
            mux_sel=1'b0;
    end
    else begin
        PC_write=1'b1; 
        IF_ID_write=1'b1; 
        mux_sel=1'b1;
    end
end
endmodule

module forwarding_unit(
input [4:0] ID_EX_rs1,ID_EX_rs2,IF_ID_rs1,IF_ID_rs2,EX_MEM_rs2,ID_EX_rd,MEM_WB_rd,EX_MEM_rd,
input MEM_WB_RegWrite,EX_MEM_RegWrite,EX_MEM_MemWrite,EX_MEM_MemRead,MEM_WB_MemRead,ID_EX_MemRead,ID_EX_MemWrite,ID_EX_RegWrite,IF_ID_branch,
output reg [1:0] forwardA=2'b0, forwardB=2'b0,
output reg forward1=0,forward2=0,MemSrc=0
);
always @(*) begin
    if (EX_MEM_RegWrite && EX_MEM_rd!=0 && EX_MEM_rd == ID_EX_rs1 && !EX_MEM_MemRead && !ID_EX_MemRead && !ID_EX_MemWrite) forwardA = 2'b10;//EX hazard
    else if (MEM_WB_RegWrite && MEM_WB_rd!=0 && MEM_WB_rd == ID_EX_rs1) forwardA = 2'b01;// MEM hazard
    else forwardA = 2'b00;
    
    if (EX_MEM_RegWrite && EX_MEM_rd!=0 && EX_MEM_rd == ID_EX_rs2 && !EX_MEM_MemRead && !ID_EX_MemRead && !ID_EX_MemWrite) forwardB = 2'b10;//EX hazard
    else if (MEM_WB_RegWrite && MEM_WB_rd!=0 && MEM_WB_rd == ID_EX_rs2) forwardB = 2'b01;// MEM hazard
    else forwardB = 2'b00;
    
    if (EX_MEM_MemWrite && MEM_WB_rd==EX_MEM_rs2) MemSrc=1'b1;//lw + sw
    else MemSrc=1'b0;

    if (IF_ID_branch && EX_MEM_RegWrite && EX_MEM_rd!=0 && EX_MEM_rd == IF_ID_rs1) forward1 = 1'b1;//R+branch
    else forward1 = 1'b0;
    
    if (IF_ID_branch && EX_MEM_RegWrite && EX_MEM_rd!=0 && EX_MEM_rd == IF_ID_rs2) forward2 = 1'b1;//R+branch
    else forward2 = 1'b0;
    end
endmodule

module Comparator (
input [6:0] opcode,
input [2:0] funct,
input [31:0] rs1, rs2,
output reg comp=1'b0
);
always @ (*) begin
    if (opcode==7'b1100011) begin
        case (funct)
            3'b0: comp=(rs1==rs2);//beq
            3'b001:comp=(rs1!=rs2);//bne
            3'b100: comp=(rs1<rs2);//blt
            3'b101: comp=!(rs1<rs2);//bge
            default: comp = 1'b1;
        endcase
    end
    end
endmodule

module Register_File(clock,RegWrite,W_data,R_addr1,R_addr2,W_addr,R_data1,R_data2);
parameter addr_width=5;
parameter width=32;
parameter number=2**addr_width;

output [width-1:0] R_data1,R_data2;
input [width-1:0] W_data;
input [addr_width-1:0] R_addr1,R_addr2,W_addr;
input RegWrite,clock;

reg [width-1:0] Register_File [number-1:0];
integer i;
initial begin
    for (i=0;i<32;i=i+1) Register_File[i] = 32'b0;
end
always@(*) begin
    if (RegWrite&&W_addr!=0) Register_File[W_addr]=W_data;
end
assign R_data1=Register_File[R_addr1];
assign R_data2=Register_File[R_addr2];
endmodule

module ALU(sel,rs1,rs2,result);
input [3:0] sel;
input [31:0] rs1,rs2;
output reg [31:0] result=0;
always @ (*) begin
case (sel)
    4'b0010: result=rs1+rs2; //add, jal, jalr
    4'b1000: result=rs1-rs2;//beq, sub 
    4'b1001: result=rs1-rs2;//bne                
    4'b1100: result=$signed(rs1)-$signed(rs2);//blt
    4'b1110: result=$signed(rs1)-$signed(rs2);//bge
    4'b0110: result=rs1|rs2; //or
    4'b0111: result=rs1&rs2; //and
    4'b0001: result=rs1<<rs2; //sll
    4'b0101: result=rs1>>rs2; //srl
    4'b1101: result=$signed(($signed(rs1))>>>rs2); //sra
    default: result=0;
endcase
end
endmodule

module ALU_Control (ALU_op,instruction,ALU_sel);
input [1:0] ALU_op;
input [3:0] instruction;
output reg [3:0] ALU_sel=0;
    always @ (*) begin
        case (ALU_op)
            2'b00:  begin
                        ALU_sel = 4'b0010; //add,jal,jalr
                    end
            2'b01:  begin
                        if (instruction[2:0] == 3'b101) ALU_sel = 4'b1110; //bge
                        else ALU_sel={1'b1, instruction[2:0]}; //beq,bne,blt
                    end
            2'b10:  begin
                        if (instruction == 0) ALU_sel = 4'b0010; //add
                        else ALU_sel=instruction; //sub,and,or,sll,srl,sra
                    end
            2'b11:  begin
                        if (instruction[2:0] == 0) ALU_sel = 4'b0010; //addi
                        else ALU_sel={1'b0, instruction[2:0]}; //andi,slli,srli
                    end
            default:ALU_sel = 4'b0000;
        endcase
    end
endmodule

module sign_extension(in,out);
    parameter in_len=12;
    parameter out_len=32;
    input [in_len-1:0] in;
    output [out_len-1:0] out;
    assign out={{(out_len-in_len){in[in_len-1]}},in};
endmodule

module imm_gen(num_in,num_out);
    input [31:0] num_in;
    output [31:0] num_out;
    wire [31:0] num_out_I,num_out_S,num_out_B,num_out_U,num_out_J;
    sign_extension uut1(num_in[31:20],num_out_I);
    sign_extension uut2({num_in[31:25],num_in[11:7]},num_out_S);
    sign_extension uut3({num_in[31],num_in[7],num_in[30:25],num_in[11:8]},num_out_B);
    sign_extension #(20,32) uut4(num_in[31:12],num_out_U);
    sign_extension #(20,32) uut5({num_in[31],num_in[19:12],num_in[20],num_in[30:21]},num_out_J);
    reg [31:0] num_out=32'b0;
    always @ (num_in) begin
        if (num_in[6:0]==7'b0000011 || num_in[6:0]==7'b0001111 || num_in[6:0]==7'b0010011 || num_in[6:0]==7'b1100111 || num_in[6:0]==7'b1110011) num_out=num_out_I;
        else if (num_in[6:0]==7'b0100011) num_out=num_out_S;
        else if (num_in[6:0]==7'b1100011) num_out=num_out_B;
        else if (num_in[6:0]==7'b0010111 || num_in[6:0]==7'b0110111) num_out=num_out_U;
        else if (num_in[6:0]==7'b1101111) num_out=num_out_J;
        else num_out=32'b0;
    end
endmodule

module Control_Unit (
input [6:0] opcode,
output reg [9:0] control_signal=0
//  control_signal={ALUOp,MemtoReg,Branch,MemRead,MemWrite,ALUSrc,Jump,RegWrite}
);
always @ (opcode) begin 
case (opcode)
7'b0000011: control_signal<=10'b0000010101;//load
7'b0010011: control_signal<=10'b1111000101;//addi,subi,...
7'b0100011: control_signal<=10'b0011001100;//store
7'b1100011: control_signal<=10'b0111100000;//branch
7'b1100111: control_signal<=10'b0001000111;//jalr
7'b1101111: control_signal<=10'b0001100111;//jal
7'b0110011: control_signal<=10'b1011000001;//R
default: control_signal<=10'b0; 
endcase
end
endmodule

module Instruction_Memory(addr,instruction_out);
input [31:0] addr;
output reg [31:0] instruction_out;
reg [31:0] Instruction [127:0];
initial begin
Instruction[0] <= 32'b00111001100100000000001100010011;
Instruction[1] <= 32'b00000000011000000010001000100011;
Instruction[2] <= 32'b00000000010000000000001010000011;
Instruction[3] <= 32'b00000000010100000010000000100011;
Instruction[4] <= 32'b00000010000000110000000001100011;
Instruction[5] <= 32'b00000000000000000010111000000011;
Instruction[6] <= 32'b00000001110000101001110001100011;
Instruction[7] <= 32'b00000001110000101000001110110011;
Instruction[8] <= 32'b00000001110000111111001100110011;
Instruction[9] <= 32'b00000000000000111111001100010011;
Instruction[10] <= 32'b01000000000000110000001010110011;
Instruction[11] <= 32'b00000000011000101101010001100011;
Instruction[12] <= 32'b00000000000000000000001110110011;
Instruction[13] <= 32'b00000000110000000000000011101111;
Instruction[14] <= 32'b00000001010000000000000011101111;
Instruction[15] <= 32'b00000000000000000000111000110011;
Instruction[16] <= 32'b00000000011111100110111000110011;
Instruction[17] <= 32'b00000000000000001000000001100111;
Instruction[18] <= 32'b00000100100000000000001100010011;
Instruction[19] <= 32'b00001010110000000000001010010011;
end   
always @ (*) begin
    instruction_out = Instruction[addr>>2];
end
endmodule

module Data_Memory(MemWrite,MemRead,funct3,addr,W_data,R_data);
input MemWrite,MemRead;
input [2:0] funct3;
input [31:0] addr,W_data;
output reg [31:0] R_data;
reg [7:0] Dmem[31:0];
    always @ (*) begin
        if (MemWrite) begin
            case (funct3)
                3'b010: begin //store word
                    Dmem[addr]=W_data[7:0];
                    Dmem[addr+1]=W_data[15:8]; 
                    Dmem[addr+2]=W_data[23:16]; 
                    Dmem[addr+3]=W_data[31:24]; 
                end
                3'b000: begin //store byte
                    Dmem[addr]=W_data[7:0];
                end
                default: Dmem[addr]=Dmem[addr];
            endcase
        end
        if (MemRead) begin
            case (funct3)
                3'b010: R_data={Dmem[addr+3], Dmem[addr+2], Dmem[addr+1], Dmem[addr]}; //load word
                3'b000: R_data={{24{Dmem[addr][7]}}, Dmem[addr]}; //load byte
                3'b100: R_data={{24{1'b0}}, Dmem[addr]}; //load byte unsigned
                default: R_data=R_data;
            endcase
        end
    end
endmodule

module IF_ID_Reg(clock,currentPC, nextPC, instruction, currentPC_out, nextPC_out, instruction_out,IF_Flush,IF_ID_Write);
input clock,IF_Flush,IF_ID_Write;
input [31:0] currentPC,nextPC,instruction;
output reg  [31:0] currentPC_out=0,nextPC_out=0,instruction_out=0;
always @ (posedge clock) begin
if (IF_Flush) begin
instruction_out=32'b0;
end
else if (IF_ID_Write) begin
currentPC_out<= currentPC;
nextPC_out<= nextPC;
instruction_out<=instruction;
end
end 
endmodule

module ID_EX_Reg(clock,RegWrite,MemtoReg,MemRead,MemWrite,Jump,ALUSrc,ALUOp,crntPC,nextPC,Reg1,Reg2,Reg1_addr,Reg2_addr,imm,Reg_rd,ALU_Instrct,RegWrite_out,MemtoReg_out,MemRead_out,MemWrite_out,Jump_out,ALUSrc_out,ALUOp_out,crntPC_out,nextPC_out,Reg1_out,Reg2_out,Reg1_addr_out,Reg2_addr_out,imm_out,Reg_rd_out,ALU_Instrct_out);
input clock,RegWrite,MemRead,MemWrite,Jump,ALUSrc;
input [1:0] MemtoReg,ALUOp;   
input [31:0] Reg1,Reg2,nextPC,crntPC,imm;
input [4:0] Reg_rd,Reg1_addr,Reg2_addr;
input [3:0] ALU_Instrct;
output reg RegWrite_out=0,MemRead_out=0,MemWrite_out=0,Jump_out=0,ALUSrc_out=0;   
output reg [1:0] MemtoReg_out=0,ALUOp_out=0;  
output reg [31:0] Reg1_out=0,Reg2_out=0,crntPC_out=0,nextPC_out=0;
output reg [31:0] imm_out=0;
output reg [4:0] Reg_rd_out=0,Reg1_addr_out=0,Reg2_addr_out=0;
output reg [3:0] ALU_Instrct_out=0;

always @ (posedge clock) begin
RegWrite_out <= RegWrite;
MemtoReg_out<= MemtoReg;
MemRead_out <= MemRead;
MemWrite_out<= MemWrite;
Jump_out<= Jump;
ALUSrc_out<= ALUSrc;
ALUOp_out <= ALUOp;
crntPC_out<= crntPC;
nextPC_out<= nextPC;
Reg1_out<= Reg1;
Reg2_out<= Reg2;
Reg_rd_out<= Reg_rd;
imm_out<= imm;
ALU_Instrct_out <= ALU_Instrct;
Reg1_addr_out<=Reg1_addr;
Reg2_addr_out<=Reg2_addr;
end
endmodule

module EX_MEM_Reg(clock,RegWrite,MemtoReg,MemRead,MemWrite,imm,nextPC,ALUResult,Reg_rs2,Reg_rs2_addr,funct3,Reg_rd,RegWrite_out,MemtoReg_out,MemRead_out,MemWrite_out,imm_out,nextPC_out,ALUResult_out,Reg_rs2_out,Reg_rs2_addr_out,funct3_out,Reg_rd_out);
input clock,RegWrite,MemRead,MemWrite;
input [1:0] MemtoReg;
input [31:0] nextPC,ALUResult,Reg_rs2,imm;
input [2:0] funct3;
input [4:0] Reg_rd,Reg_rs2_addr;
output reg RegWrite_out=0,MemRead_out=0,MemWrite_out=0; 
output reg [1:0] MemtoReg_out;  
output reg [31:0] nextPC_out;
output reg [31:0]ALUResult_out,Reg_rs2_out,imm_out;
output reg [2:0] funct3_out;
output reg  [4:0] Reg_rd_out,Reg_rs2_addr_out;
always @ (posedge clock) begin
RegWrite_out <= RegWrite;
MemtoReg_out<= MemtoReg;
MemRead_out <= MemRead;
MemWrite_out<= MemWrite;
nextPC_out <= nextPC;
ALUResult_out <= ALUResult;
Reg_rs2_out<= Reg_rs2;
funct3_out<= funct3;
Reg_rd_out <= Reg_rd;
imm_out<=imm;
Reg_rs2_addr_out<=Reg_rs2_addr;
end
endmodule

module MEM_WB_Reg(clock,RegWrite,MemRead,MemtoReg,nextPC,imm,Read_data,ALUResult,Reg_rd,MemRead_out,RegWrite_out,MemtoReg_out,imm_out,nextPC_out,Read_data_out,ALUResult_out,Reg_rd_out);
input clock,RegWrite,MemRead;
input [1:0] MemtoReg; 
input [31:0] nextPC,Read_data,ALUResult,imm;
input [4:0] Reg_rd;
output reg RegWrite_out=0,MemRead_out=0;   
output reg [1:0] MemtoReg_out=0;   
output reg [31:0] nextPC_out=0,Read_data_out=0,ALUResult_out=0,imm_out=0;
output reg [4:0] Reg_rd_out=0;
always @ (posedge clock) begin
RegWrite_out<= RegWrite;
MemtoReg_out<= MemtoReg;
nextPC_out<= nextPC;
Read_data_out<= Read_data;
ALUResult_out<= ALUResult;
Reg_rd_out<= Reg_rd;
MemRead_out<=MemRead;
imm_out<=imm;
end
endmodule

module pipeline2(input clk);
wire [31:0] instruction, imm, imm2, rs1, rs2, alu_mux_result, nextPC, W_data, adder_result1, adder_result2, adder_result3,ALUResult, Read_Mem,ALU_in1, ALU_in2;
wire Branch, bc,jp, MemRead, MemWrite, ALUSrc,RegWrite, comp;
wire IF_Flush, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite,EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, MEM_WB_RegWrite, MEM_WB_MemRead,PC_write, IF_ID_write, mux_sel, MemSrc; 
wire [31:0] IF_ID_nextPC, IF_ID_currentPC, IF_ID_instruction;
wire [31:0] ID_EX_nextPC, ID_EX_currentPC,ID_EX_rs1, ID_EX_rs2, ID_EX_imm, EX_MEM_imm, MEM_WB_imm;
wire [4:0] ID_EX_Reg_rd, ID_EX_Rs1_addr, ID_EX_Rs2_addr, EX_MEM_Rs2_addr, EX_MEM_Reg_rd, MEM_WB_Reg_rd;
wire [31:0] EX_MEM_nextPC, EX_MEM_ALUResult, EX_MEM_rs2, mem_W_data;
wire [31:0] MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult;
wire [31:0] mux_result1, mux_result2; 
wire [3:0] ALUControl, ID_EX_ALUInstrct;
wire [2:0] EX_MEM_funct3;
wire [1:0] ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg,forwardA,forwardB; 
wire forward1,forward2;
wire [9:0] control_signal_in,control_signal_out;
reg [31:0] PC=0;

assign {ALUOp,MemtoReg,Branch,MemRead,MemWrite,ALUSrc,jp,RegWrite}=control_signal_out;
always @ (posedge clk) if (PC_write == 1'b1) PC <= nextPC;
and(bc, Branch, comp); 

adder uut1(PC,32'h00000004,adder_result1); //PC+4
//assign IF_ID_currentPC=adder_result2+imm2;//PC+imm
//assign rs1=adder_result3+imm;//addr+imm
adder uut7(IF_ID_currentPC,imm2,adder_result2); //PC+imm
adder uut8(rs1,imm,adder_result3); //addr+imm 

mux21 uut2(rs1,EX_MEM_ALUResult,forward1,mux_result1); 
mux21 uut3(rs2,EX_MEM_ALUResult,forward2,mux_result2);    
mux42 uut4(adder_result1,adder_result2,adder_result3,adder_result2,{jp,bc},nextPC);
mux42 uut5(MEM_WB_Read_Mem,MEM_WB_nextPC,MEM_WB_imm,MEM_WB_ALUResult,MEM_WB_MemtoReg,W_data);
mux21 #(10) uut6(10'b0,control_signal_in,mux_sel,control_signal_out); 

assign imm2 = {imm[30:0],1'b0}; 
hazard_detection uut13 (.IF_ID_rs1 (IF_ID_instruction[19:15]),.IF_ID_rs2 (IF_ID_instruction[24:20]),.ID_EX_rd (ID_EX_Reg_rd),.EX_MEM_rd (EX_MEM_Reg_rd),.ID_EX_MemRead (ID_EX_MemRead),.EX_MEM_MemRead (EX_MEM_MemRead),
.IF_ID_branch (control_signal_in[5]),.IF_ID_MemWrite (control_signal_in[3]),.ID_EX_RegWrite (ID_EX_RegWrite),.PC_write (PC_write),.IF_ID_write (IF_ID_write),.mux_sel (mux_sel));
    
forwarding_unit uut14 (.ID_EX_rs1 (ID_EX_Rs1_addr),.ID_EX_rs2 (ID_EX_Rs2_addr),.IF_ID_rs1 (IF_ID_instruction[19:15]),.IF_ID_rs2 (IF_ID_instruction[24:20]),.EX_MEM_rs2 (EX_MEM_Rs2_addr),.MEM_WB_rd (MEM_WB_Reg_rd),
.EX_MEM_rd (EX_MEM_Reg_rd),.ID_EX_rd (ID_EX_Reg_rd),.MEM_WB_RegWrite (MEM_WB_RegWrite),.EX_MEM_RegWrite (EX_MEM_RegWrite),.EX_MEM_MemWrite (EX_MEM_MemWrite),.MEM_WB_MemRead (MEM_WB_MemRead),.EX_MEM_MemRead (EX_MEM_MemRead),
.ID_EX_MemRead (ID_EX_MemRead),.ID_EX_MemWrite (ID_EX_MemWrite),.ID_EX_RegWrite (ID_EX_RegWrite),.IF_ID_branch (Branch),.forwardA (forwardA),.forwardB (forwardB),.forward1 (forward1),.forward2 (forward2),
.MemSrc (MemSrc));
    
Instruction_Memory uut15 (.instruction_out (instruction),.addr (PC));
Control_Unit uut16 (.opcode (IF_ID_instruction[6:0]),.control_signal (control_signal_in));
Register_File uut17 (.clock (clk),.RegWrite (MEM_WB_RegWrite),.W_data (W_data),.R_addr1 (IF_ID_instruction[19:15]),.R_addr2 (IF_ID_instruction[24:20]),.W_addr (MEM_WB_Reg_rd),.R_data1 (rs1),.R_data2 (rs2));
Comparator uut18 (.opcode(IF_ID_instruction[6:0]),.funct (IF_ID_instruction[14:12]),.rs1 (mux_result1),.rs2 (mux_result2),.comp (comp));
imm_gen uut19 (.num_in (IF_ID_instruction),.num_out (imm));
ALU uut20 (.rs1 (ALU_in1),.rs2 (ALU_in2),.sel (ALUControl),.result (ALUResult)); 
ALU_Control uut21(.ALU_op (ID_EX_ALUOp),.instruction (ID_EX_ALUInstrct),.ALU_sel (ALUControl));
Data_Memory uut22 (.MemWrite (EX_MEM_MemWrite),.MemRead (EX_MEM_MemRead),.funct3 (EX_MEM_funct3),.addr (EX_MEM_ALUResult),.W_data (mem_W_data),.R_data (Read_Mem));

or(IF_Flush, bc, jp);
IF_ID_Reg IF_ID(.clock (clk),.currentPC (PC),.nextPC (adder_result1),.instruction (instruction),.currentPC_out (IF_ID_currentPC),.nextPC_out (IF_ID_nextPC),.instruction_out (IF_ID_instruction),.IF_Flush(IF_Flush),.IF_ID_Write(IF_ID_write));

ID_EX_Reg ID_EX(.clock (clk),.RegWrite (RegWrite),.MemtoReg (MemtoReg),.MemRead (MemRead),.MemWrite (MemWrite),.Jump (jp),.ALUSrc (ALUSrc),.ALUOp (ALUOp),.crntPC (IF_ID_currentPC),
.nextPC (IF_ID_nextPC),.Reg1 (rs1),.Reg2 (rs2),.Reg1_addr(IF_ID_instruction[19:15]),.Reg2_addr(IF_ID_instruction[24:20]),.imm (imm),.Reg_rd (IF_ID_instruction[11:7]),.ALU_Instrct ({IF_ID_instruction[30],IF_ID_instruction[14:12]}),.RegWrite_out (ID_EX_RegWrite),
.MemtoReg_out (ID_EX_MemtoReg),.MemRead_out (ID_EX_MemRead),.MemWrite_out (ID_EX_MemWrite),.Jump_out (ID_EX_isJump),.ALUSrc_out (ID_EX_ALUSrc),.ALUOp_out (ID_EX_ALUOp),
.crntPC_out (ID_EX_currentPC),.nextPC_out (ID_EX_nextPC),.Reg1_out (ID_EX_rs1),.Reg2_out (ID_EX_rs2),.Reg1_addr_out(ID_EX_Rs1_addr),.Reg2_addr_out(ID_EX_Rs2_addr),.imm_out (ID_EX_imm),.Reg_rd_out (ID_EX_Reg_rd),.ALU_Instrct_out (ID_EX_ALUInstrct));

EX_MEM_Reg EX_MEM(.clock (clk),.RegWrite (ID_EX_RegWrite),.MemtoReg (ID_EX_MemtoReg),.MemRead (ID_EX_MemRead),.MemWrite (ID_EX_MemWrite),.nextPC (ID_EX_nextPC),.imm(ID_EX_imm),.Reg_rs2_addr(ID_EX_Rs2_addr),
.ALUResult (ALUResult),.Reg_rs2 (ID_EX_rs2),.funct3 (ID_EX_ALUInstrct[2:0]),.Reg_rd (ID_EX_Reg_rd),.RegWrite_out (EX_MEM_RegWrite),.MemtoReg_out (EX_MEM_MemtoReg),.MemRead_out (EX_MEM_MemRead),
.MemWrite_out (EX_MEM_MemWrite),.imm_out(EX_MEM_imm),.nextPC_out (EX_MEM_nextPC),.ALUResult_out (EX_MEM_ALUResult),.Reg_rs2_out (EX_MEM_rs2),.Reg_rs2_addr_out(EX_MEM_Rs2_addr),.funct3_out (EX_MEM_funct3),.Reg_rd_out (EX_MEM_Reg_rd));

MEM_WB_Reg MEM_WB(.clock (clk),.RegWrite (EX_MEM_RegWrite),.MemRead(EX_MEM_MemRead),.MemtoReg (EX_MEM_MemtoReg),.nextPC (EX_MEM_nextPC),.imm(EX_MEM_imm),.Read_data (Read_Mem),.ALUResult (EX_MEM_ALUResult),.Reg_rd (EX_MEM_Reg_rd),
.RegWrite_out (MEM_WB_RegWrite),.MemRead_out(MEM_WB_MemRead),.MemtoReg_out (MEM_WB_MemtoReg),.nextPC_out (MEM_WB_nextPC),.Read_data_out (MEM_WB_Read_Mem),.ALUResult_out (MEM_WB_ALUResult),.Reg_rd_out (MEM_WB_Reg_rd));

mux42 uut9(ID_EX_rs1,W_data,EX_MEM_ALUResult,0,forwardA,ALU_in1);
mux42 uut10(alu_mux_result,W_data,EX_MEM_ALUResult,0,forwardB,ALU_in2); 
mux21 uut11(EX_MEM_rs2,W_data,MemSrc,mem_W_data); 
mux21 uut12(ID_EX_rs2,ID_EX_imm,ID_EX_ALUSrc,alu_mux_result); 
endmodule
