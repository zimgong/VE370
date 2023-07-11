module immGen (instruction, extend);
    parameter width = 32;
    output reg [width-1:0] extend;
    input  [width-1:0] instruction;
    always @(instruction) begin
       case (instruction[6:0])
        7'b0000011: // I with sign
            if (instruction[14] | !instruction[31]) extend = instruction[31:20];
            else extend = 32'b11111111111111111111000000000000 + instruction[31:20];
        7'b0001111: // I
            if (!instruction[31]) extend = instruction[31:20];
            else extend = 32'b11111111111111111111000000000000 + instruction[31:20];
        7'b0010011: // I with sign
            if (instruction[14:12] == 3'b011 | !instruction[31]) extend = instruction[31:20];
            else extend = 32'b11111111111111111111000000000000 + instruction[31:20];
        7'b0010111: // U
            if (!instruction[31]) extend = instruction[31:12];
            else extend = 32'b11111111111100000000000000000000 + instruction[31:12];
        7'b0100011: // S
            if (!instruction[31]) extend = instruction[11:7] + instruction[31:25]*2**5;
            else extend = 32'b11111111111111111111000000000000 + instruction[11:7] + instruction[31:25]*2**5;
        7'b0110111: // U
            if (instruction[14:12] == 3'b011 | !instruction[31]) extend = instruction[31:12];
            else extend = 32'b11111111111100000000000000000000 + instruction[31:12];
        7'b1100011: // B with sign
            if (!instruction[31] | instruction[14:13] == 2'b11) extend = instruction[31]*2**11 + instruction[7]*2**10 + instruction[30:25]*2**4 + instruction[11:8];
            else extend = 32'b11111111111111111111000000000000 + instruction[31]*2**11 + instruction[7]*2**10 + instruction[30:25]*2**4 + instruction[11:8];
        7'b1100111: // I
            if (!instruction[31]) extend = instruction[31:20];
            else extend = 32'b11111111111111111111000000000000 + instruction[31:20];
        7'b1101111: // J
            if (!instruction[31]) extend = instruction[31]*2**19 + instruction[19:12]*2**11 + instruction[20]*2**10 + instruction[30:21];
            else extend = 32'b11111111111100000000000000000000 +  instruction[31]*2**19 + instruction[19:12]*2**11 + instruction[20]*2**10 + instruction[30:21];
        7'b1110011: // I
            if (!instruction[31]) extend = instruction[31:20];
            else extend = 32'b11111111111111111111000000000000 + instruction[31:20];
        default: extend = 'bz;
       endcase 
    end
endmodule

module tb_;
parameter width = 32;

wire [width-1:0]      extend;
reg  [width-1:0]      instruction;
 
immGen IG (
    .extend (extend),
    .instruction (instruction)
);

initial begin
    $dumpfile("tb_.vcd");
    $dumpvars(0, tb_);
end

initial begin
    #0   instruction = 32'b00000000010010100000010010010011;  
    #100 instruction = 32'b11111111110010100010010010000011;
    #100 instruction = 32'b11111110100110100010111000100011;
    #100 instruction = 32'b11111111010110100000110011100011;
    #100 instruction = 32'b00010010001101000101101000110111;
    #100 instruction = 32'b00000001110000000000000011101111;
    #100 $finish;
end
endmodule