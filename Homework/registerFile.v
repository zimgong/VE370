module registerFile (readRegister1, readRegister2, writeRegister, writeData, regWrite, readData1, readData2, clock);
    parameter width = 32;
    parameter addr_width = 5;
    parameter number = 2**addr_width;

    output [width-1:0]      readData1, readData2;
    input  [width-1:0]      writeData;
    input  [addr_width-1:0] readRegister1, readRegister2, writeRegister;
    input                   regWrite, clock;

    reg [width-1:0] readData1, readData2;
    reg [width-1:0] memory [number-1:0];

    always @(posedge clock) begin
        readData1 = 'bz;
        readData2 = 'bz;
        if (regWrite) memory[writeRegister] = writeData;
        else 
            readData1 = memory[readRegister1];
            readData2 = memory[readRegister2];      
    end
endmodule

module tb_;
parameter width = 32;
parameter addr_width = 5;

wire [width-1:0]      readData1, readData2;
reg  [width-1:0]      writeData;
reg  [addr_width-1:0] readRegister1, readRegister2, writeRegister;
reg                   regWrite, clock;
 
registerFile RF (
    .readData1 (readData1),
    .readData2 (readData2),
    .writeData (writeData),
    .readRegister1 (readRegister1),
    .readRegister2 (readRegister2),
    .writeRegister (writeRegister),
    .regWrite (regWrite),
    .clock (clock)
);

localparam CLK_PERIOD = 100;
always #(CLK_PERIOD/2) clock=~clock;

initial begin
    $dumpfile("tb_.vcd");
    $dumpvars(0, tb_);
end

initial begin
    #0 clock = 0; regWrite = 1; readRegister1 = 0; readRegister2 = 0; writeRegister = 20; writeData = 32'b10101010101010101010101010101010;  
    #100 regWrite = 0;
    #100 regWrite = 1; writeRegister = 21; writeData = 32'b01010101010101010101010101010101;
    #100 regWrite = 0;
    #100 readRegister1 = 20; readRegister2 = 21; writeRegister = 9; 
    #100 writeData = readData1 + readData2; regWrite = 1;
    #100 regWrite = 0;
    #100 readRegister1 = 9;
    #100 $finish;
end
endmodule