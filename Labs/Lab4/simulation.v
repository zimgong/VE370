`timescale  1ns/1ps

module tb_;
reg clk;
 
top cpu1 (
    .clk (clk)
);

initial begin
    #0 clk = 1;
    #10
    forever #10 clk=~clk;  
end

initial #5000 $stop;
endmodule
