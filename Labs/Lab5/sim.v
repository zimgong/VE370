`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/11/12 10:54:42
// Design Name: 
// Module Name: sim
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


module sim;
reg clk;
pipeline2 test(clk);
//initial clk = 0;
//always #5 clk = ~clk;
//initial #300 $stop;

initial begin
clk=0;
        while ($time < 100) @(posedge clk)begin
            $display("===============================================");
            $display("Clock cycle %d, PC = %H", $time/2, test.PC);
            $display("ra = %H, t0 = %H, t1 = %H", test.uut17.Register_File[1], test.uut17.Register_File[5], test.uut17.Register_File[6]);
            $display("t2 = %H, t3 = %H, t4 = %H", test.uut17.Register_File[7], test.uut17.Register_File[28], test.uut17.Register_File[29]);
            $display("===============================================");
        end
        $finish();
    end
    always #1 clk = ~clk;
endmodule
