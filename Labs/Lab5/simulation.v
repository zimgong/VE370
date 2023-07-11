//`include "pipelined_cpu_hazard_resolved.v"

module tb_;
reg clk;
 
top test (
    .clk (clk)
);

    initial begin  
        while ($time < 120) @(posedge clk)begin
            $display("===============================================");
            $display("Clock cycle %d, PC = %H", $time/2-1, 4*test.IF_PC);
            $display("ra = %H, t0 = %H, t1 = %H", test.registerfile.registers[1], test.registerfile.registers[5], test.registerfile.registers[6]);
            $display("t2 = %H, t3 = %H, t4 = %H", test.registerfile.registers[7], test.registerfile.registers[28], test.registerfile.registers[29]);
            $display("===============================================");
        end
        $finish();
    end

    initial begin
        #0 clk = 0;
        #1
        forever #1 clk = ~clk;
    end

    initial #5000 $stop;
endmodule
