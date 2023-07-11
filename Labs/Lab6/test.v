`timescale 1ns / 1ps

module top( input clock);
    wire CPU_rOw;
    wire cache_hit;
    wire [9:0]   CPU_address;
    wire [31:0]  cache_readData; 
    wire [31:0]  CPU_writeData;
    // interface between cache and main memory
    wire cache_rOw;
    wire [31:0]  cache_writeData;
    wire [31:0]  main_readData;
    wire [9:0]   cache_address;
    wire Done;
    // You may add the signal you need. However, you cannot change the signals above.

    Cache   Cache(.CPU_rOw(CPU_rOw), .CPU_address(CPU_address), .CPU_writeData(CPU_writeData), .cache_readData(cache_readData), .cache_hit(cache_hit), .cache_rOw(cache_rOw), .cache_address(cache_address), .cache_writeData(cache_writeData), .main_readData(main_readData));
    main_memory            mem_db(.cache_rOw(cache_rOw), .cache_address(cache_address), .cache_writeData(cache_writeData), .main_readData(main_readData));
    CPU                 CPU_db(.clock(clock), .CPU_rOw(CPU_rOw), .CPU_address(CPU_address), .CPU_writeData(CPU_writeData), .cache_readData(cache_readData), .cache_hit(cache_hit));
endmodule

module CacheTest;
    reg clock;

    parameter half_period = 10;
    integer t = 0;
    
    top test(clock);
    
    initial begin
        #0 clock = 0;
    end
    
    always #half_period begin
        clock = ~clock;
        t=t+1;
    end
    
    always @(posedge clock) begin
            $display("===============================================");
            $display("Clock cycle %d", t/2);
            $display("Read data = %H", test.cache_readData);
            $display("hit_miss = %d", test.cache_hit);
            $display("===============================================");
    end
    initial
        #1000 $stop;

endmodule