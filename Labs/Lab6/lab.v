`timescale 1ns / 1ps

module Cache (input CPU_rOw, input [9:0] CPU_address, input [31:0] CPU_writeData, output reg [31:0] cache_readData, output cache_hit, 
              output reg [1:0] cache_rOw, output reg [9:0] cache_address, output reg [31:0] cache_writeData, input [31:0] main_readData, input main_processing);
// name: source + name
// rOw: read = 1, write = 0

    reg valid [3:0];
    reg dirty [3:0];
    reg [3:0] tag [3:0];
    reg [127:0] cache [3:0];

    wire [3:0] tagNum;
    wire [5:0] blockNum;
    wire [1:0] blockIndex;
    wire [1:0] wordIndex;
    wire [3:0] byteIndex;
    assign tagNum = CPU_address[9:6];
    assign blockNum = CPU_address[9:4];
    assign blockIndex = CPU_address[5:4];
    assign wordIndex = CPU_address[3:2];
    assign byteIndex = CPU_address[3:0];
    
    assign cache_hit = ((valid[blockIndex] == 1) && (tag[blockIndex] == tagNum));

    initial begin 
        cache_rOw = 'bz; 
        valid[0] = 1'b0; valid[1] = 1'b0; valid[2] = 1'b0; valid[3] = 1'b0;
        dirty[0] = 1'b0; dirty[1] = 1'b0; dirty[2] = 1'b0; dirty[3] = 1'b0;
        tag[0]   = 4'b0; tag[1]   = 4'b0; tag[2]   = 4'b0; tag[3]   = 4'b0; 
    end

    always @(*) begin
        // read
        if (CPU_rOw == 0) begin
            if (cache_hit == 0) begin
                if (dirty[blockIndex] == 1) begin
                // write previous data to main memory
                    cache_rOw = 1;
                    #1 /* word 00 */ cache_address = {tag[blockIndex], blockIndex, 4'b0000}; cache_writeData = cache[blockIndex][31:0];
                    #1 /* word 01 */ cache_address = {tag[blockIndex], blockIndex, 4'b0100}; cache_writeData = cache[blockIndex][63:32];
                    #1 /* word 10 */ cache_address = {tag[blockIndex], blockIndex, 4'b1000}; cache_writeData = cache[blockIndex][95:64];
                    #1 /* word 11 */ cache_address = {tag[blockIndex], blockIndex, 4'b1100}; cache_writeData = cache[blockIndex][127:96];
                    #1 cache_rOw = 'bz;
                end
                // bring data from main memory
                #1 tag[blockIndex] = tagNum; valid[blockIndex] = 1;
                cache_rOw = 0;
                     /* word 00 */ cache_address = {blockNum, 4'b0000}; #0.1 cache[blockIndex][31:0] = main_readData;
                #0.9 /* word 01 */ cache_address = {blockNum, 4'b0100}; #0.1 cache[blockIndex][63:32] = main_readData;
                #0.9 /* word 10 */ cache_address = {blockNum, 4'b1000}; #0.1 cache[blockIndex][95:64] = main_readData;
                #0.9 /* word 11 */ cache_address = {blockNum, 4'b1100}; #0.1 cache[blockIndex][127:96] = main_readData;
                #0.9 cache_rOw = 'bz;
                // mark cache as not dirty
                dirty[blockIndex] = 1;
                cache_address = 10'bz;
            end
            // return read data
            cache_readData = $signed(cache[blockIndex][{byteIndex, 3'b111} -: 8]);
        end
        // write
        if (CPU_rOw == 1) begin
            #0.1 cache_readData = 32'bz;
            if (cache_hit == 0) begin
                if (dirty[blockIndex] == 1) begin
                // write previous data to main memory
                    cache_rOw = 1;
                    #1 /* word 00 */ cache_address = {tag[blockIndex], blockIndex, 4'b0000}; cache_writeData = cache[blockIndex][31:0];
                    #1 /* word 01 */ cache_address = {tag[blockIndex], blockIndex, 4'b0100}; cache_writeData = cache[blockIndex][63:32];
                    #1 /* word 10 */ cache_address = {tag[blockIndex], blockIndex, 4'b1000}; cache_writeData = cache[blockIndex][95:64];
                    #1 /* word 11 */ cache_address = {tag[blockIndex], blockIndex, 4'b1100}; cache_writeData = cache[blockIndex][127:96];
                    #1 cache_rOw = 'bz; cache_address = 10'bz;
                end
                // bring data from main memory
                #1 tag[blockIndex] = tagNum; valid[blockIndex] = 1;
                cache_rOw = 0;
                     /* word 00 */ cache_address = {blockNum, 4'b0000}; #0.1 cache[blockIndex][31:0] = main_readData;
                #0.9 /* word 01 */ cache_address = {blockNum, 4'b0100}; #0.1 cache[blockIndex][63:32] = main_readData;
                #0.9 /* word 10 */ cache_address = {blockNum, 4'b1000}; #0.1 cache[blockIndex][95:64] = main_readData;
                #0.9 /* word 11 */ cache_address = {blockNum, 4'b1100}; #0.1 cache[blockIndex][127:96] = main_readData;
                #0.9 cache_rOw = 'bz; cache_address = 10'bz;
            end
            // write data into cache
            cache[blockIndex][{wordIndex, 5'b11111} -: 32] = CPU_writeData;
            // mark cache as dirty
            dirty[blockIndex] = 1;
            cache_address = 10'bz;
        end
    end

endmodule

module main_memory(input cache_rOw, input [9:0] cache_address, input [31:0] cache_writeData, output reg [31:0] main_readData);
    reg [31:0] main [255:0];

    initial begin
        $readmemb("E:/Semesters/FA22/VE370/Labs/Lab6/mem.txt", main);
    end

    always @(*) begin
        case(cache_rOw)
            1'b0: main_readData = main[cache_address[9:2]];
            1'b1: main[cache_address[9:2]] = cache_writeData;
            default: main_readData = 32'bz;
        endcase
    end

endmodule

module CPU (input cache_hit, input clock, input [31:0] cache_readData, output CPU_rOw, output [9:0] CPU_address, output [31:0] CPU_writeData);
    parameter  request_total = 12; 
    reg [4:0]  request_num;
    reg        read_write_test[request_total-1:0];
    reg [9:0]  address_test[request_total-1:0];
    reg [31:0] write_data_test[request_total-1:0]; 
    
    initial begin
        #10 request_num = 0;
        read_write_test[0] = 1; address_test[0] = 10'b0110101000; write_data_test[0] = 10'h3ab;             // write 6a() with 000003ab
        read_write_test[1] = 0; address_test[1] = 10'b0110101000; write_data_test[1] = 0;                   // read from 6a() [ffffffab]
        read_write_test[2] = 1; address_test[2] = 10'b0110101000; write_data_test[2] = 10'h3ac;             // write 6a() with 000003ac
        read_write_test[3] = 0; address_test[3] = 10'b0110101000; write_data_test[3] = 0;                   // read from 6a()
        read_write_test[4] = 0; address_test[4] = 10'b0100001000; write_data_test[4] = 0;                   // read from 42()
        read_write_test[5] = 0; address_test[5] = 10'b0100101000; write_data_test[5] = 0;                   // read from 4a()
        read_write_test[6] = 0; address_test[6] = 10'b0110101000; write_data_test[6] = 0;                   // read from 6a()
        read_write_test[7] = 1; address_test[7] = 10'b0110101000; write_data_test[7] = 10'h3ad;             // write 6a() with 000003ad
        read_write_test[8] = 1; address_test[8] = 10'b0101101000; write_data_test[8] = 10'h3ae;             // write 5a() with 000003ae
        read_write_test[9] = 0; address_test[9] = 10'b0101101000; write_data_test[9] = 0;                   // read from 5a()
        read_write_test[10] = 0; address_test[10] = 10'b0110101000; write_data_test[10] = 0;                // read from 6a() [000003ad]
        read_write_test[11] = 0; address_test[11] = 10'b0110101001; write_data_test[11] = 0;                // read from 6a.01 [00000003]         
    end
    
    always @(posedge clock) begin
        if (cache_hit == 1) request_num = request_num + 1;
        if (cache_hit == 0) request_num = request_num;
    end
    
    assign CPU_address   = address_test[request_num];
    assign CPU_rOw       = read_write_test[request_num];
    assign CPU_writeData = write_data_test[request_num];
    
endmodule