`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
//           Joseph Callenes
//           
// 
// Create Date: 02/06/2020 06:40:37 PM
// Design Name: 
// Module Name: dcache
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

package cache_def;

parameter int TAG_MSB = 31;
parameter int TAG_LSB = 12;

typedef struct packed{
    logic valid;
    logic dirty;
    logic [TAG_MSB:TAG_LSB] tag;
}cache_tag_type;

typedef struct {
    logic [9:0] index;
    logic we;
}cache_req_type;

//128-bit cache line
typedef logic [127:0] cache_data_type;

//CPU request (CPU ->cache controller)
typedef struct{
    logic [31:0] addr;
    logic [31:0] data;
    logic rw;
    logic valid;
}cpu_req_type;

//Cache result (cache controller -> CPU)
typedef struct {
    logic [31:0]data;
    logic ready;
}cpu_result_type;

//memory request (cache controller -> memory)
typedef struct {
    logic [31:0]addr;
    logic [127:0]data;
    logic rw;
    logic valid;
}mem_req_type;

//memory controller response (memory -> cache controller)
typedef struct {
cache_data_type data;
logic ready;
}mem_data_type;

endpackage

import cache_def::*;
import memory_bus_sizes::*; 

module L1_cache_data ( 
    input clk,
    input cache_req_type data_req,
    input cache_data_type data_write,
    input [3:0] be,
    input [1:0] block_offset,
    input from_ram,
    output cache_data_type data_read);
    
    cache_data_type data_mem[0:255];
    
    initial begin
        for(int i=0; i<256; i++)
            data_mem[i]='0;
    end

    always_ff @(posedge clk) begin
        if(data_req.we) begin
            if(from_ram) 
                data_mem[data_req.index] <= data_write;
            if(!from_ram) begin
              for (int b = 0; b < WORD_SIZE; b++) begin
                if (be[b]) begin
                    data_mem[data_req.index][block_offset*WORD_WIDTH+b*8+:8] <= data_write[block_offset*WORD_WIDTH+b*8+:8];  //[b*8+:8];
                end
              end
            end
        end
    end
    
    always_comb begin //asynch read
        data_read = data_mem[data_req.index];
    end
        
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req,
    input cache_tag_type tag_write,
    output cache_tag_type tag_read);

	cache_tag_type tag_mem[0:255];
    
    initial begin
        for(int i=0; i<256; i++) begin
            tag_mem[i].valid = 0;
            tag_mem[i].dirty = 0;
        end
    end
    
    always_comb //read (asynchronous)
        tag_read = tag_mem[tag_req.index];
    
    always_ff @(posedge clk) begin
        if (tag_req.we) begin
            tag_mem[tag_req.index].tag <= tag_write.tag;
            tag_mem[tag_req.index].valid <= tag_write.valid;
            tag_mem[tag_req.index].dirty <= tag_write.dirty;
        end
    end
	
endmodule


module dcache(
    input clk, RESET,
    i_mhub_to_dcache.device mhub,
    i_dcache_to_ram.controller ram
    );

    cpu_req_type cpu_req;     //CPU->cache
    mem_data_type mem_data;   //memory->cache
    
    mem_req_type mem_req;    //cache->memory
    cpu_result_type cpu_res;  //cache->CPU
    
    logic [1:0] block_offset, saved_bo;
    logic [3:0] be, saved_be;
    logic miss;
    logic from_ram;
    logic wait_read, next_wait_read; //Delay for when memory is outputting data to cache
    logic data_being_read; //Delay for when cache is outputting data to OTTER/Memory Hub
    logic [1:0] cache_update; //Delay for when cache has to allocate and then update for write
    
    assign be = mhub.be;
    assign block_offset = mhub.waddr[3:2];
    assign data_blk_msb = ((31 * (block_offset + 1)) + block_offset);
    assign data_blk_lsb = 32 * block_offset;
   
    assign cpu_req.addr[ADDR_WIDTH-1:WORD_ADDR_LSB] = mhub.waddr;
    assign cpu_req.data = mhub.din;
    assign cpu_req.rw = mhub.we;
    assign cpu_req.valid = mhub.en;
     
    assign mhub.dout = cpu_res.data;            //async read
    assign mhub.hold = ~cpu_res.ready; 	 //Inverse     //ready goes high before edge that data is available
    
    assign ram.baddr = mem_req.addr[ADDR_WIDTH-1:BLOCK_ADDR_LSB];
    assign ram.din = mem_req.data;
    assign ram.we = mem_req.rw;
    assign ram.en = mem_req.valid;
   
    assign mem_data.data = ram.dout;
    assign mem_data.ready = ~ram.hold;       //Inverse
    
    typedef enum {idle, compare_tag, allocate, writeback} cache_state_type;
   
    cache_state_type state, next_state;

    cache_tag_type tag_read;
    cache_tag_type tag_write;
    cache_req_type tag_req;
    
    cache_data_type data_read;
    cache_data_type data_write;
    cache_req_type data_req;
    
    cpu_result_type next_cpu_res;
    
	//FSM for Cache Controller
    L1_cache_tag L1_tags(.clk(clk), .tag_req(tag_req), .tag_write(tag_write), .tag_read(tag_read));
    L1_cache_data L1_data(.clk(clk), .data_req(data_req), .data_write(data_write), .be(saved_be),
                          .block_offset(saved_bo), .from_ram(from_ram), .data_read(data_read));
    
    always_ff @ (posedge clk) begin
        if (RESET)
            state <= idle; //beginning of program
        else
            state <= next_state;  
        wait_read <= next_wait_read;
    end
    
    initial begin //to avoid warnings
        mem_req.valid = 0;
        mem_req.rw = 0;
    end
    
//=============Combinational Aspect of FSM====================//
    always_comb
        if (state == idle) begin
            mem_req.addr[31:4] = mhub.waddr[31:4];
            mem_req.addr[3:0] = 4'd0; //to avoid warnings
            if (cpu_req.valid) begin //request from OTTER received
                if (data_being_read) begin //layover from previous cache access
                    next_state = idle;
                    cpu_res.ready = 1;
                end
                else begin //actual request from OTTER
                    next_state = compare_tag;
                    cpu_res.ready = 0;
                end
            end
            else begin
                next_state = idle;
                cpu_res.ready = 1;
            end
            miss = 0;
            next_wait_read = 0;
            data_being_read = 0;
        end
        //============================================================================//
        else if (state == compare_tag) begin
            if (tag_read.valid && (tag_read.tag == mhub.waddr[31:12])) begin //hit
                next_state = idle;
                next_wait_read = 0;
                mem_req.addr[31:4] = mhub.waddr[31:4];
                mem_req.addr[3:0] = 4'd0;
                data_being_read = 1;
            end
            else begin
                miss = 1;
                data_being_read = 0;
                if (tag_read.dirty) begin //need to write to memory before reading
                    next_state = writeback;
                    next_wait_read = 0;
                    mem_req.addr[31:12] = tag_read.tag;
                    mem_req.addr[11:0] = 12'd0;
                end
                else begin
                    next_state = allocate;
                    next_wait_read = 1;
                    mem_req.addr[31:12] = mhub.waddr[31:12];
                    mem_req.addr[11:0] = 12'd0;
                end
            end
            cpu_res.ready = 0; //keep hold on
        end
        //=============================================================//
        else if (state == allocate) begin //Allocate
            if (!wait_read) begin //ram access finished
                if (cache_update == 2'b10 || !cpu_req.rw) begin
                    next_state = idle;
                    next_wait_read = 0;
                    data_being_read = 1;
                end
                else begin
                    next_state = allocate;
                    next_wait_read = 0;
                    data_being_read = 0;
                end
            end
            else begin
                next_state = allocate;
                if (mem_data.ready)
                    next_wait_read = 0;
                else
                    next_wait_read = 1;
                data_being_read = 0;
                
            end
            cpu_res.ready = 0;
            mem_req.addr[31:12] = mhub.waddr[31:12];
            mem_req.addr[11:0] = 12'd0;
        end
        //==================================================================//
        else if (state == writeback) begin //Writeback
            if (mem_data.ready) begin
                next_state = allocate;
                next_wait_read = 1;
            end
            else begin
                next_state = writeback;
                next_wait_read = 0;
            end
            cpu_res.ready = 0;
            mem_req.addr[31:12] = tag_read.tag;
            mem_req.addr[11:0] = 12'd0;
            data_being_read = 0;
        end
        //==============================================================//
        else begin //Because of combinational warnings
            next_state = idle;
            cpu_res.ready = 1;
            next_wait_read = 0;
            mem_req.addr[31:4] = mhub.waddr[31:4];
            mem_req.addr[3:0] = 4'd0;
            data_being_read = 0;
        end
 
 
 
//=============Sequential Aspect of FSM====================//
    always_ff @ (posedge clk) begin
        if (state == idle) begin
            tag_req.we <= 0;
            cache_update <= 0;
            if (cpu_req.valid) begin    
                tag_req.index <= mhub.waddr[11:4]; //update addr for cache
                data_req.index <= mhub.waddr[11:4];
                data_req.we <= 0;
            end
        end
    //====================================================//                
        else if (state == compare_tag) begin
            if (tag_read.valid && (tag_read.tag == mhub.waddr[31:12])) begin //hit
                mem_req.valid <= 0;
                from_ram <= 0; //not using ram                
                tag_req.we <= 1;
                tag_write.tag <= mhub.waddr[31:12];
                tag_write.valid <= 1; 
                if (cpu_req.rw) begin //write
                    data_write[block_offset*WORD_WIDTH+:WORD_WIDTH] <= mhub.din; //grab correct index
                    data_req.we <= 1;
                    tag_write.dirty <= 1;
                    saved_bo <= block_offset; //saving block offset and byte enable to prevent repeated hits from losing info
                    saved_be <= be;
                end
                else begin
                    if (tag_read.dirty)
                        tag_write.dirty <= 1;
                    else
                        tag_write.dirty <= 0;
                    data_req.we <= 0;
                    cpu_res.data <= data_read[block_offset*WORD_WIDTH+:WORD_WIDTH];
                end
            end
            else begin //miss
                mem_req.valid <= 1;
                from_ram <= 1;
                data_req.we <= 0;
                if (tag_read.dirty) begin
                    mem_req.data <= data_read;
                    mem_req.rw <= 1;
                end
                else begin
                    mem_req.data[block_offset*WORD_WIDTH+:WORD_WIDTH] <= mhub.din;
                    mem_req.rw <= 0;
                end
            end
        end
        //====================================================// For neatness homie 
        else if (state == allocate) begin
            if (!wait_read) begin
                if (cache_update == 2'b10) begin
                    cache_update <= 2'b00;
                    from_ram <= 0;
                    data_write[block_offset*WORD_WIDTH+:WORD_WIDTH] <= mhub.din;
                end
                else if (cache_update == 2'b01)
                    cache_update <= cache_update + 1;
                else begin
                    if (cpu_req.rw)
                        cache_update <= cache_update + 1;
                    tag_req.we <= 1;
                    tag_write.tag <= mhub.waddr[31:12];
                    tag_write.valid <= 1;
                    if (cpu_req.rw) begin
                        tag_write.dirty <= 1;
                    end
                    else begin
                        tag_write.dirty <= 0;
                    end
                    data_write <= mem_data.data;
                    data_req.we <= 1;
                    saved_bo <= block_offset;
                    saved_be <= be;
                    cpu_res.data <= mem_data.data[block_offset*WORD_WIDTH+:WORD_WIDTH];
                end
            end
            else if (mem_data.ready)
                mem_req.valid <= 0;
        end
        //====================================================// For neatness homie 
        else if (state == writeback) begin
            mem_req.rw <= 0;
            mem_req.data <= mhub.din;
        end
        else;
    end

endmodule

module icache(
    input clk, RESET,
    i_mhub_to_icache.device mhub,
    i_icache_to_ram.controller ram
    );

    cpu_req_type cpu_req;     //CPU->cache
    mem_data_type mem_data;   //memory->cache
    
    mem_req_type mem_req;    //cache->memory
    cpu_result_type cpu_res;  //cache->CPU
    
    logic [1:0] block_offset, saved_bo;
    logic [3:0] be, saved_be;
    logic miss;
    logic from_ram;
    logic wait_read, next_wait_read; //Delay for when memory is outputting data to cache
    logic data_being_read; //Delay for when cache is outputting data to OTTER/Memory Hub
    
    assign be = 0;
    assign block_offset = mhub.waddr[3:2];
    assign data_blk_msb = ((31 * (block_offset + 1)) + block_offset);
    assign data_blk_lsb = 32 * block_offset;
   
    assign cpu_req.addr[ADDR_WIDTH-1:WORD_ADDR_LSB] = mhub.waddr;
    assign cpu_req.data = 0;
    assign cpu_req.rw = 0;
    assign cpu_req.valid = mhub.en;
     
    assign mhub.dout = cpu_res.data;            //async read
    assign mhub.hold = ~cpu_res.ready; 	 //Inverse     //ready goes high before edge that data is available
    
    assign ram.baddr = mem_req.addr[ADDR_WIDTH-1:BLOCK_ADDR_LSB];
    assign ram.en = mem_req.valid;
   
    assign mem_data.data = ram.dout;
    assign mem_data.ready = ~ram.hold;       //Inverse
    
    typedef enum {idle, compare_tag, allocate, writeback} cache_state_type;
   
    cache_state_type state, next_state;

    cache_tag_type tag_read;
    cache_tag_type tag_write;
    cache_req_type tag_req;
    
    cache_data_type data_read;
    cache_data_type data_write;
    cache_req_type data_req;
    
    cpu_result_type next_cpu_res;
    
	//FSM for Cache Controller
    L1_cache_tag L1_tags(.clk(clk), .tag_req(tag_req), .tag_write(tag_write), .tag_read(tag_read));
    L1_cache_data L1_data(.clk(clk), .data_req(data_req), .data_write(data_write), .be(saved_be),
                          .block_offset(saved_bo), .from_ram(from_ram), .data_read(data_read));
    
    always_ff @ (posedge clk) begin
        if (RESET) begin
            state <= idle;
        end
        else begin
            state <= next_state;
        end   
        wait_read <= next_wait_read;
    end
    
    initial begin
        mem_req.valid = 0;
        mem_req.rw = 0;
    end
    
//=============Combinational Aspect of FSM====================//
    always_comb begin
        if (state == idle) begin
            mem_req.addr[31:4] = mhub.waddr[31:4];
            mem_req.addr[3:0] = 4'd0;
            if (cpu_req.valid) begin
                if (data_being_read) begin
                    next_state = idle;
                    cpu_res.ready = 1;
                end
                else begin
                    next_state = compare_tag;
                    cpu_res.ready = 0;
                end
            end
            else begin
                next_state = idle;
                cpu_res.ready = 1;
            end
            miss = 0;
            next_wait_read = 0;
            data_being_read = 0;
        end
        //============================================================================//
        else if (state == compare_tag) begin
            if (tag_read.valid && (tag_read.tag == mhub.waddr[31:12])) begin //hit
                next_state = idle;
                next_wait_read = 0;
                mem_req.addr[31:4] = mhub.waddr[31:4];
                mem_req.addr[3:0] = 4'd0;
                data_being_read = 1;
            end
            else begin
                miss = 1;
                data_being_read = 0;
                if (tag_read.dirty) begin
                    next_state = writeback;
                    next_wait_read = 0;
                    mem_req.addr[31:12] = tag_read.tag;
                    mem_req.addr[11:0] = 12'd0;
                end
                else begin
                    next_state = allocate;
                    next_wait_read = 1;
                    mem_req.addr[31:12] = mhub.waddr[31:12];
                    mem_req.addr[11:0] = 12'd0;
                end
            end
            cpu_res.ready = 0;
        end
        //=============================================================//
        else if (state == allocate) begin //Allocate
            if (!wait_read) begin
                next_state = idle;
                next_wait_read = 0;
                data_being_read = 1;
            end
            else begin
                next_state = allocate;
                if (mem_data.ready)
                    next_wait_read = 0;
                else
                    next_wait_read = 1;
                data_being_read = 0;               
            end
            cpu_res.ready = 0;
            mem_req.addr[31:12] = mhub.waddr[31:12];
            mem_req.addr[11:0] = 12'd0;
        end
        //==================================================================//
        else if (state == writeback) begin //Writeback
            if (mem_data.ready) begin
                next_state = allocate;
                next_wait_read = 1;
            end
            else begin
                next_state = writeback;
                next_wait_read = 0;
            end
            cpu_res.ready = 0;
            mem_req.addr[31:12] = tag_read.tag;
            mem_req.addr[11:0] = 12'd0;
            data_being_read = 0;
        end
        //==============================================================//
        else begin //Because of combinational warnings
            next_state = idle;
            cpu_res.ready = 1;
            next_wait_read = 0;
            mem_req.addr[31:4] = mhub.waddr[31:4];
            mem_req.addr[3:0] = 4'd0;
            data_being_read = 0;
        end
    end
 
 
//=============Sequential Aspect of FSM====================//
    always_ff @ (posedge clk) begin
        if (state == idle) begin
            tag_req.we <= 0;
            if (cpu_req.valid) begin    
                tag_req.index <= mhub.waddr[11:4];
                data_req.index <= mhub.waddr[11:4];
                data_req.we <= 0;
            end
        end
    //====================================================// For neatness homie                
        else if (state == compare_tag) begin
            if (tag_read.valid && (tag_read.tag == mhub.waddr[31:12])) begin //hit
                mem_req.valid <= 0;
                from_ram <= 0;                
                tag_req.we <= 1;
                tag_write.tag <= mhub.waddr[31:12];
                tag_write.valid <= 1; 
                if (cpu_req.rw) begin //write
                    data_write[block_offset*WORD_WIDTH+:WORD_WIDTH] <= 32'd0;
                    data_req.we <= 1;
                    tag_write.dirty <= 1;
                    saved_bo <= block_offset;
                    saved_be <= be;
                end
                else begin
                    if (tag_read.dirty)
                        tag_write.dirty <= 1;
                    else
                        tag_write.dirty <= 0;
                    data_req.we <= 0;
                    cpu_res.data <= data_read[block_offset*WORD_WIDTH+:WORD_WIDTH];
                end
            end
            else begin //miss
                mem_req.valid <= 1;
                from_ram <= 1;
                data_req.we <= 0;
                if (tag_read.dirty) begin
                    mem_req.data <= data_read;
                    mem_req.rw <= 1;
                end
                else begin
                    mem_req.data[block_offset*WORD_WIDTH+:WORD_WIDTH] <= 32'd0;
                    mem_req.rw <= 0;
                end
            end
        end
        //====================================================// For neatness homie 
        else if (state == allocate) begin
            if (!wait_read) begin
                tag_req.we <= 1;
                tag_write.tag <= mhub.waddr[31:12];
                tag_write.valid <= 1;
                if (cpu_req.rw) begin
                    tag_write.dirty <= 1;
                end
                else begin
                    tag_write.dirty <= 0;
                end
                data_write <= mem_data.data;
                data_req.we <= 1;
                saved_bo <= block_offset;
                saved_be <= be;
                cpu_res.data <= mem_data.data[block_offset*WORD_WIDTH+:WORD_WIDTH];
            end
            else if (mem_data.ready)
                mem_req.valid <= 0;
        end
        //====================================================// For neatness homie 
        else if (state == writeback) begin
            mem_req.rw <= 0;
            mem_req.data <= 32'd0;
        end
        else;
    end

endmodule