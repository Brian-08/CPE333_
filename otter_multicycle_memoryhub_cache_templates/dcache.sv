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
            
        data_read <= data_mem[data_req.index];
    end
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req,
    input cache_tag_type tag_write,
    output cache_tag_type tag_read);
    
	// tag storage 
	// incluces valid and dirty bits
	// async read, sync write
	
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
            tag_mem[tag_req.index].valid <= 1;
            tag_mem[tag_req.index].dirty <= 0;
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
    
    logic [1:0] block_offset;
    logic [3:0] be;
    logic miss;
    logic from_ram;
    logic wait_read, next_wait_read;   
    
    assign be = mhub.be;
    assign block_offset = mhub.waddr[3:2];
   
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
    L1_cache_data L1_data(.clk(clk), .data_req(data_req), .data_write(data_write), .be(be),
                          .block_offset(block_offset), .from_ram(from_ram), .data_read(data_read));
    
    initial begin
        state = idle;
        next_state = compare_tag;
    end
    
//=============Combinational Aspect of FSM====================//
    always_comb begin
        if (state == idle) begin
            miss = 0;
            if (cpu_req.valid) begin
                next_state = compare_tag;
                cpu_res.ready = 1;
            end
            else
                next_state = idle;
        end
        else if (state == compare_tag)
            if (tag_read.valid && (tag_read.tag == mhub.addr[31:12])) begin //hit
                next_state = idle;
            end
            else begin
                miss = 1;
                if (tag_read.dirty)
                    next_state = writeback;
                else begin
                    next_state = allocate;
                    next_wait_read = 1;
                end
            end
        else if (state == allocate)
            if (!wait_read)
                next_state = idle;
            else begin
                next_state = allocate;
                if (mem_data.ready)
                    next_wait_read = 0;
                else
                    next_wait_read = 1;
            end
        else if (state == writeback)
            if (mem_data.ready) begin
                next_state = allocate;
                next_wait_read = 1;
            end
            else
                next_state = writeback;
    end
    
//=============Sequential Aspect of FSM====================//
    always_ff @(posedge clk) begin
        if (state == idle)
            if (cpu_req.valid) begin    
                tag_req.index <= mhub.waddr[11:4];
                tag_req.we <= 0;
                data_req.index <= mhub.waddr[11:4];
                data_req.we <= 0;
            end
    //====================================================// For neatness homie                
        else if (state == compare_tag) begin
            if (tag_read.valid && (tag_read.tag == mhub.addr[31:12])) begin //hit
                mem_req.valid <= 0;
                from_ram <= 0;
                tag_req.we <= 1;
                tag_write.valid <= 1;
                tag_write.tag <= mhub.waddr[31:12]; //20 MSB (Due to given code)
                if (cpu_req.rw) begin //write
                    tag_write.dirty <= 1;
                    data_req.we <= 1;
                    data_write <= mhub.din;
                end
                else
                    cpu_res.data <= data_read;
                    tag_write.dirty <= 0;
            end
            else begin //miss
                mem_req.valid <= 1;
                from_ram <= 1;
                if (tag_read.dirty) begin
                    mem_req.addr[31:12] <= tag_read.tag;
                    mem_req.data <= data_read;
                    mem_req.rw <= 1;
                end
                else begin
                    mem_req.addr[31:12] <= mhub.waddr[31:12];
                    mem_req.data <= mhub.din;
                    mem_req.rw <= mhub.we;
                end
            end
        end
        //====================================================// For neatness homie 
        else if (state == allocate)
            if (!wait_read) begin
                tag_req.we <= 1;
                tag_write.tag <= mhub.waddr[31:12];
                tag_write.valid <= 1;
                if (cpu_req.rw)
                    tag_write.dirty <= 1;
                else
                    tag_write.dirty <= 0;
                data_write <= mem_data.data;
                mem_req.valid <= 0;
            end
        //====================================================// For neatness homie 
        else if (state == writeback) begin
            mem_req.rw <= 0;
            mem_req.addr[31:12] <= mhub.waddr[31:12];
            mem_req.data <= mhub.din;
        end
        
        state <= next_state;
        wait_read <= next_wait_read;
    end

endmodule