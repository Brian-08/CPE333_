`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/18/2020 05:05:37 PM
// Design Name: 
// Module Name: d-cache
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


module dcache(input CLK,
                input waddr,
                input din,
                input en,
                input we,
                output dout,
                output dc_hold
);

    //tag
    
    //data
    logic cache_ready, block_clean, block_invalid, block_dirty, memory_wait;
    
    //FSM
    logic[1:0] dcache_state; //0=Idle, 1=Compare Tag, 2=Allocate, 3=Write Back
    
    always_ff @(posedge CLK) begin
        if (dcache_state == 0) //Idle
            if (en)
                dcache_state <= 1;
            else
                dcache_state <= 0;
        else if (dcache_state == 1) begin //Compare
            if (tag_match && entry_valid) begin
                cache_ready <= 1;
                if (we) begin
                    //set bits
                    if (block_dirty)
                        dcache_state <= 3;
                    else 
                        dcache_state <= 0;
                end
                else
                    dcache_state <= 0;
            end
            else begin
                cache_ready <= 0;
                if (block_clean || block_invalid)
                    dcache_state <= 2;
                else
                    dcache_state <= 0;   
            end
        else if (dcache_state == 2) begin //
            
endmodule
