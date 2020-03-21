`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 03/04/2020 01:53:45 AM
// Updated Date: 03/04/2020 03:00:00 AM
// Design Name: 
// Module Name: stall_dout
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This module is designed to stall the output of a VLM interface,
//     and also save dout from a module that doesn't comply with the no-change
//     rule of the VLM protocol. The new_data_coming input indicates that
//     orig_dout will be valid in the next cycle; that data will be saved going
//     forward. Map new_data_coming to (en && !we && !hold) of a VLM interface,
//     and map orig_dout to dout of that VLM interface. If stall is asserted, this
//     module's dout maintains its previous value, the last one saved from the VLM
//     interface's dout (orig_dout).
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


import memory_bus_sizes::*;

module stall_dout(
    input clk,
    input rst,
    input stall,
    input new_data_coming,
    input [WORD_WIDTH-1:0] orig_dout,
    output [WORD_WIDTH-1:0] dout
    );
    
    logic r_new_data_here;
    logic r_was_stalled;
    logic [WORD_WIDTH-1:0] r_data;
    
    early_change_register #(.WIDTH(WORD_WIDTH)) data_from_vlm(
        .clk(clk), .rst(rst), .en(r_new_data_here), .d(orig_dout), .q(r_data)
    );
    
    early_change_register #(.WIDTH(WORD_WIDTH)) output_data(
        .clk(clk), .rst(rst), .en(!r_was_stalled), .d(r_data), .q(dout)
    );

    always_ff @(posedge clk) begin
        r_new_data_here <= rst ? 0 : new_data_coming;
        r_was_stalled <= rst ? 0 : stall;
    end
    
endmodule


// With this flipflop+mux combination, the output can be updated immediately
// (combinationally) while still preserving its value (sequentially) when not
// being changed. Essentially, this is like an unclocked latch but is safe for
// sequential designs!
// Credit goes to Trevor McKay and Hayden Rinn for the idea.
module early_change_register #(
    parameter WIDTH = -1  // define in parent
    )(
    input clk,
    input rst,
    input en,
    input [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
    );
    
    logic [WIDTH-1:0] r_reg;
    
    assign q = rst ? '0 : en ? d : r_reg;
    
    always_ff @(posedge clk) begin
        r_reg <= q;
    end
    
endmodule
