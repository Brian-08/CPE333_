`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/20/2020 10:17:25 PM
// Updated Date: 02/13/2020 11:00:00 AM
// Design Name: 
// Module Name: testbench
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


module testbench();

    // 100MHz clock
    logic clk = 1; always #5 clk = ~clk;
    default clocking cb @(posedge clk); endclocking

    logic BTNL = '0;
    logic BTNC = '0;
    logic [15:0] SWITCHES = 16'b0000000000000000;
    logic RX = '0;
    wire TX;
    wire [15:0] LEDS;
    wire [7:0] CATHODES;
    wire [3:0] ANODES;

    OTTER_Wrapper_Programmable DUT (.CLK(clk), .BTNL(BTNL), .BTNC(BTNC), .SWITCHES(SWITCHES), .RX(RX), .TX(TX), .LEDS(LEDS), .CATHODES(CATHODES), .ANODES(ANODES));

    initial begin

    end    

endmodule
