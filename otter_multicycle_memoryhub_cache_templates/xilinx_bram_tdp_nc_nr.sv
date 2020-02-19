`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// Credits: Xilinx UG901 for bram inference style
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/13/2020 08:00:00 AM
// Design Name: 
// Module Name: xilinx_bram_tdp_nc_nr
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Infers a Xilinx BRAM with two independently clocked read/write
//              ports in no change mode (no simultaneous read and write, as dout
//              doesn't change when writing), with no additional output registers
//              (BRAM internal latches still provide synchronous reads). Provides
//              multiple choices of initialization. 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module xilinx_bram_tdp_nc_nr #(
    parameter ADDR_WIDTH = -1,  // define in parent
    parameter DATA_WIDTH = -1,  // define in parent
    parameter RAM_DEPTH = -1,  // define in parent
    parameter INIT_FILENAME = ""  // define in parent
    )(
    input clka, clkb,
    input ena, enb,
    input wea, web,
    input [ADDR_WIDTH-1:0] addra, addrb,
    input [DATA_WIDTH-1:0] dina, dinb,
    output [DATA_WIDTH-1:0] douta, doutb
    );

    logic [DATA_WIDTH-1:0] r_douta, r_doutb;

    assign douta = r_douta;
    assign doutb = r_doutb;

    logic [DATA_WIDTH-1:0] r_ram [RAM_DEPTH-1:0];

    initial $readmemh(INIT_FILENAME, r_ram, 0, RAM_DEPTH-1);

    always_ff @(posedge clka) begin 
        if (ena) begin
            if (wea) begin
                r_ram[addra] <= dina;
            end else begin
                r_douta <= r_ram[addra];
            end
        end
    end
    
    always_ff @(posedge clkb) begin 
        if (enb) begin
            if (web) begin
                r_ram[addrb] <= dinb;
            end else begin
                r_doutb <= r_ram[addrb];
            end
        end
    end

endmodule
