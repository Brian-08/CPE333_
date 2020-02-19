`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/13/2020 08:00:00 AM
// Design Name: 
// Module Name: slow_ram
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


import memory_bus_sizes::*;

module slow_ram #(
    parameter MEM_DELAY = 10,  // accept/process one command every MEM_DELAY+1 clock cycles
    parameter RAM_DEPTH = -1,  // define in parent
    parameter INIT_FILENAME = ""  // define in parent
    )(
    input clk,
    i_icache_to_ram.device icache,
    i_dcache_to_ram.device dcache
    );

    logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] rami_baddr, ramd_baddr;
    logic [BLOCK_WIDTH-1:0] rami_din, rami_dout, ramd_din, ramd_dout;
    logic rami_en, ramd_en, ramd_we; 
    
    delay_ram #(
        .PORT_NAME("Instruction"), .MEM_DELAY(MEM_DELAY)
    ) delay_rami (
        .clk(clk), .baddr(icache.baddr), .en(icache.en), .we('0), .din('0), .dout(icache.dout), .hold(icache.hold),
        .ram_baddr(rami_baddr), .ram_en(rami_en), .ram_we(), .ram_din(), .ram_dout(rami_dout)
    );
    delay_ram #(
        .PORT_NAME("Data"), .MEM_DELAY(MEM_DELAY)
    ) delay_ramd (
        .clk(clk), .baddr(dcache.baddr), .en(dcache.en), .we(dcache.we), .din(dcache.din), .dout(dcache.dout), .hold(dcache.hold),
        .ram_baddr(ramd_baddr), .ram_en(ramd_en), .ram_we(ramd_we), .ram_din(ramd_din), .ram_dout(ramd_dout)
    );
    xilinx_bram_tdp_nc_nr #(
        .ADDR_WIDTH(ADDR_WIDTH - BLOCK_ADDR_LSB), .DATA_WIDTH(BLOCK_WIDTH), .RAM_DEPTH(RAM_DEPTH),
        .INIT_FILENAME(INIT_FILENAME)
    ) bram (
        .clka(clk), .addra(rami_baddr), .dina('0), .douta(rami_dout), .ena(rami_en), .wea('0), 
        .clkb(clk), .addrb(ramd_baddr), .dinb(ramd_din), .doutb(ramd_dout), .enb(ramd_en), .web(ramd_we) 
    );

endmodule

module delay_ram #(
    PORT_NAME = "",  // define in parent, for debug output labeling
    MEM_DELAY = -1  // define in parent
    )(
    input clk,
    input [ADDR_WIDTH-1:BLOCK_ADDR_LSB] baddr,
    input en, we,
    input [BLOCK_WIDTH-1:0] din,
    output [BLOCK_WIDTH-1:0] dout,
    output hold,
    output [ADDR_WIDTH-1:BLOCK_ADDR_LSB] ram_baddr,
    output ram_en, ram_we,
    output [BLOCK_WIDTH-1:0] ram_din,
    input [BLOCK_WIDTH-1:0] ram_dout
    );

    localparam CNT_WIDTH = (MEM_DELAY == 0) ? 1 : $clog2(MEM_DELAY+1);
    logic [CNT_WIDTH-1:0] r_cycle_cnt = 0;
    logic r_we;
    logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] r_baddr;
    logic [BLOCK_WIDTH-1:0] r_din;

    logic s_hold;  // if command present, tells controller to hold it's command steady while we process
    logic s_available;  // available unless processing, but also includes all of the acceptance partial cycle
    logic s_processing;  // includes acceptance partial cycle
    logic s_accepting, s_in_delay_cycles;  // if s_processing, exactly one true, otherwise all false
    logic s_reading, s_writing;  // if s_processing, exactly one true, otherwise all false
    logic s_in_final_cycle;  // the operation completes on the edge after this cycle, which may be the acceptance cycle if MEM_DELAY==0
    logic s_we;  // command or saved
    logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] s_baddr;  // command or saved
    logic [BLOCK_WIDTH-1:0] s_din;  // command or saved

    assign s_in_delay_cycles = r_cycle_cnt > 0;
    assign s_available = !s_in_delay_cycles;
    assign s_accepting = s_available && en;
    assign s_processing = s_accepting || s_in_delay_cycles;
    assign s_in_final_cycle = s_processing && r_cycle_cnt == MEM_DELAY;
    assign s_we = s_accepting ? we : r_we;
    assign s_reading = s_processing && !s_we;
    assign s_writing = s_processing && s_we;
    assign s_baddr = s_accepting ? baddr : r_baddr;
    assign s_din = s_accepting ? din : r_din;
    assign s_hold = en ? ((s_reading && !s_in_final_cycle) || (s_writing && !s_accepting)) : 0;

    assign hold = s_hold;
    assign dout = ram_dout;

    assign ram_baddr = s_baddr;
    assign ram_en = s_in_final_cycle;
    assign ram_we = s_writing;
    assign ram_din = s_din;

    always_ff @(posedge clk) begin
        if (s_in_final_cycle) begin
            r_cycle_cnt <= 0;
        end else if (s_processing) begin
            r_cycle_cnt <= r_cycle_cnt + 1;
        end
        if (s_accepting && !s_in_final_cycle) begin
            r_baddr <= baddr;
            r_we <= we;
            if (s_writing) begin
                r_din <= din;
            end
        end
    end
    
endmodule
