`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/14/2020 05:00:00 AM
// Design Name: 
// Module Name: memory_interfaces
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


package memory_bus_sizes;
    // user-defined
    parameter ADDR_WIDTH = 32;  // bits
    parameter WORD_SIZE = 4;  // bytes, power of 2
    parameter BLOCK_SIZE = 16;  // bytes, power of 2
    parameter MMIO_START_ADDR = 'h11000000;
    // derived
    parameter WORD_WIDTH = WORD_SIZE * 8;
    parameter BLOCK_WIDTH = BLOCK_SIZE * 8;
    parameter WORD_ADDR_LSB = $clog2(WORD_SIZE);
    parameter BLOCK_ADDR_LSB = $clog2(BLOCK_SIZE);
endpackage
import memory_bus_sizes::*;

interface i_cpui_to_mhub();
    logic [ADDR_WIDTH-1:0] addr;
    logic [WORD_WIDTH-1:0] dout;
    logic en, hold;
    modport controller (output addr, en, input  dout, hold);
    modport device     (input  addr, en, output dout, hold);
    wire [$size(addr)-1:0] read_args = {addr};
    wire [0:0] write_args = {1'b0};
    wire [$size(dout)-1:0] read_results = {dout};
    wire we = 0;
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_cpud_to_mhub();
    logic [ADDR_WIDTH-1:0] addr;
    logic [WORD_WIDTH-1:0] din, dout;
    logic [1:0] size;
    logic lu;
    logic en, we, hold;
    modport controller (output addr, size, lu, en, we, din, input  dout, hold);
    modport device     (input  addr, size, lu, en, we, din, output dout, hold);
    wire [$size(addr)+$size(size)+1-1:0] read_args = {addr, size, lu};
    wire [$size(addr)+$size(size)+$size(din)-1:0] write_args = {addr, size, din};
    wire [$size(dout)-1:0] read_results = {dout};
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_prog_to_mhub();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] din;
    logic en, we, flush, hold;
    modport controller (output waddr, en, we, flush, din, input  hold);
    modport device     (input  waddr, en, we, flush, din, output hold);
    wire [$size(waddr)-1:0] read_args = {waddr};
    wire [$size(waddr)+$size(din)-1:0] write_args = {waddr, din};
    wire [0:0] read_results = {1'b0};
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_mhub_to_mmio();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] din, dout;
    logic [WORD_SIZE-1:0] be;
    logic en, we, hold;
    modport controller (output waddr, be, en, we, din, input  dout, hold);
    modport device     (input  waddr, be, en, we, din, output dout, hold);
    wire [$size(waddr)-1:0] read_args = {waddr};
    wire [$size(waddr)+$size(be)+$size(din)-1:0] write_args = {waddr, be, din};
    wire [$size(dout)-1:0] read_results = {dout};
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_mhub_to_icache();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] dout;
    logic en, flush, hold;
    modport controller (output waddr, en, flush, input  dout, hold);
    modport device     (input  waddr, en, flush, output dout, hold);
    wire [$size(waddr)-1:0] read_args = {waddr};
    wire [0:0] write_args = {1'b0};
    wire [$size(dout)-1:0] read_results = {dout};
    wire we = 0;
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_mhub_to_dcache();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] din, dout;
    logic [WORD_SIZE-1:0] be;
    logic en, we, flush, hold;
    modport controller (output waddr, be, en, we, flush, din, input  dout, hold);
    modport device     (input  waddr, be, en, we, flush, din, output dout, hold);
    wire [$size(waddr)-1:0] read_args = {waddr};
    wire [$size(waddr)+$size(be)+$size(din)-1:0] write_args = {waddr, be, din};
    wire [$size(dout)-1:0] read_results = {dout};
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_icache_to_ram();
    logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] baddr;
    logic [BLOCK_WIDTH-1:0] dout;
    logic en, hold;
    modport controller (output baddr, en, input  dout, hold);
    modport device     (input  baddr, en, output dout, hold);
    wire [$size(baddr)-1:0] read_args = {baddr};
    wire [$size(baddr)-1:0] write_args = {baddr};
    wire [$size(dout)-1:0] read_results = {dout};
    wire we = 0;
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

interface i_dcache_to_ram();
    logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] baddr;
    logic [BLOCK_WIDTH-1:0] din, dout;
    logic en, we, hold;
    modport controller (output baddr, en, we, din, input  dout, hold);
    modport device     (input  baddr, en, we, din, output dout, hold);
    wire [$size(baddr)-1:0] read_args = {baddr};
    wire [$size(baddr)+$size(din)-1:0] write_args = {baddr, din};
    wire [$size(dout)-1:0] read_results = {dout};
    modport vlm_validator (input en, we, hold, read_args, write_args, read_results);
endinterface

module vlm_protocol_validator #(
    parameter DISPLAY_NAME = ""  // parent should define
    )(
    input clk,
    interface vlm,
    output err
    );

    logic r_last_en, r_last_we, r_last_hold;
    logic [$bits(vlm.read_args)-1:0] r_last_read_args;
    logic [$bits(vlm.write_args)-1:0] r_last_write_args;
    logic [$bits(vlm.read_results)-1:0] r_last_read_results;
    logic r_first_edge = 1;
    logic r_read_finished = 0; 

    logic s_err_ctrl_sigs;  // en, we (if en=1), or hold is X/Z
    logic s_err_other_sigs;  // other currently-relevant signal is X/Z
    logic s_err_hold_nothing;  // hold asserted when no command
    logic s_err_ctrl_changed;  // en or we changed when should be held
    logic s_err_input_changed;  // inputs changed when should be held
    logic s_err_output_changed;  // outputs changed when not finishing a read
    
    assign err = s_err_ctrl_sigs || s_err_other_sigs || s_err_hold_nothing || s_err_ctrl_changed || s_err_input_changed || s_err_output_changed;
    
    always_comb begin
        s_err_ctrl_sigs = 0;
        s_err_other_sigs = 0;
        s_err_hold_nothing = 0;
        s_err_ctrl_changed = 0;
        s_err_input_changed = 0;
        s_err_output_changed = 0;
        // reduction-xor (^) yields 'X if any X or Z bits in signal
        if (^vlm.en === 'X  // en needs to be always good
            || ^vlm.hold === 'X  // hold needs to be always good
            || (vlm.en === 1 && ^vlm.we === 'X)) begin  // if enabled, then "we" needs to be good
            s_err_ctrl_sigs = 1;
        end else begin
            if ((vlm.en === 1 && vlm.we === 0 && ^vlm.read_args === 'X)  // if reading, read_args needs to be good
                || (vlm.en === 1 && vlm.we === 1 && ^vlm.write_args === 'X)  // if writing, write_args needs to be good
                || (r_read_finished && ^vlm.read_results === 'X)) begin  // if any previous read completed, read_results needs to be good
                s_err_other_sigs = 1;
            end
            if (vlm.hold && !vlm.en) begin
                s_err_hold_nothing = 1;
            end
            if (!r_first_edge) begin
                if (r_last_hold) begin
                    if (vlm.en != r_last_en || vlm.we != r_last_we) begin
                        s_err_ctrl_changed = 1;
                    end
                    if (!r_last_we && vlm.read_args != r_last_read_args) begin
                        s_err_input_changed = 1;
                    end
                    if (r_last_we && vlm.write_args != r_last_write_args) begin
                        s_err_input_changed = 1;
                    end
                end
                if (!(r_last_en && !r_last_we && !r_last_hold)) begin  // if a read didn't just complete
                    if (vlm.read_results != r_last_read_results) begin
                        s_err_output_changed = 1;
                    end
                end
            end
        end
    end

    initial $timeformat(-9, 3, "ns", 10);

    always @(posedge clk) begin
        if (s_err_ctrl_sigs) $display("%t: [%s] VLM protocol warning: unexpected X's or Z's in control signals", $time, DISPLAY_NAME);
        if (s_err_other_sigs) $display("%t: [%s] VLM protocol warning: unexpected X's or Z's in currently-relevant signals", $time, DISPLAY_NAME);
        if (s_err_hold_nothing) $display("%t: [%s] VLM protocol violation: hold should not be asserted unless a command is starting or in progress", $time, DISPLAY_NAME);
        if (s_err_ctrl_changed) $display("%t: [%s] VLM protocol violation: en and we must remain constant while hold is asserted", $time, DISPLAY_NAME);
        if (s_err_input_changed) $display("%t: [%s] VLM protocol violation: currently-relevant inputs must remain constant while hold is asserted", $time, DISPLAY_NAME);
        if (s_err_output_changed) $display("%t: [%s] VLM protocol violation: outputs (besides hold) must remain constant except at a read completion", $time, DISPLAY_NAME);
        if (vlm.en && !vlm.we && !vlm.hold) begin
            r_read_finished <= 1; 
        end
        r_last_en <= vlm.en;
        r_last_we <= vlm.we;
        r_last_hold <= vlm.hold;
        r_first_edge <= 0;
        r_last_read_args <= vlm.read_args;
        r_last_write_args <= vlm.write_args;
        r_last_read_results <= vlm.read_results;
    end

endmodule

// empty module just to force Vivado to show this file in source hierarchy
module memory_interfaces(); endmodule
