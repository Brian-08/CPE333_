`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/13/2020 08:00:00 AM
// Design Name: 
// Module Name: memory_hub
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

module memory_hub(
    input clk,
    output err,
    i_cpui_to_mhub.device cpui,
    i_cpud_to_mhub.device cpud,
    i_prog_to_mhub.device prog,
    i_mhub_to_mmio.controller mmio,
    i_mhub_to_icache.controller icache,
    i_mhub_to_dcache.controller dcache
    );

    i_cpudt_to_mhub s_cpudt_to_mhub();
    i_mhub s_mhub();
    logic s_trans_err;

    size_translator trans(
        .clk(clk), .cpud(cpud), .cpudt(s_cpudt_to_mhub), .err(s_trans_err)
    );
    merge_controllers merge(
        .clk(clk), .cpudt(s_cpudt_to_mhub), .prog(prog), .mhub(s_mhub)
    );
    split_devices split(
        .clk(clk), .mhub(s_mhub), .mmio(mmio), .dcache(dcache)
    );
    
    assign err = s_trans_err || cpui.addr[WORD_ADDR_LSB-1:0] != '0;
    assign icache.waddr = cpui.addr[ADDR_WIDTH-1:WORD_ADDR_LSB]; 
    assign icache.en = cpui.en;
    assign icache.flush = 0;
    assign cpui.dout = icache.dout;
    assign cpui.hold = icache.hold;
        
endmodule

module size_translator(
    input clk,
    i_cpud_to_mhub.device cpud,
    i_cpudt_to_mhub.controller cpudt,
    output err
    );

    typedef enum logic [1:0] {B=2'b00, H=2'b01, W=2'b10, D=2'b11} t_size;

    logic [$clog2(WORD_SIZE)-1:0] r_dout_byte_sel = '0;
    logic [1:0] r_dout_size = '0;
    logic r_dout_lu = '0;

    logic [$clog2(WORD_SIZE)-1:0] s_byte_sel;
    logic s_din_err, s_dout_err;
    logic s_sign;
    
    assign {cpudt.waddr, s_byte_sel} = cpud.addr;
    assign cpudt.en = cpud.en;
    assign cpudt.we = cpud.we;
    assign cpud.hold = cpudt.hold;    
    assign err = s_din_err || s_dout_err;

    // this module should work for both 32-bit and 64-bit RISC-V
    // the synthesizer should optimize out the 64-bit parts when WORD_SIZE < 8

    // translate din for write ops (stores)
    always_comb begin
        cpudt.be = '0;
        cpudt.din = '0;
        s_din_err = cpud.en && cpud.we;  // if a write op, assume error unless acceptable size and byte_sel
        case (t_size'(cpud.size))
            B: begin
                cpudt.be[s_byte_sel] = 'b1;
                cpudt.din[s_byte_sel*8+:8] = cpud.din[7:0];
                s_din_err = 0;
            end
            H: if (WORD_SIZE >= 2 && (s_byte_sel & 'b1) == 0) begin
                cpudt.be[s_byte_sel+:2] = 'b11;
                cpudt.din[s_byte_sel*8+:16] = cpud.din[15:0];
                s_din_err = 0;
            end
            W: if (WORD_SIZE >= 4 && (s_byte_sel & 'b11) == 0) begin
                cpudt.be[s_byte_sel+:4] = 'b1111;
                cpudt.din[s_byte_sel*8+:32] = cpud.din[31:0];
                s_din_err = 0;
            end
            D: if (WORD_SIZE >= 8 && (s_byte_sel & 'b111) == 0) begin
                cpudt.be[s_byte_sel+:8] = 'b11111111;
                cpudt.din[s_byte_sel*8+:64] = cpud.din[63:0];
                s_din_err = 0;
            end
        endcase
    end

    // translate dout for read ops (loads)
    always_comb begin
        cpud.dout = '0;
        s_dout_err = 1;  // assume error unless acceptable size and byte_sel
        case (t_size'(r_dout_size))
            B: begin
                if (r_dout_lu) cpud.dout = unsigned'(cpudt.dout[r_dout_byte_sel*8+:8]);
                else cpud.dout = signed'(cpudt.dout[r_dout_byte_sel*8+:8]);
                s_dout_err = 0;
            end
            H: if (WORD_SIZE >= 2 && (r_dout_byte_sel & 'b1) == 0) begin
                if (r_dout_lu) cpud.dout = unsigned'(cpudt.dout[r_dout_byte_sel*8+:16]);
                else cpud.dout = signed'(cpudt.dout[r_dout_byte_sel*8+:16]);
                s_dout_err = 0;
            end
            W: if (WORD_SIZE >= 4 && (r_dout_byte_sel & 'b11) == 0) begin
                if (r_dout_lu) cpud.dout = unsigned'(cpudt.dout[r_dout_byte_sel*8+:32]);
                else cpud.dout = signed'(cpudt.dout[r_dout_byte_sel*8+:32]);
                s_dout_err = 0;
            end
            D: if (WORD_SIZE >= 8 && (r_dout_byte_sel & 'b111) == 0) begin
                if (r_dout_lu) cpud.dout = unsigned'(cpudt.dout[r_dout_byte_sel*8+:64]);
                else cpud.dout = signed'(cpudt.dout[r_dout_byte_sel*8+:64]);
                s_dout_err = 0;
            end
        endcase
    end
    
    // latch info for translating dout at the same time dout is latched by its source
    always_ff @(posedge clk) begin
        if (cpud.en && !cpud.we && !cpud.hold) begin
            r_dout_byte_sel <= s_byte_sel; 
            r_dout_size <= cpud.size;
            r_dout_lu <= cpud.lu;
        end
    end 
    
endmodule

interface i_cpudt_to_mhub();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] din, dout;
    logic [WORD_SIZE-1:0] be;
    logic en, we, hold;
    modport controller (output waddr, be, en, we, din, input  dout, hold);
    modport device     (input  waddr, be, en, we, din, output dout, hold);
endinterface

interface i_mhub();
    logic [ADDR_WIDTH-1:WORD_ADDR_LSB] waddr;
    logic [WORD_WIDTH-1:0] din, dout;
    logic [WORD_SIZE-1:0] be;
    logic en, we, flush, hold;
    modport controller (output waddr, be, en, we, flush, din, input  dout, hold);
    modport device     (input  waddr, be, en, we, flush, din, output dout, hold);
endinterface

module merge_controllers(
    input clk,
    i_cpudt_to_mhub.device cpudt,
    i_prog_to_mhub.device prog,
    i_mhub.controller mhub
    );

    // prog has priority over cpudt when both start issuing a command in the same
    //   cycle, but operations won't be interrupted once in progress

    // NOTE: implementing the no-change rule for dout would be non-trivial if both
    //   controllers had the ability to issue read commands, but for now, prog can
    //   only write and has no dout

    // TODO: determine how prog can trigger icache flush also

    typedef enum {ACCEPTING, PROG_ACTIVE, CPUDT_ACTIVE} t_state;
    t_state r_state = ACCEPTING;
    t_state s_next_state;

    always_comb begin
        s_next_state = r_state;
        mhub.waddr = prog.waddr;
        mhub.be = {WORD_SIZE{1'b1}};
        mhub.en = prog.en;
        mhub.we = prog.we;
        mhub.flush = prog.flush;
        mhub.din = prog.din;
        prog.hold = prog.en;  // by default, hold all commands unless below logic says otherwise
        cpudt.hold = cpudt.en;  // by default, hold all commands unless below logic says otherwise
        cpudt.dout = mhub.dout;  // prog can't read, so mhub.dout should only change on cpudt reads 
        if (r_state == PROG_ACTIVE || (r_state == ACCEPTING && prog.en)) begin
            prog.hold = mhub.hold;
            if (mhub.hold) begin
                s_next_state = PROG_ACTIVE;
            end else begin
                s_next_state = ACCEPTING;
            end
        end else if (r_state == CPUDT_ACTIVE || (r_state == ACCEPTING && cpudt.en)) begin
            cpudt.hold = mhub.hold;
            mhub.waddr = cpudt.waddr;
            mhub.be = cpudt.be;
            mhub.en = cpudt.en;
            mhub.we = cpudt.we;
            mhub.flush = 0;  // only from prog for now
            mhub.din = cpudt.din;
            if (mhub.hold) begin
                s_next_state = CPUDT_ACTIVE;
            end else begin
                s_next_state = ACCEPTING;
            end
        end
    end
    
    always_ff @(posedge clk) begin
        r_state <= s_next_state;
    end

endmodule

module split_devices(
    input clk,
    i_mhub.device mhub,
    i_mhub_to_mmio.controller mmio,
    i_mhub_to_dcache.controller dcache
    );

    typedef enum {DCACHE, MMIO} t_sel_device;

    t_sel_device r_dout_sel_device = DCACHE;

    t_sel_device s_sel_device;
    
    assign dcache.waddr = mhub.waddr;
    assign dcache.be = mhub.be;
    assign dcache.en = s_sel_device == DCACHE && mhub.en;
    assign dcache.we = mhub.we;
    assign dcache.flush = mhub.flush;
    assign dcache.din = mhub.din;
    assign mmio.waddr = mhub.waddr;
    assign mmio.be = mhub.be;
    assign mmio.en = s_sel_device == MMIO && mhub.en;
    assign mmio.we = mhub.we;
    assign mmio.din = mhub.din;
    assign mhub.dout = r_dout_sel_device == DCACHE ? dcache.dout : mmio.dout;
    assign mhub.hold = s_sel_device == DCACHE ? dcache.hold : mmio.hold;
    assign s_sel_device = mhub.waddr < MMIO_START_ADDR[ADDR_WIDTH-1:WORD_ADDR_LSB] ? DCACHE : MMIO;

    always_ff @(posedge clk) begin
        if (mhub.en && !mhub.we && !mhub.hold) begin
            r_dout_sel_device <= s_sel_device;
        end
    end    
    
endmodule
