`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/11/2020 07:30:00 PM
// Design Name: 
// Module Name: slow_ram_testbench
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

module slow_ram_testbench();

    // 100MHz clock
    logic clk = 1; always #5 clk = ~clk;
    default clocking cb @(posedge clk); endclocking

    i_icache_to_ram s_icache_to_ram();
    i_dcache_to_ram s_dcache_to_ram();

    slow_ram #(
        .MEM_DELAY(3),
        .INIT_SOURCE(2)  // 0 = zeros, 1 = mem.txt, 2 = random (sim only), 3 = no init (sim X) 
    ) ram (
        .clk(clk), .icache(s_icache_to_ram), .dcache(s_dcache_to_ram)
    );

    initial begin
        s_icache_to_ram.baddr = 'Z;
        s_icache_to_ram.en = 0;
        s_dcache_to_ram.baddr = 'Z;
        s_dcache_to_ram.en = 0;
        s_dcache_to_ram.we = 'Z;
        s_dcache_to_ram.din = 'Z;

        // do various operations, idling before and after, delaying each start by a half cycle to distinguish combinational responses

        // read data from block
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h1234567;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        ##20;

        // write data to the same block
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h1234567;
        s_dcache_to_ram.we <= 1;
        s_dcache_to_ram.din <= 128'h12345678123456781234567812345678;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        s_dcache_to_ram.din <= 'Z; 
        ##20;

        // read it back
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h1234567;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        ##20;

        // read data twice back-to-back from different blocks
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000011;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000012;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        ##20;

        // write data twice back-to-back to different blocks
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000013;
        s_dcache_to_ram.we <= 1;
        s_dcache_to_ram.din <= 128'h11111111111111111111111111111133;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        s_dcache_to_ram.din <= 'Z; 
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000014;
        s_dcache_to_ram.we <= 1;
        s_dcache_to_ram.din <= 128'h11111111111111111111111111111144;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        s_dcache_to_ram.din <= 'Z; 
        ##20;

        // read then write back-to-back on different blocks
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000015;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000016;
        s_dcache_to_ram.we <= 1;
        s_dcache_to_ram.din <= 128'h11111111111111111111111111111166;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        s_dcache_to_ram.din <= 'Z; 
        ##20;

        // write then read back-to-back on different blocks
        ##20;
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000017;
        s_dcache_to_ram.we <= 1;
        s_dcache_to_ram.din <= 128'h11111111111111111111111111111177;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        s_dcache_to_ram.din <= 'Z; 
        #2;  // @(negedge clk);
        s_dcache_to_ram.baddr <= 28'h0000018;
        s_dcache_to_ram.we <= 0;
        s_dcache_to_ram.en <= 1;
        @(posedge clk iff !s_dcache_to_ram.hold);
        s_dcache_to_ram.en <= 0;
        s_dcache_to_ram.baddr <= 'Z;
        s_dcache_to_ram.we <= 'Z;
        ##20;

        // read instruction
        ##20;
        @(negedge clk);
        s_icache_to_ram.baddr <= 28'h111111A;
        s_icache_to_ram.en <= 1;
        @(posedge clk iff !s_icache_to_ram.hold);
        s_icache_to_ram.en <= 0;
        s_icache_to_ram.baddr <= 'Z;
        ##20;

        // read instructions twice back-to-back from different blocks
        ##20;
        @(negedge clk);
        s_icache_to_ram.baddr <= 28'h111111B;
        s_icache_to_ram.en <= 1;
        @(posedge clk iff !s_icache_to_ram.hold);
        s_icache_to_ram.en <= 0;
        s_icache_to_ram.baddr <= 'Z;
        @(negedge clk);
        s_icache_to_ram.baddr <= 28'h111111C;
        s_icache_to_ram.en <= 1;
        @(posedge clk iff !s_icache_to_ram.hold);
        s_icache_to_ram.en <= 0;
        s_icache_to_ram.baddr <= 'Z;
        ##20;

    end

    /*
    // debug display for simulation
    initial $timeformat(-9, 3, "ns", 10);
    always @(posedge clk) begin
        // first look at our signals before nonblocking assignments on the clock edge change them
        automatic logic v_bc_finishing_read = s_in_final_cycle && s_reading;
        automatic logic v_bc_finishing_write = s_in_final_cycle && s_writing;
        automatic logic [ADDR_WIDTH-1:BLOCK_ADDR_LSB] v_bc_baddr = s_baddr;
        automatic integer v_bc_time = $time;
        // then use #1 to postpone further processing until nonblocking assignments have completed
        // this makes the bram output available for display
        #1;
        if (v_bc_finishing_read) $display("%t: [Memory] %s Read finished @ baddr=%x with data=%x", v_bc_time, PORT_NAME, v_bc_baddr, dout); 
        if (v_bc_finishing_write) $display("%t: [Memory] %s Write finished @ baddr=%x", v_bc_time, PORT_NAME, v_bc_baddr); 
        // we should now also get an early look at what the inputs will be right before the next positive clock edge
        if (s_accepting) begin
            if (s_reading) $display("%t: [Memory] %s Read started @ baddr=%x", $time, PORT_NAME, s_baddr); 
            if (s_writing) $display("%t: [Memory] %s Write started @ baddr=%x with data=%x", $time, PORT_NAME, s_baddr, s_din); 
        end else begin
            // check again right after the negative edge, for another early look at what the inputs will be right
            //   before the next positive clock edge (the same upcoming positive edge as for the first early look)
            @(negedge clk);
            #1;
            if (s_accepting) begin
                if (s_reading) $display("%t: [Memory] %s Read started @ baddr=%x", $time, PORT_NAME, s_baddr); 
                if (s_writing) $display("%t: [Memory] %s Write started @ baddr=%x with data=%x", $time, PORT_NAME, s_baddr, s_din);
            end 
        end
    end
    */

endmodule
