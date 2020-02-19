`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Updated Date: 02/13/2020 08:00:00 AM
// Design Name: 
// Module Name: OTTER_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.10 - (Keefe Johnson, 1/14/2020) Added serial programmer.
// Revision 0.20 - (Keefe Johnson, 2/13/2020) Replaced the basic memory system
//                 with a new hub using the VLM protocol with cache support.
//                 Restructured how the programmer interfaces with the CPU. 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


import memory_bus_sizes::*;  // ADDED FOR VLM SYSTEM

module OTTER_MCU(input CLK,
                input INTR,
                input EXT_RESET,  // CHANGED RESET TO EXT_RESET FOR PROGRAMMER
                i_mhub_to_mmio.controller MMIO,  // CHANGED IOBUS SIGNALS TO MMIO INTERFACE FOR VLM SYSTEM
                input PROG_RX,  // ADDED FOR PROGRAMMER
                output PROG_TX  // ADDED FOR PROGRAMMER
);           

    // ADDED SECOND RESET SOURCE FOR PROGRAMMER
    wire RESET;
    wire s_prog_mcu_reset;
    assign RESET = s_prog_mcu_reset | EXT_RESET;
    
    wire s_stall;  // ADDED FOR VLM SYSTEM
 
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    wire [3:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
   

    assign opcode = IR[6:0]; // opcode shortcut
    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                 .PC_DIN(pc_value), .PC_COUNT(pc));   
    
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
    Mult6to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, mtvec, mepc, pc_sel, pc_value);
    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (B, I_immed, S_immed, pc, opB_sel, aluBin);
    
    Mult2to1 ALUAinput (A, U_immed, opA_sel, aluAin);
    // Creates a RISC-V ALU
    // Inputs are ALUCtl (the ALU control), ALU value inputs (ALUAin, ALUBin)
    // Outputs are ALUResultOut (the 64-bit output) and Zero (zero detection output)
    OTTER_ALU ALU (alu_fun, aluAin, aluBin, aluResult); // the ALU
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], IR[11:7], rfIn, regWrite, A, B, CLK); // Register file
 
    //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (next_pc,csr_reg,mem_data,aluResult,wb_sel,rfIn);
  
    //pc target calculations 
    assign next_pc = pc + 4;    //PC is byte aligned, memory is word aligned
    assign jalr_pc = I_immed + A;
    //assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_pc = pc + {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = pc + {{12{IR[31]}}, IR[19:12], IR[20],IR[30:21],1'b0};
    assign int_pc = 0;
    
    logic br_lt,br_eq,br_ltu;
    //Branch Condition Generator
    always_comb
    begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(A) < $signed(B)) br_lt=1;
        if(A==B) br_eq=1;
        if(A<B) br_ltu=1;
    end
    
    // Generate immediates
    assign S_immed = {{20{IR[31]}},IR[31:25],IR[11:7]};
    assign I_immed = {{20{IR[31]}},IR[31:20]};
    assign U_immed = {IR[31:12],{12{1'b0}}};

    // REMOVED OLD MEMORY SYSTEM

     OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(opcode), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_BR_EQ(br_eq),.CU_BR_LT(br_lt),.CU_BR_LTU(br_ltu),.CU_PCSOURCE(pc_sel),
             .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel),.intTaken(intTaken));
            
     logic prev_INT=0;

    // ADDED STALL SIGNAL FOR VLM SYSTEM     
     OTTER_CU_FSM CU_FSM (.CU_CLK(CLK), .CU_INT(INTR), .CU_RESET(RESET), .CU_STALL(s_stall), .CU_OPCODE(opcode), //.CU_OPCODE(opcode),
                     .CU_FUNC3(IR[14:12]),.CU_FUNC12(IR[31:20]),
                     .CU_PCWRITE(pcWrite), .CU_REGWRITE(regWrite), .CU_MEMWRITE(memWrite), 
                     .CU_MEMREAD1(memRead1),.CU_MEMREAD2(memRead2),.CU_intTaken(intTaken),.CU_intCLR(intCLR),.CU_csrWrite(csrWrite),.CU_prevINT(prev_INT));
    
    //CSR registers and interrupt logic
     CSR CSRs(.clk(CLK),.rst(RESET),.intTaken(intTaken),.addr(IR[31:20]),.next_pc(pc),.wd(aluResult),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));
    
    always_ff @ (posedge CLK)
    begin
         if(INTR && mie)
            prev_INT=1'b1;
         if(intCLR || RESET)
            prev_INT=1'b0;
    end

    // CHANGED IOBUS SIGNALS TO MMIO INTERFACE FOR VLM SYSTEM

    // ADDED NEW VLM SYSTEM BELOW         

    i_cpui_to_mhub s_cpui();
    i_cpud_to_mhub s_cpud();
    i_prog_to_mhub s_prog();
    i_mhub_to_mmio s_mmio();
    i_mhub_to_icache s_mhub_to_icache();
    i_mhub_to_dcache s_mhub_to_dcache();
    i_icache_to_ram s_icache_to_ram();
    i_dcache_to_ram s_dcache_to_ram();

    programmer #(
        .CLK_RATE(50), .BAUD(115200), .IB_TIMEOUT(200), .WAIT_TIMEOUT(500)
    ) programmer (
        .clk(CLK), .rst(EXT_RESET), .srx(PROG_RX), .stx(PROG_TX),
        .mcu_reset(s_prog_mcu_reset), .mhub(s_prog)
    );
    memory_hub mhub(
        .clk(CLK), .err(), .cpui(s_cpui), .cpud(s_cpud), .prog(s_prog),
        .mmio(MMIO), .icache(s_mhub_to_icache), .dcache(s_mhub_to_dcache)
    );
    icache icache(
        .clk(CLK), .mhub(s_mhub_to_icache), .ram(s_icache_to_ram)
    );
    dcache dcache(
        .clk(CLK), .mhub(s_mhub_to_dcache), .ram(s_dcache_to_ram)
    );
    slow_ram #(
        .RAM_DEPTH(2**12),  // 4096 blocks * 16 bytes/block = 64KiB 
        .INIT_FILENAME("otter_memory_blocks.mem")  // load 16-byte blocks
    ) ram (
        .clk(CLK), .icache(s_icache_to_ram), .dcache(s_dcache_to_ram)
    );

    assign s_cpui.addr = pc;
    assign s_cpui.en = memRead1;
    assign IR = s_cpui.dout;

    assign s_cpud.addr = aluResult;
    assign s_cpud.size = IR[13:12];
    assign s_cpud.lu = IR[14];
    assign s_cpud.en = memRead2 || memWrite;
    assign s_cpud.we = memWrite;
    assign s_cpud.din = B;
    assign mem_data = s_cpud.dout;

    assign s_stall = s_cpui.hold || s_cpud.hold;
    
endmodule
