`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
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
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_MCU(input CLK,
                //input INTR, Note: not used in pipelining for now
                input EXT_RESET,  // CHANGED RESET TO EXT_RESET FOR PROGRAMMER
                i_mhub_to_mmio.controller MMIO,
                input PROG_RX,  // ADDED PROG_RX FOR PROGRAMMER
                output PROG_TX  // ADDED PROG_TX FOR PROGRAMMER
);           

    typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
    } opcode_t;
    
    
    typedef enum logic [2:0] {
        Func3_CSRRW  = 3'b001,
        Func3_CSRRS  = 3'b010,
        Func3_CSRRC  = 3'b011,
        Func3_CSRRWI = 3'b101,
        Func3_CSRRSI = 3'b110,
        Func3_CSRRCI = 3'b111,
        Func3_PRIV   = 3'b000       //mret
    } funct3_system_t;

    wire RESET;
    wire [31:0] s_prog_ram_addr;
    wire [31:0] s_prog_ram_data;
    wire s_prog_ram_we;
    wire s_prog_mcu_reset;
    wire [31:0] mem_addr_after;
    wire [31:0] mem_data_after;
    wire [1:0] mem_size_after;
    wire mem_sign_after;
    wire mem_we_after;

    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,
        I_immed,S_immed,U_immed,rfIn,csr_reg, mem_data;
        
    wire memRead1;
    
    wire pcWrite, memWrite, op1_sel,mem_op,IorD,pcWriteCond;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [3:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;

    logic s_stall;
    
    always_comb begin
        if (s_cpui.hold || s_cpud.hold)
            s_stall = 1;
        else
            s_stall = 0;
    end

    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                 .PC_DIN(pc_value), .PC_S_STALL(s_stall), .PC_COUNT(pc));   
    
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
    Mult6to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, mtvec, mepc, pc_sel, pc_value);
    
    logic br_lt,br_eq,br_ltu;
    
    //pipelining variables
    logic [31:0] if_de_pc, de_ex_pc, ex_mem_pc, mem_wb_pc, wb_inst_pc, rf_next_pc;
    wire [31:0] if_de_inst;
    logic [31:0] de_ex_inst, ex_mem_inst, mem_wb_inst, wb_inst;
    logic de_ex_memRead2, de_ex_regWrite, de_ex_memWrite, ex_mem_memRead2, ex_mem_regWrite, ex_mem_memWrite,
          mem_wb_regWrite, mem_wb_memWrite, mem_wb_memRead2, wb_regWrite;
    logic [1:0] de_ex_rf_wr_sel, ex_mem_rf_wr_sel, mem_wb_rf_wr_sel, wb_rf_wr_sel;
    logic [31:0] de_ex_rs1, de_ex_rs2, de_ex_opA, de_ex_opB, ex_opA, ex_opB, ex_mem_rs2, mem_wb_rs2, mem_wb_mem_in,
                 ex_mem_aluResult, mem_wb_aluResult, wb_aluResult, rf_w_notused, rf_w_notused2,
                 de_ex_rs1_forwarded, de_ex_rs2_forwarded, forwarded_valA, forwarded_valB, alu_opA, alu_opB;
    logic [4:0] de_inst_rd, de_inst_rs1, de_inst_rs2;
    logic [3:0] de_ex_alu_fun, ex_alu_fun;
    
    //hazard solution variables
    logic stall_fetch=0, branch_taken=0, jump_called=0, changed_pc=0;
    logic [1:0] forward_A, forward_B;
    logic do_forwardA, do_forwardB, inst_notU, inst_notJ;              

    //cache implementation
    i_cpui_to_mhub s_cpui();
    i_cpud_to_mhub s_cpud();
    i_prog_to_mhub s_prog();
    i_mhub_to_icache s_mhub_to_icache();
    i_mhub_to_dcache s_mhub_to_dcache();
    i_icache_to_ram s_icache_to_ram();
    i_dcache_to_ram s_dcache_to_ram();
    
    memory_hub mhub(
        .clk(CLK), .err(), .cpui(s_cpui), .cpud(s_cpud), .prog(s_prog),
        .mmio(MMIO), .icache(s_mhub_to_icache), .dcache(s_mhub_to_dcache)
    );
    icache icache(
        .clk(CLK), .RESET(EXT_RESET), .mhub(s_mhub_to_icache), .ram(s_icache_to_ram)
    );
    dcache dcache(
        .clk(CLK), .RESET(EXT_RESET), .mhub(s_mhub_to_dcache), .ram(s_dcache_to_ram)
    );
    slow_ram #(
        .RAM_DEPTH(2**12),  // 4096 (2**12) blocks * 16 bytes/block = 64KiB 
        .INIT_FILENAME("otter_memory_blocks.mem")  // load 16-byte blocks
    ) ram (
        .clk(CLK), .icache(s_icache_to_ram), .dcache(s_dcache_to_ram)
    );
    
    stall_dout STALL_CPUI_DOUT(
    .clk(CLK), .rst(EXT_RESET), .stall(s_stall),
    .new_data_coming(s_cpui.en && !s_cpui.we && !s_cpui.hold),
    .orig_dout(s_cpui.dout), .dout(if_de_inst)
    );
    stall_dout STALL_CPUD_DOUT(
    .clk(CLK), .rst(EXT_RESET), .stall(s_stall),
    .new_data_coming(s_cpud.en && !s_cpud.we && !s_cpud.hold),
    .orig_dout(s_cpud.dout), .dout(mem_data)
    );
    
    assign s_cpui.addr = pc;
    assign s_cpui.en = memRead1;
    
    assign s_cpud.addr = mem_wb_aluResult;
    assign s_cpud.size = mem_wb_inst[13:12];
    assign s_cpud.lu = mem_wb_inst[14];
    assign s_cpud.en = mem_wb_memRead2 || mem_wb_memWrite;  // important to OR both read and write signals!
    assign s_cpud.we =  mem_wb_memWrite;
    assign s_cpud.din = mem_wb_mem_in;
                
    assign {s_prog.waddr, s_prog.en, s_prog.we, s_prog.flush, s_prog.din} = '0;  // stub out the programmer for now            
    
    assign RESET = EXT_RESET;
    logic prev_INT=0;
    
    //CSR registers and interrupt logic
    CSR CSRs(.clk(CLK),.rst(EXT_RESET),.intTaken(intTaken),.addr(12'b0),.next_pc(pc),.wd(32'b0),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));
    always_ff @ (posedge CLK) begin
        if (!s_stall) begin
            if(1'b0 && mie)
                prev_INT=1'b1;
        end
    end
           
    //=======================================Pipelining Logic==============================================================================
    //============Fetch==============
    always_ff @(posedge CLK) begin
        if (!s_stall)
            if (!stall_fetch)
                if_de_pc <= pc;
    end
    
    assign pcWrite = !stall_fetch;
    assign memRead1 = !stall_fetch;
    //===============================
    
    //============Decode=============
    always_ff @(posedge CLK) begin
        if (!s_stall) begin
            de_ex_pc <= if_de_pc;
            if (stall_fetch || branch_taken || jump_called || changed_pc)
                de_ex_inst <= 32'h00000013; //insert bubble instruction (nop)
            else
                de_ex_inst <= if_de_inst;
        end  
    end
    
    always_comb begin
        de_inst_rd = de_ex_inst[11:7];
        de_inst_rs1 = de_ex_inst[19:15]; //addr1
        de_inst_rs2 = de_ex_inst[24:20]; //addr2
    end
    
    // Generate immediates
    assign S_immed = {{20{de_ex_inst[31]}},de_ex_inst[31:25],de_ex_inst[11:7]};
    assign I_immed = {{20{de_ex_inst[31]}},de_ex_inst[31:20]};
    assign U_immed = {de_ex_inst[31:12],{12{1'b0}}};
            
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(de_ex_inst[6:0]), .CU_FUNC3(de_ex_inst[14:12]),.CU_FUNC7(de_ex_inst[31:25]), 
             .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(de_ex_alu_fun),.CU_RF_WR_SEL(de_ex_rf_wr_sel),.intTaken(intTaken));

    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (de_ex_rs2, I_immed, S_immed, de_ex_pc, opB_sel, de_ex_opB);
    
    Mult2to1 ALUAinput (de_ex_rs1, U_immed, opA_sel, de_ex_opA);

    OTTER_CU_FSM CU_FSM (.CU_CLK(CLK), .CU_INT(1'b0), .CU_RESET(EXT_RESET), .CU_OPCODE(de_ex_inst[6:0]), //.CU_OPCODE(opcode),
             .CU_FUNC3(de_ex_inst[14:12]), .CU_FUNC12(de_ex_inst[31:20]), .CU_REGWRITE(de_ex_regWrite), .CU_MEMWRITE(de_ex_memWrite), 
             .CU_MEMREAD2(de_ex_memRead2), .CU_intTaken(intTaken),.CU_intCLR(intCLR),.CU_csrWrite(csrWrite),
             .CU_prevINT(prev_INT));
    //=====================================
    
    //==============Execute================
    always_ff @(posedge CLK) begin
        if (!s_stall) begin
            ex_mem_pc <= de_ex_pc;
            if (branch_taken || jump_called)
                ex_mem_inst <= 32'h00000013; //stall
            else 
                ex_mem_inst <= de_ex_inst;
            ex_mem_memWrite <= de_ex_memWrite;
            ex_mem_memRead2 <= de_ex_memRead2;
            ex_mem_regWrite <= de_ex_regWrite;
            ex_mem_rf_wr_sel <= de_ex_rf_wr_sel;
            ex_opA <= de_ex_opA;
            ex_opB <= de_ex_opB;
            ex_alu_fun <= de_ex_alu_fun;
            ex_mem_rs2 <= de_ex_rs2;
        end
    end
    
    //Branch Condition Generator     
    always_comb begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(alu_opA) < $signed(alu_opB)) br_lt=1;
        if(alu_opA==alu_opB) br_eq=1;
        if(alu_opA<alu_opB) br_ltu=1;
    end
    
    logic brn_cond;
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(ex_mem_inst[6:0]);
    
    always_comb begin
        if (ex_mem_inst[6:0] == 7'b1100011)
            case(ex_mem_inst[14:12])
                3'b000: brn_cond = br_eq;     //BEQ 
                3'b001: brn_cond = ~br_eq;    //BNE
                3'b100: brn_cond = br_lt;     //BLT
                3'b101: brn_cond = ~br_lt;    //BGE
                3'b110: brn_cond = br_ltu;    //BLTU
                3'b111: brn_cond = ~br_ltu;   //BGEU
                default: brn_cond =0;
            endcase
        else
            brn_cond = 0;
        
        branch_taken = brn_cond;
        
        case(ex_mem_inst[6:0]) //pc sel
            JAL: pc_sel = 3'b011;
            JALR: pc_sel=3'b001;
            BRANCH: pc_sel=(brn_cond)?3'b010:2'b000;
            SYSTEM: pc_sel= (ex_mem_inst[14:12]==Func3_PRIV)? 3'b101:3'b000;
            default: pc_sel=3'b000;
        endcase   
    end
    
    always_comb begin
        if (do_forwardA) //forward condition met for rs1
            if (wb_inst[6:0] == 7'b0000011 & mem_wb_inst == 32'h00000013) //handles forward from load to adjacent inst
                forwarded_valA = mem_data;
            else
                forwarded_valA = de_ex_rs1_forwarded; //get forwarded data
        else
            forwarded_valA = 32'b0;
        if (do_forwardB) //forward condition met for rs2
            if (wb_inst[6:0] == 7'b0000011 & mem_wb_inst == 32'h00000013)
                forwarded_valB = mem_data;
            else if ((ex_mem_inst[6:0] == 7'b1100111) || (ex_mem_inst[6:0] == 7'b0000011)
                     || (ex_mem_inst[6:0] == 7'b0010011)) //checks if forward accidentally caught immediate
                forwarded_valB = ex_opB;
            else
                forwarded_valB = de_ex_rs2_forwarded;
        else
            forwarded_valB = 32'b0;
    end
    
    Mult2to1 ForwardingA (ex_opA, forwarded_valA, do_forwardA, alu_opA);
    Mult2to1 ForwardingB (ex_opB, forwarded_valB, do_forwardB, alu_opB);
    
    // Creates a RISC-V ALU
    // Inputs are ALUCtl (the ALU control), ALU value inputs (ALUAin, ALUBin)
    // Outputs are ALUResultOut (the 64-bit output) and Zero (zero detection output)
    OTTER_ALU ALU (ex_alu_fun, alu_opA, alu_opB, ex_mem_aluResult); // the ALU
    
    //pc target calculations
    assign next_pc = pc + 4;    //PC is byte aligned, memory is word aligned
    assign jalr_pc = {{20{ex_mem_inst[31]}},ex_mem_inst[31:20]} + alu_opA;
    assign branch_pc = ex_mem_pc + {{20{ex_mem_inst[31]}},ex_mem_inst[7],ex_mem_inst[30:25],ex_mem_inst[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = ex_mem_pc + {{12{ex_mem_inst[31]}}, ex_mem_inst[19:12], ex_mem_inst[20],ex_mem_inst[30:21],1'b0};
    assign int_pc = 0;
    
    always_comb //decide what next pc value is
        if ((wb_inst[6:0] == 7'b1100111) || (wb_inst[6:0] == 7'b1101111))
            rf_next_pc = wb_inst_pc + 4;
        else
            rf_next_pc = next_pc;
    
    always_comb begin
        jump_called = ((ex_mem_inst[6:0] == 7'b1101111) || (ex_mem_inst[6:0] == 7'b1100111)); //JAL or JALR
    end
        
    always_ff @(posedge CLK)
        if (!s_stall) begin
            if (branch_taken || jump_called)
                changed_pc <= 1;
            else if (changed_pc == 1)
                changed_pc <= 0;
        end
    //===================================
    
    //==============Memory===============
    always_ff @(posedge CLK) begin
        if (!s_stall) begin
            mem_wb_pc <= ex_mem_pc;
            mem_wb_inst <= ex_mem_inst;
            mem_wb_aluResult <= ex_mem_aluResult;
            mem_wb_regWrite <= ex_mem_regWrite;
            mem_wb_memWrite <= ex_mem_memWrite;
            mem_wb_memRead2 <= ex_mem_memRead2;
            mem_wb_rf_wr_sel <= ex_mem_rf_wr_sel;
            if (ex_mem_inst[6:0] == 7'b0100011) 
                mem_wb_rs2 <= de_ex_rs2_forwarded;
            else 
                mem_wb_rs2 <= ex_mem_rs2;
        end
    end
    
    always_comb
        if ((mem_wb_inst[6:0] == 7'b0100011) && (wb_inst[6:0] == 7'b0000011)) //store after load
            mem_wb_mem_in = mem_data;
        else
            mem_wb_mem_in = mem_wb_rs2;
    
    //===================================
    
    //============Write Back=============
    always_ff @(posedge CLK) begin
        if (!s_stall) begin
            wb_inst_pc <= mem_wb_pc;
            wb_inst <= mem_wb_inst;
            wb_aluResult <= mem_wb_aluResult;
            wb_regWrite <= mem_wb_regWrite;
            wb_rf_wr_sel <= mem_wb_rf_wr_sel;
        end
    end
    //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (rf_next_pc,csr_reg,mem_data,wb_aluResult,wb_rf_wr_sel,rfIn);
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (de_ex_inst[19:15], de_ex_inst[24:20], wb_inst[11:7], rfIn, wb_regWrite, de_ex_rs1, de_ex_rs2, CLK, s_stall);

    //===================================
    
    //========Hazard Detection===========
    //stalling (only when use right after load or doing jump)
    always_comb
        if ((de_ex_inst[11:7] != 0) && (de_ex_inst[11:7] == if_de_inst[19:15])
            && de_ex_memRead2)
            stall_fetch = 1;
        else if ((de_ex_inst[11:7] != 0) && (de_ex_inst[11:7] == if_de_inst[24:20])
            && de_ex_memRead2 && (if_de_inst[6:0] != 7'b0100011))
            stall_fetch = 1;
        else
            stall_fetch = 0;
    
    //forwarding
    always_comb begin //booleans to separate instruction types
        inst_notU = (de_ex_inst[6:0] != 7'b0110111) && (de_ex_inst[6:0] != 7'b0010111);
        inst_notJ = (de_ex_inst[6:0] != 7'b1101111);
    end
    
    always_comb begin
        //ex hazard
        if ((ex_mem_regWrite) && (ex_mem_inst[11:7] != 0) && (ex_mem_inst[11:7] == de_ex_inst[19:15]))
            forward_A = 2'b10;
        else
            forward_A = 2'b00;
        if ((ex_mem_regWrite) && (ex_mem_inst[11:7] != 0) && (ex_mem_inst[11:7] == de_ex_inst[24:20])
            && (de_ex_inst[6:0] != 7'b0010011))
            forward_B = 2'b10;
        else
            forward_B = 2'b00;
            
        //mem hazard
        if ((mem_wb_regWrite) && (mem_wb_inst[11:7] != 0) && (mem_wb_inst[11:7] == de_ex_inst[19:15])
            && (forward_A != 2'b10))
            forward_A = 2'b01;
        else if (forward_A != 2'b10)
            forward_A = 2'b00;
        if ((mem_wb_regWrite) && (mem_wb_inst[11:7] != 0) && (mem_wb_inst[11:7] == de_ex_inst[24:20])
            && (forward_B != 2'b10) && (de_ex_inst[6:0] != 7'b0010011))
            forward_B = 2'b01;
        else if (forward_B != 2'b10)
            forward_B = 2'b00;
    end
    
    always_ff @(posedge CLK) begin //set forward values based on forward_A and/or forward_B
        if (!s_stall) begin
            if (forward_A == 2'b10)
                de_ex_rs1_forwarded <= ex_mem_aluResult;
            else if (forward_A == 2'b01)
                de_ex_rs1_forwarded <= mem_wb_aluResult;
            if (forward_B == 2'b10)
                de_ex_rs2_forwarded <= ex_mem_aluResult;
            else if (forward_B == 2'b01)
                de_ex_rs2_forwarded <= mem_wb_aluResult;
            
            if (inst_notU && inst_notJ) //see if forwarding is needed or not
                do_forwardA <= forward_A > 2'b0;
            else
                do_forwardA <= 1'b0;
            if ((de_ex_inst[6:0] != 7'b0100011) && inst_notU && inst_notJ)
                do_forwardB <= forward_B > 2'b0;
            else
                do_forwardB <= 1'b0;    
        end
    end
    
    //=============================================
 
endmodule