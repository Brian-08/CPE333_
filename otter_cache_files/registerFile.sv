`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/05/2019 12:17:57 AM
// Design Name: 
// Module Name: registerFile
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

module OTTER_registerFile(Read1,Read2,WriteReg,WriteData,RegWrite,Data1,Data2,clock, RF_S_STALL);
    input [4:0] Read1,Read2,WriteReg; //the register numbers to read or write
    input [31:0] WriteData; //data to write
    input RegWrite, //the write control
        RF_S_STALL, //stall signal from cache
        clock;  // the clock to trigger write
    output logic [31:0] Data1, Data2; // the register values read
    logic [31:0] RF [31:0]; //32 registers each 32 bits long
    
    always_comb
        if(Read1==0) Data1 =0;
        else Data1 = RF[Read1];
    always_comb
        if(Read2==0) Data2 =0;
        else Data2 = RF[Read2];
        
    always@(negedge clock) begin
        if(!RF_S_STALL) begin
            if(RegWrite && WriteReg!=0) RF[WriteReg] <= WriteData;
        end
    end
 endmodule
