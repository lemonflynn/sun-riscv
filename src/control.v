`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    07:48:42 12/08/2020 
// Design Name: 
// Module Name:    control 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "common_define.h"
module control(
    input [31:0] instruction,
    input BrLt,
    input BrEq,
    output reg PCSel,
    output reg RegWen,
    output reg [2:0] ImmSel,
    output reg BrUn,
    output reg ASel,
    output reg BSel,
    output reg [3:0] ALUSel,
    output reg [3:0] MemRW,
    output reg [1:0] WBSel 
    );

parameter BEQ     = 3'b000;
parameter BNE     = 3'b001;
parameter BLT     = 3'b100;
parameter BGE     = 3'b101;
parameter BLTU    = 3'b110;
parameter BGEU    = 3'b111;

wire[4:0] opcode_5;
wire[2:0] func3;
wire func7_1;
assign opcode_5 = instruction[6:2];
assign func3 = instruction[14:12];
assign func7_1 = instruction[30];

always@(*)
begin
    //let's give these ouput signal a default value to
    //avoid create latch while systhese, otherwise we
    //have to give these all these signal a value in the
    //following decode process.
    PCSel   = `PCSel_next;
    ImmSel  = `ImmSel_I;
    BrUn = 0;
    ASel    = `ASel_reg;
    BSel    = `BSel_reg;
    ALUSel  = {func7_1, func3};
    MemRW   = `MemRead;
    RegWen  = 1;
    WBSel   = `WBSel_alu;
    case(opcode_5)
        `R_type:begin
            PCSel   = `PCSel_next;
            ALUSel  = {func7_1, func3};
            ASel    = `ASel_reg;
            BSel    = `BSel_reg;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_alu;
        end
        `I_type:begin
            PCSel   = `PCSel_next;
            ImmSel  = `ImmSel_I;
            ASel    = `ASel_reg;
            BSel    = `BSel_imm;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_alu;
            //srai and srli are indentify by func7_1
            if(func3 == 3'b101)
                ALUSel = {func7_1, func3};
            else
                ALUSel = {1'b0, func3};
        end
        `Load_type:begin
            PCSel   = `PCSel_next;
            ImmSel  = `ImmSel_I;
            ASel    = `ASel_reg;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_mem;
        end
        `S_type:begin
            PCSel   = `PCSel_next;
            ImmSel  = `ImmSel_S;
            ASel    = `ASel_reg;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            RegWen  = 0;
            case (func3)
                `FNC_SB: MemRW = `MemWrite_B;
                `FNC_SH: MemRW = `MemWrite_H;
                `FNC_SW: MemRW = `MemWrite_W;
                default: MemRW = `MemWrite_W;
            endcase
        end
        `B_type:begin
            ImmSel  = `ImmSel_B;
            ASel    = `ASel_pc;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 0;
            case(func3)
                BEQ:PCSel = BrEq?`PCSel_alu:`PCSel_next;
                BNE:PCSel = BrEq?`PCSel_next:`PCSel_alu;
                BLT:begin
                    BrUn = 0;//branch unsigned
                    PCSel = BrLt?`PCSel_alu:`PCSel_next;
                end
                BLTU:begin
                    BrUn = 1;
                    PCSel = BrLt?`PCSel_alu:`PCSel_next;
                end
                BGE:begin
                    BrUn = 0;
                    //A >= B, if !(A<B)
                    PCSel = BrLt?`PCSel_next:`PCSel_alu;
                end
                BGEU:begin
                    BrUn = 1;
                    PCSel = BrLt?`PCSel_next:`PCSel_alu;
                end
            endcase
        end
        `JAL_type:begin
            PCSel   = `PCSel_alu;
            ImmSel  = `ImmSel_J;
            ASel    = `ASel_pc;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_pc_next;
        end
        `JALR_type:begin
            PCSel   = `PCSel_alu;
            ImmSel  = `ImmSel_I;
            ASel    = `ASel_reg;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_pc_next;
        end
        `AUIPC_type:begin
            PCSel   = `PCSel_next;
            ImmSel  = `ImmSel_U;
            ASel    = `ASel_pc;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_pc_next;
        end
        `LUI_type:begin
            PCSel   = `PCSel_next;
            ImmSel  = `ImmSel_U;
            ASel    = `ASel_reg;
            BSel    = `BSel_imm;
            ALUSel  = `op_add;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_pc_next;
        end
        default:begin
            PCSel   = `PCSel_next;
            ALUSel  = {func7_1, func3};
            ASel    = `ASel_reg;
            BSel    = `BSel_reg;
            MemRW   = `MemRead;
            RegWen  = 1;
            WBSel   = `WBSel_alu;
        end
    endcase
end

endmodule
