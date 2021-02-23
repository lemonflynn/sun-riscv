`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    07:49:31 12/15/2020 
// Design Name: 
// Module Name:    imm_gen 
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
module imm_gen(
    input [31:0]        inst,
    input [2:0]         immSel,
    output reg [31:0]   out
    );

always@(*)
begin
    case(immSel)
        `ImmSel_I:out = {{20{inst[31]}},inst[31:20]};
        `ImmSel_S:out = {{20{inst[31]}},inst[31:25],inst[11:7]};
        `ImmSel_B:out = {{19{inst[31]}},inst[7],inst[30:25],inst[11:8],1'b0};
        `ImmSel_J:out = {{12{inst[31]}},inst[19:12],inst[20],inst[30:25],inst[24:21],1'b0};
        `ImmSel_U:out = {{inst[31:12]}, 12'b0};
        default:out = 32'd0;
    endcase
end

endmodule
