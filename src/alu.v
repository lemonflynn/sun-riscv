`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    07:39:33 12/08/2020 
// Design Name: 
// Module Name:    alu 
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
module alu(
    input [31:0]inputA,
    input [31:0]inputB,
    input [3:0]ALUSel,
    output reg [31:0]out
    );

always@(*)
begin
    case(ALUSel)
        `op_add:out = inputA + inputB;
        `op_sub:out = inputA - inputB;
        `op_sll:out = inputA << inputB[4:0];
        `op_slt:out = ($signed(inputA)<$signed(inputB))?32'b1:32'b0;
        `op_sltu:out = (inputA<inputB)?32'b1:32'b0;
        `op_xor:out=inputA ^ inputB;
        `op_srl:out = inputA >> inputB[4:0];
        `op_sra:out = $signed({{32{inputA[31]}}, inputA}) >> inputB[4:0];
        `op_or:out=inputA | inputB;
        `op_and:out=inputA & inputB;
        default:out = inputA + inputB;
    endcase
end

endmodule
