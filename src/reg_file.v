`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:33:17 12/03/2020 
// Design Name: 
// Module Name:    reg_file 
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
module reg_file(
    input clk,
    input we,
    input [4:0] wa,
    input [4:0] ra1,
    input [4:0] ra2,
    input [31:0] wd,
    output [31:0] rd1,
    output [31:0] rd2 
    );

reg[31:0]regs[31:0];

assign rd1 =(ra1==5'b0)?32'b0:((ra1 == wa)?wd:regs[ra1]);
assign rd2 =(ra2==5'b0)?32'b0:((ra2 == wa)?wd:regs[ra2]);

always@(posedge clk)
begin
    if(we==1'b1)
        if(wa!=5'b0)
            regs[wa] <= wd;
end

endmodule
