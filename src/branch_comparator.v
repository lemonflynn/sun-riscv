`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    07:52:45 12/16/2020 
// Design Name: 
// Module Name:    branch_comparator 
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
module branch_comparator(
    input [31:0] input1,
    input [31:0] input2,
    input BrUn,
    output reg BrLt,
    output reg BrEq
    );
always@(*)
begin
    if(BrUn==0)begin
       if(input1 == input2)begin
           BrEq = 1;
           BrLt = 0;
       end else if(input1 < input2)begin
           BrEq = 0;
           BrLt = 1;
       end else begin
           BrEq = 0;
           BrLt = 0;
       end 
    end else begin
       if($signed(input1) == $signed(input2))begin
           BrEq = 1;
           BrLt = 0;
       end else if($signed(input1) < $signed(input2))begin
           BrEq = 0;
           BrLt = 1;
       end else begin
           BrEq = 0;
           BrLt = 0;
       end 
    end
end

endmodule
