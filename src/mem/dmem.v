`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:40:11 12/18/2020 
// Design Name: 
// Module Name:    dmem 
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
module dmem (
  input clk,
  input en,
  input [3:0] we,
  input [13:0] addr,
  input [31:0] din,
  output [31:0] dout
);
    reg [31:0] mem [16384-1:0];
    //reg [31:0] mem [4096-1:0];
    wire [11:0] addr_align;

    assign addr_align = addr[13:2];
    assign dout = en?mem[addr_align]:32'b0;

    /*
    always @(posedge clk) begin
      if (en)
        dout <= mem[addr_align];
    end
    */

    genvar i;
    generate for (i = 0; i < 4; i = i+1) begin:dmem_byte
      always @(posedge clk) begin
        if (we[i] && en)
            mem[addr_align][i*8 +: 8] <= din[i*8 +: 8];
      end
    end endgenerate
endmodule
