`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   08:44:51 12/09/2020
// Design Name:   alu
// Module Name:   E:/FPGA_test/RISC-V/src/alu_tb.v
// Project Name:  RISC-V
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: alu
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module alu_tb;

	// Inputs
	reg [31:0] inputA;
	reg [31:0] inputB;
	reg [3:0] ALUSel;

	// Outputs
	wire [31:0] out;
    parameter op_add    =4'b0_000;
    parameter op_sub    =4'b1_000;
    parameter op_sll    =4'b0_001;//shift left logic
    parameter op_slt    =4'b0_010;//set if less than
    parameter op_sltu   =4'b0_011;//set if less than unsigned
    parameter op_xor    =4'b0_100;
    parameter op_srl    =4'b0_101;//shift right logic
    parameter op_sra    =4'b1_101;//shift right arithmetic
    parameter op_or     =4'b0_110;
    parameter op_and    =4'b0_111;

	// Instantiate the Unit Under Test (UUT)
	alu uut (
		.inputA(inputA), 
		.inputB(inputB), 
		.ALUSel(ALUSel), 
		.out(out)
	);

	initial begin
		// Initialize Inputs
		inputA = 0;
		inputB = 0;
		ALUSel = 0;

		// Wait 100 ns for global reset to finish
		#100;
        inputA = 32'hffff_ffff_ffff_fffe;
        inputB = 32'd2;
        ALUSel = op_add;
        $display("inputA %d %x, inputB %d %x op add", inputA, inputA, inputB, inputB);
		#100;
        ALUSel = op_sub;
        $display("inputA %d %x, inputB %d %x op add", inputA, inputA, inputB, inputB);
		#100;
        ALUSel = op_sll;
		#100;
        ALUSel = op_slt;
		#100;
        ALUSel = op_sltu;
		#100;
        ALUSel = op_xor;
		#100;
        ALUSel = op_srl;
		#100;
        ALUSel = op_sra;
		#100;
        ALUSel = op_or;
		#100;
        ALUSel = op_and;
		#100;
        $finish();
        
		// Add stimulus here

	end
      
endmodule

