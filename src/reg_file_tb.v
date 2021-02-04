`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   08:35:33 12/04/2020
// Design Name:   reg_file
// Module Name:   E:/FPGA_test/RISC-V/src/reg_file_tb.v
// Project Name:  RISC-V
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: reg_file
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module reg_file_tb;

	// Inputs
	reg clk;
	reg write_en;
	reg [4:0] RW;
	reg [4:0] RA;
	reg [4:0] RB;
	reg [31:0] busW;

	// Outputs
	wire [31:0] busA;
	wire [31:0] busB;

	// Instantiate the Unit Under Test (UUT)
	reg_file uut (
		.clk(clk), 
		.write_en(write_en), 
		.RW(RW), 
		.RA(RA), 
		.RB(RB), 
		.busW(busW), 
		.busA(busA), 
		.busB(busB)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		write_en = 0;
		RW = 0;
		RA = 0;
		RB = 0;
		busW = 0;

		// Wait 100 ns for global reset to finish
		#100;
        $monitor("bus A %d, bus B %d, RA %d, RB %d, Rw %d\n", busA, busB, RA, RB, RW);
		write_en = 1;
        RW=1;
        busW=12;
        RA=1;
        #100;
		write_en = 0;
        #1005;
		write_en = 1;
        RW=2;
        busW=14;
        RB=2;
        #100;
		write_en = 0;
        $finish();
		// Add stimulus here

	end
always #10 clk = ~clk;     
endmodule

