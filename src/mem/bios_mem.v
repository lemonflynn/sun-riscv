`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:40:56 12/18/2020 
// Design Name: 
// Module Name:    bios_mem 
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
module bios_mem (
    input clk,
    input ena,
    input [12:0] addra,
    output [31:0] douta,
    input enb,
    input [12:0] addrb,
    output reg [31:0] doutb
);
    reg [31:0] mem [4096-1:0];
	wire [10:0] addra_align;
	wire [10:0] addrb_align;
    wire[31:0] doutb_align;
	
	assign addra_align = addra[12:2];
	assign addrb_align = addrb[12:2];
    assign douta = ena?mem[addra_align]:32'b0;
    assign doutb_align = enb?mem[addrb_align]:32'b0;

    always@(*)
    begin
        case(addrb[1:0])
            2'b00: doutb = doutb_align;
            2'b01: doutb = {8'b0, {doutb_align[31:8]}};
            2'b10: doutb = {16'b0, {doutb_align[31:16]}};
            2'b11: doutb = {24'b0, {doutb_align[31:24]}};
            default: doutb = doutb_align;
        endcase
    end

    /*
    always @(posedge clk) begin
        if (ena) begin
            douta <= mem[addra_align];
        end
    end

    always @(posedge clk) begin
        if (enb) begin
            doutb <= mem[addrb_align];
        end
    end
    */

    initial begin
    	//$readmemh("/home/flynn/fpga/fpga_project_skeleton_fa20-master/software/assembly_tests/assembly_tests.hex", mem);
    end
    //`define STRINGIFY_BIOS(x) `"x/../software/bios151v3/bios151v3.hex`"
    //`ifdef SYNTHESIS
    //    initial begin
    //        $readmemh(`STRINGIFY_BIOS(`ABS_TOP), mem);
    //    end
    //`endif
endmodule
