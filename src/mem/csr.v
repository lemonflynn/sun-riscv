`timescale 1ns / 1ps

module csr(
    input clk,
    input en,
    input [11:0] addr,
    input [31:0] din,
    output [31:0] dout
);

    reg [31:0] mem[1024-1:0];

    assign dout = mem[addr];

    always@(posedge clk) begin
        if(en)
            mem[addr] <= din;
    end

endmodule
    
