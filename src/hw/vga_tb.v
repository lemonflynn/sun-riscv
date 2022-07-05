`timescale 1ns / 1ps

`define CLOCK_PERIOD 8

module vga_tb;

    reg clk = 0;
    always #(`CLOCK_PERIOD/2) clk = ~clk;
    reg rst = 0;
    wire hs_o, vs_o, de_o;
    wire [7:0] rgb_r_o, rgb_g_o, rgb_b_o;

    vga vga_inst (
        .clk(clk),
        .rst(rst),
        .hs(hs_o),
        .vs(vs_o),
        .de(de_o),
        .rgb_r(rgb_r_o),
        .rgb_g(rgb_g_o),
        .rgb_b(rgb_b_o)
    );

    initial begin

        $dumpfile("vga_tb.vcd");
        $dumpvars(0, vga_tb);
        #10;
        rst = 1;
        #10;
        rst = 0;
        repeat (1920*1080*2) @(posedge clk);
        $display("test finished");
        $finish();
    end
endmodule
