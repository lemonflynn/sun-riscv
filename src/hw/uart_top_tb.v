`timescale 1ns/1ps

`define CLOCK_PERIOD 8
`define CLOCK_FREQ 125_000_000
`define BAUD_RATE 115_200

/*
    In this testbench, we instantiate 2 UARTs. They are connected via the serial lines (FPGA_SERIAL_RX/TX).
    Our testbench is given access to the 1st UART's transmitter's ready/valid interface and the 2nd UART's
    receiver's ready/valid interface. The testbench then directs the transmitter to send a character and then
    waits for the receiver to acknowlege that data has been sent to it. It then reads the data from the receiver
    and compares it to what was transmitted.
*/
module uart_testbench();
    // Generate 125 MHz clock
    reg clk = 0;
    always #(`CLOCK_PERIOD/2) clk = ~clk;

    reg reset;
    reg serial_in, status;
    wire serial_out, off_chip_serial_out;


    uart_top # (
        .CLOCK_FREQ(`CLOCK_FREQ),
        .BAUD_RATE(`BAUD_RATE)
    ) on_chip_uart (
        .clk(clk),
        .reset(reset),
        .uart_rx_i(serial_in),
        .uart_tx_o(serial_out)
    );

    uart_top # (
        .CLOCK_FREQ(`CLOCK_FREQ),
        .BAUD_RATE(`BAUD_RATE)
    ) off_chip_uart (
        .clk(clk),
        .reset(reset),
        .uart_rx_i(serial_out),
        .uart_tx_o(off_chip_serial_out)
    );

    parameter SYMBOL_EDGE_TIME     = `CLOCK_FREQ / `BAUD_RATE;

    reg done = 0;
    initial begin
        $dumpfile("uart_top_tb.vcd");
        $dumpvars(0, uart_testbench);
        reset = 1'b0;
        serial_in = 1;
        repeat (2) @(posedge clk); #1;

        // Reset the UARTs
        reset = 1'b1;
        repeat (10) @(posedge clk);

        fork
            begin
                status = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 1;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 1;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 1;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 0;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 1;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                serial_in = 1;
                repeat(SYMBOL_EDGE_TIME)@(posedge clk);
                status = ~status;
                done = 1;
                status = 0;
                repeat (SYMBOL_EDGE_TIME * 20) @(posedge clk);
            end
            begin
                repeat (25000) @(posedge clk);
                if (!done) begin
                    $display("Failure: timing out");
                    $finish();
                end
            end
        join

        repeat (20) @(posedge clk);
        $display("Test Successful");
        $finish();
    end
endmodule
