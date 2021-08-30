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
module echo_testbench();
    // Generate 125 MHz clock
    reg clk = 0;
    always #(`CLOCK_PERIOD/2) clk = ~clk;

    /*
    * in and out is looking from cpu's perspective,
    * in: receive data from off-chip uart
    * out: send data to off-chip uart
    */
    reg reset;
    reg serial_in, status;
    wire serial_out;

    riscv# (
        .CPU_CLOCK_FREQ(`CLOCK_FREQ),
        .RESET_PC(32'h4000_0000),
        .BAUD_RATE(`BAUD_RATE)
    ) CPU(
        .clk(clk),
        .rst(reset),
        .FPGA_SERIAL_RX(serial_in),
        .FPGA_SERIAL_TX(serial_out)
    );

    parameter SYMBOL_EDGE_TIME     = `CLOCK_FREQ / `BAUD_RATE;

    reg done = 0;
    integer index;
    initial begin
        $readmemh("../sim/echo.hex", CPU.bios_mem.mem);
        for(index = 0;index < 20;index = index + 1)
                $display("%x ", CPU.bios_mem.mem[index]);

        $dumpfile("echo_testbench.vcd");
        $dumpvars(0, echo_testbench);
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
                // first byte
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
                // second byte

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
