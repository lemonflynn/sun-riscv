`timescale 1ns/10ps

/* MODIFY THIS LINE WITH THE HIERARCHICAL PATH TO YOUR REGFILE ARRAY INDEXED WITH reg_number */
`define REGFILE_ARRAY_PATH CPU.rf.regs[reg_number]
`define IVERILOG

module assembly_testbench();
    reg clk, rst;
	wire[31:0] temp;
    parameter CPU_CLOCK_PERIOD = 20;
    parameter CPU_CLOCK_FREQ = 50_000_000;

    initial clk = 0;
    always #(CPU_CLOCK_PERIOD/2) clk <= ~clk;

    riscv # (
        .CPU_CLOCK_FREQ(CPU_CLOCK_FREQ)
    ) CPU(
        .clk(clk),
        .rst(rst),
		.wd(temp),
        .FPGA_SERIAL_RX(),
        .FPGA_SERIAL_TX()
    );
	integer index;
    // A task to check if the value contained in a register equals an expected value
    task check_reg;
        input [4:0] reg_number;
        input [31:0] expected_value;
        input [10:0] test_num;
        if (expected_value !== `REGFILE_ARRAY_PATH) begin
            $display("FAIL - test %d, got: %d, expected: %d for reg %d", test_num, `REGFILE_ARRAY_PATH, expected_value, reg_number);
            $finish();
        end
        else begin
            $display("PASS - test %d, got: %d for reg %d", test_num, expected_value, reg_number);
        end
    endtask

    // A task that runs the simulation until a register contains some value
    task wait_for_reg_to_equal;
        input [4:0] reg_number;
        input [31:0] expected_value;
        while (`REGFILE_ARRAY_PATH !== expected_value) @(posedge clk);
    endtask

    reg done = 0;
    initial begin
        //$readmemh("../sim/assembly_tests.vh", CPU.bios_mem.mem, 0, 4095);
        $readmemh("../sim/assembly_tests.hex", CPU.bios_mem.mem);
		for(index = 0;index < 20;index = index + 1)
				$display("%x ", CPU.bios_mem.mem[index]);

        `ifndef IVERILOG
            $vcdpluson;
        `endif
        `ifdef IVERILOG
            $dumpfile("assembly_testbench.vcd");
            $dumpvars(0,assembly_testbench);
        `endif

        rst = 1;
		#10;

        // Reset the CPU
        rst = 0;
        repeat (30) @(posedge clk);             // Hold reset for 30 cycles
        rst = 1;
		$monitor("time: %0t x1 is %d, x2 is %d, x3 is %d, csr 0x10 is %d\n", $time,
                                                    CPU.rf.regs[1],
													CPU.rf.regs[2], 
													CPU.rf.regs[3],
                                                    CPU.csr.mem[16]);

        fork
            begin
                // Your processor should begin executing the code in /software/assembly_tests/start.s

                // Test ADD
                wait_for_reg_to_equal(10, 32'd100);
                check_reg(10, 32'd100, 5);
                wait_for_reg_to_equal(20, 32'd1);       // Run the simulation until the flag is set to 1
                check_reg(1, 32'd300, 1);               // Verify that x1 contains 300

                // Test BEQ
                wait_for_reg_to_equal(20, 32'd2);       // Run the simulation until the flag is set to 2
                check_reg(1, 32'd500, 2);               // Verify that x1 contains 500
                check_reg(2, 32'd100, 3);               // Verify that x2 contains 100
                $display("ALL ASSEMBLY TESTS PASSED");
                done = 1;
            end
            begin
                repeat (1000) @(posedge clk);
                if (!done) begin
                    $display("Failed: timing out");
                    $finish();
                end
            end
        join

        `ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end
endmodule
