`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: francisco flynn
// 
// Create Date:    17:49:34 12/19/2020 
// Design Name: 
// Module Name:    rsicv 
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
`include "common_define.h"

//used for forwarding logic
`define FORWARD_IF_ID	2'b00
`define FORWARD_ID_EX	2'b01
`define FORWARD_EX_MEM	2'b10
`define FORWARD_MEM_WB	2'b11
`define UPDATE          1'b0
`define STALL           1'b1
`define NO_FLUSH        1'b0
`define FLUSH           1'b1

`define BUBBLE_AT_D_STAGE 1'b0
`define BUBBLE_AT_E_STAGE 1'b1

module riscv#(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
)(
    input clk,
    input rst,
    input FPGA_SERIAL_RX,
    output FPGA_SERIAL_TX
);
// Memories
wire [11:0] bios_addra, bios_addrb;
wire [31:0] bios_douta, bios_doutb;
reg [31:0] bios_doutb_wd;
wire mmio_en;
wire [3:0] mmio_we;
wire [31:0] mmio_dout, mmio_din;
reg [31:0] mmio_dout_wd;
wire instruction_complete;
wire bios_ena, bios_enb;
wire [13:0] dmem_addr, mmio_addr;
wire [31:0] dmem_din, dmem_dout;
reg [31:0] dmem_dout_wd;
wire [3:0] dmem_we;
wire dmem_en;
wire [31:0] imem_dina, imem_doutb;
wire [13:0] imem_addra, imem_addrb;
wire [3:0] imem_wea;
wire imem_ena;
wire RegWen;
wire [2:0] WBSel;
reg [4:0] ra1, ra2, wa;
reg  [31:0] wd;
wire [31:0] rd1, rd2, branch_rd1, branch_rd2;
wire BrUn, BrLt, BrEq; 
wire [31:0] alu_a, alu_b, alu_forward_a, alu_forward_b, alu_out;
wire [31:0] pc_alu_base;
wire ASel, BSel;
wire[3:0] ALUSel;
reg [31:0] PC, inst, F_D_inst;
wire [31:0] imm_dout;
wire [2:0] ImmSel;
wire PCSel;
wire [3:0] MemRW;
//csr
wire CSRSel;
reg E_M_CSRSel;
wire csr_en;
wire [31:0] csr_data;
reg [31:0] forward_csr_data;
wire [31:0] csr_dout;
wire [11:0] csr_addr;

//pipeline register
reg [31:0] F_D_PC, D_E_PC, E_M_PC;
reg [31:0] D_E_rd1, D_E_rd2, E_M_rd2;
wire [31:0] D_E_rd2_forward;
reg [31:0] D_E_inst, E_M_inst, M_W_inst, complete_inst;
reg [31:0] E_M_csr_data;
reg [11:0] D_E_csr_addr, E_M_csr_addr;
reg [31:0] D_E_imm_dout;
reg [31:0] E_M_alu, pc_alu;
reg [31:0] M_W_wd;
//pipeline register to implement forwarding logic
reg [4:0]	D_E_ra1, E_M_ra1, D_E_ra2, D_E_wa;
reg [4:0]	E_M_wa;
reg [4:0]	M_W_wa;
reg [1:0]	Forward_A, Forward_B, branch_forward_1, branch_forward_2;
//bubble is insert in to pipeline if we need to stall pipeline for one clock.
//ID_Flush is used to flush F_D pipeline register when branch prediction failed.
reg bubble, bubble_delay1, bubble_location, bubble_location_delay1, ID_Flush;
wire [4:0] inst_opcode_5, E_M_inst_opcode_5;
wire [2:0] func3, E_M_func3;
//pipeline control register
// Signal E_ASel, E_BSel, E_ALUSel will
// be create in Decode stage, and consume in Execute stage, 
// others will flow to next stage.
reg E_ASel, E_BSel, E_CSRSel;
reg [3:0] E_ALUSel;
reg [3:0] E_MemRW;
reg [2:0] E_WBSel;
reg E_RegWen;
//M_MemRW and M_WBSel will be consumed in Memory stage,
//M_RegWen flow to next stage.
reg [3:0] M_MemRW;
reg [2:0] M_WBSel;
reg M_RegWen;
//The last signal W_RegWen will be consumed in Write
//back stage.
reg W_RegWen;

assign bios_ena = 1;
assign bios_enb = 1;
/*
* we should check carefully wheter we are accessing mmio or not,
* because access mmio would triggle hand-shaking logic of UART,
* such as
* addi x5, x5, 8
* if the alu out is 0x8000_0008, this would make mmio think this
* instruction going to transmit a byte, thus issue a wrong
* hand-shaking signal.
*/
assign mmio_en = (E_M_inst_opcode_5 == `Load_type || E_M_inst_opcode_5 == `S_type) && (E_M_alu[31] == 1'b1);
assign bios_addra = PC[11:0];
assign bios_addrb = E_M_alu[11:0];
assign dmem_addr = E_M_alu[13:0];
assign mmio_addr = E_M_alu[13:0];
assign dmem_din = E_M_rd2;
assign mmio_din = E_M_rd2;
assign dmem_we = M_MemRW;
assign mmio_we = M_MemRW;
assign dmem_en = 1;
assign imem_ena = 1;
assign imem_addra = E_M_alu[13:0];
assign imem_addrb = PC[13:0];
assign imem_dina = E_M_rd2;
assign imem_wea = M_MemRW;
assign alu_a = (E_ASel==`ASel_reg) ? D_E_rd1:D_E_PC;
//assign alu_b = (E_BSel==`BSel_reg) ? D_E_rd2:D_E_imm_dout;
assign inst_opcode_5 = inst[6:2];
assign E_M_inst_opcode_5 = E_M_inst[6:2];
assign func3 = inst[14:12];
assign E_M_func3 = E_M_inst[14:12];

assign csr_en = (M_WBSel == `WBSel_csr) ? 1:0;
assign csr_addr = inst[31:20];
assign csr_data = (CSRSel == `CSRSel_reg) ? D_E_rd1:D_E_imm_dout;

assign instruction_complete = (complete_inst != 32'b0) && complete_inst != M_W_inst;

/* decode rs1, rs2, and rd according to their instruction format,
* this can prevent the wrong value to malfunction our hazard and
* forwarding logic.
*/
always@(*)
begin
    case(inst_opcode_5)
        `R_type:begin
            ra1 = inst[19:15];
            ra2 = inst[24:20];
            wa = inst[11:7];
        end
        `I_type, `Load_type:begin
            ra1 = inst[19:15];
            ra2 = 5'b0;
            wa = inst[11:7];
        end
        `S_type, `B_type:begin
            ra1 = inst[19:15];
            ra2 = inst[24:20];
            wa = 5'b0;
        end
        `JAL_type, `LUI_type, `AUIPC_type:begin
            ra1 = 5'b0;
            ra2 = 5'b0;
            wa = inst[11:7];
        end
        `JALR_type:begin
            ra1 = inst[19:15];
            ra2 = 5'b0;
            wa = inst[11:7];
        end
        `CSR_type:begin
            if(func3 == 3'b001)
                ra1 = inst[19:15];
            else
                ra1 = 5'b0;
            ra2 = 5'b0;
            wa = inst[11:7];
        end
        default:begin
            ra1 = inst[19:15];
            ra2 = inst[24:20];
            wa = inst[11:7];
        end
    endcase
end

/* forwarding unit */
always@(*)
begin
	if(M_RegWen && E_M_wa != 5'b0 && E_M_wa == D_E_ra1)
		Forward_A = `FORWARD_EX_MEM;
	else if(W_RegWen && M_W_wa != 5'b0 && M_W_wa == D_E_ra1)
		Forward_A = `FORWARD_MEM_WB;
	else
		Forward_A = `FORWARD_ID_EX;
end

always@(*)
begin
	if(M_RegWen && E_M_wa != 5'b0 && E_M_wa == D_E_ra2)
		Forward_B = `FORWARD_EX_MEM;
	else if(W_RegWen && M_W_wa != 5'b0 && M_W_wa == D_E_ra2)
		Forward_B = `FORWARD_MEM_WB;
	else
		Forward_B = `FORWARD_ID_EX;
end

assign alu_forward_a = (Forward_A==`FORWARD_EX_MEM)?E_M_alu:((Forward_A==`FORWARD_MEM_WB)?M_W_wd:alu_a);
assign D_E_rd2_forward = (Forward_B==`FORWARD_EX_MEM)?E_M_alu:((Forward_B==`FORWARD_MEM_WB)?M_W_wd:D_E_rd2);
assign alu_forward_b = (E_BSel==`BSel_reg) ? D_E_rd2_forward:D_E_imm_dout;

/* hazard detection unit */
always@(*)
begin
    /*
    * should we use E_MemRW here to detect whether a stall is needed? 
    * because most of the instruct will set E_MemRW to MemRead.
    */
    if(E_MemRW == `MemRead && E_WBSel == `WBSel_mem &&
        D_E_wa != 0 && (D_E_wa == ra1 || D_E_wa == ra2))begin
        /* stall the pipeline */
        bubble  = `STALL;
        bubble_location = `BUBBLE_AT_D_STAGE;
    end else if(E_WBSel == `WBSel_csr && D_E_wa != 0 && (D_E_wa == ra1 || D_E_wa == ra2)) begin
        /*
        * we can detect hazard in Decode stage for csrrw instruction
        * csrrw x3, 0x10, x1
        * add x4, x2, x3
        */
        bubble  = `STALL;
        bubble_location = `BUBBLE_AT_D_STAGE;
    end else if(inst_opcode_5 == `B_type &&
        /*
        * if branch instruction need the result from previous instruction, and
        * when forwarding cannot help, we need to stall the pipeline
        * such as:
        * add x1, x3, x4
        * beq x1, x2, 8
        */
        D_E_wa != 0 && (D_E_wa == ra1 || D_E_wa == ra2))begin
        bubble  = `STALL;
        bubble_location = `BUBBLE_AT_D_STAGE;
    end else if(inst_opcode_5 == `B_type &&
        /*
        * if load followed by branch, we need to stall one more clock
        * such as:
        * ld x1, 0(x3)
        * beq x1, x2, 8
        */
        M_WBSel == `WBSel_mem && E_M_wa != 0 && (E_M_wa == ra1 || E_M_wa == ra2))begin
        bubble  = `STALL;
        bubble_location = `BUBBLE_AT_D_STAGE;
    end else if(inst_opcode_5 == `JALR_type && ra1 != 0)begin
        if(D_E_wa == ra1) begin
            /*
            * handle hazard, such as
            * add x1, x2, x3
            * jalr x4, x1, 4
            */
            bubble = `STALL;
            bubble_location = `BUBBLE_AT_D_STAGE;
        end else if(M_WBSel == `WBSel_mem && E_M_wa == ra1) begin
            /*
            * handle hazard, we need to stall one more clock, such as
            * ld x1, (x2)
            * jalr x4, x1, 4
            */
            bubble = `STALL;
            bubble_location = `BUBBLE_AT_D_STAGE;
        end else begin
            bubble = `UPDATE;
        end
    end else if(M_MemRW != `MemRead && E_WBSel == `WBSel_mem
        && E_M_alu[13:2] == alu_out[13:2])begin
        /* handle mem read after write */
        bubble  = `STALL;
        bubble_location = `BUBBLE_AT_E_STAGE;
    end else begin
        bubble  = `UPDATE;
    end
end

always@(posedge clk or negedge rst)
begin
    if(rst == 1'b0) begin
        bubble_delay1 <= 1'b0;
        bubble_location_delay1 <= 1'b0;
    end else begin
        bubble_delay1 <= bubble;
        bubble_location_delay1 <= bubble_location;
    end
end

/*
* forwarding unit for branch comparator
* since the destnatioin address of jalr instruction
* is also caculated in Decode stage, so we borrow
* the branch_forward_1 signal and branch_rd1 to
* handle data forward for jalr address caculation
* logic.
*/
always@(*)
begin
    /* do we have to check M_RegWen or W_RegWen ? */
    if(ra1 != 5'b0 && ra1 == E_M_wa)
        branch_forward_1 = `FORWARD_EX_MEM;
    else if(ra1 != 5'b0 && ra1 == M_W_wa)
        branch_forward_1 = `FORWARD_MEM_WB;
    else
        branch_forward_1 = `FORWARD_IF_ID;
end

always@(*)
begin
    if(ra2 != 5'b0 && ra2 == E_M_wa)
        branch_forward_2 = `FORWARD_EX_MEM;
    else if(ra2 != 5'b0 && ra2 == M_W_wa)
        branch_forward_2 = `FORWARD_MEM_WB;
    else
        branch_forward_2 = `FORWARD_IF_ID;
end

assign branch_rd1 = (branch_forward_1==`FORWARD_EX_MEM)?E_M_alu:((branch_forward_1==`FORWARD_MEM_WB)?M_W_wd:rd1);
assign branch_rd2 = (branch_forward_2==`FORWARD_EX_MEM)?E_M_alu:((branch_forward_2==`FORWARD_MEM_WB)?M_W_wd:rd2);

always@(posedge clk or negedge rst)
begin
    /*
    * if the first instruction is a branch instruction, we need to hold the instruction
    * until the rst is high, and ignore the control signal.
    */
    if(rst != 1'b0 &&  bubble == `UPDATE) begin
        if(ID_Flush == `NO_FLUSH)
            F_D_inst <= (PC[30]==1'b1) ? bios_douta:imem_doutb;
        else
            F_D_inst <= 32'b0;
    end else begin
            F_D_inst <= F_D_inst;
    end
end

/* borrow branch_rd1 from forwarding logic of branch comparator*/
assign pc_alu_base = (inst_opcode_5 == `JALR_type) ? branch_rd1 : F_D_PC;
always@(*)pc_alu = pc_alu_base + imm_dout;
/*
* If PCSel is not PCSel_next, which means we have a branch instruction,
* we need to flush the pipeline.
*/
always@(*)ID_Flush = (PCSel == `PCSel_next) ? `NO_FLUSH:`FLUSH;

/* forwarding unit for csr */
/* can fix such data hazard
* ld x10, (x1)
* csrrw x3, 0x10, x10
* or
* add x10, x11, x12 
* csrrw x3, 0x10, x10
*/
/* what if M_W_wa is x0 ? */
//assign forward_csr_data = (M_WBSel == `WBSel_csr) ? ((M_W_wa == E_M_ra1)? M_W_wd:E_M_csr_data):E_M_csr_data;
always@(*)
begin
    if(M_WBSel == `WBSel_csr && E_M_CSRSel == `CSRSel_reg && E_M_ra1 !=0 && M_W_wa == E_M_ra1)
        forward_csr_data = M_W_wd;
    else
        forward_csr_data = E_M_csr_data;
end

/* we need to handle big to little ending transform if we use 
* riscv32-unknown-elf-objcopy to generate our hex file, but 
* with riscv32-unknown-elf-bin2hex, we do not need it.
*/
always@(*)
begin
	//inst = {F_D_inst[7:0], F_D_inst[15:8], F_D_inst[23:16], F_D_inst[31:24]};
	inst = F_D_inst;
end

mmio # (
    .CPU_CLOCK_FREQ(CPU_CLOCK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) mmio_mem (
    .clk(clk),
    .reset(rst),
    .en(mmio_en),
    .we(mmio_we),
    .instruction_complete(instruction_complete),
    .addr(mmio_addr),
    .din(mmio_din),
    .dout(mmio_dout),
    .serial_in(FPGA_SERIAL_RX),
    .serial_out(FPGA_SERIAL_TX)
);

bios_mem bios_mem (
    .clk(clk),
    .ena(bios_ena),
    .addra(bios_addra),
    .douta(bios_douta),
    .enb(bios_enb),
    .addrb(bios_addrb),
    .doutb(bios_doutb)
);

dmem dmem (
    .clk(clk),
    .en(dmem_en),
    .we(dmem_we),
    .addr(dmem_addr),
    .din(dmem_din),
    .dout(dmem_dout)
);

imem imem (
    .clk(clk),
    .ena(imem_ena),
    .wea(imem_wea),
    .addra(imem_addra),
    .dina(imem_dina),
    .addrb(imem_addrb),
    .doutb(imem_doutb)
);

csr csr (
    .clk(clk),
    .en(csr_en),
    .addr(E_M_csr_addr),
    .din(forward_csr_data),
    .dout(csr_dout)
);

always@(*)
begin
    if(M_WBSel == `WBSel_mem) begin
        case (E_M_func3)
            `FNC_LB: begin
                mmio_dout_wd = {{24{mmio_dout[7]}}, mmio_dout[7:0]};
                bios_doutb_wd = {{24{bios_doutb[7]}}, bios_doutb[7:0]};
                dmem_dout_wd = {{24{dmem_dout[7]}}, dmem_dout[7:0]};
            end
            `FNC_LH: begin
                mmio_dout_wd = {{16{mmio_dout[15]}}, mmio_dout[15:0]};
                bios_doutb_wd = {{16{bios_doutb[15]}}, bios_doutb[15:0]};
                dmem_dout_wd = {{16{dmem_dout[15]}}, dmem_dout[15:0]};
            end
            `FNC_LW: begin
                mmio_dout_wd = mmio_dout;
                bios_doutb_wd = bios_doutb;
                dmem_dout_wd = dmem_dout;
            end
            `FNC_LBU: begin
                mmio_dout_wd = {{24'b0}, mmio_dout[7:0]};
                bios_doutb_wd = {{24'b0}, bios_doutb[7:0]};
                dmem_dout_wd = {{24'b0}, dmem_dout[7:0]};
            end
            `FNC_LHU: begin
                mmio_dout_wd = {{16'b0}, mmio_dout[15:0]};
                bios_doutb_wd = {{16'b0}, bios_doutb[15:0]};
                dmem_dout_wd = {{16'b0}, dmem_dout[15:0]};
            end
            default: begin
                mmio_dout_wd = mmio_dout;
                bios_doutb_wd = bios_doutb;
                dmem_dout_wd = dmem_dout;
            end
        endcase
    end else begin
        mmio_dout_wd = 32'b0;
        bios_doutb_wd = 32'b0;
        dmem_dout_wd = 32'b0;
    end
end

always@(*)
begin
    case (M_WBSel)
        `WBSel_mem: begin
            /* we should use E_M_alu to decide memory resource */
            if(E_M_alu[31] == 1'b1)
                wd = mmio_dout_wd;
            else if(E_M_alu[30] == 1'b1)
                wd = bios_doutb_wd;
            else if(E_M_alu[28] == 1'b1)
                wd = dmem_dout_wd;
            else
                wd = dmem_dout_wd;
            end
        `WBSel_alu: wd = E_M_alu;
        `WBSel_pc_next: wd = E_M_PC + 31'd4;
        `WBSel_csr: wd = csr_dout;
        default: wd = E_M_alu;
    endcase
end

reg_file rf (
    .clk(clk),
    .we(W_RegWen),
    .ra1(ra1), .ra2(ra2), .wa(M_W_wa),
    .wd(M_W_wd),
    .rd1(rd1), .rd2(rd2)
);

branch_comparator br(
    .input1(branch_rd1),
    .input2(branch_rd2),
    .BrUn(BrUn),
    .BrLt(BrLt),
    .BrEq(BrEq)
);

alu alu_0(
    .inputA(alu_forward_a), 
    .inputB(alu_forward_b), 
    .ALUSel(E_ALUSel), 
    .out(alu_out)
);

imm_gen imm_gen0(
    .inst(inst),
    .immSel(ImmSel),
    .out(imm_dout)
);

control control_0(
    .instruction(inst),
    .BrLt(BrLt),
    .BrEq(BrEq),
    .PCSel(PCSel),
    .RegWen(RegWen),
    .ImmSel(ImmSel),
    .BrUn(BrUn),
    .ASel(ASel),
    .BSel(BSel),
    .CSRSel(CSRSel),
    .ALUSel(ALUSel),
    .MemRW(MemRW),
    .WBSel(WBSel)
);

always@(posedge clk or negedge rst)
begin
    if(rst == 1'b0)begin
        PC <= RESET_PC; 
    end else begin
        if(bubble == `UPDATE)begin
            if(PCSel == `PCSel_next)
                PC <= PC + 31'd4;
            else
                PC <= pc_alu;
        end else begin
            PC <= PC;
        end
    end
end

//update pipeline register
always@(posedge clk or negedge rst)
begin
    if(rst == 1'b0)begin
        F_D_PC  <= RESET_PC;
        D_E_PC  <= RESET_PC;
        D_E_rd1 <= 32'd0;
        D_E_rd2 <= 32'd0;
        D_E_ra1 <= 5'd0;
        D_E_ra2 <= 5'd0;
        D_E_wa  <= 5'd0;
        E_M_wa  <= 5'd0;
        M_W_wa  <= 5'd0;
        E_M_ra1 <= 5'd0;
        E_M_rd2 <= 32'd0;
        E_M_PC  <= RESET_PC;
        E_M_alu <= 32'd0;
        M_W_wd  <= 32'd0;
        D_E_inst<= 32'd0;
        E_M_inst<= 32'd0;
        M_W_inst<= 32'd0;
        E_ASel      <= 1'b0;
        E_BSel      <= 1'b0;
        E_CSRSel    <= `CSRSel_reg;
        E_ALUSel    <= 4'b0;
        E_MemRW     <= 4'b0;
        E_WBSel     <= 3'b0;
        E_RegWen    <= 1'b0;
        M_MemRW     <= 4'b0;
        M_WBSel     <= 3'b0;
        M_RegWen    <= 1'b0;
        W_RegWen    <= 1'b0;
        bubble      <= `UPDATE;
        ID_Flush    <= `NO_FLUSH;
        pc_alu      <= RESET_PC;
    end else begin
        //fetch->Decode
        if(bubble == `UPDATE)begin
            if(ID_Flush == `NO_FLUSH)
                F_D_PC  <= PC;
            else
            /* should be zeor ? */
                F_D_PC  <= RESET_PC;
        end else begin
            F_D_PC  <= F_D_PC;
        end
        //Decode->Execute
        D_E_PC      <= F_D_PC;
        D_E_rd1     <= rd1;
        D_E_rd2     <= rd2;
        D_E_inst    <= inst;
        D_E_ra1     <= ra1;
        D_E_ra2     <= ra2;
        D_E_imm_dout<= imm_dout;
        D_E_csr_addr<= csr_addr;
        //--control signal
        E_ASel      <= ASel;
        E_BSel      <= BSel;
        E_ALUSel    <= ALUSel;
        E_WBSel     <= WBSel;
        if(bubble == `STALL)begin
            E_MemRW     <= 4'b0;
            E_RegWen    <= 1'b0;
            D_E_wa      <= 5'd0;
        end else begin
            E_MemRW     <= MemRW;
            E_RegWen    <= RegWen;
            D_E_wa      <= wa;
        end

        if(bubble_delay1 == `STALL && bubble_location_delay1 == `BUBBLE_AT_D_STAGE)
            E_M_alu     <= 32'd0;
        else
            E_M_alu     <= alu_out;
        E_M_PC      <= D_E_PC;
        E_M_rd2     <= D_E_rd2_forward;
        E_M_inst    <= D_E_inst;
        E_M_wa		<= D_E_wa;
        E_M_csr_data<= csr_data;
        E_M_csr_addr<= D_E_csr_addr;
        E_M_CSRSel  <= CSRSel;
        //Execute->Memory
        M_WBSel     <= E_WBSel;
        M_MemRW     <= E_MemRW;
        M_RegWen    <= E_RegWen;
        E_M_ra1     <= D_E_ra1;
        //Memory->Write back
        M_W_wd      <= wd;
        M_W_wa		<= E_M_wa;
        M_W_inst    <= E_M_inst;
        complete_inst <= M_W_inst;
        W_RegWen    <= M_RegWen;
    end
end
// On-chip UART

endmodule
