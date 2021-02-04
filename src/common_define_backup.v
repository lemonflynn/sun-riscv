`ifndef OPCODE
`define OPCODE
//second most significant bit in func7, concatenate with 3 bits in func3
parameter op_add    = 4'b0_000;
parameter op_sub    = 4'b1_000;
parameter op_sll    = 4'b0_001;//shift left logic
parameter op_slt    = 4'b0_010;//set if less than
parameter op_sltu   = 4'b0_011;//set if less than unsigned
parameter op_xor    = 4'b0_100;
parameter op_srl    = 4'b0_101;//shift right logic
parameter op_sra    = 4'b1_101;//shift right arithmetic
parameter op_or     = 4'b0_110;
parameter op_and    = 4'b0_111;
//srl: padding zero on the MSB.
//sra: padding sign bit on the MSB.

//register type
parameter R_type = 5'b01100;
// Loads are encoded in the I-type format and stores are S-type.
parameter I_type    = 5'b00100;
parameter Load_type = 5'b00000;
parameter S_type    = 5'b01000;
parameter B_type    = 5'b11000;
parameter JAL_type  = 5'b11011;
parameter JALR_type = 5'b11001;
parameter AUIPC_type= 5'b00101;
parameter LUI_type  = 5'b01101;

// Load and store function codes
parameter FNC_LB    = 3'b000
parameter FNC_LH    = 3'b001
parameter FNC_LW    = 3'b010
parameter FNC_LBU   = 3'b100
parameter FNC_LHU   = 3'b101
parameter FNC_SB    = 3'b000
parameter FNC_SH    = 3'b001
parameter FNC_SW    = 3'b010

//Control signal from control module
parameter ImmSel_I      = 3'd0;
parameter ImmSel_S      = 3'd1;
parameter ImmSel_B      = 3'd2;
parameter ImmSel_J      = 3'd3;
parameter ImmSel_U      = 3'd4;
parameter PCSel_next    = 1'b0;
parameter PCSel_alu     = 1'b1;
parameter ASel_reg      = 0;
parameter ASel_pc       = 1;
parameter BSel_reg      = 0;
parameter BSel_imm      = 1;
parameter WBSel_mem     = 2'd0;
parameter WBSel_alu     = 2'd1;
parameter WBSel_pc_next = 2'd2;
parameter MemRead       = 4'b0000;
parameter MemWrite_B    = 4'b0001; //store byte
parameter MemWrite_H    = 4'b0011; //store half word
parameter MemWrite_W    = 4'b1111; //store word
`endif