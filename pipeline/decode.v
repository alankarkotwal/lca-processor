`ifndef _DECODE
`define _DECODE

module decode(fromPipe1PC, fromPipe1IR, PC_Imm, rA1, rA2, wA, Sext_out, Imm970);

	output [15:0] PC_Imm, Sext_out, Imm970;
	output rA1, rA2;
	input [15:0] fromPipe1PC, fromPipe1IR;
	reg [15:0] imm6, imm9;
	reg select, offset;
	assign imm6 = {10'd0, IR[5:0]};
	assign imm9 = {7'd0, IR[8:0]};
	assign select = (IR[15:12]==4'B1000)?0:1;	//If opcode is 1000 then select data0.
	mux16x2 m1(.data0(imm9), .data1(imm6), .selectInput(select), .out(offset));
	add add1(.in1(fromPipe1PC),.in2(offset),.out(PC_Imm));
	sext6 s1(.in(IR[5:0]), .out(Sext_out));
	Imm970 = {IR[8:0], 7'd0};
	
	

endmodule

`endif