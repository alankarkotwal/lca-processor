`ifndef _ALU
`define _ALU

`include "../misc/mux.v"
`include "../alu/adder.v"
`include "../alu/nand.v"

module alu(in1, in2, op, out, zero, carry); 
	
	output [15:0] out;
	output        zero, carry;
	input  [15:0] in1, in2;
	input         op;
	wire   [15:0] outAdd, outNand;
	wire          carryAdd;
	
	nor n1(zero,out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7],out[8],out[9],out[10],out[11],out[12],out[13],out[14],out[15]);
	
	mux16x2 m1(outAdd, outNand, op, out);
	adder16 add1(.in1(in1), .in2(in2), .out(outAdd), .carry(carryAdd));
	nand16 nand1(.in1(in1), .in2(in2), .out(outNand));
	
	assign carry = (op==0)?carryAdd:1'b0;
	
endmodule

`endif
