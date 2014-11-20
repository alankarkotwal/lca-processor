`ifndef _EXECUTE
`define _EXECUTE

`include "../alu/alu.v"
`include "../misc/mux.v"
`include "../reg-file/register.v"

module execute(	clk, reset, fromPlusOneMem, fromRFOut, RASelectInput, CCRWrite, CCRWriteValue
		RAOut, CCR, CCRTem);

	output [15:0] RAOut;
	output [ 1:0] CCR;
	input  [15:0] fromPlusOneMem, fromRFOut;
	input  [ 1:0] CCRWriteValue;
	input         RASelectInput, CCRWrite, ALUOp;
	wire   [15:0] ALUIn1, ALUIn2;
	wire   [ 1:0] CCRTemp;
	wire          ALUZero, ALUCarry;
	
	mux16x2 RAMux(.data0(fromPlusOneMem), .data1(fromRFOut), .selectInput(RASelectInput), .out(RAOut));
	register2 CCRReg(.clk(clk), .out(CCR), .in(CCRWriteValue), .write(CCRWrite), .reset(reset));
	alu me(.in1(ALUIn1), .in2(ALUIn2), .op(ALUOp), .out(ALUOut), .zero(ALUZero), .carry(ALUCarry));
	
	assign CCRTemp = {ALUZero, ALUCarry};

endmodule

`endif
