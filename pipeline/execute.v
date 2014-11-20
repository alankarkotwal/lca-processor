`ifndef _EXECUTE
`define _EXECUTE

`include "../alu/alu.v"
`include "../misc/mux.v"
`include "../reg-file/register.v"

module execute(	clk, reset, ALUOut, fromPlusOneMem, fromRFOut1, fromRFOut2, RASelectInput, CCRWrite, CCRWriteValue, fromSImm6, ExMux1Select, ExMux2Select,
		RAOut, CCR, CCRTemp);

	output [15:0] RAOut, ALUOut;
	output [ 1:0] CCR, CCRTemp;
	input  [15:0] fromPlusOneMem, fromRFOut1, fromRFOut2, fromSImm6;
	input  [ 1:0] CCRWriteValue;
	input         clk, reset, RASelectInput, CCRWrite, ALUOp, ExMux1Select, ExMux2Select;
	wire   [15:0] ALUIn1, ALUIn2, ExMux1Out, ExMux2Out;
	wire   [ 2:0] ExMux3Select, ExMux4Select;			// These come from the forwarding unit. Do this.
	wire          ALUZero, ALUCarry;
	
	mux16x2 RAMux(.data0(fromPlusOneMem), .data1(fromRFOut1), .selectInput(RASelectInput), .out(RAOut));
	mux16x2 ExMux1(.data0(fromRFOut1), .data1(fromSImm6), .selectInput(ExMux1Select), .out(ExMux1Out));
	mux16x2 ExMux2(.data0(fromRFOut2), .data1(fromSImm6), .selectInput(ExMux2Select), .out(ExMux2Out));
	mux16x8 ExMux3(.data0(ExMux1Out), .data1(SignalA), .data2(SignalB), .data3(SignalC), .data4(SignalG), .data5(SignalI), .data6(SignalJ), .data7(16'b0), .selectInput(ExMux3Select), .out(ALUIn1));
	mux16x8 ExMux4(.data0(ExMux2Out), .data1(SignalA), .data2(SignalB), .data3(SignalC), .data4(SignalG), .data5(SignalI), .data6(SignalJ), .data7(16'b0), .selectInput(ExMux3Select), .out(ALUIn2));
	register2 CCRReg(.clk(clk), .out(CCR), .in(CCRWriteValue), .write(CCRWrite), .reset(reset));
	alu me(.in1(ALUIn1), .in2(ALUIn2), .op(ALUOp), .out(ALUOut), .zero(ALUZero), .carry(ALUCarry));
	
	assign CCRTemp = {ALUZero, ALUCarry};

endmodule

`endif
