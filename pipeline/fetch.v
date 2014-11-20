`ifndef _FETCH
`define _FETCH

`include "../memory/instr_mem.v"
`include "../misc/plus_one.v"
`include "../reg-file/register.v"
`include "../misc/mux.v"

module fetch(fromDecode, fromPipe2, fromPipe3RFOut, fromPipe3PCInc, fromPipe4, fromPipe5, fromForwarding, PCWrite, PCOut, IROut, incPCOut, clk, reset);

	output [15:0] PCOut, IROut, incPCOut;
	input  [15:0] fromDecode, fromPipe2, fromPipe3RFOut, fromPipe3PCInc, fromPipe4, fromPipe5;
	input  [ 2:0] fromForwarding;
	input         PCWrite, clk, reset;
	
	wire   [15:0] PCWriteWire;
		
	mux16x8 PCWriteSelect(.data0(16'b0), .data1(fromPipe3RFOut), .data2(incPCOut), .data3(fromDecode), .data4(fromPipe3PCInc), .data5(fromPipe2), .data6(fromPipe4), .data7(fromPipe5), .selectInput(fromForwarding), .out(PCWriteWire));
	register16 PCReg(.clk(clk), .out(PCOut), .in(PCWriteWire), .write(PCWrite), .reset(reset));
	plus_one PlusOne(.in(PCOut), .out(incPCOut));
	instr_mem InstructionMemory(.readAdd(PCOut), .out(IROut));

endmodule

`endif
