`ifndef _FETCH
`define _FETCH

`include "../memory/instr_mem.v"
`include "../misc/plus_one.v"
`include "../reg-file/register.v"
`include "../misc/mux.v"

module fetch(fromPipe2_PCim, fromPipe2, fromPipe3RFOut, fromPipe3PCInc, fromPipe4_Aluout, fromPipe5Mem, fromForwarding, PCWrite, PCOut, IROut, incPCOut, clk, reset);

	output [15:0] PCOut, IROut, incPCOut;
	input  [15:0] fromPipe2_PCim, fromPipe2_970, fromPipe3RFOut, fromPipe3PCInc, fromPipe4_Aluout, fromPipe5Mem;
	input  [ 2:0] fromForwarding;
	input         PCWrite, clk, reset;
	
	wire   [15:0] PCWriteWire;
		
	mux16x8 PCWriteSelect(.data0(incPCOut), .data1(fromPipe3RFOut), .data2(fromPipe5Mem), .data3(fromPipe2_PCim), .data4(fromPipe3PCInc), .data5(fromPipe2_970), .data6(fromPipe4_Aluout), .data7(16'b0), .selectInput(fromForwarding), .out(PCWriteWire));
	register16 PCReg(.clk(clk), .out(PCOut), .in(PCWriteWire), .write(PCWrite), .reset(reset));
	plus_one PlusOne(.in(PCOut), .out(incPCOut));
	instr_mem InstructionMemory(.readAdd(PCOut), .out(IROut));

endmodule

`endif
