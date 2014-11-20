`ifndef _MEM_ACCESS
`define _MEM_ACCESS

`include "../misc/mux.v"
`include "../misc/plus_one.v"
`include "../memory/data_mem.v"

module mem_access(RAFromPipe, ALUOut, RASelectInput, WASelectInput, MemData, DataIn, WriteMem, RAFromPipeInc);

	output [15:0] MemData, RAFromPipeInc;
	input  [15:0] ALUOut, RAFromPipe, DataIn;
	input         RASelectInput, WASelectInput, WriteMem;
	wire   [15:0] readAddSelected, writeAddSelected;
	
	mux16x2 RASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(RASelectInput), .out(readAddSelected));
	mux16x2 WASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(WASelectInput), .out(writeAddSelected));
	data_mem DataMemory(.readAdd(readAddSelected), .out(MemData), .writeAdd(writeAddSelected), .in(DataIn), .write(WriteMem));
	plus_one Inc(.in(RAFromPipe), .out(RAFromPipeInc));

endmodule

`endif
