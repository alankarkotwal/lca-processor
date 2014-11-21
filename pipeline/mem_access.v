`ifndef _MEM_ACCESS
`define _MEM_ACCESS

`include "../misc/mux.v"
`include "../misc/plus_one.v"
`include "../memory/data_mem.v"

module mem_access(RAFromPipe, ALUOut, RASelectInput, WASelectInput, MemData, DataIn, DataInSelect, WriteMem, RAFromPipeInc, SignalC);

	output [15:0] MemData, RAFromPipeInc;
	input  [15:0] ALUOut, RAFromPipe, DataIn, SignalC;
	input         RASelectInput, WASelectInput, WriteMem, DataInSelect;
	wire   [15:0] readAddSelected, writeAddSelected, DataInSelected;
	
	mux16x2 RASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(RASelectInput), .out(readAddSelected));
	mux16x2 WASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(WASelectInput), .out(writeAddSelected));
	mux16x2 RASelect(.data0(DataIn), .data1(SignalC), .selectInput(DataInSelect), .out(DataInSelected));
	data_mem DataMemory(.readAdd(readAddSelected), .out(MemData), .writeAdd(writeAddSelected), .in(DataInSelected), .write(WriteMem));
	plus_one Inc(.in(RAFromPipe), .out(RAFromPipeInc));

endmodule

`endif
