`ifndef _MEM_ACCESS
`define _MEM_ACCESS

`include "../misc/mux.v"
`include "../misc/plus_one.v"
`include "../memory/data_mem.v"

module mem_access(RAFromPipe, ALUOut, RASelectInput, WASelectInput, MemData, DataIn, DataInSelect, WriteMem, RAFromPipeInc, SignalC, Forwarded, F3, Rfout1, Rfout2);

	output [15:0] MemData, RAFromPipeInc, DataIn;
	input  [15:0] ALUOut, RAFromPipe, Rfout1, Rfout2, SignalC, Forwarded;
	input         RASelectInput, WASelectInput, WriteMem, DataInSelect;
	wire   [15:0] readAddSelected, writeAddSelected, DataInSelected;
	
	mux16x2 RASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(RASelectInput), .out(readAddSelected));
	mux16x2 WASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(WASelectInput), .out(writeAddSelected));
	mux16x4 DataSelect2(.data0(SignalC), .data1(SignalB), .data2(DataInSelected), .data3(Forwarded), .SelectInput(F3), .out(DataIn));
	mux16x2 DataSelect1(.data0(Rfout1), .data1(Rfout2), .selectInput(DataInSelect), .out(DataInSelected));
	data_mem DataMemory(.readAdd(readAddSelected), .out(MemData), .writeAdd(writeAddSelected), .in(DataIn), .write(WriteMem));
	plus_one Inc(.in(RAFromPipe), .out(RAFromPipeInc));

endmodule

`endif
