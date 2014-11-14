`ifndef _DEMUX
`define _DEMUX

module decode8(selectInput, out);  // 8-output decoder

	output [7:0] out;
	input  [2:0] selectInput;
	
	assign out = a<<selectInput;
	
endmodule

`endif
