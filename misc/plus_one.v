`ifndef _PLUS_ONE
`define _PLUS_ONE

module plus_one(in, out);

	output [15:0] out;
	input  [15:0] in;
	
	assign out = in + 1;

endmodule

`endif
