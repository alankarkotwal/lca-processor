`ifndef _ADDER
`define _ADDER

module sext6(in, out);		// Sign Extension 6 to 16
	
	input  [5:0] in;
	output [15:0] out;
	assign out = {10{in[5]}, in[5:0]};
	
endmodule

`endif
