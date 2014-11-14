`ifndef _EQUAL
`define _EQUAL

module equal(in1, in2, out);
	
	output out;
	input [15:0] in1, in2;
	
	assign out = (in1==in2);
	
endmodule

`endif
