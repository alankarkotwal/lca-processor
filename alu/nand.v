`ifndef _AND
`define _AND

module nand16(in1, in2, out);		// Implements bitwise NAND for two 16-bit numbers

	input  [15:0] in1, in2;
	output [15:0] out;
	
	assign out = ~(in1 & in2);
	
endmodule

`endif
