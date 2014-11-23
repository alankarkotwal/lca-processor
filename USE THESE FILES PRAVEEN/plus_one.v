module plus_one(in, out);

	output [15:0] out;
	input  [15:0] in;
	
	assign out = in + 1;

endmodule