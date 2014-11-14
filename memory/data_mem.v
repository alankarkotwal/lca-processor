`ifndef _DATA_MEM
`define _DATA_MEM

module data_mem(readAdd, out, writeAdd, in, write);

	output [15:0] out;
	input  [15:0] readAdd, writeAdd, in;
	input  write;
	
	reg [15:0] mem [0:255];
	integer i;
	
	assign out = mem[readAdd];
	
	initial begin
		for(i=0;i<256;i=i+1) begin
			mem[i] = 16'b0;
		end
	end
	
	always@(*) begin
		if(!write) begin
			mem[i] = in;
		end
	end

endmodule

`endif
