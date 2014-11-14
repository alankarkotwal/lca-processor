`ifndef _INSTR_MEM
`define _INSTR_MEM

module instr_mem(readAdd, out);

	output [15:0] out;
	input  [15:0] readAdd;
	
	reg [15:0] mem [0:255];
	integer i;
	
	assign out = mem[readAdd];
	
	initial begin
		for(i=0;i<256;i=i+1) begin
			mem[i] = 16'b0;
		end
		// Initialise program here
	end

endmodule

`endif
