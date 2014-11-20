`ifndef _REGISTER
`define _REGISTER

module register16(clk, out, in, write, reset);  // Negedge-triggered flipflop register with active-low write signal and reset

	output reg [15:0] out;
	input      [15:0] in;
	input      clk, write, reset;
	
	always@(posedge clk) begin
		if(reset==0) begin
			out = 16'b0;
		end
		else if(write == 1'b0) begin
			out = in;
		end
	end
	
endmodule


module register3(clk, out, in, write, reset);  // Negedge-triggered flipflop register with active-low write signal and reset

	output reg [2:0] out;
	input      [2:0] in;
	input      clk, write, reset;
	
	always@(posedge clk) begin
		if(reset==0) begin
			out = 3'b0;
		end
		else if(write == 1'b0) begin
			out = in;
		end
	end
	
endmodule

`endif
