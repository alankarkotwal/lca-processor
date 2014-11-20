`ifndef _LCA_TEST
`define _LCA_TEST

`include "../processor/lca_processor.v"

module lca_test();

	reg clk, reset;

	initial begin
		$dumpfile("lca_test.vcd");
		$dumpvars(0);
		clk = 1'b0;
		reset = 1'b0;
		#15 reset = 1;
		#500 $finish;
	end
	
	always begin
		#2 clk = ~clk;
	end

endmodule

`endif
