`ifndef _PIPELINE_REGS
`define _PIPELINE_REGS

`include "../reg-file/register.v"

module pipeline_reg1(clk, reset, toPCInc, toPC, toIR, PCInc, PC, IR); // First pipeline register
	
	output [15:0] PCInc, PC, IR;
	input  [15:0] toPCInc, toPC, toIR;
	input	      reset, clk;
	
	register16 pipe1IncPC(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 pipe1PC(.clk(clk), .out(PC), .in(toPC), .write(1'b0), .reset(reset));
	register16 pipe1IR(.clk(clk), .out(IR), .in(toIR), .write(1'b0), .reset(reset));
	
endmodule


module pipeline_reg2();

endmodule


module pipeline_reg3();

endmodule


module pipeline_reg4();

endmodule


module pipeline_reg5(toCCR, toCCRWrite, toMemData, toWriteRF, toImm970s, toPCImmInc, toALUOut, toWriteMem);

endmodule

`endif
