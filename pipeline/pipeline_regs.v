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


module pipeline_reg5(	clk, reset, toCCR, toCCRWrite, toMemData, toWriteRF, toImm970s, toPCImmInc, toALUOut, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect,
			CCR, CCRWrite, MemData, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect);
			
	output [15:0] MemData, Imm970s, PCImmInc, ALUOut, PCInc;
	output [ 2:0] WriteAdd, R7WriteSelect;
	output [ 1:0] CCR, RegWriteSelect;
	output        CCRWrite, WriteRF, WriteR7;
	
	input [15:0] toMemData, toImm970s, toPCImmInc, toALUOut, toPCInc;
	input [ 2:0] toWriteAdd, toR7WriteSelect;
	input [ 1:0] toCCR, toRegWriteSelect;
	input        toCCRWrite, toWriteRF, toWriteR7, clk, reset;
	
	register16 MemDataReg(.clk(clk), .out(MemData), .in(toMemData), .write(1'b0), .reset(reset));
	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 ALUOutReg(.clk(clk), .out(ALUOut), .in(toALUOut), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	register3  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
	register2  RegWriteSelectReg(.clk(clk), .out(RegWriteSelect), .in(toRegWriteSelect), .write(1'b0), .reset(reset));
	register2  CCRReg(.clk(clk), .out(CCR), .in(toCCR), .write(1'b0), .reset(reset));
	register1  CCRWriteReg(.clk(clk), .out(CCRWrite), .in(toCCRWrite), .write(1'b0), .reset(reset));
	register1  WriteRFReg(.clk(clk), .out(WriteRF), .in(toWriteRF), .write(1'b0), .reset(reset));
	register1  WriteR7Reg(.clk(clk), .out(WriteR7), .in(toWriteR7), .write(1'b0), .reset(reset));

endmodule

`endif
