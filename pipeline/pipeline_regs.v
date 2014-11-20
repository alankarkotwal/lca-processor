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


module pipeline_reg3(	clk, reset, toCCR, toCCRWrite, toWriteRF, toImm970s, toPCImmInc, toALUOut, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect, toWriteMem, toRFOut, toRAOut,
			CCR, CCRWrite, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect, WriteMem, RFOut, RAOut);

endmodule


module pipeline_reg4(	clk, reset, toCCR, toCCRWrite, toWriteRF, toImm970s, toPCImmInc, toALUOut, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect, toWriteMem, toRFOut, toRAOut,
			CCR, CCRWrite, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect, WriteMem, RFOut, RAOut);

	output [15:0] Imm970s, PCImmInc, ALUOut, PCInc, RFOut, RAOut;
	output [ 2:0] WriteAdd, R7WriteSelect;
	output [ 1:0] CCR, RegWriteSelect;
	output        CCRWrite, WriteRF, WriteR7, WriteMem, RAMemSelectInput, WAMemSelectInput;
	
	input [15:0] toImm970s, toPCImmInc, toALUOut, toPCInc, toRFOut, toRAOut;
	input [ 2:0] toWriteAdd, toR7WriteSelect;
	input [ 1:0] toCCR, toRegWriteSelect;
	input        toCCRWrite, toWriteRF, toWriteR7, toWriteMem, toRAMemSelectInput, toWAMemSelectInput, clk, reset;

	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 ALUOutReg(.clk(clk), .out(ALUOut), .in(toALUOut), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 RFOutReg(.clk(clk), .out(RFOut), .in(toRFOut), .write(1'b0), .reset(reset));
	register16 RAReg(.clk(clk), .out(RAOut), .in(toRAOut), .write(1'b0), .reset(reset));
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	register3  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
	register2  RegWriteSelectReg(.clk(clk), .out(RegWriteSelect), .in(toRegWriteSelect), .write(1'b0), .reset(reset));
	register2  CCRReg(.clk(clk), .out(CCR), .in(toCCR), .write(1'b0), .reset(reset));
	register1  CCRWriteReg(.clk(clk), .out(CCRWrite), .in(toCCRWrite), .write(1'b0), .reset(reset));
	register1  WriteRFReg(.clk(clk), .out(WriteRF), .in(toWriteRF), .write(1'b0), .reset(reset));
	register1  WriteR7Reg(.clk(clk), .out(WriteR7), .in(toWriteR7), .write(1'b0), .reset(reset));
	register1  WriteMemReg(.clk(clk), .out(WriteMem), .in(toWriteMem), .write(1'b0), .reset(reset));
	register1  RAMemSelect(.clk(clk), .out(RAMemSelectInput), .in(toRAMemSelectInput), .write(1'b0), .reset(reset));
	register1  WAMemSelect(.clk(clk), .out(WAMemSelectInput), .in(toWAMemSelectInput), .write(1'b0), .reset(reset));

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
