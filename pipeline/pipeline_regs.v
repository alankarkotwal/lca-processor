`ifndef _PIPELINE_REGS
`define _PIPELINE_REGS

`include "../reg-file/register.v"

module pipeline_reg1(clk, reset, toPCInc, toPC, toIR, PCInc, PC, IR,tofirst_multiple,first_multiple,flush); // First pipeline register
	
	output first_multiple
	output [15:0] PCInc, PC, IR;
	input  [15:0] toPCInc, toPC, toIR;
	input	      reset, clk;
	input tofirst_multiple,flush;
	assign inIR = (flush==1'b1?toIR:{4'b0,toIR[11:0]);//introduce a NOP, in the event of a flush
	register1 pipe1first_multiple(.clk(clk), .out(first_multiple), .in(tofirst_multiple), .write(1'b0), .reset(reset));
	register16 pipe1IncPC(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 pipe1PC(.clk(clk), .out(PC), .in(toPC), .write(1'b0), .reset(reset));
	register16 pipe1IR(.clk(clk), .out(IR), .in(inIR), .write(1'b0), .reset(reset));
	
endmodule


module pipeline_reg2(clk,reset,toMex1,Mex1,toMex2,Mex2,toMmemData,MmemData,toMmemR,MmemR,toMmemW,MmemW,toMregWB,MregWB,toMr7WB,Mr7WB,toPCInc,PCInc,toPC,PC,toIR,IR,tofirst_multiple,
first_multiple,toPCImmInc,PCImmInc,toWriteMem,WriteMem,torA1,rA1,torA2,rA2,toWriteAdd,WriteAdd,toSImm6,SImm6,toImm970s,Imm970s,toalu_ctrl,alu_ctrl,flush);
output Mex1,Mex2,MmemData,MmemR,MmemW,first_multiple,WriteMem,alu_ctrl;
output [2:0] MregWB,rA1,rA2,WriteAdd;
output [3:0] Mr7WB;
output [15:0] PCInc,PC,IR,PCImmInc,SImm6,Imm970s;
input toMex1,toMex2,toMmemData,toMmemR,toMmemW,tofirst_multiple,toWriteMem,toalu_ctrl
input [2:0] toMregWB,torA1,torA2,toWriteAdd;
input [3:0] toMr7WB;
input [15:0] toPCInc,toPC,toIR,toPCImmInc,toSImm6,toImm970s;

register1 Mex1_reg (.clk(clk), .out(Mex1), .in(toMex1) , .write(1'b0), .reset(reset));
register1 Mex2_reg (.clk(clk), .out(Mex2), .in(toMex2) , .write(1'b0), .reset(reset));
register1 MmemData_reg (.clk(clk), .out(MmemData), .in(toMmemData) , .write(1'b0), .reset(reset));
register1 MmemR_reg (.clk(clk), .out(MmemR), .in(toMmemR) , .write(1'b0), .reset(reset));
register1 MmemW_reg (.clk(clk), .out(MmemW), .in(toMmemW) , .write(1'b0), .reset(reset));
register1 first_multiple_reg (.clk(clk), .out(first_multiple), .in(tofirst_multiple) , .write(1'b0), .reset(reset));
register1 WriteMem_reg (.clk(clk), .out(WriteMem), .in(toWriteMem) , .write(1'b0), .reset(reset));
register1 alu_ctrl_reg (.clk(clk), .out(alu_ctrl), .in(toalu_ctrl) , .write(1'b0), .reset(reset));

register3 MregWB_reg (.clk(clk), .out(MregWB), .in(toMregWB) , .write(1'b0), .reset(reset));
register3 rA1_reg (.clk(clk), .out(rA1), .in(torA1) , .write(1'b0), .reset(reset));
register3 rA2_reg (.clk(clk), .out(rA2), .in(torA2) , .write(1'b0), .reset(reset));
register3 WriteAdd_reg (.clk(clk), .out(WriteAdd), .in(toWriteAdd) , .write(1'b0), .reset(reset));

register4 Mr7WB_reg (.clk(clk), .out(Mr7WB), .in(toMr7WB) , .write(1'b0), .reset(reset));


register16 PCInc_reg (.clk(clk), .out(PCInc), .in(toPCInc) , .write(1'b0), .reset(reset));
register16 PC_reg (.clk(clk), .out(PC), .in(toPC) , .write(1'b0), .reset(reset));
register16 IR_reg (.clk(clk), .out(IR), .in(toIR) , .write(1'b0), .reset(reset));
register16 PCImmInc_reg (.clk(clk), .out(PCImmInc), .in(toPCImmInc) , .write(1'b0), .reset(reset));
register16 SImm6_reg (.clk(clk), .out(SImm6), .in(toSImm6) , .write(1'b0), .reset(reset));
register16 Imm970s_reg (.clk(clk), .out(Imm970s), .in(toImm970s) , .write(1'b0), .reset(reset));


endmodule


module pipeline_reg3(	clk, reset, toWriteRF, toImm970s, toPCImmInc, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect, toWriteMem, toRFOut1,
 toRFOut2, toEqu, toSImm6, toIR,, WriteRF, Imm970s, PCImmInc, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect, WriteMem, RFOut1, RFOut2, Equ, SImm6, IR);
	
	output [15:0] Imm970s, PCImmInc, PCInc, RFOut1, RFOut2, IR, SImm6;
	output [ 2:0] WriteAdd, R7WriteSelect;
	output [ 1:0] RegWriteSelect;
	output        WriteRF, WriteR7, WriteMem, RAMemSelectInput, WAMemSelectInput, Equ;
	
	input [15:0] toImm970s, toPCImmInc, toPCInc, toRFOut, toRFOut2, toIR, toSImm6;
	input [ 2:0] toWriteAdd, toR7WriteSelect;
	input [ 1:0] toRegWriteSelect;
	input        toWriteMem, toRAMemSelectInput, toWAMemSelectInput, toEqu, clk, reset;
	
	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 RFOut1Reg(.clk(clk), .out(RFOut1), .in(toRFOut1), .write(1'b0), .reset(reset));
	register16 RFOut2Reg(.clk(clk), .out(RFOut2), .in(toRFOut2), .write(1'b0), .reset(reset));
	register16 SImm6Reg(.clk(clk), .out(SImm6), .in(toSImm6), .write(1'b0), .reset(reset));
	register16 IRReg(.clk(clk), .out(IR), .in(toIR), .write(1'b0), .reset(reset));
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	register3  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
	register2  RegWriteSelectReg(.clk(clk), .out(RegWriteSelect), .in(toRegWriteSelect), .write(1'b0), .reset(reset));
	
	register1  WriteMemReg(.clk(clk), .out(WriteMem), .in(toWriteMem), .write(1'b0), .reset(reset));
	register1  RAMemSelect(.clk(clk), .out(RAMemSelectInput), .in(toRAMemSelectInput), .write(1'b0), .reset(reset));
	register1  WAMemSelect(.clk(clk), .out(WAMemSelectInput), .in(toWAMemSelectInput), .write(1'b0), .reset(reset));
	register1  EquReg(.clk(clk), .out(Equ), .in(toEqu), .write(1'b0), .reset(reset));


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
