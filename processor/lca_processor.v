



module lca_processor(clk,reset);
input clk,reset;
fetch stage1(.IR_load_mux(IR_load_mux),.new_IR_multi(new_IR_multi), .equ(equalValue), .pr2_IR(pr2_IR) ,.pr3_IR(pr3_IR) , .pr4_IR(pr4_IR), .pr5_IR(pr5_IR), .fromPipe2_PCim(fromPipe2_PCim), 
.fromPipe2_970(fromPipe2_970), .fromPipe3RFOut(fromPipe3RFOut), .fromPipe3PCInc(fromPipe3PCInc), .fromPipe4_Aluout(fromPipe4_Aluout), .fromPipe5Mem(fromPipe5Mem), .PCWrite(pc_write),
 .PCOut(to_pr1_PC), .IROut(to_pr1IR), .incPCOut(to_pr1_PCInc), .clk(clk), .reset(reset));
pipeline_reg1 p1(.clk(clk), .reset(reset), .toPCInc(to_pr1_PCInc), .toPC(to_pr1_PC), .toIR(to_pr1_IR), .PCInc(pr1_PCInc), .PC(pr1_PC), .IR(pr1_IR), .tofirst_multiple(to_pr1_first_multiple),
.first_multiple(pr1_first_multiple),.flush(flush_if_id));


decode stage2(.MmemData(to_pr2_Memdata), .fromPipe1PC(pr1_PC), .IR(pr1_IR), .PC_Imm(to_pr2_PCImmInc), .rA1(to_pr2_rA1), .rA2(to_pr2_rA2), .wA(to_pr2_WriteAdd)
, .Sext_Imm6(to_pr2_SImm6), .Imm970(to_pr2_Imm970), .Mex1(to_pr2_Mex1), .Mex2(to_pr2_Mex2), .wMem(to_pr2_WriteMem), .alu_ctrl(to_pr2_alu_ctrl), 
.MregWB(to_pr2_MregWB), .MmemR(to_pr2_MmemR), .MmemW(to_pr2_MmemW), .Mr7WB(to_pr2_Mr7WB));

pipeline_reg2 p2(.clk(clk), .reset(reset),.toMex1(to_pr2_Mex1),.Mex1(pr2_Mex1),.toMex2(to_pr2_Mex2),.Mex2(pr2_Mex2),toMmemData(to_pr2_Memdata),.MmemData(pr2_Mmemdata),
.toMmemR(to_pr2_MmemR),.MmemR(pr2_MmemR),.toMmemW(to_pr2_MmemW),.MmemW(pr2_MmemW),.toMregWB(to_pr2_MregWB), .MregWB(pr2_MregWB), .toMr7WB(to_pr2_Mr7WB), .Mr7WB(pr2_Mr7WB),
 .toPCInc(pr1_PCInc), .PCInc(pr2_PCInc), .toPC(pr1_PC), .PC(pr2_PC), .toIR(pr1_IR),.IR(pr2_IR), .tofirst_multiple(pr1_first_multiple), .first_multiple(pr2_first_multiple), 
 .toPCImmInc(to_pr2_PCImmInc),  .PCImmInc(pr2_PCImmInc), .toWriteMem(to_pr2_WriteMem), .WriteMem(pr2_WriteMem), .torA1(to_pr2_rA1), .rA1(pr2_rA1), .torA2(to_pr2_rA2),
 .rA2(pr2_rA2),toWriteAdd(to_pr2_WriteAdd), .WriteAdd(pr2_WriteAdd), .toSImm6(to_pr2_SImm6), .SImm6(pr2_SImm6), toImm970s(to_pr2_Imm970), .Imm970s(pr2_Imm970) ,
 .toalu_ctrl(to_pr2_alu_ctrl) ,.alu_ctrl(pr2_alu_ctrl),  .flush(flush_id_reg))


 
 
			//from wb
reg_read stage3(.in(writeData), .readAdd1(pr2_rA1), .readAdd2(pr2_rA2), .regValue1(regValue1), .regValue2(regValue2), .equalValue(equalValue), .write(pr5_WriteRF), 
.writeAdd(pr5_WriteAdd), .writeR7(pr5_WriteR7), .inR7(writeR7Data), .clk(clk), .reset(reset),);

pipeline_reg3 p3(.clk(clk), .reset(reset), .toalu_ctrl(pr2_alu_ctrl) ,.alu_ctrl(pr3_alu_ctrl),
  .toImm970s(pr2_Imm970), .toPCImmInc(pr2_PCImmInc), .toPCInc(pr2_PCInc), .toWriteAdd(pr2_WriteAdd), .toRegWriteSelect(pr2_MregWB), .toR7WriteSelect(pr2_Mr7WB),
  .toWriteMem(pr2_WriteMem), .toRFOut1(regValue1), .toRFOut2(regValue2), .toEqu(equalValue), .toSImm6(pr2_SImm6), .toIR(pr2_IR),
.WriteRF(pr3_WriteRF), .Imm970s(pr3_Imm970s), .PCImmInc(pr3_PCImmInc), .PCInc(pr3_PCInc), .WriteAdd(pr3_WriteAdd), .WriteR7(pr3_WriteR7), .RegWriteSelect(pr3_RegWriteSelect), 
.R7WriteSelect(pr3_R7WriteSelect), .WriteMem(pr3_WriteMem), .RFOut1(pr3_RFOut1), .RFOut2(pr3_RFOut2), .Equ(pr3_Equ), .SImm6(pr3_SImm6), .IR(pr3_IR)
.MemdataSelectInput(pr3_Mmemdata),.toMemdataSelectInput(pr2_Mmemdata), .RAMemSelectInput(pr3_MmemR),.toRAMemSelectInput(pr2_MmemR), .WAMemSelectInput(pr3_MmemW),
.toWAMemSelectInput(pr2_MmemW),.tofirst_multiple(pr2_first_multiple), .first_multiple(pr3_first_multiple),.Mex1(pr3_Mex1), .toMex1(pr2_mex1), .Mex2(pr3_Mex2) ,.toMex2(pr2_Mex2));
			
			
																							//
execute stage4(	.clk(clk), .reset(reset), .ALUOut(ALUOut), .ALUOp(pr3_alu_ctrl),.fromPlusOneMem(RAFromPipeInc), .fromRFOut1(pr3_RFOut1), .fromRFOut2(pr3_RFOut2),
 .RASelectInput(pr3_first_multiple), .CCRWrite(CCRWrite), .CCR_Write_from_wb(pr5_CCR_Write),.CCRWriteValue(CCRWriteValue),.CCRWriteValue_from_wb(pr5_CCR), 
.fromSImm6(pr3_SImm6), .ExMux1Select(pr3_Mex1), .ExMux2Select(pr3_Mex2), .RAOut(RAOut), .CCR(CCR), .IR(pr3_IR), .SignalA(pr4_ALUOut), .SignalB(pr5_ALUOut), 
.SignalC(pr5_MemData), .SignalG(pr4_PCImmInc), .SignalI(pr4_Imm970s), .SignalJ(pr5_Imm970s), .SignalK(pr5_PCInc), .SignalX(pr4_CCR), .SignalY(pr5_CCR),
		.mem_wb_op({pr5_IR[15:12],pr5_IR[1:0]}), .mem_wb_regA(pr5_IR[11:9]),.mem_wb_regB(pr5_IR[8:6]),.mem_wb_regC(pr5_IR[5:3]), .ex_mem_op({pr4_IR[15:12],pr4_IR[1:0]}),
		.ex_mem_regA(pr4_IR[11:9]),.ex_mem_regB(pr4_IR[8:6]),.ex_mem_regC(pr4_IR[5:3]),
		.regread_ex_op({pr3_IR[15:12],pr3_IR[1:0]}),regread_ex_regA(pr3_IR[11:9]),regread_ex_regB(pr3_IR[8:6]),.regread_ex_regC(pr3_IR[5:3]), .mem_wb_CCR_write(pr5_CCR_Write),
		.ex_mem_CCR_write(pr4_CCRWrite),.r7(r7_write),.rf(rf_write));
		
pipeline_reg4 pr4(	.clk(clk), .reset(reset), .toCCR(CCR), toCCRWrite(CCRWrite), .toWriteRF(rf_write), .toImm970s(pr3_Imm970s), .toPCImmInc(pr3_PCImmInc),
 .toALUOut(ALUOut), .toPCInc(pr3_PCInc), .toWriteAdd(pr3_WriteAdd), .toWriteR7(r7_write), .toRegWriteSelect(pr3_RegWriteSelect), 
.toR7WriteSelect(pr3_R7WriteSelect), .toWriteMem(pr3_WriteMem), .toRFOut1(pr3_RFOut1), .toRFOut2(pr3_RFOut2), .toRAOut(RAOut), .CCR(pr4_CCR),
 .CCRWrite(pr4_CCRWrite), .WriteRF(pr4_rf_write), .Imm970s(pr4_Imm970s), .PCImmInc(pr4_PCImmInc), .ALUOut(pr4_ALUOut),
 .PCInc(pr4_PCInc), .WriteAdd(pr4_WriteAdd), .WriteR7(pr4_r7_write), .RegWriteSelect(pr4_RegWriteSelect), .R7WriteSelect(pr4_R7WriteSelect), .WriteMem(pr4_WriteMem), 
 .RFOut1(pr4_RFOut1), .RFOut2(pr4_RFOut2), .RAOut(pr4_RAOut), .toIR(pr3_IR), .IR(pr4_IR),.RAMemSelectInput(pr4_MmemR),.toRAMemSelectInput(pr3_MmemR), 
 .WAMemSelectInput(pr4_MmemW),.toWAMemSelectInput(pr3_MmemW),.MemdataSelectInput(pr4_Mmemdata),.toMemdataSelectInput(pr3_Mmemdata));
			
			
			
			
			
																												
mem_access stage5(.IRfrompipe4(pr4_IR), .IRfrompipe5(pr5_IR), .RAFromPipe(pr4_RAOut), .ALUOut(pr4_ALUOut), .RAMemSelectInput(pr4_MmemR), WAMemSelectInput(pr4_MmemW)
, .MemData(MemData), DataInSelect(pr4_Mmemdata), .WriteMem(pr4_WriteMem), .RAFromPipeInc(RAFromPipeInc), .SignalC(pr5_MemData), , .Rfout1(pr4_RFOut1), 
.Rfout2(pr4_RFOut2), .mem_wb_CCR_write(pr5_CCR_Write), .ex_mem_CCR_write(pr4_CCRWrite));
			
pipeline_reg5 pr5(	.clk(clk), .reset(reset), .toCCR(pr4_CCR), .toCCRWrite(pr4_CCRWrite), .toMemData(MemData), .toWriteRF(pr4_rf_write), .toImm970s(pr4_Imm970s), 
.toPCImmInc(pr4_PCImmInc), .toALUOut(pr4_ALUOut), .toPCInc(pr4_PCInc),.toWriteAdd(pr4_WriteAdd), .toWriteR7(pr4_r7_write), .toRegWriteSelect(pr4_RegWriteSelect),
.toR7WriteSelect(pr4_R7WriteSelect),.CCR(pr5_CCR), .CCRWrite(pr5_CCR_Write), .MemData(pr5_MemData), .WriteRF(pr5_WriteRF), .Imm970s(pr5_Imm970s),
 .PCImmInc(pr5_PCImmInc), .ALUOut(pr5_ALUOut), .PCInc(pr5_PCInc),.WriteAdd(pr5_WriteAdd), .WriteR7(pr5_WriteR7), .RegWriteSelect(pr5_RegWriteSelect),
 .R7WriteSelect(pr4_R7WriteSelect),.toIR(pr4_IR), .IR(pr5_IR));
			
			


write_back stage6(clk, reset, Imm970, MemData, PCImmInc, ALUOut, PCInc, regSelect, r7Select, writeData, writeR7Data);

hazard_detection hdu(IR_load_mux,new_IR_multi,pr1_first_multiple,clk,flush_reg_ex,flush_id_reg,flush_if_id,pr1_IR,pr1_pc,pr2_IR,pr2_pc,pr3_IR,pr4_IR,pc_write,equ);
endmodule


//pipeline register modules