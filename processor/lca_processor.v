



module lca_processor(clk,reset);
input clk,reset;
fetch stage1(.IR_load_mux(IR_load_mux),.new_IR_multi(new_IR_multi), .equ(equalValue), .pr2_IR(pr2_IR) ,.pr3_IR(pr3_IR) , .pr4_IR(pr4_IR), .pr5_IR(pr5_IR), .fromPipe2_PCim(fromPipe2_PCim), 
.fromPipe2_970(fromPipe2_970), .fromPipe3RFOut(fromPipe3RFOut), .fromPipe3PCInc(fromPipe3PCInc), .fromPipe4_Aluout(fromPipe4_Aluout), .fromPipe5Mem(fromPipe5Mem), .PCWrite(pc_write),
 .PCOut(to_pr1_PC), .IROut(to_pr1IR), .incPCOut(to_pr1_PCInc), .clk(clk), .reset(reset));
pipeline_reg1 p1(.clk(clk), .reset(reset), .toPCInc(to_pr1_PCInc), .toPC(to_pr1_PC), .toIR(to_pr1_IR), .PCInc(pr1_PCInc), .PC(pr1_PC), .IR(pr1_IR), .tofirst_multiple(to_pr1_first_multiple),
.first_multiple(pr1_first_multiple),.flush(flush_if_id));


decode stage2(.MmemData(to_pr2_Memdata), .fromPipe1PC(pr1_PC), .IR(pr1_IR), .PC_Imm(to_pr2_PCImmInc), .rA1(to_pr2_rA1), .rA2(to_pr2_rA2), .wA(to_pr2_WriteAdd)
, .Sext_Imm6(to_pr2_SImm6), .Imm970(to_pr2_Imm970), .Mex1(to_pr2_Mex1), .Mex2(to_pr2_Mex2), .wMem(to_pr2_WriteMem), .alu_ctrl(to_pr1_alu_ctrl), .MregWB(to_pr2_MregWB), 
.MmemR(to_pr2_MmemR), .MmemW(to_pr2_MmemW), .Mr7WB(to_pr2_Mr7WB));

pipeline_reg2 p2(.clk(clk), .reset(reset),.toMex1(to_pr2_Mex1),.Mex1(pr2_Mex1),.toMex2(to_pr2_Mex2),.Mex2(pr2_Mex2),toMmemData(to_pr2_Memdata),.MmemData(pr2_Mmemdata),
.toMmemR(to_pr2_MmemR),.MmemR(pr2_MmemR),.toMmemW(to_pr2_MmemW),.MmemW(pr2_MmemW),.toMregWB(to_pr2_MregWB), .MregWB(pr2_MregWB), .toMr7WB(to_pr2_Mr7WB), .Mr7WB(pr2_Mr7WB),
 .toPCInc(pr1_PCInc), .PCInc(pr2_PCInc), .toPC(pr1_PC), .PC(pr2_PC), .toIR(pr1_IR),.IR(pr2_IR), .tofirst_multiple(pr1_first_multiple), .first_multiple(pr2_first_multiple), 
 .toPCImmInc(to_pr2_PCImmInc),  .PCImmInc(pr2_PCImmInc), .toWriteMem(to_pr2_WriteMem), .WriteMem(pr2_WriteMem), .torA1(to_pr2_rA1), .rA1(pr2_rA1), .torA2(to_pr2_rA2),
 .rA2(pr2_rA2),toWriteAdd(to_pr2_WriteAdd), .WriteAdd(pr2_WriteAdd), .toSImm6(to_pr2_SImm6), .SImm6(pr2_SImm6), toImm970s(to_pr2_Imm970), .Imm970s(pr2_Imm970) ,
 .toalu_ctrl(to_pr1_alu_ctrl) ,.alu_ctrl(pr2_alu_ctrl),  .flush(flush_id_reg))


 
 
			//from wb
reg_read stage3(.in(writeData), .readAdd1(pr2_rA1), .readAdd2(pr2_rA2), .regValue1(regValue1), .regValue2(regValue2), .equalValue(equalValue), .write(pr5_WriteRF), 
.writeAdd(pr5_WriteAdd), .writeR7(pr5_WriteR7), .inR7(writeR7Data), .clk(clk), .reset(reset));
pipeline_reg3 p3(.clk(clk), .reset(reset), .toalu_ctrl(pr2_alu_ctrl) ,.alu_ctrl(pr3_alu_ctrl),
  .toImm970s(pr2_Imm970), .toPCImmInc(pr2_PCImmInc), .toPCInc(pr2_PCInc), .toWriteAdd(pr2_WriteAdd), .toRegWriteSelect(pr2_MregWB), .toR7WriteSelect(pr2_Mr7WB),
  .toWriteMem(pr2_WriteMem), .toRFOut(regValue1), .toRFOut2(regValue2), .toEqu(equalValue), .toSImm6(pr2_SImm6), .toIR(pr2_IR),
.WriteRF(pr3_WriteRF), .Imm970s(pr3_Imm970s), .PCImmInc(pr3_PCImmInc), .PCInc(pr3_PCInc), .WriteAdd(pr3_WriteAdd), .WriteR7(pr3_WriteR7), .RegWriteSelect(pr3_RegWriteSelect), 
.R7WriteSelect(pr3_R7WriteSelect), .WriteMem(pr3_WriteMem), .RFOut1(pr3_RFOut1), .RFOut2(pr3_RFOut2), .Equ(pr3_Equ), .SImm6(pr3_SImm6), .IR(pr3_IR));
			
			
																							//
execute stage4(	.clk(clk), .reset(reset), .ALUOut(to_pr4_ALUOut), .ALUOp(pr3_alu_ctrl),.fromPlusOneMem(, fromRFOut1, fromRFOut2, RASelectInput, CCRWrite, CCR_Write_from_wb,CCRWriteValue,CCRWriteValue_from_wb, 
fromSImm6, ExMux1Select, ExMux2Select,RAOut, CCR,IR,SignalA,SignalB,SignalC,SignalG,SignalI,SignalJ,SignalK,SignalX,SignalY,
		mem_wb_op,mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_op,ex_mem_regA,ex_mem_regB,ex_mem_regC,
		regread_ex_op,regread_ex_regA,regread_ex_regB,regread_ex_regC,,mem_wb_CCR_write,ex_mem_CCR_write,r7,rf);
		
pipeline_reg4 pr4(	clk, reset, toCCR, toCCRWrite, toWriteRF, toImm970s, toPCImmInc, .toALUOut(to_pr4_ALUOut), toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect, toWriteMem, toRFOut, toRAOut,
			CCR, CCRWrite, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect, WriteMem, RFOut, RAOut);
			
			
pipeline_reg5 pr5(	clk, reset, toCCR, toCCRWrite, toMemData, toWriteRF, toImm970s, toPCImmInc, toALUOut, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect,
			CCR, CCRWrite, MemData, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect);
			
			


mem_access stage5(IRfrompipe4, IRfrompipe5, RAFromPipe, ALUOut, RASelectInput, WASelectInput, MemData, DataIn, DataInSelect, WriteMem, RAFromPipeInc, SignalC,
				  Forwarded, F3, Rfout1, Rfout2, mem_wb_CCR_write, ex_mem_CCR_write);
write_back stage6(clk, reset, Imm970, MemData, PCImmInc, ALUOut, PCInc, regSelect, r7Select, writeData, writeR7Data);

hazard_detection hdu(IR_load_mux,new_IR_multi,pr1_first_multiple,clk,flush_reg_ex,flush_id_reg,flush_if_id,pr1_IR,pr1_pc,pr2_IR,pr2_pc,pr3_IR,pr4_IR,pc_write,equ);
endmodule


//pipeline register modules