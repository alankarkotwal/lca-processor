module lca_processor(clk,reset);
input clk,reset;
wire [15:0] new_IR_multi, pr2_IR ,pr3_IR ,pr4_IR, pr5_IR, fromPipe2_PCim, fromPipe2_970, fromPipe3RFOut,
				fromPipe3PCInc, fromPipe4_Aluout, fromPipe5Mem, to_pr1_PC, to_pr1_IR, to_pr1_PCInc, pr1_PC,
				to_pr2_PCImmInc, pr1_PCInc, pr1_IR, to_pr2_SImm6, to_pr2_Imm970, pr2_PCInc, pr2_PC, pr2_PCImmInc,
				pr2_SImm6, pr2_Imm970, writeData, regValue1, regValue2, writeR7Data, pr3_Imm970s, pr3_PCImmInc,
				pr3_PCInc, pr3_RFOut1, pr3_RFOut2, pr3_SImm6, ALUOut, RAFromPipeInc, RAOut, pr4_ALUOut, pr5_ALUOut,
				pr5_MemData, pr4_PCImmInc, pr4_Imm970s, pr5_Imm970s, pr5_PCInc, pr4_PCInc, pr4_RFOut1, pr4_RFOut2,
				pr4_RAOut, MemData, pr5_PCImmInc, pr5_RFOut2;
wire [5:0]	mem_wb_op,ex_mem_op,regread_ex_op;
wire [2:0]	to_pr2_rA1, to_pr2_rA2, to_pr2_WriteAdd, pr2_rA1, pr2_rA2, pr2_WriteAdd, pr5_WriteAdd,
				pr3_WriteAdd, pr4_WriteAdd,modify_pr2_ra, pr4_R7WriteSelect, pr3_R7WriteSelect,pr5_R7WriteSelect,
				to_pr2_Mr7WB, pr2_Mr7WB;//ra is modified in case of lm/sm
wire [1:0]	to_pr2_MregWB,pr2_MregWB, pr3_RegWriteSelect, CCRWriteValue, pr5_CCR, CCR, pr4_CCR,
				pr4_RegWriteSelect, pr5_RegWriteSelect;
wire			IR_load_mux, equalValue, pc_write, to_pr1_first_multiple, pr1_first_multiple, flush_if_id,
				to_pr2_Memdata, to_pr2_Mex1, to_pr2_Mex2, to_pr2_WriteMem, to_pr2_alu_ctrl, to_pr2_MmemR, 
				to_pr2_MmemW, pr2_Mex1, pr2_Mex2, pr2_Mmemdata, pr2_MmemR, pr2_MmemW , pr2_first_multiple,
				pr2_WriteMem, pr2_alu_ctrl, flush_id_reg, pr5_WriteRF, pr5_WriteR7, flush_reg_ex, pr3_alu_ctrl,
				pr3_WriteRF, pr3_WriteMem, pr3_Equ, pr3_Mmemdata, pr3_MmemR, pr3_MmemW, pr3_first_multiple, pr3_Mex1,
				pr3_Mex2, CCRWrite, pr5_CCR_Write, r7_write, rf_write, pr4_CCRWrite, pr4_rf_write, pr4_r7_write,
				pr4_WriteMem, pr4_MmemR, pr4_MmemW, pr4_Mmemdata,modify_ir;//modify_ir to signal change in ir of pr2	 
			
assign 	mem_wb_op = {pr5_IR[15:12],pr5_IR[1:0]};
assign 	ex_mem_op = {pr4_IR[15:12],pr4_IR[1:0]};	
assign 	regread_ex_op = {pr3_IR[15:12],pr3_IR[1:0]};	
			 
fetch stage1(.IR_load_mux(IR_load_mux),.new_IR_multi(new_IR_multi), .equ(equalValue), .pr2_IR(pr2_IR) ,
				.pr3_IR(pr3_IR) , .pr4_IR(pr4_IR), .pr5_IR(pr5_IR), .fromPipe2_PCim(fromPipe2_PCim),
				.fromPipe2_970(fromPipe2_970), .fromPipe3RFOut(fromPipe3RFOut), .fromPipe3PCInc(fromPipe3PCInc), 
				.fromPipe4_Aluout(fromPipe4_Aluout), .fromPipe5Mem(fromPipe5Mem), .PCWrite(pc_write),
				.PCOut(to_pr1_PC), .IROut(to_pr1_IR), .incPCOut(to_pr1_PCInc), .clk(clk), .reset(reset));

pipeline_reg1 p1(.clk(clk), .reset(reset), .toPCInc(to_pr1_PCInc), .toPC(to_pr1_PC), .toIR(to_pr1_IR), 
					  .PCInc(pr1_PCInc), .PC(pr1_PC), .IR(pr1_IR), .tofirst_multiple(to_pr1_first_multiple),
					  .first_multiple(pr1_first_multiple),.flush(flush_if_id));


decode stage2(.MmemData(to_pr2_Memdata), .fromPipe1PC(pr1_PC), .IR(pr1_IR), .PC_Imm(to_pr2_PCImmInc),
					.rA1(to_pr2_rA1), .rA2(to_pr2_rA2), .wA(to_pr2_WriteAdd), .Sext_Imm6(to_pr2_SImm6), 
					.Imm970(to_pr2_Imm970), .Mex1(to_pr2_Mex1), .Mex2(to_pr2_Mex2), .wMem(to_pr2_WriteMem),
					.alu_ctrl(to_pr2_alu_ctrl), .MregWB(to_pr2_MregWB), .MmemR(to_pr2_MmemR), .MmemW(to_pr2_MmemW), 
					.Mr7WB(to_pr2_Mr7WB),.modify_pr2_ra(modify_pr2_ra),.modify_ir(modify_ir));
										
pipeline_reg2 p2(.clk(clk), .reset(reset),.toMex1(to_pr2_Mex1),.Mex1(pr2_Mex1),.toMex2(to_pr2_Mex2),
					  .Mex2(pr2_Mex2),.toMmemData(to_pr2_Memdata),.MmemData(pr2_Mmemdata),.toMmemR(to_pr2_MmemR),
					  .MmemR(pr2_MmemR),.toMmemW(to_pr2_MmemW),.MmemW(pr2_MmemW),.toMregWB(to_pr2_MregWB),
					  .MregWB(pr2_MregWB), .toMr7WB(to_pr2_Mr7WB), .Mr7WB(pr2_Mr7WB), .toPCInc(pr1_PCInc), 
					  .PCInc(pr2_PCInc), .toPC(pr1_PC), .PC(pr2_PC), .toIR(pr1_IR),.IR(pr2_IR), 
					  .tofirst_multiple(pr1_first_multiple), .first_multiple(pr2_first_multiple),
					  .toPCImmInc(to_pr2_PCImmInc),  .PCImmInc(pr2_PCImmInc), .toWriteMem(to_pr2_WriteMem), 
					  .WriteMem(pr2_WriteMem), .torA1(to_pr2_rA1), .rA1(pr2_rA1), .torA2(to_pr2_rA2),.rA2(pr2_rA2),
					  .toWriteAdd(to_pr2_WriteAdd), .WriteAdd(pr2_WriteAdd), .toSImm6(to_pr2_SImm6),.SImm6(pr2_SImm6),
					  .toImm970s(to_pr2_Imm970),.Imm970s(pr2_Imm970),.toalu_ctrl(to_pr2_alu_ctrl),
					  .alu_ctrl(pr2_alu_ctrl),  .flush(flush_id_reg),.modify_pr2_ra(modify_pr2_ra),.modify_ir(modify_ir));

			//from wb
reg_read stage3(.in(writeData),.readAdd1(pr2_rA1),.readAdd2(pr2_rA2),.regValue1(regValue1),.regValue2(regValue2),
					 .equalValue(equalValue), .write(pr5_WriteRF), .writeAdd(pr5_WriteAdd), .writeR7(pr5_WriteR7),
					 .inR7(writeR7Data), .clk(clk), .reset(reset));

pipeline_reg3 p3(.toR7WriteSelect(pr2_Mr7WB),.R7WriteSelect(pr3_R7WriteSelect),.flush(flush_reg_ex),.clk(clk),.reset(reset)
					  ,.toalu_ctrl(pr2_alu_ctrl), .alu_ctrl(pr3_alu_ctrl),
					  .toImm970s(pr2_Imm970),.toPCImmInc(pr2_PCImmInc),.toPCInc(pr2_PCInc),.toWriteAdd(pr2_WriteAdd),
					  .toRegWriteSelect(pr2_MregWB),.toWriteMem(pr2_WriteMem),
					  .toRFOut1(regValue1),.toRFOut2(regValue2),.toEqu(equalValue),.toSImm6(pr2_SImm6),.toIR(pr2_IR),
					  .Imm970s(pr3_Imm970s), .PCImmInc(pr3_PCImmInc), .PCInc(pr3_PCInc),
					  .WriteAdd(pr3_WriteAdd), .RegWriteSelect(pr3_RegWriteSelect), 
					  .WriteMem(pr3_WriteMem),.RFOut1(pr3_RFOut1),.RFOut2(pr3_RFOut2),
					  .Equ(pr3_Equ), .SImm6(pr3_SImm6), .IR(pr3_IR),.MemdataSelectInput(pr3_Mmemdata),
					  .toMemdataSelectInput(pr2_Mmemdata), .RAMemSelectInput(pr3_MmemR),.toRAMemSelectInput(pr2_MmemR),
					  .WAMemSelectInput(pr3_MmemW),.toWAMemSelectInput(pr2_MmemW),.tofirst_multiple(pr2_first_multiple),
					  .first_multiple(pr3_first_multiple),.Mex1(pr3_Mex1), .toMex1(pr2_Mex1), .Mex2(pr3_Mex2),
					  .toMex2(pr2_Mex2));
																							//
execute stage4(.clk(clk), .reset(reset), .ALUOut(ALUOut), .ALUOp(pr3_alu_ctrl),.fromPlusOneMem(RAFromPipeInc),
					.fromRFOut1(pr3_RFOut1), .fromRFOut2(pr3_RFOut2), .RASelectInput(pr3_first_multiple),
					.CCRWrite(CCRWrite), .CCR_Write_from_wb(pr5_CCR_Write),.CCRWriteValue(CCRWriteValue),
					.CCRWriteValue_from_wb(pr5_CCR), .fromSImm6(pr3_SImm6), .ExMux1Select(pr3_Mex1),
					.ExMux2Select(pr3_Mex2), .RAOut(RAOut), .IR(pr3_IR), .SignalA(pr4_ALUOut),
					.SignalB(pr5_ALUOut), .SignalC(pr5_MemData), .SignalG(pr4_PCImmInc), .SignalI(pr4_Imm970s),
					.SignalJ(pr5_Imm970s), .SignalK(pr5_PCInc), .SignalX(pr4_CCR), .SignalY(pr5_CCR),
					.mem_wb_op(mem_wb_op), .mem_wb_regA(pr5_IR[11:9]),.mem_wb_regB(pr5_IR[8:6]),
					.mem_wb_regC(pr5_IR[5:3]), .ex_mem_op(ex_mem_op),.ex_mem_regA(pr4_IR[11:9]),
					.ex_mem_regB(pr4_IR[8:6]),.ex_mem_regC(pr4_IR[5:3]),.regread_ex_op(regread_ex_op),
					.regread_ex_regA(pr3_IR[11:9]),.regread_ex_regB(pr3_IR[8:6]),.regread_ex_regC(pr3_IR[5:3]),
					.ex_mem_CCR_write(pr4_CCRWrite),.r7(r7_write),.rf(rf_write));

			
pipeline_reg4 pr4(.clk(clk), .reset(reset), .toCCR(CCRWriteValue), .toCCRWrite(CCRWrite), .toWriteRF(rf_write),
						.toImm970s(pr3_Imm970s), .toPCImmInc(pr3_PCImmInc), .toALUOut(ALUOut), .toPCInc(pr3_PCInc),
						.toWriteAdd(pr3_WriteAdd), .toWriteR7(r7_write), .toRegWriteSelect(pr3_RegWriteSelect),	
						.toR7WriteSelect(pr3_R7WriteSelect), .toWriteMem(pr3_WriteMem), .toRFOut1(pr3_RFOut1),
						.toRFOut2(pr3_RFOut2), .toRAOut(RAOut), .CCR(pr4_CCR), .CCRWrite(pr4_CCRWrite),
						.WriteRF(pr4_rf_write), .Imm970s(pr4_Imm970s), .PCImmInc(pr4_PCImmInc), .ALUOut(pr4_ALUOut),
						.PCInc(pr4_PCInc), .WriteAdd(pr4_WriteAdd), .WriteR7(pr4_r7_write),
						.RegWriteSelect(pr4_RegWriteSelect), .R7WriteSelect(pr4_R7WriteSelect),.WriteMem(pr4_WriteMem),	
						.RFOut1(pr4_RFOut1), .RFOut2(pr4_RFOut2), .RAOut(pr4_RAOut), .toIR(pr3_IR), .IR(pr4_IR),
						.RAMemSelectInput(pr4_MmemR),.toRAMemSelectInput(pr3_MmemR), .WAMemSelectInput(pr4_MmemW),
						.toWAMemSelectInput(pr3_MmemW),.MemdataSelectInput(pr4_Mmemdata),
						.toMemdataSelectInput(pr3_Mmemdata));
			

mem_access stage5(.IRfrompipe4(pr4_IR), .IRfrompipe5(pr5_IR), .RAFromPipe(pr4_RAOut), .ALUOut(pr4_ALUOut),
						.RAMemSelectInput(pr4_MmemR), .WAMemSelectInput(pr4_MmemW), .MemData(MemData),
						.DataInSelect(pr4_Mmemdata), .WriteMem(pr4_WriteMem), .RAFromPipeInc(RAFromPipeInc),
						.SignalC(pr5_MemData),.SignalB(pr5_ALUOut),.Rfout1(pr4_RFOut1),.Rfout2(pr4_RFOut2),.mem_wb_CCR_write(pr5_CCR_Write),
						.ex_mem_CCR_write(pr4_CCRWrite));
			
pipeline_reg5 pr5(.clk(clk),.reset(reset),.toCCR(pr4_CCR),.toCCRWrite(pr4_CCRWrite),.toMemData(MemData),
						.toWriteRF(pr4_rf_write),.toImm970s(pr4_Imm970s),.toPCImmInc(pr4_PCImmInc),.toALUOut(pr4_ALUOut),
						.toPCInc(pr4_PCInc),.toWriteAdd(pr4_WriteAdd), .toWriteR7(pr4_r7_write),
						.toRegWriteSelect(pr4_RegWriteSelect),.toR7WriteSelect(pr4_R7WriteSelect),.CCR(pr5_CCR),
						.CCRWrite(pr5_CCR_Write), .MemData(pr5_MemData), .WriteRF(pr5_WriteRF), .Imm970s(pr5_Imm970s),
						.PCImmInc(pr5_PCImmInc), .ALUOut(pr5_ALUOut), .PCInc(pr5_PCInc),.WriteAdd(pr5_WriteAdd),
						.WriteR7(pr5_WriteR7), .RegWriteSelect(pr5_RegWriteSelect),.R7WriteSelect(pr5_R7WriteSelect),
						.toIR(pr4_IR), .IR(pr5_IR), .toRFOut2(pr4_RFOut2), .RFOut2(pr5_RFOut2));
			
			


write_back stage6(.clk(clk), .reset(clk), .Imm970(pr5_Imm970s), .MemData(pr5_MemData), .PCImmInc(pr5_PCImmInc), .ALUOut(pr5_ALUOut), .PCInc(pr5_PCInc)
, .regSelect(pr5_RegWriteSelect), .r7Select(pr5_R7WriteSelect), .writeData(writeData), .writeR7Data(writeData),
.RFOut2(pr5_RFOut2));

hazard_detection hdu(.IR_load_mux(IR_load_mux),.new_IR_multi(new_IR_multi),.first_multiple(to_pr1_first_multiple),.clk(clk),.flush_reg_ex(flush_reg_ex),
.flush_id_reg(flush_id_reg), .flush_if_id(flush_if_id) ,.pr1_IR(pr1_IR), .pr1_pc(pr1_PC), .pr2_IR(pr2_IR), .pr2_pc(pr2_PC),.pr3_IR(pr3_IR), .pr4_IR(pr4_IR)
, .pc_write(pc_write),.equ(pr3_Equ));
endmodule


//pipeline register modules
module fetch(IR_load_mux,new_IR_multi,equ,pr2_IR ,pr3_IR ,pr4_IR, pr5_IR,fromPipe2_PCim, fromPipe2_970, fromPipe3RFOut, fromPipe3PCInc, fromPipe4_Aluout, fromPipe5Mem, PCWrite, PCOut, IROut, incPCOut, clk, reset);

	output [15:0] PCOut, IROut, incPCOut;
	input [15:0]new_IR_multi;
	wire [15:0] next_IR;//read from instruction memory
	input IR_load_mux;
	input  [15:0] fromPipe2_PCim, fromPipe2_970, fromPipe3RFOut, fromPipe3PCInc, fromPipe4_Aluout, fromPipe5Mem;
	input [15:0] pr2_IR ,pr3_IR ,pr4_IR, pr5_IR;
	wire  [ 2:0] fromForwarding;
	input         PCWrite, clk, reset;
	input equ;
	wire   [15:0] PCWriteWire;
	pc_forwarding f_u(.clk(clk),.equ(equ),.pr2_IR(pr2_IR),.pr3_IR(pr3_IR),.pr4_IR(pr4_IR),.pr5_IR(pr5_IR),.pc_mux_select(fromForwarding))	;
	mux16x8 PCWriteSelect(.data0(incPCOut), .data1(fromPipe3RFOut), .data2(fromPipe5Mem), .data3(fromPipe2_PCim), .data4(fromPipe3PCInc), .data5(fromPipe2_970), .data6(fromPipe4_Aluout), .data7(16'b0), .selectInput(fromForwarding), .out(PCWriteWire));
	register16 PCReg(.clk(clk), .out(PCOut), .in(PCWriteWire), .write(PCWrite), .reset(reset));
	plus_one PlusOne(.in(PCOut), .out(incPCOut));
	instr_mem InstructionMemory(.readAdd(PCOut), .out(next_IR));
	mux16x2 IR_write_select(.data0(next_IR), .data1(new_IR_multi), .selectInput(IR_load_mux), .out(IROut));
endmodule

module pc_forwarding(clk,equ,pr2_IR,pr3_IR,pr4_IR,pr5_IR,pc_mux_select);

parameter ADD = 6'b000000;
parameter NDU = 6'b001000;
parameter ADC = 6'b000010;
parameter ADZ = 6'b000001;
parameter ADI = 4'b0001;
parameter NDC = 6'b001010;
parameter NDZ = 6'b001001;
parameter LHI = 4'b0011;
parameter LW  = 4'b0100;
parameter SW  = 4'b0101;
parameter LM  = 4'b0110;
parameter SM  = 4'b0111;
parameter BEQ = 4'b1100;
parameter JAL = 4'b1000;
parameter JLR = 4'b1001;

parameter rb=3'd1;
parameter c=3'd2;
parameter m= 3'd3;
parameter one = 3'd4;
parameter h = 3'd5;
parameter a = 3'd6;


output reg [2:0] pc_mux_select;
input [15:0] pr2_IR,pr3_IR,pr4_IR,pr5_IR;
input equ,clk;
wire [5:0] op2,op3,op4,op5;
wire[2:0] pr2RA,pr2RB,pr4RC,pr5RA;
assign op2={pr2_IR[15:12],pr2_IR[1:0]};
assign op3={pr3_IR[15:12],pr3_IR[1:0]};
assign op4={pr4_IR[15:12],pr4_IR[1:0]};
assign op5={pr5_IR[15:12],pr5_IR[1:0]};

assign pr2RA = pr2_IR[11:9];
assign pr2RB = pr2_IR[8:6];



assign pr4RC = pr2_IR[5:3];
assign pr5RA = pr5_IR[11:9];

always @(negedge clk)
begin
if((op5[5:2]==LW||op5[5:2]==LM)&&pr5RA==3'b111)
pc_mux_select=c;//from mem
else if(op2[5:2]==LHI&&pr2RA==3'b111)
pc_mux_select=h;//970 from pr2

else if((op4==ADD||op4==NDU||op4==ADC||op4==ADZ||op4==NDC||op4==NDC||op4==NDZ)&&(pr4RC==3'b111))
	pc_mux_select=a;//ALU_out in pr4
else if(op4[5:2]==ADI&&pr2RB==3'b111)
	pc_mux_select=a;//ALU_out in pr4
else if(equ==1&&op3[5:2]==BEQ)
	pc_mux_select=one;//pc+Im6, in pr3
else if(op3[5:2]==JLR)
	pc_mux_select=rb;//from RFout2 of pr3
else if(op2[5:2]==JAL)
	pc_mux_select=m;//PC+Im6 , in pr2
else
	pc_mux_select=0;
	
end//always

endmodule






module decode(MmemData, fromPipe1PC, IR, PC_Imm, rA1, rA2, wA, Sext_Imm6, Imm970, Mex1, Mex2, wMem, alu_ctrl, MregWB, MmemR, MmemW, Mr7WB,modify_pr2_ra,modify_ir);
	output  [15:0] PC_Imm, Sext_Imm6, Imm970;

	output reg [2:0] rA1, rA2, wA;
	output reg [1:0] MregWB;
	output wire [2:0]modify_pr2_ra;
	output reg modify_ir;
	assign modify_pr2_ra=wA;
	output reg MmemData;
	integer i;
	output reg  Mex1, Mex2, wMem, alu_ctrl, MmemR, MmemW;
	output reg [2:0] Mr7WB;
	input [15:0] fromPipe1PC, IR;//from pipe1
	wire [15:0] imm6, imm9;
	wire select;
	wire [15:0] offset;
	wire [8:0] LM_Imm;	//Only for LM & SM instruction
	assign LM_Imm = IR[8:0];
	assign imm6 = {10'd0, IR[5:0]};
	assign imm9 = {7'd0, IR[8:0]};
	assign select = (IR[15:12]==4'B1000)?1'b0:1'b1;	//If opcode is 1000 then select data0.
	mux16x2 m1(.data0(imm9), .data1(imm6), .selectInput(select), .out(offset));
	add add1(.in1(fromPipe1PC),.in2(offset),.out(PC_Imm));
	sext6 s1(.in(IR[5:0]), .out(Sext_Imm6));
	assign Imm970 = {IR[8:0], 7'd0};
	
	always@(*)	
	begin
	
		case (IR[15:12])
			
			4'b0000:	//ADD, ADC, ADZ
			begin
				modify_ir <= 1'b0;
				rA2<= IR[8:6];	//RB
				wA<= IR[5:3];	//RC
				rA1<= IR[11:9];	//RA
				Mex1<= 0;	//Rfout1
				Mex2<= 0;	//Rfout2
				alu_ctrl<=0;	//Add operation
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1;	//Write back Alu_out
				if(IR[5:3]==3'b111)	//If RC is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0001:	//ADI
			begin
				modify_ir <= 1'b0;
				rA1<= IR[11:9];	//RA
				rA2<= 3'b000;	//Don't Care
				wA<= IR[8:6];	//RB
				Mex1<= 0;	//Rfout1
				Mex2<= 1;	//Sext_Imm6;	
				alu_ctrl<=0;	//ADD
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1; 	//Write back Alu_out
				if(IR[8:6]==3'b111)	//If RB is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0010:	//NDU, NDC, NDZ
			begin
				modify_ir <= 1'b0;
				rA2<= IR[8:6];	//RB
				wA<= IR[5:3];	//RC
				rA1<= IR[11:9];	//RA
				Mex1<= 0;	//Rfout1
				Mex2<= 0;	//Rfout2
				alu_ctrl<=1;	//Nand operation
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1;	//Write back Alu_out
				if(IR[5:3]==3'b111)	//If RC is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0011:	//LHI
			begin
				modify_ir <= 1'b0;
				wA<= IR[11:9];	//RA
				rA1<= 3'b000;	//Don't Care
				rA2<= 3'b000;	//Don't Care
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't care
				alu_ctrl<=0;	//Don't care
				wMem<=1;	//No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=2;	//Write back Imm970 to RA
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=0;	// Write back Imm970 to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0100:	//LW
			begin
				modify_ir <= 1'b0;
				wA<= IR[11:9];	//RA
				rA2<=IR[8:6];	//RB
				rA1<=3'b000;	//Don't Care
				Mex1<=1;	//Sign Extended Immediate six bit
				Mex2<=0;	//Rfout2
				alu_ctrl<=0;	//ADD
				wMem<=1;	//No memory write
				MmemR<=1;	//Read from memory using address in Alu_out
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=0;	//Write back mem_data
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=1;	// Write back mem_data to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0101:	//SW
			begin
				modify_ir <= 1'b0;
				rA2<= IR[8:6];	//RB
				rA1<= IR[11:9];	//RA
				wA<= 3'b000;	//Don't Care
				Mex1<=1;	//Sign Extended Immediate six bit
				Mex2<=0;	//Rfout2
				alu_ctrl<=0;	//Add
				wMem<=0;	// Write to memory
				MmemR<=0;	//Don't Care
				MmemW<=1;	//Write to memory. address in Alu_out
				MmemData<=0;	//Write to memory. Data present in rfout1
				MregWB<=0;	//Don't Care
				Mr7WB<=5;	//PC Increment
			end
			
			4'b0110:	//LM
			begin
				
				rA1<= IR[11:9];	//RA
				rA2<=3'b000;	//Don't Care
				if(LM_Imm[0]==1)
					wA <=3'b000;
				else if(LM_Imm[1]==1)
					wA <=3'b001;
				else if(LM_Imm[2]==1)
					wA <=3'b010;
				else if(LM_Imm[3]==1)
					wA <=3'b011;
				else if(LM_Imm[4]==1)
					wA <=3'b100;
				else if(LM_Imm[5]==1)
					wA <=3'b101;
				else if(LM_Imm[6]==1)
					wA <=3'b110;
				else if(LM_Imm[7]==1)
					wA <=3'b111;
				else 
					wA <=3'b000;
				modify_ir <= 1'b1;
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//No memory write operation
				MmemR<=0;	//Read from memory using address stored in RA
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=0;	//Write back the value in mem_data
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=1;	// Write back mem_data to R7
				else
					Mr7WB<=5;	//PC Increment
			end
			
			4'b0111:	//SM
			begin
				modify_ir <= 1'b0;
				rA1<= IR[11:9];	//RA
				wA<= 3'b000;	//Don't Care
				if(LM_Imm[0]==1)
					rA2 <=3'b000;
				else if(LM_Imm[1]==1)
					rA2 <=3'b001;
				else if(LM_Imm[2]==1)
					rA2 <=3'b010;
				else if(LM_Imm[3]==1)
					rA2 <=3'b011;
				else if(LM_Imm[4]==1)
					rA2 <=3'b100;
				else if(LM_Imm[5]==1)
					rA2 <=3'b101;
				else if(LM_Imm[6]==1)
					rA2 <=3'b110;
				else if(LM_Imm[7]==1)
					rA2 <=3'b111;
				else 
					rA2 <=3'b000;
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//Write memory operation done
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Write to memory. address in RA
				MmemData<=1;	//Write to memory. data in Rfout2
				MregWB<=0;	//Don't Care
				Mr7WB<=5;	//PC Increment
			end
			
			4'b1100:	//BEQ
			begin
				modify_ir <= 1'b0;
				rA1<= IR[11:9];	//RA
				rA2<= IR[8:6];	//RB
				wA<= 3'b000;	//Don't Care
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=0;	//Don't Care
				Mr7WB<=2;	//PC_Imm -> r7
			end
			
			4'b1000:	//JAL
			begin
				modify_ir <= 1'b0;
				rA1<= 3'b000;	//Don't Care
				rA2<= 3'b000;	//Don't Care
				wA<= IR[11:9];	//RA
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//Don't disturb your memory
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=3;	//Write back PC+1
				Mr7WB<=2;	//PC_Imm -> r7
			end
			
			4'b1001:	//JLR
			begin
				modify_ir <= 1'b0;
				rA1<= 3'b000;	//Don't Care
				rA2<= IR[8:6];	//RB
				wA<= IR[11:9];	//RA
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//Don't disturb your memory
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=3;	//Write back PC+1
				Mr7WB<=4;	//Write Rfout2 to R7
			end
			
			default:
			begin
				modify_ir <= 1'b0;
				rA1<=3'b000;	
				rA2<=3'b000;	
				wA<= 3'b000;
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't Care
				alu_ctrl<=0;	//Don't Care
				wMem<=1;	//Don't disturb your memory
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=0;	
				Mr7WB<=5;	//PC Increment
			end
		endcase
	end
endmodule


module sext6(in, out);		// Sign Extension 6 to 16
	input  [5:0] in;
	output [15:0] out;
	assign out = {{10{in[5]}}, in[5:0]};
endmodule

module add(in1, in2 , out);		// Implements a full 16-bit adder	
	output [15:0] out;
	input  [15:0] in1, in2;
	wire   [16:0] outTemp;
	assign outTemp = in1 + in2;
	assign out     = outTemp[15:0];
endmodule


module reg_read(in, readAdd1, readAdd2, regValue1, regValue2, equalValue, write, writeAdd, writeR7, inR7, clk, reset);

	output [15:0] regValue1, regValue2;
	output 	      equalValue;
	input  [15:0] in, inR7;
	input  [2:0]  readAdd1, readAdd2, writeAdd;
	input	      write, writeR7, clk, reset;
	
	register_file rfile(.clk(clk), .out1(regValue1), .out2(regValue2), .readAdd1(readAdd1), .readAdd2(readAdd2), .write(write), .writeAdd(writeAdd), .writeR7(writeR7), .inR7(inR7), .in(in), .reset(reset));
	equal eqCheck(.in1(regValue1), .in2(regValue2), .out(equalValue));

endmodule

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

module decode8(selectInput, out);  // 8-output decoder

	output reg [7:0] out;
	input  [2:0] selectInput;
	
	always@(selectInput) begin
		case(selectInput)
			0: out = 8'b00000001;
			1: out = 8'b00000010;
			2: out = 8'b00000100;
			3: out = 8'b00001000;
			4: out = 8'b00010000;
			5: out = 8'b00100000;
			6: out = 8'b01000000;
			7: out = 8'b10000000;
		endcase
	end
	
endmodule

module equal(in1, in2, out);
	
	output out;
	input [15:0] in1, in2;
	
	assign out = (in1==in2);
	
endmodule

module execute(	clk, reset, ALUOut, ALUOp,fromPlusOneMem, fromRFOut1, fromRFOut2, RASelectInput, CCRWrite, CCR_Write_from_wb,CCRWriteValue,
CCRWriteValue_from_wb, fromSImm6, ExMux1Select, ExMux2Select,
		RAOut,IR,SignalA,SignalB,SignalC,SignalG,SignalI,SignalJ,SignalK,SignalX,SignalY,
		mem_wb_op,mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_op,ex_mem_regA,ex_mem_regB,ex_mem_regC,
		regread_ex_op,regread_ex_regA,regread_ex_regB,regread_ex_regC,ex_mem_CCR_write,r7,rf);

		
parameter ADD = 6'b000000;
parameter NDU = 6'b001000;
parameter ADC = 6'b000010;
parameter ADZ = 6'b000001;
parameter ADI = 4'b0001;
parameter NDC = 6'b001010;
parameter NDZ = 6'b001001;
	output [15:0] RAOut, ALUOut;
	
	output r7,rf;
	output    reg    CCRWrite;//send to pipeline register
	output  [ 1:0] CCRWriteValue;//send to pipeline register
	input CCR_Write_from_wb;
	input  [15:0] fromPlusOneMem, fromRFOut1, fromRFOut2, fromSImm6;
	input [15:0] SignalA,SignalB,SignalC,SignalG,SignalI,SignalJ,SignalK;
	input [1:0] SignalX,SignalY;
	input [1:0] CCRWriteValue_from_wb;
	input         clk, reset, RASelectInput, ALUOp, ExMux1Select, ExMux2Select;
	input [15:0]IR;
	
	//inputs required for forwarding
	input [2:0] mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_regA,ex_mem_regB,ex_mem_regC,regread_ex_regA,regread_ex_regB,regread_ex_regC;
	input [5:0]mem_wb_op,ex_mem_op,regread_ex_op;
	input ex_mem_CCR_write;
	//
	
	
	wire   [15:0] ALUIn1, ALUIn2, ExMux1Out, ExMux2Out;
	wire[1:0] CCRMux_out;//CCRMux_out[1] = zero, CCRMux_out[0] = carry
	wire[1:0] CCR_muxSelect;
	wire [1:0] CCR;
	wire   [ 2:0] ExMux3Select, ExMux4Select;			// These come from the forwarding unit. Do this.
	wire          ALUZero, ALUCarry;
	wire [5:0]ALU_op;//to check whether we must write to CCR or not
	assign ALU_op = {IR[15:12],IR[1:0]};
	mux2x4 CCR_mux(.data0(CCR),.data1(SignalX),.data2(SignalY),.data3(2'b0),.selectInput(CCR_muxSelect),.out(CCRMux_out));
	mux16x2 RAMux(.data0(fromPlusOneMem), .data1(fromRFOut1), .selectInput(RASelectInput), .out(RAOut));
	mux16x2 ExMux1(.data0(fromRFOut1), .data1(fromSImm6), .selectInput(ExMux1Select), .out(ExMux1Out));//for input 1 of ALU
	mux16x2 ExMux2(.data0(fromRFOut2), .data1(fromSImm6), .selectInput(ExMux2Select), .out(ExMux2Out));//for input 2 of ALU
	mux16x8 ExMux3(.data0(ExMux1Out), .data1(SignalA), .data2(SignalB), .data3(SignalC), .data4(SignalG), .data5(SignalI), .data6(SignalJ), .data7(SignalK), .selectInput(ExMux3Select), .out(ALUIn1));
	mux16x8 ExMux4(.data0(ExMux2Out), .data1(SignalA), .data2(SignalB), .data3(SignalC), .data4(SignalG), .data5(SignalI), .data6(SignalJ), .data7(SignalK), .selectInput(ExMux4Select), .out(ALUIn2));
	register2 CCRReg(.clk(clk), .out(CCR), .in(CCRWriteValue_from_wb), .write(CCR_Write_from_wb), .reset(reset));
	alu me(.in1(ALUIn1), .in2(ALUIn2), .op(ALUOp), .out(ALUOut), .zero(ALUZero), .carry(ALUCarry));
	forward_ex_stage f_ex(.mem_wb_op(mem_wb_op),.mem_wb_regA(mem_wb_regA),.mem_wb_regB(mem_wb_regB),.mem_wb_regC(mem_wb_regC),.ex_mem_op(ex_mem_op),.ex_mem_regA(ex_mem_regA),.ex_mem_regB(ex_mem_regB),.ex_mem_regC(ex_mem_regC),.regread_ex_op(regread_ex_op),.regread_ex_regA(regread_ex_regA),.regread_ex_regB(regread_ex_regB),
.regread_ex_regC(regread_ex_regC),.F1(ExMux3Select),.F2(ExMux4Select),.FCCR(CCR_muxSelect), .mem_wb_CCR_write(CCR_Write_from_wb), .ex_mem_CCR_write(ex_mem_CCR_write), .writerf(rf), .writer7(r7));
	
	assign CCRWriteValue = {ALUZero, ALUCarry};
	always @(*)
	begin
	
	if(ALU_op==ADD||ALU_op==ADI||ALU_op==NDU)
		CCRWrite=1'b0;
	else if((ALU_op==ADC||ALU_op==NDC)&&	(CCRMux_out[0]==1'b1))//previous carry set or not
		CCRWrite=1'b0;
	else if((ALU_op==ADZ||ALU_op==NDZ) &&(CCRMux_out[1]==1'b1)) //previous zero flag set or not
		CCRWrite=1'b0;
	else 
		CCRWrite=1'b1;
	end
	
		
endmodule


module alu(in1, in2, op, out, zero, carry); 
	
	output [15:0] out;
	output        zero, carry;
	input  [15:0] in1, in2;
	input         op;
	wire   [15:0] outAdd, outNand;
	wire          carryAdd;
	
	nor n1(zero,out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7],out[8],out[9],out[10],out[11],out[12],out[13],out[14],out[15]);
	
	mux16x2 m1(outAdd, outNand, op, out);
	adder16 add1(.in1(in1), .in2(in2), .out(outAdd), .carry(carryAdd));
	nand16 nand1(.in1(in1), .in2(in2), .out(outNand));
	
	assign carry = (op==0)?carryAdd:1'b0;
	
endmodule


module adder16(in1, in2 , out, carry);		// Implements a full 16-bit adder
	
	output [15:0] out;
	output        carry;
	input  [15:0] in1, in2;
	wire   [16:0] outTemp;

	assign outTemp = in1 + in2;
	assign out     = outTemp[15:0];
	assign carry   = outTemp[16];
	
endmodule

module nand16(in1, in2, out);		// Implements bitwise NAND for two 16-bit numbers

	input  [15:0] in1, in2;
	output [15:0] out;
	
	assign out = ~(in1 & in2);
	
endmodule


module forward_ex_stage(mem_wb_op,mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_op,ex_mem_regA,ex_mem_regB,ex_mem_regC,regread_ex_op,regread_ex_regA,regread_ex_regB,
regread_ex_regC,F1,F2,FCCR,mem_wb_CCR_write,ex_mem_CCR_write,writer7,writerf);

parameter ADD = 6'b000000;
parameter NDU = 6'b001000;
parameter ADC = 6'b000010;
parameter ADZ = 6'b000001;
parameter ADI = 4'b0001;
parameter NDC = 6'b001010;
parameter NDZ = 6'b001001;
parameter LHI = 4'b0011;
parameter LW  = 4'b0100;
parameter SW  = 4'b0101;
parameter LM  = 4'b0110;
parameter SM  = 4'b0111;
parameter BEQ = 4'b1100;
parameter JAL = 4'b1000;
parameter JLR = 4'b1001;

input [2:0] mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_regA,ex_mem_regB,ex_mem_regC,regread_ex_regA,regread_ex_regB,regread_ex_regC;
input [5:0]mem_wb_op,ex_mem_op,regread_ex_op;
input mem_wb_CCR_write,ex_mem_CCR_write;
output reg [2:0]F1,F2;
output reg [1:0]FCCR;
output reg writerf,writer7;


			
			
always @ (*)
begin

	if(regread_ex_op==ADD||regread_ex_op==NDU||regread_ex_op==ADC||regread_ex_op==ADZ||regread_ex_op[5:2]==ADI||regread_ex_op==NDC||regread_ex_op==NDZ)
				begin        // for  operators
						
								if((regread_ex_regA==ex_mem_regC)&&(ex_mem_op==ADD||ex_mem_op==NDU||ex_mem_op==ADC||ex_mem_op==ADZ
								||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))
								
									F1 = 3'b1;//a
								
								
								
						
							else if((regread_ex_regA==ex_mem_regA)&&(ex_mem_op[5:2]==LHI))
								F1 = 3'd5;//i
					
					
							else if((regread_ex_regA==mem_wb_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
								||mem_wb_op==ADZ||mem_wb_op==NDC
								||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
									F1 = 3'd2;//b
							
							else if((regread_ex_regA==mem_wb_regA)&&(mem_wb_op[5:2]==LHI))
								F1 = 3'd6;//j
							else if((regread_ex_regA == mem_wb_regA)&&(mem_wb_op[5:2] ==LW||mem_wb_op[5:2] ==LM))//wait till praveen completes LM to verify
							
									F1 = 3'd3; //forwarded from memory
							else if((regread_ex_regA == mem_wb_regA)&&(mem_wb_op[5:2] == JAL))
									
									F1 = 3'd7; //forwarded PC+1
									
									
									
								
						else if((regread_ex_regA == ex_mem_regB)&&(ex_mem_op[5:2]==ADI)
								&&(ex_mem_CCR_write==1'b0))
								
									F1 = 3'b1;//a
								else if((regread_ex_regA == ex_mem_regB)&&(mem_wb_op[5:2]==ADI)
								&&(mem_wb_CCR_write==1'b0))
									F1 = 3'd2;//b
								
									
						else //no hazard, given the current instruction is op
							F1 = 3'b0;
						
				end			// for  operators
		
		
		
		
	/*	else if(regread_ex_op[5:2]==LM) //wrong
			begin
				
						if((regread_ex_regA == ex_mem_regC)&&(ex_mem_op==ADD||ex_mem_op==NDU
						||ex_mem_op==ADC||ex_mem_op==ADZ
									||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))
							F1 = 3'b1;//a
						else if((regread_ex_regA == mem_wb_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
									||mem_wb_op==ADZ||mem_wb_op==NDC
									||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
							F1 = 3'd2;//b
						
					else if((regread_ex_regA==ex_mem_regA)&&(ex_mem_op==LHI))
							F1 = 3'd5;//i
					else if((regread_ex_regA==mem_wb_regA)&&(mem_wb_op==LHI))
							F1 = 3'd6;//j		
					
					else	if((regread_ex_regA == mem_wb_regA)&&(mem_wb_op[5:2]==LW||mem_wb_op[5:2]==LM))
							F1 = 3'd3;
						else if((regread_ex_regA == mem_wb_regA)&& (mem_wb_op[5:2] ==JAL))
							F1 = 3'd7;//k -> PC+1
						
				else 
					F1 = 3'b0; //no hazards,given current instruction is LM
					
			end */
		
		
	
		else 

				F1 = 3'b0;
end


			
			
			
			always @ (*)
begin

	if(regread_ex_op==ADD||regread_ex_op==NDU||regread_ex_op==ADC||regread_ex_op==ADZ||regread_ex_op==NDC||regread_ex_op==NDZ)//NO ADI as ADI has only regA
				begin        // for  operators
						
							
								if((regread_ex_regB==ex_mem_regC)&&(ex_mem_op==ADD||ex_mem_op==NDU||ex_mem_op==ADC||ex_mem_op==ADZ
								||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))
								
									F2 = 3'b1;//a
								
								
								else if((regread_ex_regB==ex_mem_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
								||mem_wb_op==ADZ||mem_wb_op==NDC
								||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
									F2 = 3'd2;//b
							
							else if((regread_ex_regB==ex_mem_regA)&&(ex_mem_op[5:2]==LHI))
								F2 = 3'd5;//i
					
							else if((regread_ex_regB==mem_wb_regA)&&(mem_wb_op[5:2]==LHI))
								F2 = 3'd6;//j
							else	if((regread_ex_regB == mem_wb_regA)&&(mem_wb_op[5:2] ==LW||mem_wb_op[5:2] ==LM))//wait till praveen completes LM to verify
								
									F2 = 3'd3; //forwarded from memory
								else if((regread_ex_regB == mem_wb_regA)&&(mem_wb_op[5:2] == JAL))
								
								F2 = 3'd7; //forwarded PC+1
								
						
							
							else	if((regread_ex_regB == ex_mem_regB)&&(ex_mem_op[5:2]==ADI)&&(ex_mem_CCR_write==1'b0))
									F2 = 3'b1;//a
								else if((regread_ex_regB == ex_mem_regB)&&(mem_wb_op[5:2]==ADI)&&(mem_wb_CCR_write==1'b0))
									F2 = 3'd2;//b
								else 
									F2 = 3'd0;//no hazards when current instruction is op
							
						
				end			// for  operators
		
		
		
		else if(regread_ex_op[5:2]==LW)
			begin
				
						if((regread_ex_regB == ex_mem_regC)&&(ex_mem_op==ADD||ex_mem_op==NDU||ex_mem_op==ADC||ex_mem_op==ADZ
									||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))
							F2 = 3'b1;//a
						else if((regread_ex_regB == ex_mem_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
									||mem_wb_op==ADZ||mem_wb_op==NDC
									||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
							F2 = 3'd2;//b
						
					else if((regread_ex_regB==ex_mem_regA)&&(ex_mem_op==LHI))
							F2 = 3'd5;//i
					else if((regread_ex_regB==mem_wb_regA)&&(mem_wb_op==LHI))
							F2 = 3'd6;//j		
					
					else	if((regread_ex_regB == mem_wb_regA)&&(mem_wb_op[5:2]==LW||mem_wb_op[5:2]==LM))
							F2 = 3'd3;
						else if((regread_ex_regB == mem_wb_regA)&& (mem_wb_op[5:2] ==JAL))
							F2 = 3'd7;//k -> PC+1
						
				else 
					F2 = 3'b0; //no hazards,given current instruction is LW
					
			end
			
			else if(regread_ex_op[5:2]==SW)
			begin
				if((regread_ex_regB == ex_mem_regC)&&(ex_mem_op==ADD||ex_mem_op==NDU||ex_mem_op==ADC||ex_mem_op==ADZ
									||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))
					F2 = 3'b1;//a
				else if((regread_ex_regB == mem_wb_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
									||mem_wb_op==ADZ||mem_wb_op==NDC
									||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
					F2 = 3'd2;//b	
				else if((regread_ex_regB==ex_mem_regA)&&(ex_mem_op==LHI))
							F2 = 3'd5;//i
				else if((regread_ex_regB==mem_wb_regA)&&(mem_wb_op==LHI))
							F2 = 3'd6;//j		
				else if((regread_ex_regB == mem_wb_regA)&& (mem_wb_op[5:2] ==JAL))
							F2 = 3'd7;//k -> PC+1
				else	if((regread_ex_regB == mem_wb_regA)&&(mem_wb_op[5:2] ==LW||mem_wb_op[5:2] ==LM))//wait till praveen completes LM to verify
								
									F2 = 3'd3; //forwarded from memory
				else 
					F2 = 3'd0;
			end
		else 
			F2 = 3'b0;
end


			

always @(*)
begin
if(regread_ex_op==ADC||regread_ex_op==ADZ||regread_ex_op==NDC||regread_ex_op==NDZ) 
		begin
	
				if((ex_mem_op==ADD||ex_mem_op==NDU||ex_mem_op==ADC||ex_mem_op==ADZ||ex_mem_op[5:2]==ADI||ex_mem_op==NDC||ex_mem_op==NDZ)&&(ex_mem_CCR_write==1'b0))//if the current op is conditional on CCR, CCR needs to be forwarded
					begin
					FCCR = 2'b1;
						if(regread_ex_regC==3'b111)
						begin
							writer7=1'b0;
							writerf=1'b1;
						end
						else 
						writer7=1'b1;
						writerf=1'b0;
					end
				else if((mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC||mem_wb_op==ADZ||mem_wb_op[5:2]==ADI||mem_wb_op==NDC||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
					begin
					FCCR = 2'd2;
						if(regread_ex_regC==3'b111)
						begin
							writer7=1'b0;
							writerf=1'b1;
						end
						else 
						begin
							writer7=1'b1;
							writerf=1'b0;
						end
					end
				else if((regread_ex_op==ADZ||regread_ex_op==NDZ)&&(ex_mem_op==LW)&&(ex_mem_CCR_write==1'b0))
					begin
					FCCR = 2'b1;
						if(regread_ex_regC==3'b111)
						begin
							writer7=1'b0;
							writerf=1'b1;
						end
						else
						begin
							writer7=1'b1;
							writerf=1'b0;			
						end
					end
				
				else if((regread_ex_op==ADZ||regread_ex_op==NDZ)&&(mem_wb_op==LW)&&(mem_wb_CCR_write==1'b0))
					begin
					FCCR = 2'd2;
						if(regread_ex_regC==3'b111)
						begin
							writer7=1'b0;
							writerf=1'b1;
						end
						else 
						begin
							writer7=1'b1;
							writerf=1'b0;	
						end
					end
				else 
					begin
						FCCR = 2'b0;
						if(regread_ex_regC==3'b111)
							begin
								writer7=1'b0;
								writerf=1'b1;
							end
						else 
							begin
								writer7=1'b1;
								writerf=1'b0;	
							end
					end
				
			
		end

else if(regread_ex_op==ADC||regread_ex_op==NDU)
	begin
	FCCR = 2'b0;
				if(regread_ex_regC==3'b111)
				begin
					writer7=1'b0;
					writerf=1'b1;
				end
				else 
				begin
					writer7=1'b1;
					writerf=1'b0;	
				end
	end//unconditional operations
else if(regread_ex_op==LW||regread_ex_op==LM||regread_ex_op==LHI)
	begin
	FCCR = 2'b0;
				if(regread_ex_regA==3'b111)
				begin
					writer7=1'b0;
					writerf=1'b1;
				end
				else 
				begin
					writer7=1'b1;
					writerf=1'b0;	
				end
	end//load operations
else if(regread_ex_op==JAL||regread_ex_op==JLR)
	begin
	FCCR=2'b0;
		writer7=1'b1;
		writerf=1'b0;
	end//jump operations, which modiy registers
else
	begin
	FCCR=1'b0;
	writer7=1'b1;
	writerf=1'b1;
	end
	
end

endmodule


module hazard_detection(IR_load_mux,new_IR_multi,first_multiple,clk,flush_reg_ex,flush_id_reg,flush_if_id,pr1_IR,pr1_pc,pr2_IR,pr2_pc,pr3_IR,pr4_IR,pc_write,equ);//equ comes from PR reg/ex


parameter ADD = 6'b000000;
parameter NDU = 6'b001000;
parameter ADC = 6'b000010;
parameter ADZ = 6'b000001;
parameter ADI = 4'b0001;
parameter NDC = 6'b001010;
parameter NDZ = 6'b001001;
parameter LHI = 4'b0011;
parameter LW  = 4'b0100;
parameter SW  = 4'b0101;
parameter LM  = 4'b0110;
parameter SM  = 4'b0111;
parameter BEQ = 4'b1100;
parameter JAL = 4'b1000;
parameter JLR = 4'b1001;
	input clk,equ;
	input [15:0] pr1_IR,pr2_IR,pr3_IR,pr4_IR,pr1_pc,pr2_pc;
	output reg[15:0] new_IR_multi;
	output reg flush_id_reg,flush_if_id,flush_reg_ex,pc_write,first_multiple,IR_load_mux;
	wire [5:0] op1,op2,op3,op4;
	wire [7:0]LM_Imm;
	assign LM_Imm=pr1_IR[7:0];
	wire[2:0] pr1RA,pr2RA,pr3RA,pr1RB,pr2RB,pr1RC,pr2RC,pr3RC,pr4RA;
	assign op1 = {pr1_IR[15:12],pr1_IR[1:0]};
	assign op2 = {pr2_IR[15:12],pr2_IR[1:0]};
	assign op3 = {pr3_IR[15:12],pr3_IR[1:0]};
	assign op4 = {pr4_IR[15:12],pr4_IR[1:0]};
	assign pr1RA = pr1_IR[11:9];
	assign pr1RB = pr1_IR[8:6];
	assign pr1RC = pr1_IR[5:3];
	assign pr2RA = pr2_IR[11:9];
	assign pr2RB = pr2_IR[8:6];
	assign pr2RC = pr2_IR[5:3];
	assign pr3RA = pr3_IR[11:9];
	
	assign pr3RC = pr3_IR[5:3];
	assign pr4RA = pr4_IR[11:9];

	
	always@(negedge clk) //first multiple detection
		begin
		if((op1[5:2]==LM||op1[5:2]==SM)&&(op1!=op2))
			first_multiple=1'b1;
		else if ((op1[5:2]==LM||op1[5:2]==SM)&&(op1==op2)&&(pr1_pc!=pr2_pc))
			first_multiple=1'b1;
		else 
			first_multiple=1'b0;
		end//always
	always @(negedge clk)
	
		begin
		new_IR_multi[15:8]=pr1_IR[15:8];
			if(op3[5:2]==BEQ&&equ==1'b1)
				begin
				flush_reg_ex=1'b1;
				flush_id_reg=1'b1;
				pc_write=1'b0;//write the forwarded value i.e. the jump location into PC
				end
			else if((op1==ADD||op1==NDU||op1==ADC||op1==ADZ||op1==NDC||op1==NDC||op1==NDZ)&&(pr1RC==3'b111)) //if rc = R7 and an operation is performed onto Rc
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;//do not allow pc to be written into
				end
			else if((op2==ADD||op2==NDU||op2==ADC||op2==ADZ||op2==NDC||op2==NDC||op2==NDZ)&&(pr2RC==3'b111))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;//do not allow pc to be written into
				end
			else if((op3==ADD||op3==NDU||op3==ADC||op3==ADZ||op3==NDC||op3==NDC||op3==NDZ)&&(pr3RC==3'b111))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id = 1'b1;
				pc_write = 1'b1;
				end
			else if(op1[5:2]==ADI&&pr1RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op2[5:2]==ADI&&pr2RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op3[5:2]==ADI&&pr2RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==LW||op1[5:2]==LM)&&pr1RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op2[5:2]==LW||op2[5:2]==LM)&&pr2RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op3[5:2]==LW||op3[5:2]==LM)&&pr3RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op4[5:2]==LW||op4[5:2]==LM)&&pr4RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1[5:2]==LM||op1[5:2]==SM)
			begin
					if(LM_Imm[0]==1)
					begin
					IR_load_mux=1'b1;
					pc_write=1'b1;
					new_IR_multi[0]=1'b0;
					end
				else if(LM_Imm[1]==1)
				begin
				IR_load_mux=1'b1;
				pc_write=1'b1;
					new_IR_multi[1]=1'b0;
				end
				else if(LM_Imm[2]==1)
				begin
				IR_load_mux=1'b1;
				pc_write=1'b1;
				new_IR_multi[2]=1'b0;
				end
				else if(LM_Imm[3]==1)
				begin
				IR_load_mux=1'b1;
				pc_write=1'b1;
				new_IR_multi[3]=1'b0;
				end
					else if(LM_Imm[4]==1)
					begin
					IR_load_mux=1'b1;
					pc_write=1'b1;
				new_IR_multi[4]=1'b0;
				end
					else if(LM_Imm[5]==1)
					begin
					IR_load_mux=1'b1;
					pc_write=1'b1;
					new_IR_multi[5]=1'b0;
					end
					else if(LM_Imm[6]==1)
					begin
					IR_load_mux=1'b1;
					pc_write=1'b1;
					new_IR_multi[6]=1'b0;
					end
					else if(LM_Imm[7]==1)
					begin
					IR_load_mux=1'b0;
					new_IR_multi[7]=1'b0;
					pc_write=1'b0;
					end
					else begin //LM/sm 00000000
					IR_load_mux=1'b0;
					new_IR_multi[7]=1'b0;
					pc_write=1'b0;
					end
			end
			
			else if ((op1==ADD||op1==NDU||op1==ADC||op1==ADZ||op1==NDC||op1==NDC||op1==NDZ)
			&&((pr1RA==pr2RA)||pr1RB==pr2RA)&&(op2==LW||op2==LM))//load followed by op
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==ADI)&&(op2==LW||op2==LM)&&(pr1RA==pr2RA))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==LW)&&(op2[5:2]==LW||op2[5:2]==LM)&&(pr1RB==pr2RA)) // lw/lm followed by lw
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==LM)&&(op2[5:2]==LW||op2[5:2]==LM)&&(pr1RA==pr2RA)) //lw/lm followed by LM
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==SW)&&(op2[5:2]==LW||op2[5:2]==LM)&&(pr1RB==pr2RA))        //load followed by store
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1[5:2]==JAL)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1[5:2]==JLR)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op2[5:2]==JLR)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			 
			else if((op1[5:2]==SM||op1[5:2]==LM)&&(op2==ADD||op2==NDU||op2==ADC||op2==ADZ||op2==NDC||op2==NDC||op2==NDZ)&&(pr1RA==pr2RC))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==SM||op1[5:2]==LM)&&(op2[5:2]==ADI)&&(pr1RA==pr2RB))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==SM||op1[5:2]==LM)&&(op2[5:2]==LHI)&&(pr1RA==pr2RA))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1[5:2]==SM||op1[5:2]==LM)&&(op2[5:2]==LW||op2[5:2]==LM)&&(pr1RA==pr2RA))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			
			else 
			begin
			flush_reg_ex=1'b0;
			flush_id_reg=1'b0;
			flush_if_id=1'b0;
			pc_write=1'b0;
			end
		end		
endmodule
		
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



module mem_access(IRfrompipe4, IRfrompipe5, RAFromPipe, ALUOut, RAMemSelectInput, WAMemSelectInput, MemData, DataInSelect, WriteMem, RAFromPipeInc,SignalB, SignalC
				  , Rfout1, Rfout2, mem_wb_CCR_write, ex_mem_CCR_write);

	output [15:0] MemData, RAFromPipeInc;
	wire [15:0] DataIn;
	input  [15:0] ALUOut, RAFromPipe, Rfout1, Rfout2, SignalC, SignalB, IRfrompipe5, IRfrompipe4;
	input         RAMemSelectInput, WAMemSelectInput, WriteMem, DataInSelect, mem_wb_CCR_write, ex_mem_CCR_write;
	wire   [15:0] readAddSelected, writeAddSelected, DataInSelected;
	wire [1:0] F3;
	mux16x2 RASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(RAMemSelectInput), .out(readAddSelected));
	mux16x2 WASelect(.data0(RAFromPipe), .data1(ALUOut), .selectInput(WAMemSelectInput), .out(writeAddSelected));
	mux16x4 DataSelect2(.data0(DataInSelected), .data1(SignalB), .data2(SignalC), .data3(16'b0), .selectInput(F3), .out(DataIn));
	mux16x2 DataSelect1(.data0(Rfout1), .data1(Rfout2), .selectInput(DataInSelect), .out(DataInSelected));
	data_mem DataMemory(.readAdd(readAddSelected), .out(MemData), .writeAdd(writeAddSelected), .in(DataIn), .write(WriteMem));
	plus_one Inc(.in(RAFromPipe), .out(RAFromPipeInc));
	forward_mem_stage f_mem(.mem_wb_op({IRfrompipe5[15:12],IRfrompipe5[1:0]}), .mem_wb_regA(IRfrompipe5[11:9]), .mem_wb_regC(IRfrompipe5[5:3]), .ex_mem_op({IRfrompipe4[15:12],IRfrompipe4[1:0]}),
					  .ex_mem_regA(IRfrompipe4[11:9]), .F3(F3) ,.mem_wb_CCR_write(mem_wb_CCR_write), .ex_mem_CCR_write(ex_mem_CCR_write));

endmodule


module forward_mem_stage(mem_wb_op,mem_wb_regA,mem_wb_regC,ex_mem_op,ex_mem_regA,F3,mem_wb_CCR_write,ex_mem_CCR_write);


	parameter ADD = 6'b000000;
	parameter NDU = 6'b001000;
	parameter ADC = 6'b000010;
	parameter ADZ = 6'b000001;
	parameter ADI = 4'b0001;
	parameter NDC = 6'b001010;
	parameter NDZ = 6'b001001;
	parameter LHI = 4'b0011;
	parameter LW  = 4'b0100;
	parameter SW  = 4'b0101;
	parameter LM  = 4'b0110;
	parameter SM  = 4'b0111;
	parameter BEQ = 4'b1100;
	parameter JAL = 4'b1000;
	parameter JLR = 4'b1001;

	input [2:0] mem_wb_regA,mem_wb_regC,ex_mem_regA;
	input [5:0]mem_wb_op,ex_mem_op;
	input mem_wb_CCR_write,ex_mem_CCR_write;
	output reg [1:0]F3;

	always @(*)
	begin
		if(ex_mem_op[5:2]==SW)
		begin
			if((ex_mem_regA == mem_wb_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC||mem_wb_op==ADZ
									||mem_wb_op==NDC||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
				F3 = 2'd1;//b				
			else if((ex_mem_regA==mem_wb_regA)&&(mem_wb_op[5:2]==LW))
				F3 = 2'd2;//c
			else	
				F3 = 2'b0;
		end
		else 
			F3 = 2'b0;
	end
	
endmodule




module mux16x8(data0, data1, data2, data3, data4, data5, data6, data7, selectInput, out);  // 8-16bit-input mux

	output reg [15:0] out;
	input  [15:0] data0, data1, data2, data3, data4, data5, data6, data7;
	input  [2:0] selectInput;
	
	always@(data0 or data1 or data2 or data3 or data4 or data5 or data6 or data7 or selectInput) begin
		case(selectInput)
			0: out = data0;
			1: out = data1;
			2: out = data2;
			3: out = data3;
			4: out = data4;
			5: out = data5;
			6: out = data6;
			7: out = data7;
		endcase
	end
	
endmodule


module mux16x4(data0, data1, data2, data3, selectInput, out);  // 4-16bit-input mux

	output reg [15:0] out;
	input  [15:0] data0, data1, data2, data3;
	input  [1:0] selectInput;
	
	always@(data0 or data1 or data2 or data3 or selectInput) begin
		case(selectInput)
			0: out = data0;
			1: out = data1;
			2: out = data2;
			3: out = data3;
		endcase
	end
	
endmodule
module mux2x4(data0, data1, data2, data3, selectInput, out);  // 4-16bit-input mux

	output reg [1:0] out;
	input  [1:0] data0, data1, data2, data3;
	input  [1:0] selectInput;
	
	always@(data0 or data1 or data2 or data3 or selectInput) begin
		case(selectInput)
			0: out = data0;
			1: out = data1;
			2: out = data2;
			3: out = data3;
		endcase
	end
	
endmodule


module mux16x2(data0, data1, selectInput, out);  // 2-16bit-input mux

	output reg [15:0] out;
	input  [15:0] data0, data1;
	input  selectInput;
	
	always@(data0 or data1 or selectInput) begin
		case(selectInput)
			0: out = data0;
			1: out = data1;
		endcase
	end
	
endmodule




module pipeline_reg1(clk, reset, toPCInc, toPC, toIR, PCInc, PC, IR,tofirst_multiple,first_multiple,flush); // First pipeline register
	
	output first_multiple;
	output [15:0] PCInc, PC, IR;
	input  [15:0] toPCInc, toPC, toIR;
	input	      reset, clk;
	input tofirst_multiple,flush;
	wire [15:0]inIR;
	wire infirst_multiple;
	assign infirst_multiple = (flush==1'b1)?1'b0:tofirst_multiple;//introduce a NOP, in the event of a flush
	assign inIR = (flush==1'b1)?16'b1111000000000000:toIR;//introduce a NOP, in the event of a flush
	register1 pipe1first_multiple(.clk(clk), .out(first_multiple), .in(infirst_multiple), .write(1'b0), .reset(reset));
	register16 pipe1IncPC(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 pipe1PC(.clk(clk), .out(PC), .in(toPC), .write(1'b0), .reset(reset));
	register16 pipe1IR(.clk(clk), .out(IR), .in(inIR), .write(1'b0), .reset(reset));
	
endmodule


module pipeline_reg2(clk,reset,toMex1,Mex1,toMex2,Mex2,toMmemData,MmemData,toMmemR,MmemR,toMmemW,MmemW,toMregWB,MregWB,toMr7WB,Mr7WB,toPCInc,PCInc,toPC,PC,toIR,IR,tofirst_multiple,
first_multiple,toPCImmInc,PCImmInc,toWriteMem,WriteMem,torA1,rA1,torA2,rA2,toWriteAdd,WriteAdd,toSImm6,SImm6,toImm970s,Imm970s,toalu_ctrl,alu_ctrl,flush,modify_pr2_ra,modify_ir);
	output Mex1,Mex2,MmemData,MmemR,MmemW,first_multiple,WriteMem,alu_ctrl;
	output [2:0] rA1,rA2,WriteAdd;
	output [2:0] Mr7WB;
	output [1:0] MregWB;
	output [15:0] PCInc,PC,IR,PCImmInc,SImm6,Imm970s;
	input toMex1,toMex2,toMmemData,toMmemR,toMmemW,tofirst_multiple,toWriteMem,toalu_ctrl;
	input [2:0] torA1,torA2,toWriteAdd,modify_pr2_ra;
	input  modify_ir;
	input [2:0] toMr7WB;
	input [1:0] toMregWB;
	input [15:0] toPCInc,toPC,toIR,toPCImmInc,toSImm6,toImm970s;
	input flush,clk,reset;
	wire [15:0] inIR;
	wire infirst_multiple;
	wire inWriteMem;

assign inIR = (flush==1'b1)?16'b1111000000000000:
					((modify_ir ==1'b1)?{toIR[15:12],modify_pr2_ra,toIR[8:0]}:toIR);//introduce a NOP, in the event of a flush
assign inWriteMem = (flush==1'b1)?1'b1:toWriteMem;//introduce a NOP, in the event of a flush
assign infirst_multiple = (flush==1'b1)?1'b0:tofirst_multiple;//introduce a NOP, in the event of a flush
register1 Mex1_reg (.clk(clk), .out(Mex1), .in(toMex1) , .write(1'b0), .reset(reset));
register1 Mex2_reg (.clk(clk), .out(Mex2), .in(toMex2) , .write(1'b0), .reset(reset));
register1 MmemData_reg (.clk(clk), .out(MmemData), .in(toMmemData) , .write(1'b0), .reset(reset));
register1 MmemR_reg (.clk(clk), .out(MmemR), .in(toMmemR) , .write(1'b0), .reset(reset));
register1 MmemW_reg (.clk(clk), .out(MmemW), .in(toMmemW) , .write(1'b0), .reset(reset));
register1 first_multiple_reg (.clk(clk), .out(first_multiple), .in(infirst_multiple) , .write(1'b0), .reset(reset));
register1 WriteMem_reg (.clk(clk), .out(WriteMem), .in(inWriteMem) , .write(1'b0), .reset(reset));
register1 alu_ctrl_reg (.clk(clk), .out(alu_ctrl), .in(toalu_ctrl) , .write(1'b0), .reset(reset));

register2 MregWB_reg (.clk(clk), .out(MregWB), .in(toMregWB) , .write(1'b0), .reset(reset));
register3 rA1_reg (.clk(clk), .out(rA1), .in(torA1) , .write(1'b0), .reset(reset));
register3 rA2_reg (.clk(clk), .out(rA2), .in(torA2) , .write(1'b0), .reset(reset));
register3 WriteAdd_reg (.clk(clk), .out(WriteAdd), .in(toWriteAdd) , .write(1'b0), .reset(reset));

register2 Mr7WB_reg (.clk(clk), .out(Mr7WB), .in(toMr7WB) , .write(1'b0), .reset(reset));


register16 PCInc_reg (.clk(clk), .out(PCInc), .in(toPCInc) , .write(1'b0), .reset(reset));
register16 PC_reg (.clk(clk), .out(PC), .in(toPC) , .write(1'b0), .reset(reset));
register16 IR_reg (.clk(clk), .out(IR), .in(inIR) , .write(1'b0), .reset(reset));
register16 PCImmInc_reg (.clk(clk), .out(PCImmInc), .in(toPCImmInc) , .write(1'b0), .reset(reset));
register16 SImm6_reg (.clk(clk), .out(SImm6), .in(toSImm6) , .write(1'b0), .reset(reset));
register16 Imm970s_reg (.clk(clk), .out(Imm970s), .in(toImm970s) , .write(1'b0), .reset(reset));


endmodule


module pipeline_reg3(clk, reset, toalu_ctrl, alu_ctrl, toImm970s, toPCImmInc, toPCInc, toWriteAdd, toRegWriteSelect, toR7WriteSelect, toWriteMem, toRFOut1,
 toRFOut2, toEqu, toSImm6, toIR, Imm970s, PCImmInc, PCInc, WriteAdd, RegWriteSelect, R7WriteSelect, WriteMem, RFOut1, RFOut2, Equ, SImm6, IR,
 MemdataSelectInput,toMemdataSelectInput,
 RAMemSelectInput,toRAMemSelectInput, WAMemSelectInput,toWAMemSelectInput,tofirst_multiple,first_multiple,Mex1,toMex1,Mex2,toMex2,flush);
	
	output [15:0] Imm970s, PCImmInc, PCInc, RFOut1, RFOut2, IR, SImm6;
	output [ 2:0] WriteAdd;
	output [2:0]  R7WriteSelect;
	output [1:0] RegWriteSelect;
	output        WriteMem, RAMemSelectInput, WAMemSelectInput, Equ,Mex1,Mex2, alu_ctrl, MemdataSelectInput,first_multiple;
	
	input [15:0] toImm970s, toPCImmInc, toPCInc, toRFOut1, toRFOut2, toIR, toSImm6;
	input [ 2:0] toWriteAdd;
	input [2:0]  toR7WriteSelect;
	input [ 1:0] toRegWriteSelect;
	input        toWriteMem, toRAMemSelectInput, toWAMemSelectInput, toEqu, clk, reset,tofirst_multiple,toMex1,toMex2, flush,
				 toalu_ctrl, toMemdataSelectInput;
	
	wire [15:0] inIR;
	wire infirst_multiple;
	wire inWriteMem,inEqu;

	assign inIR = (flush==1'b1)?16'b1111000000000000:toIR;//introduce a NOP, in the event of a flush
	assign inWriteMem = (flush==1'b1)?1'b1:toWriteMem;//introduce a NOP, in the event of a flush
	assign inEqu = (flush==1'b1)?1'b1:toEqu;//introduce a NOP, in the event of a flush
	assign infirst_multiple = (flush==1'b1)?1'b0:tofirst_multiple;//introduce a NOP, in the event of a flush
	
	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 RFOut1Reg(.clk(clk), .out(RFOut1), .in(toRFOut1), .write(1'b0), .reset(reset));
	register16 RFOut2Reg(.clk(clk), .out(RFOut2), .in(toRFOut2), .write(1'b0), .reset(reset));
	register16 SImm6Reg(.clk(clk), .out(SImm6), .in(toSImm6), .write(1'b0), .reset(reset));
	register16 IRReg(.clk(clk), .out(IR), .in(inIR), .write(1'b0), .reset(reset));
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	
	register2  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
	register2  RegWriteSelectReg(.clk(clk), .out(RegWriteSelect), .in(toRegWriteSelect), .write(1'b0), .reset(reset));
	
	register1  WriteMemReg(.clk(clk), .out(WriteMem), .in(inWriteMem), .write(1'b0), .reset(reset));
	register1  RAMemSelect(.clk(clk), .out(RAMemSelectInput), .in(toRAMemSelectInput), .write(1'b0), .reset(reset));
	register1  WAMemSelect(.clk(clk), .out(WAMemSelectInput), .in(toWAMemSelectInput), .write(1'b0), .reset(reset));
	register1  MemdataSelect(.clk(clk), .out(MemdataSelectInput), .in(toMemdataSelectInput), .write(1'b0), .reset(reset));
	register1  EquReg(.clk(clk), .out(Equ), .in(inEqu), .write(1'b0), .reset(reset));
	register1 first_multiple_reg (.clk(clk), .out(first_multiple), .in(infirst_multiple) , .write(1'b0), .reset(reset));
	register1 Mex1_reg (.clk(clk), .out(Mex1), .in(toMex1) , .write(1'b0), .reset(reset));
	register1 Mex2_reg (.clk(clk), .out(Mex2), .in(toMex2) , .write(1'b0), .reset(reset));
	register1 alu_ctrl_reg (.clk(clk), .out(alu_ctrl), .in(toalu_ctrl) , .write(1'b0), .reset(reset));
endmodule


module pipeline_reg4(	clk, reset, toCCR, toCCRWrite, toWriteRF, toImm970s, toPCImmInc, toALUOut, toPCInc, toWriteAdd, toWriteR7, toRegWriteSelect, toR7WriteSelect, 
	toWriteMem, toRFOut1,toRFOut2, toRAOut,CCR, CCRWrite, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect, 
	WriteMem, RFOut1,RFOut2, RAOut,toIR,IR,MemdataSelectInput,toMemdataSelectInput, RAMemSelectInput, toRAMemSelectInput, WAMemSelectInput,
	toWAMemSelectInput);

	output [15:0] Imm970s, PCImmInc, ALUOut, PCInc, RFOut1, RFOut2, RAOut,IR;
	output [2:0]  WriteAdd;
	output [2:0] R7WriteSelect;
	output [1:0] CCR, RegWriteSelect;
	output        CCRWrite, WriteRF, WriteR7, WriteMem, RAMemSelectInput, WAMemSelectInput, MemdataSelectInput;
	
	input [15:0] toImm970s, toPCImmInc, toALUOut, toPCInc, toRFOut1, toRFOut1, toRFOut2, toRAOut,toIR;
	input [ 2:0] toWriteAdd;
	output [2:0] toR7WriteSelect;
	input [ 1:0] toCCR, toRegWriteSelect;
	input        toCCRWrite, toWriteRF, toWriteR7, toWriteMem, toRAMemSelectInput, toWAMemSelectInput, clk, reset, toMemdataSelectInput;

	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 ALUOutReg(.clk(clk), .out(ALUOut), .in(toALUOut), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 RAReg(.clk(clk), .out(RAOut), .in(toRAOut), .write(1'b0), .reset(reset));
	register16 RfOut1_Reg(.clk(clk), .out(RFOut1), .in(toRFOut1), .write(1'b0), .reset(reset));
	register16 RfOut2_Reg(.clk(clk), .out(RFOut2), .in(toRFOut2), .write(1'b0), .reset(reset));
	register16 IR_Reg(.clk(clk), .out(IR), .in(toIR), .write(1'b0), .reset(reset));
	register1  MemdataSelect(.clk(clk), .out(MemdataSelectInput), .in(toMemdataSelectInput), .write(1'b0), .reset(reset));
	
	
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	
	register2  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
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
			CCR, CCRWrite, MemData, WriteRF, Imm970s, PCImmInc, ALUOut, PCInc, WriteAdd, WriteR7, RegWriteSelect, R7WriteSelect,toIR,IR,
			RFOut2, toRFOut2);
			
	output [15:0] MemData, Imm970s, PCImmInc, ALUOut, PCInc,IR, RFOut2;
	output [ 2:0] WriteAdd;
	output [2:0] R7WriteSelect;
	output [ 1:0] CCR, RegWriteSelect;
	output        CCRWrite, WriteRF, WriteR7;
	
	input [15:0] toMemData, toImm970s, toPCImmInc, toALUOut, toPCInc,toIR, toRFOut2;
	input [ 2:0] toWriteAdd;
	input [2:0] toR7WriteSelect;
	input [ 1:0] toCCR, toRegWriteSelect;
	input        toCCRWrite, toWriteRF, toWriteR7, clk, reset;
	
	register16 MemDataReg(.clk(clk), .out(MemData), .in(toMemData), .write(1'b0), .reset(reset));
	register16 Imm970Reg(.clk(clk), .out(Imm970s), .in(toImm970s), .write(1'b0), .reset(reset));
	register16 PCImmIncReg(.clk(clk), .out(PCImmInc), .in(toPCImmInc), .write(1'b0), .reset(reset));
	register16 ALUOutReg(.clk(clk), .out(ALUOut), .in(toALUOut), .write(1'b0), .reset(reset));
	register16 PCIncReg(.clk(clk), .out(PCInc), .in(toPCInc), .write(1'b0), .reset(reset));
	register16 IR_Reg(.clk(clk), .out(IR), .in(toIR), .write(1'b0), .reset(reset));
	register16 RFOut2_Reg(.clk(clk), .out(RFOut2), .in(toRFOut2), .write(1'b0), .reset(reset));
	
	register3  WriteAddReg(.clk(clk), .out(WriteAdd), .in(toWriteAdd), .write(1'b0), .reset(reset));
	register2  R7WriteSelectReg(.clk(clk), .out(R7WriteSelect), .in(toR7WriteSelect), .write(1'b0), .reset(reset));
	register2  RegWriteSelectReg(.clk(clk), .out(RegWriteSelect), .in(toRegWriteSelect), .write(1'b0), .reset(reset));
	register2  CCRReg(.clk(clk), .out(CCR), .in(toCCR), .write(1'b0), .reset(reset));
	register1  CCRWriteReg(.clk(clk), .out(CCRWrite), .in(toCCRWrite), .write(1'b0), .reset(reset));
	register1  WriteRFReg(.clk(clk), .out(WriteRF), .in(toWriteRF), .write(1'b0), .reset(reset));
	register1  WriteR7Reg(.clk(clk), .out(WriteR7), .in(toWriteR7), .write(1'b0), .reset(reset));

endmodule


module plus_one(in, out);

	output [15:0] out;
	input  [15:0] in;
	
	assign out = in + 16'd1;

endmodule



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


module register4(clk, out, in, write, reset);  // Negedge-triggered flipflop register with active-low write signal and reset

	output reg [3:0] out;
	input      [3:0] in;
	input      clk, write, reset;
	
	always@(posedge clk) begin
		if(reset==0) begin
			out = 4'b0;
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


module register2(clk, out, in, write, reset);  // Negedge-triggered flipflop register with active-low write signal and reset

	output reg [1:0] out;
	input      [1:0] in;
	input      clk, write, reset;
	
	always@(posedge clk) begin
		if(reset==0) begin
			out = 2'b0;
		end
		else if(write == 1'b0) begin
			out = in;
		end
	end
	
endmodule


module register1(clk, out, in, write, reset);  // Negedge-triggered flipflop register with active-low write signal and reset

	output reg out;
	input      in;
	input      clk, write, reset;
	
	always@(posedge clk) begin
		if(reset==0) begin
			out = 1'b0;
		end
		else if(write == 1'b0) begin
			out = in;
		end
	end
	
endmodule



module register_file(clk, out1, out2, readAdd1, readAdd2, write, writeAdd, writeR7, inR7, in, reset); // Modify to include R7 effects

	output [15:0] out1, out2;
	input  [15:0] in, inR7;
	input  [2:0]  readAdd1, readAdd2, writeAdd;
	input         write, clk, reset, writeR7;
	
	wire   [15:0] data0,  data1,  data2,  data3,  data4,  data5,  data6,  data7;
	wire   [6:0]   writeLines;
	wire [7:0] writeLinesInit;
	decode8 dem(writeAdd, writeLinesInit);
	mux16x8 mux1(data0, data1, data2, data3, data4, data5, data6, data7, readAdd1, out1);
	mux16x8 mux2(data0, data1, data2, data3, data4, data5, data6, data7, readAdd2, out2);
	
	or a0(writeLines[0], write, ~writeLinesInit[0]);
	or a1(writeLines[1], write, ~writeLinesInit[1]);
	or a2(writeLines[2], write, ~writeLinesInit[2]);
	or a3(writeLines[3], write, ~writeLinesInit[3]);
	or a4(writeLines[4], write, ~writeLinesInit[4]);
	or a5(writeLines[5], write, ~writeLinesInit[5]);
	or a6(writeLines[6], write, ~writeLinesInit[6]);
	
	register16 r0(clk, data0, in, writeLines[0], reset);
	register16 r1(clk, data1, in, writeLines[1], reset);
	register16 r2(clk, data2, in, writeLines[2], reset);
	register16 r3(clk, data3, in, writeLines[3], reset);
	register16 r4(clk, data4, in, writeLines[4], reset);
	register16 r5(clk, data5, in, writeLines[5], reset);
	register16 r6(clk, data6, in, writeLines[6], reset);
	register16 r7(clk, data7, inR7, writeR7, reset);
	
endmodule

module write_back(clk, reset, Imm970, MemData, PCImmInc, ALUOut, PCInc, regSelect, r7Select, writeData, writeR7Data, RFOut2);

	output [15:0] writeData, writeR7Data;
	input  [15:0] Imm970, MemData, PCImmInc, ALUOut, PCInc, RFOut2;
	input  [ 2:0] r7Select;
	input  [ 1:0] regSelect;
	input         clk, reset;

	mux16x4 regWriteMux(.data0(MemData), .data1(ALUOut), .data2(Imm970), .data3(PCInc), .selectInput(regSelect), .out(writeData));
	mux16x8 r7WriteMux(.data0(Imm970), .data1(MemData), .data2(PCImmInc), .data3(ALUOut), .data4(RFOut2), .data5(PCInc), .selectInput(r7Select), .out(writeR7Data));

endmodule

			

