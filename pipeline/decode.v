`ifndef _DECODE
`define _DECODE
`include "../misc/sext.v"

module decode(fromPipe1PC, fromPipe1IR, PC_Imm, rA1, rA2, wA, Sext_Imm6, Imm970, Mex1, Mex2, wCCR, wMem, alu_ctrl, MregWB, MmemR, MmemW, Mr7WB);
	output [15:0] PC_Imm, Sext_Imm6, Imm970;
	output [2:0] rA1, rA2, wA, i, MregWB;
	output Mex1, Mex2, wCCR, wMem, alu_ctrl, MmemR, MmemW;
	output [3:0] Mr7WB;
	input [15:0] fromPipe1PC, fromPipe1IR;
	reg [15:0] imm6, imm9;
	reg select, offset;
	reg [8:0] LM_Imm;	//Only for LM & SM instruction
	assign LM_Imm = IR[8:0];
	assign imm6 = {10'd0, IR[5:0]};
	assign imm9 = {7'd0, IR[8:0]};
	assign select = (IR[15:12]==4'B1000)?0:1;	//If opcode is 1000 then select data0.
	mux16x2 m1(.data0(imm9), .data1(imm6), .selectInput(select), .out(offset));
	add add1(.in1(fromPipe1PC),.in2(offset),.out(PC_Imm));
	sext6 s1(.in(IR[5:0]), .out(Sext_Imm6));
	Imm970 = {IR[8:0], 7'd0};
	
	always@(*)	
	begin
		case (IR[15:12])
		begin
			0000:	//ADD, ADC, ADZ
				rA2<= IR[8:6];	//RB
				wA<= IR[5:3];	//RC
				rA1<= IR[11:9];	//RA
				Mex1<= 0;	//Rfout1
				Mex2<= 0;	//Rfout2
				wCCR<= 0;	//Write CCR
				alu_ctrl<=0;	//Add operation
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1;	//Write back Alu_out
				if(IR[5:3]==3'b111)	//If RC is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0001:	//ADI
				rA1<= IR[11:9];	//RA
				wA<= IR[8:6];	//RB
				Mex1<= 0;	//Rfout1
				Mex2<= 1;	//Sext_Imm6;
				wCCR<= 0;	//Write CCR
				alu_ctrl<=0;	//ADD
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1; 	//Write back Alu_out
				if(IR[8:6]==3'b111)	//If RB is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0010:	//NDU, NDC, NDZ
				rA2<= IR[8:6];	//RB
				wA<= IR[5:3];	//RC
				rA1<= IR[11:9];	//RA
				Mex1<= 0;	//Rfout1
				Mex2<= 0;	//Rfout2
				wCCR<= 0;	//Write CCR
				alu_ctrl<=1;	//Nand operation
				wMem<=1;	// No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=1;	//Write back Alu_out
				if(IR[5:3]==3'b111)	//If RC is R7
					Mr7WB<=3;	//Write back Alu_out to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0011:	//LHI
				wA<= IR[11:9];	//RA
				Mex1<=0;	//Don't care
				Mex2<=0;	//Don't care
				wCCR<=1;	//CCR write not required
				alu_ctrl<=0;	//Don't care
				wMem<=1;	//No memory write
				MmemR<=0;	//Don't Care
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=2;	//Write back Imm970 to RA
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=0;	// Write back Imm970 to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0100:	//LW
				wA<= IR[11:9];	//RA
				rA2<=IR[8:6];	//RB
				Mex1<=1;	//Sign Extended Immediate six bit
				Mex2<=0;	//Rfout2
				wCCR<=0;	//Zero flag is set by this instruction
				alu_ctrl<=0;	//ADD
				wMem<=1;	//No memory write
				MmemR<=1;	//Read from memory using address in Alu_out
				MmemW<=0;	//Don't Care
				MmemData<=0;	//Don't Care
				MregWB<=0;	//Write back mem_data
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=1;	// Write back mem_data to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0101:	//SW
				rA2<= IR[8:6];	//RB
				rA1<= IR[11:9];	//RA
				Mex1<=1;	//Sign Extended Immediate six bit
				Mex2<=0;	//Rfout2
				wCCR<=1;
				alu_ctrl<=0;	//Add
				wMem<=0;	// Write to memory
				MmemR<=0;	//Don't Care
				MmemW<=1;	//Write to memory. address in Alu_out
				MmemData<=2;	//Write to memory. Data present in rfout1
				MregWB<=0;	//Don't Care
				Mr7WB<=0;	//Don't Care
				break;
			0110:	//LM
				rA1<= IR[11:9];	//RA
				for(i=0;i<8;i++)
				{
					if(LM_Imm[i]==1)
					{
						case (i)
						0:	wA<=3'b000; break;	//R0
						1:	wA<=3'b001; break;	//R1
						2:	wA<=3'b010; break;	//R2
						3:	wA<=3'b011; break;	//R3
						4:	wA<=3'b100; break;	//R4
						5:	wA<=3'b101; break;	//R5
						6:	wA<=3'b110; break;	//R6
						7:	wA<=3'b111;	//R7
						break;
					}
				}
				wCCR<=1;	//No disturbance to CCR
				wMem<=1;	//No memory write operation
				MmemR<=0;	//Read from memory using address stored in RA
				MmemW<=0;	//Don't Care
				MregWB<=0;	//Write back the value in mem_data
				if(IR[11:9]==3'b111) //If RA is R7
					Mr7WB<=1;	// Write back mem_data to R7
				else
					Mr7WB<=0;	//Don't Care
				break;
			0111:	//SM
				rA1<= IR[11:9];	//RA
				for(i=0;i<8;i++)
				{
					if(LM_Imm[i]==1)
					{
						case (i)
						0:	rA2<=3'b000; break;	//R0
						1:	rA2<=3'b001; break;	//R1
						2:	rA2<=3'b010; break;	//R2
						3:	rA2<=3'b011; break;	//R3
						4:	rA2<=3'b100; break;	//R4
						5:	rA2<=3'b101; break;	//R5
						6:	rA2<=3'b110; break;	//R6
						7:	rA2<=3'b111;	//R7
						break;
					}
				}
				wCCR<=1;	//CCR write not to be done
				wMem<=1;	//Write memory operation done
				MmemW<=0;	//Write to memory. address in RA
				MmemData<=3;	//Write to memory. data in Rfout2
				MregWB<=0;	//Don't Care
				Mr7WB<=0;	//Don't Care
				break;
			1100:	//BEQ
				rA1<= IR[11:9];	//RA
				rA2<= IR[8:6];	//RB
				wMem<=1;	//No memory write
				wCCR<=1;	//No disturbance to CCR
				MregWB<=0;	//Don't Care
				Mr7WB<=2;	//PC_Imm -> r7
				break;
			1000:	//JAL
				wA<= IR[11:9];	//RA
				wMem<=1;	//Don't disturb your memory
				wCCR<=1;	//Let these kids rest in peace
				MregWB<=3;	//Write back PC+1
				Mr7WB<=2;	//PC_Imm -> r7
				break;
			1001:	//JLR
				rA2<= IR[8:6];	//RB
				wA<= IR[11:9];	//RA
				wMem<=1;	//Don't disturb your memory
				wCCR<=1;	//Let these kids rest in peace
				MregWB<=3;	//Write back PC+1
				Mr7WB<=4;	//Write Rfout2 to R7
				break;
		end
	end
	
	
endmodule

`endif