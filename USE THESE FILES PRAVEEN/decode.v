




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


