`ifndef _DECODE
`define _DECODE

module decode(fromPipe1PC, fromPipe1IR, PC_Imm, rA1, rA2, wA, Sext_Imm6, Imm970, Mex1, Mex2, wCCR, wMem, wRF);
	output [15:0] PC_Imm, Sext_Imm6, Imm970;
	output [2:0] rA1, rA2, wA;
	output Mex1, Mex2, wCCR, wMem, wRF;
	input [15:0] fromPipe1PC, fromPipe1IR;
	reg [15:0] imm6, imm9;
	reg select, offset;
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
				wMem<=1;	// No memory write
/*				wR7<=1;	//R7 not being written
				case(IR[1:0])
				begin
					00:
						wRF<= 0; //Register file write
						break;
					10:
						if(carry set)
							wRF<= 0; //Register file write
						else
							wRF<= 1; //Do not store in RegC
					01:*/
				break;
			0001:	//ADI
				rA1<= IR[11:9];
				wA<= IR[8:6];
				Mex1<= 0;	//Rfout1
				Mex2<= 1;	//Sext_Imm6;
				wCCR<= 0;	//Write CCR
				wMem<=1;	// No memory write
				
				break;
			0011:	//LHI
				wA<= IR[11:9];
				
				break;
			0100:	//LW
				wA<= IR[11:9];
				break;
			0101:	//SW
				rA2<= IR[8:6];
				rA1<= IR[11:9];
				break;
			0110:	//LM
				rA1<= IR[11:9];
				break;
			0111:	//SM
				rA1<= IR[11:9];
				break;
			1100:	//BEQ
				rA1<= IR[11:9];
				rA2<= IR[8:6];
				break;
			1000:	//JAL
				wA<= IR[11:9];
				break;
			1001:	//JLR
				rA2<= IR[8:6];
				wA<= IR[11:9];
				break;
			default:
				rA2<= IR[8:6];	//RB
				wA<= IR[5:3];	//RC
				rA1<= IR[11:9];	//RA
		end
	end
	
	
endmodule

`endif