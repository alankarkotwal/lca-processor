module execute(	clk, reset, ALUOut, ALUOp,fromPlusOneMem, fromRFOut1, fromRFOut2, RASelectInput, CCRWrite, CCR_Write_from_wb,CCRWriteValue,CCRWriteValue_from_wb, fromSImm6, ExMux1Select, ExMux2Select,
		RAOut, CCR,IR,SignalA,SignalB,SignalC,SignalG,SignalI,SignalJ,SignalK,SignalX,SignalY,
		mem_wb_op,mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_op,ex_mem_regA,ex_mem_regB,ex_mem_regC,
		regread_ex_op,regread_ex_regA,regread_ex_regB,regread_ex_regC,,mem_wb_CCR_write,ex_mem_CCR_write,r7,rf);

		
parameter ADD = 6'b000000;
parameter NDU = 6'b001000;
parameter ADC = 6'b000010;
parameter ADZ = 6'b000001;
parameter ADI = 4'b0001;
parameter NDC = 6'b001010;
parameter NDZ = 6'b001001;
	output [15:0] RAOut, ALUOut;
	output [ 1:0] CCR;
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
	input mem_wb_CCR_write,ex_mem_CCR_write;
	//
	
	
	wire   [15:0] ALUIn1, ALUIn2, ExMux1Out, ExMux2Out;
	wire[1:0] CCRMux_out;//CCRMux_out[1] = zero, CCRMux_out[0] = carry
	wire[1:0] CCR_muxSelect;
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
.regread_ex_regC(regread_ex_regC),.F1(ExMux3Select),.F2(ExMux4Select),.FCCR(CCR_muxSelect),.mem_wb_CCR_write(mem_wb_CCR_write),.ex_mem_CCR_write(ex_mem_CCR_write).rf(rf),.r7(r7));
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

module mux2x4(data0, data1, data2, data3,selectInput,out);
	output reg[1:0] out;
	input [1:0] data0, data1, data2, data3;
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
output reg rf,r7;


			
			
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






