

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



