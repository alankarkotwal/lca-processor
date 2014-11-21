


//It's actually forward Ex stage
module forward_Ex_stage(mem_wb_op,mem_wb_regA,mem_wb_regB,mem_wb_regC,ex_mem_op,ex_mem_regA,ex_mem_regB,ex_mem_regC,regread_ex_op,regread_ex_regA,regread_ex_regB,
regread_ex_regC,F1,F2,FCCR,mem_wb_CCR_write,ex_mem_CCR_write);

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
		
		
		
		
		else if(regread_ex_op[5:2]==LM)
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
					
			end
		
		
		else if(regread_ex_op[5:2]==SM)
			begin
				if((regread_ex_regA == mem_wb_regC)&&(mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC
									||mem_wb_op==ADZ||mem_wb_op==NDC
									||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
							F1 = 3'd2;//b
				else if((regread_ex_regA == mem_wb_regA)&&(mem_wb_op[5:2]==LW||mem_wb_op[5:2]==LM))
							F1 = 3'd3;//c
		
				else if((regread_ex_regA==mem_wb_regA)&&(mem_wb_op==LHI))
							F1 = 3'd6;//j	
				else if((regread_ex_regA == mem_wb_regA)&& (mem_wb_op[5:2] ==JAL))
							F1 = 3'd7;//k -> PC+1	
				else 
					F1 = 3'b0;
			end	
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
			FCCR = 2'b1;
		else if((mem_wb_op==ADD||mem_wb_op==NDU||mem_wb_op==ADC||mem_wb_op==ADZ||mem_wb_op[5:2]==ADI||mem_wb_op==NDC||mem_wb_op==NDZ)&&(mem_wb_CCR_write==1'b0))
			FCCR = 2'd2;
		else if((regread_ex_op==ADZ||regread_ex_op==NDZ)&&(ex_mem_op==LW)&&(ex_mem_CCR_write==1'b0))
		
			FCCR = 2'b1;
		
		else if((regread_ex_op==ADZ||regread_ex_op==NDZ)&&(mem_wb_op==LW)&&(mem_wb_CCR_write==1'b0))
			FCCR = 2'd2;
			
		else 
			FCCR = 2'b0;
		end

else 
	FCCR = 2'b0;
end


endmodule






