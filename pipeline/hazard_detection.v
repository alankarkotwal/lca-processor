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
			if(op3==BEQ&&equ==1'b1)
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
			else if(op1==ADI&&pr1RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op2==ADI&&pr2RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op3==ADI&&pr2RB==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==LW||op1==LM)&&pr1RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op2==LW||op2==LM)&&pr2RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op3==LW||op3==LM)&&pr3RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op4==LW||op4==LM)&&pr4RA==3'b111)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1==LM||op1==SM)
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
			else if((op1==LW)&&(op2==LW||op2==LM)&&(pr1RB==pr2RA)) // lw/lm followed by lw
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==LM)&&(op2==LW||op2==LM)&&(pr1RA==pr2RA)) //lw/lm followed by LM
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==SW)&&(op2==LW||op2==LM)&&(pr1RB==pr2RA))        //load followed by store
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1==JAL)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op1==JLR)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if(op2==JLR)
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			 
			else if((op1==SM||op1==LM)&&(op2==ADD||op2==NDU||op2==ADC||op2==ADZ||op2==NDC||op2==NDC||op2==NDZ)&&(pr1RA==pr2RC))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==SM||op1==LM)&&(op2==ADI)&&(pr1RA==pr2RB))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==SM||op1==LM)&&(op2==LHI)&&(pr1RA==pr2RA))
				begin
				flush_reg_ex=1'b0;
				flush_id_reg=1'b0;
				flush_if_id=1'b1;
				pc_write = 1'b1;
				end
			else if((op1==SM||op1==LM)&&(op2==LW||op2==LM)&&(pr1RA==pr2RA))
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
		
			
			