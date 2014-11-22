`ifndef _WRITE_BACK
`define _WRITE_BACK

`include "../misc/mux.v"

module write_back(clk, reset, Imm970, MemData, PCImmInc, ALUOut, PCInc, regSelect, r7Select, writeData, writeR7Data);

	output [15:0] writeData, writeR7Data;
	input  [15:0] Imm970, MemData, PCImmInc, ALUOut, PCInc;
	input  [ 1:0] r7Select;
	input  [ 1:0] regSelect;
	input         clk, reset;

	mux16x4 regWriteMux(.data0(MemData), .data1(ALUOut), .data2(Imm970), .data3(PCInc), .selectInput(regSelect), .out(writeData));
	mux16x4 r7WriteMux(.data0(Imm970), .data1(MemData), .data2(PCImmInc), .data3(ALUOut), .selectInput(r7Select), .out(writeR7Data));

endmodule

`endif
