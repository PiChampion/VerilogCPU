module proj1(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);

	input clk;
	input rst;
	output MemRW_IO;		//Memory Read/Write Signal
	output [7:0]MemAddr_IO;	//Memory Address
	output [15:0]MemD_IO; 	//Memory Data

	wire MemRW;		//Memory Read/Write Signal
	wire [7:0]MemAddr;	//Memory Address
	wire [15:0]MemD; 	//Memory Data
	wire [15:0]MemQ;

	wire zflag;
	wire [7:0]opcode;
	wire muxPC;
	wire muxMAR;
	wire muxACC;
	wire loadMAR;
	wire loadPC;
	wire loadACC;
	wire loadMDR;
	wire loadIR;
	wire opALU;
	wire mul;
	wire loadMUL;
	

//one instance of memory
ram r1(MemRW, MemD, MemQ, MemAddr);

//one instance of controller
ctr c1(
	clk,
	rst,
	zflag,
	opcode,
	muxPC,
	muxMAR,
	muxACC,
	loadMAR,
	loadPC,
	loadACC,
	loadMDR,
	loadIR,
	opALU,
	MemRW, mul, loadMUL
);

//one instance of datapath1
datapath datapath1(
	clk,
	rst,
	muxPC,
	muxMAR,
	muxACC,
	loadMAR,
	loadPC,
	loadACC,
	loadMDR,
	loadIR,
	opALU,
	zflag,
	opcode,
	MemAddr,
	MemD,
	MemQ, mul, loadMUL
);

//these are just to observe the signals.
assign MemAddr_IO = MemAddr;
assign MemD_IO = MemD;
assign MemRW_IO = MemRW;


endmodule

module  ram(we, d, q, addr);

	input we; 				//	Read(0)/Write(1) Enable
	input [15:0] d;		//	Data Input
	input [7:0] addr;		// Input Address
	output reg [15:0] q;		// Data Output

	reg [15:0]mem[0:255]; // 256x16 Memory
	
	always@(addr or d or we) begin
		if(we) 	mem[addr] = d;
		else 	q = mem[addr];
	end

	
endmodule

module alu(A, B, opALU, Rout);

	input [15:0] A;		//	Input 1
	input [15:0] B;		//	Input 2
	input opALU;			//	Input (1:A+B) (0:A|B)
	output reg [15:0] Rout;	//	Output

	always@(*) begin
		case(opALU)
		1'b0: Rout = A | B;
		1'b1: Rout = A + B;
		default: ;
		endcase
	end

endmodule

module ctr (
	clk,
	rst,
	zflag,
	opcode,
	muxPC,
	muxMAR,
	muxACC,
	loadMAR,
	loadPC,
	loadACC,
	loadMDR,
	loadIR,
	opALU,
	MemRW, mul, loadMUL
);

	input clk;
	input rst;
	input zflag;
	input [7:0]opcode;
	
	output reg muxPC;
	output reg muxMAR;
	output reg muxACC;
	output reg loadMAR;
	output reg loadPC;
	output reg loadACC;
	output reg loadMDR;
	output reg loadIR;
	output reg opALU;
	output reg MemRW;
	output reg mul;
	output reg loadMUL;
		   
	reg [3:0] reg_state;
	reg [4:0] mulcounter;
	
//These opcode representation need to be followed for proper operation
parameter op_add=8'b001;
parameter op_or= 8'b010;
parameter op_jump=8'b011;
parameter op_jumpz=8'b100;
parameter op_load=8'b101;
parameter op_store=8'b110;
parameter op_mul=8'b1001;

//These are the states
parameter Fetch_1=4'b0000;
parameter Fetch_2=4'b0001;
parameter Fetch_3=4'b0010;
parameter Decode=4'b0011;
parameter ExecADD_1=4'b0100;
parameter ExecOR_1=4'b0101;
parameter ExecLoad_1=4'b0110;
parameter ExecStore_1=4'b0111;
parameter ExecJump=4'b1000;
parameter ExecADD_2=4'b1001;
parameter ExecOR_2=4'b1010;
parameter ExecLoad_2=4'b1011;
parameter ExecMUL_1=4'b1100;
parameter ExecMUL_2=4'b1101;
parameter ExecMUL_3=4'b1110;
parameter ExecMUL_4=4'b1111;

	
always@(posedge clk) begin
	if(rst) begin
	    muxPC <= 1'b0; 
		muxMAR <= 1'b0; 
		muxACC <= 1'b0; 
		loadMAR <= 1'b0; 
		loadPC <= 1'b0; 
		loadACC <= 1'b0; 
		loadMDR <= 1'b0; 
		loadIR <= 1'b0; 
		opALU <= 1'b0; 
		MemRW <= 1'b0; 
		mul <= 1'b0;
		loadMUL <= 1'b0;
		reg_state <= Fetch_1;    
	end
	else begin
		case(reg_state)
			Fetch_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b1; 
				loadPC <= 1'b1; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_2;
			end
			Fetch_2: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b1; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_3;
			end
			Fetch_3: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b1; 
				opALU <= 2'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Decode;
			end
			Decode: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b1; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b1; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				case(opcode)
					op_add: reg_state <= ExecADD_1;
					op_or: reg_state <= ExecOR_1;
					op_mul: reg_state <= ExecMUL_1;
					op_jump: reg_state <= ExecJump;
					op_jumpz: if(zflag) reg_state <= ExecJump; else reg_state <= Fetch_1;
					op_load: reg_state <= ExecLoad_1;
					op_store: reg_state <= ExecStore_1;
					default: reg_state <= Fetch_1;
				endcase
			end
			ExecADD_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b1; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= ExecADD_2;
			end
			ExecOR_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b1; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= ExecOR_2;
			end
			ExecMUL_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b1;
				loadIR <= 1'b0; 
				opALU <= 1'b0;
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				mulcounter <= 5'b0;
				reg_state <= ExecMUL_2;
			end
			ExecLoad_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b1; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0; 
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= ExecLoad_2;
			end
			ExecStore_1: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b1;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_1;
			end
			ExecJump: begin
				muxPC <= 1'b1; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b1; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_1;
			end
			ExecADD_2: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b1; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b1; 
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_1;
			end
			ExecOR_2: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b1; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				reg_state <= Fetch_1;
			end
			ExecMUL_2: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0;
				loadIR <= 1'b0; 
				opALU <= 1'b0;
				MemRW <= 1'b0;
				mul <= 1'b1;
				loadMUL <= 1'b0;
				reg_state <= ExecMUL_3;
			end
			ExecLoad_2: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b1; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b1; 
				loadMDR <= 1'b0; 
				loadIR <= 1'b0; 
				opALU <= 1'b0; 
				MemRW <= 1'b0;	
				mul <= 1'b0;
				loadMUL <= 1'b0;				
				reg_state <= Fetch_1;
			end
				ExecMUL_3: begin
				muxPC <= 1'b0; 
				muxMAR <= 1'b0; 
				muxACC <= 1'b0; 
				loadMAR <= 1'b0; 
				loadPC <= 1'b0; 
				loadACC <= 1'b0; 
				loadMDR <= 1'b0;
				loadIR <= 1'b0; 
				opALU <= 1'b0;
				MemRW <= 1'b0;
				mul <= 1'b0;
				loadMUL <= 1'b0;
				mulcounter <= mulcounter + 1;
				reg_state <= ExecMUL_4;
			end
				ExecMUL_4: begin
				if(mulcounter == 5'b01010) begin
					muxPC <= 1'b0; 
					muxMAR <= 1'b0; 
					muxACC <= 1'b0; 
					loadMAR <= 1'b0; 
					loadPC <= 1'b0; 
					loadACC <= 1'b1; 
					loadMDR <= 1'b0;
					loadIR <= 1'b0; 
					opALU <= 1'b0;
					MemRW <= 1'b0;
					mul <= 1'b0;
					loadMUL <= 1'b1;
					reg_state <= Fetch_1;
				end
				else begin
					muxPC <= 1'b0; 
					muxMAR <= 1'b0; 
					muxACC <= 1'b0; 
					loadMAR <= 1'b0; 
					loadPC <= 1'b0; 
					loadACC <= 1'b0; 
					loadMDR <= 1'b0;
					loadIR <= 1'b0; 
					opALU <= 1'b0;
					MemRW <= 1'b0;
					mul <= 1'b0;
					loadMUL <= 1'b0;
					reg_state <= ExecMUL_3;
				end
			end
			default: reg_state <= 4'bzzzz;
		endcase
	end
end

endmodule

module registers(
	clk,
	rst,
	PC_reg, PC_next,
	IR_reg,  IR_next,  
	ACC_reg,  ACC_next,  
	MDR_reg,  MDR_next,  
	MAR_reg,  MAR_next,  
	Zflag_reg, zflag_next
);

	input wire clk;
	input wire rst;
	output reg  [7:0]PC_reg;
	input wire  [7:0]PC_next;
 
	output reg  [15:0]IR_reg;
	input wire  [15:0]IR_next;
	
	output reg  [15:0]ACC_reg;
	input wire  [15:0]ACC_next;
	
	output reg  [15:0]MDR_reg;
	input wire  [15:0]MDR_next;
	
	output reg  [7:0]MAR_reg;
	input wire  [7:0]MAR_next;
	
	output reg Zflag_reg;
	input wire zflag_next;

always @ (posedge clk) begin
	if(rst) begin
		PC_reg <= 8'b0;
		IR_reg <= 16'b0;
		ACC_reg <= 16'b0; 
		MDR_reg <= 16'b0;
		MAR_reg <= 8'b0; 
		Zflag_reg <= 1'b0;
	end
	else begin
		PC_reg <= PC_next;
		IR_reg <= IR_next;
		ACC_reg <= ACC_next;
		MDR_reg <= MDR_next;
		MAR_reg <= MAR_next;
		Zflag_reg <= zflag_next;
	end
end

endmodule

module datapath(
	clk,
	rst,
	muxPC,
	muxMAR,
	muxACC,
	loadMAR,
	loadPC,
	loadACC,
	loadMDR,
	loadIR,
	opALU,
	zflag,
	opcode,
	MemAddr,
	MemD,
	MemQ, mul, loadMUL
);

	input clk;
	input  rst;
	input  muxPC;
	input  muxMAR;
	input  muxACC;
	input  loadMAR;
	input  loadPC;
	input  loadACC;
	input  loadMDR;
	input  loadIR;
	input  opALU; 
	output   zflag;
	output   [7:0]opcode;
	output   [7:0]MemAddr;
	output   [15:0]MemD;
	input   [15:0]MemQ;
	input mul;
	input loadMUL;

	reg  [7:0]PC_next;
	reg  [15:0]IR_next;    //CHANGED to REG
	reg  [15:0]ACC_next;  
	reg  [15:0]MDR_next;  //CHANGED to REG
	reg  [7:0]MAR_next;  
	reg zflag_next;
	
	
	wire  [7:0]PC_reg;
	wire  [15:0]IR_reg;  
	wire  [15:0]ACC_reg;  
	wire  [15:0]MDR_reg;  
	wire  [7:0]MAR_reg;  
	wire zflag_reg;
	
	wire  [15:0]ALU_out;  
	wire  [15:0]MUL_out; 

//one instance of ALU
	alu alu1(ACC_reg, MDR_reg, opALU, ALU_out);
//one instance of register.
	registers reg1(
		clk,
		rst,
		PC_reg, PC_next,
		IR_reg,  IR_next,  
		ACC_reg,  ACC_next,  
		MDR_reg,  MDR_next,  
		MAR_reg,  MAR_next,  
		Zflag_reg, zflag_next
	);

	HW6_1 multiplier(clk, rst, mul, ACC_reg[7:0], MDR_reg[7:0], MUL_out);
	
always@(*)
	begin
// [7:0]PC_next;
// Only change if loadpc is enabled.
// Mux pc decides between pc+1 or branch address
// Reset address is 0, Hence nothing for the datapath to do at reset.
	if(loadPC) begin
		case(muxPC)
			1'b0: PC_next = PC_reg + 1'b1;
			1'b1: PC_next = IR_reg[15:8];
			default: PC_next = PC_reg;
		endcase
	end
	else PC_next = PC_reg;



// [15:0]IR_next;  
// Gets value of mdr_reg if loadir is set

	if(loadIR) IR_next = MDR_reg;
	else IR_next = IR_reg;


 // [15:0]ACC_next;  
// Only change when loaddacc is enabled.
// Muxacc decides between mdr_reg and alu out

	if(loadACC) begin
		if(loadMUL) ACC_next = MUL_out;
		else begin
		case(muxACC)
			1'b0: ACC_next = ALU_out;
			1'b1: ACC_next = MDR_reg;
			default: ACC_next = ACC_reg;
		endcase
		end
	end
	else ACC_next = ACC_reg;


 // [15:0]MDR_next;  
// Gets value from memeory,  if load mdr is set

	if(loadMDR)
		MDR_next = MemQ;
	else MDR_next = MDR_reg;


 // [7:0]MAR_next;  
// Only change if loadmar is enabled.
// Mux mar decides between  pcreg or IR[15:8]reg
	
	if(loadMAR)
		case(muxMAR)
			1'b0: MAR_next = PC_reg;
			1'b1: MAR_next = IR_reg[15:8];
			default: MAR_next = MAR_reg;
		endcase
	else MAR_next = MAR_reg;

 // zflag_next;
// Decide  based on the content of acc_reg
	
	if(ACC_reg) zflag_next = 1'b0;
	else zflag_next = 1'b1;

end

   // output   zflag; => based on ACC reg
	assign zflag = !ACC_reg;
   // output   [7:0]opcode; => based on IR_reg
	assign opcode = IR_next[7:0];
   // output   [7:0]MemAddr => Same as MAR_reg
	assign MemAddr = MAR_reg;
   // output   [15:0]MemD => Same as ACC reg
	assign MemD = ACC_reg;


endmodule


module HW6_1 (input clk, input rst, input ld, input [7:0] a, b, output reg [15:0] O);

reg [7:0] A, B;
reg [1:0] state;
reg [2:0] C;
wire [15:0] middleman;
wire [15:0] decoyO, tempO;
reg [15:0] middleO;
reg done;

shifter shift({8'b0,A}, C, middleman);
fulladder add(O, middleman, tempO);
fulladder add2(O, 16'b0, decoyO);

always@(posedge clk) begin
if(rst) begin 
	state <= 2'b00; 
	O <= 0;
	done <= 0; 
end
else begin
	if(ld && (state == 2'b00)) begin
		A <= a;
		B <= b;
		C <= 3'b000;
		state <= 2'b01;
	end
	else begin
		case({done,state})
			3'b000 : ;
			3'b001 : begin 
				if(B[C]) begin 
					middleO <= tempO;
					end
				else begin
					middleO <= decoyO;
					end
				C <= C + 1;
				state = 2'b10;
				end	
			3'b010 : begin 
				O <= middleO;
				if(C) state <= 3'b001;
				else state <= 2'b11;
				end
			3'b011 : begin 
				done <= 1'b1;
				state <= 2'b00;
				end
			3'b1xx : ;
			default : ;
		endcase
	end
end
end

endmodule


module fulladder (input [15:0] x, input [15:0] y, output [15:0] O);

assign O = y + x;

endmodule

module shifter (input [15:0] in, input [2:0] N, output [15:0] O);

reg [15:0] out_reg; 

assign O = out_reg;

always @(N or in) begin 

case (N) 
7 : out_reg <= {in[7:0],7'b0}; 
6 : out_reg <= {in[7:0],6'b0}; 
5 : out_reg <= {in[7:0],5'b0}; 
4 : out_reg <= {in[7:0],4'b0}; 
3 : out_reg <= {in[7:0],3'b0}; 
2 : out_reg <= {in[7:0],2'b0}; 
1 : out_reg <= {in[7:0],1'b0}; 
0 : out_reg <= in[7:0]; 

endcase 

end

endmodule


module proj1_tb;

	reg clk;
	reg rst;
	wire MemRW_IO;
	wire [7:0]MemAddr_IO;
	wire [15:0]MemD_IO;
	   
always @(t.c1.reg_state) begin
	case (t.c1.reg_state)
		0: $display($time," Fetch_1");
		1: $display($time," Fetch_2");
		2: $display($time," Fetch_3");
		3: $display($time," Decode");
		4: $display($time," ExecADD_1");
		5: $display($time," ExecOR_1");
		6: $display($time," ExecLoad_1");
		7: $display($time," ExecStore_1");
		8: $display($time," ExecJump");
		9: $display($time," ExecADD_2");
		10: $display($time," ExecOR_2");
		11: $display($time," ExecLoad_2");
		12: $display($time," ExecMUL_1");
		13: $display($time," ExecMUL_2");
		14: $display($time," ExecMUL_3");
		15: $display($time," ExecMUL_4");
		default: $display($time," Unrecognized State");
	endcase
end

always 
begin
      #5  clk =  !clk; 
end
		
initial begin
	$dumpfile("Proj1_waves.vcd");
  	$dumpvars;
	clk=1'b0;
	rst=1'b1;
	$readmemh("memory.txt", proj1_tb.t.r1.mem);
	#20 rst=1'b0;
	#4350 
	$display("Final value\n");
	$display("0x00d %d\n",proj1_tb.t.r1.mem[16'h000d]);
	$finish;
end

proj1 t(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);

endmodule
