/**********************
* Description
*	This is the top-level of a MIPS processor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
**********************/


module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 32
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//**********************/
//**********************/
assign  PortOut = 0;

//**********************/
//**********************/
// Data types to connect modules
wire BranchNE_wire;
wire BranchEQ_wire;
wire RegDst_wire;
wire NotZeroANDBrachNE;
wire ZeroANDBrachEQ;
wire ORForBranch;
wire ALUSrc_wire;
wire RegWrite_wire;
wire Zero_wire;
wire Jump_wire;
wire JAL_wire;
wire JR_wire;
wire JOrPC4OrBranchOrJR_wire;
wire JumpOrPC4OrBranch_wire;
wire RegWriteORJAL_wire;
wire [2:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [4:0] WriteRegister_wire;
wire [31:0] MUX_PC_wire;
wire [31:0] PC_wire;
wire [31:0] Instruction_wire;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] InmmediateExtend_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] ALUResult_wire;
wire [31:0] PC_4_wire;
wire [31:0] InmmediateExtendAnded_wire;
wire [31:0] JumpOrPC4OrBranch_wire;
wire [31:0] BranchOrPC4_wire;
wire [31:0] JumpAddrSh2_wire; //Jump address shifted 2 bits
wire [31:0] BranchAddrSh2_wire	//Branch address shifted 2 bits
wire [31:0] Link_wire;
wire [31:0] JumpAddr;
wire [31:0] BranchToPC_wire;
integer ALUStatus;


//**********************/

ANDGate
Gate_JumpANDLeast
(
	.A(Jump_wire)
	.B(Instruction_wire[26]) //bit menos significativo del opcode porque J 000010 y JAL 000011
	.C(JAL_wire)
);

//**********************/

ANDGate
Gate_BranchEQANDZero
(
	.A(BranchEQ_wire)
	.B(zero) //bit menos significativo del opcode porque J 000010 y JAL 000011
	.C(ZeroANDBrachEQ)
);

//**********************/

ANDGate
Gate_BranchNEANDZero
(
	.A(BranchNE_wire)
	.B(!zero) //Si zero es diferente de 1,, significa que es diferente
	.C(NotZeroANDBrachNE)
);

//**********************/

ORGate
Gate_BeqOrBNE
(
	.A(NotZeroANDBrachNE)
	.B(ZeroANDBrachEQ)
	.C(ORForBranch)
);

//**********************/
Control
ControlUnit
(
	.OP(Instruction_wire[31:26]),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.ALUOp(ALUOp_wire),
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire),
	.Jump(Jump_wire)
);

 PC_Register
#(
	.N(32)
)

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o
ProgramCounter
(
	.clk(clk),
	.reset(reset),
	.NewPC(JOrPC4OrBranchOrJR_wire),
	.PCValue(PC_wire)
);


ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);

Adder32bits
PC_Puls_4
(
	.Data0(PC_wire),
	.Data1(4),

	.Result(PC_4_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
JumpShifter
(
	.DataInput({6'b0,Instruction_wire[25:0]}),
   .DataOutput(JumpAddrSh2_wire)
);

Adder32bits
PC_Puls_4
(
	.Data0(PC_4_wire),
	.Data1({PC_4_wire[31:28], JumpAddrSh2_wire[28:0]}),

	.Result(JumpAddr)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
BranchShifter
(
	 .DataInput({6'b0,InmmediateExtend_wire[15:0]}),
   .DataOutput(BranchAddrSh2_wire)
);

Adder32bits
PC_Puls_4
(
	.Data0(PC_4_wire),
	.Data1({PC_4_wire[31:28], BranchAddrSh2_wire[28:0]}),

	.Result(BranchToPC_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForBranch
(
	.Selector(ORForBranch),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(BranchToPC_wire),

	.MUX_Output(BranchOrPC4_wire)

);


Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJump
(
	.Selector(Jump_wire),
	.MUX_Data0(BranchOrPC4_wire),
	.MUX_Data1(JumpAddr),

	.MUX_Output(JumpOrPC4OrBranch_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJumpRegister
(
	.Selector(JR_wire),
	.MUX_Data0(JumpOrPC4OrBranch_wire),
	.MUX_Data1(ReadData1_wire),

	.MUX_Output(JOrPC4OrBranchOrJR_wire)

);

//**********************/
//**********************/
//**********************/
//**********************/
//**********************/
Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire),
	.MUX_Data0(Instruction_wire[20:16]),
	.MUX_Data1(Instruction_wire[15:11]),

	.MUX_Output(WriteRegister_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForLink
(
	.Selector(JAL_wire),
	.MUX_Data0(ALUResult_wire),
	.MUX_Data1(PC_4_wire),

	.MUX_Output(Link_wire)

);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForJalorRorIType
(
	.Selector(JAL_wire),
	.MUX_Data0(WriteRegister_wire),
	.MUX_Data1(31),

	.MUX_Output(RAorWriteReg_wire)

);
assign RegWriteORJAL_wire = RegWrite_wire | JAL_wire;


RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(RegWriteORJAL_wire),
	.WriteRegister(RAorWriteReg_wire),
	.ReadRegister1(Instruction_wire[25:21]),
	.ReadRegister2(Instruction_wire[20:16]),
	.WriteData(Link_wire),
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

SignExtend
SignExtendForConstants
(
	.DataInput(Instruction_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);



Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSrc_wire),
	.MUX_Data0(ReadData2_wire),
	.MUX_Data1(InmmediateExtend_wire),

	.MUX_Output(ReadData2OrInmmediate_wire)

);


ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire),
	.ALUFunction(Instruction_wire[5:0]),
	.ALUOperation(ALUOperation_wire),
	.JR(JR_wire)
);



ALU
Arithmetic_Logic_Unit
(
	.ALUOperation(ALUOperation_wire),
	.A(ReadData1_wire),
	.B(ReadData2OrInmmediate_wire),
	.shamt(Instruction_wire[10:6]),
	.Zero(Zero_wire),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;


endmodule
