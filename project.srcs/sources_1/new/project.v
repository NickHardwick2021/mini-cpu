`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/18/2019 09:29:09 AM
// Design Name: 
// Module Name: project
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module project(clk, m2regMUXOut);
    input clk;
    wire wwreg;
    wire wm2reg;
    wire [4:0] FMuxOUT;
    wire [31:0] wALUOutOUT;
    wire [31:0] doDataMemOUT;
    wire wregOUT, m2regOUT, wmemOUT, aluimmOUT;
    wire [3:0] alucOUT;
    wire [4:0] mux1OutOUT;
    wire [31:0] qaOUT, qbOUT, eOUT;
    
    wire [31:0] currentPC;
    wire [31:0] do;
    wire [31:0] PCD;
    wire [31:0] qaOut;
    wire [31:0] qbOut;
    
    wire [31:0] e;    
    wire [4:0] rs, rt, rd;
    wire [5:0] op, func;
    wire [15:0] imm;
    wire regrt, wreg, m2reg, wmem, aluimm;
    wire [3:0] aluc;
    wire [4:0] mux1Out;
    wire [31:0] mux2Out;
    wire [31:0] ALUOut;
    
    wire mwreg, mm2reg, mwmem; //Lab 4 Output
    wire [4:0] mmux1OutOUT;
    wire [31:0] ALUOutOUT; //Lab 4 Output
    wire [31:0] mqbOUT; //Lab 4 Output
    wire [31:0] doDataMem; //Lab 4 Output
    
    //New Project Wires
    wire [31:0] qa, qb;
    wire wpcir;
    wire [1:0] fwdb, fwda;
    
    output [31:0] m2regMUXOut;
    
    program_counter P1(.clk(clk),.pc(currentPC), .wpcir(wpcir));
    instMem M1(.a(currentPC), .do(do));
    ifid_ff FF1(.clk(clk), .do(do), .q(PCD), .wpcir(wpcir));
    assign op = PCD[31:26];
    assign rs = PCD[25:21];
    assign rt = PCD[20:16];
    assign rd = PCD[15:11];
    assign imm = PCD[15:0];
    assign func = PCD[5:0];
    
    signExtender SE(.imm(imm), .e(e));
    
    controlUnit CU(.op(op), .func(func), .wreg(wreg), .m2reg(m2reg), .wmem(wmem), .aluc(aluc), .aluimm(aluimm), .regrt(regrt),
    .rs(rs), .rt(rt), .wpcir(wpcir), .fwda(fwda), .fwdb(fwdb), .mrn(mmux1OutOUT), .mm2reg(mm2reg), .mwreg(mm2reg), .ern(mux1OutOUT), .em2reg(m2regOUT), .ewreg(wregOUT));
    
    mux_01 MUX1(.regrt(regrt), .instr(PCD), .out(mux1Out));
    
    mux_41 MUX41(.fwda(fwda), .qa(qa), .ALUOut(ALUOut), .ALUOutOUT(ALUOutOUT), .doDataMem(doDataMem), .qaOut(qaOut));
    
    mux_42 MUX42(.fwdb(fwdb), .qb(qb), .ALUOut(ALUOut), .ALUOutOUT(ALUOutOUT), .doDataMem(doDataMem), .qbOut(qbOut));
    
    id_exe FF2(.wreg(wreg), .m2reg(m2reg), .wmem(wmem), .aluc(aluc), .aluimm(aluimm), .mux1Out(mux1Out), .qaOut(qaOut), .qbOut(qbOut), .e(e), .wpcir(wpcir),
               .wregOUT(wregOUT), .m2regOUT(m2regOUT), .wmemOUT(wmemOUT), .alucOUT(alucOUT), .aluimmOUT(aluimmOUT), .mux1OutOUT(mux1OutOUT), .qaOUT(qaOUT), .qbOUT(qbOUT), .eOUT(eOUT),.clk(clk));
               
               
    // Lab 4 Starts Here
    mux2 MUX2(.qbOUT(qbOUT), .eOUT(eOUT), .aluimmOUT(aluimmOUT), .mux2Out(mux2Out));
    
    ALU ALU(.alucOUT(alucOUT), .qaOUT(qaOUT), .mux2Out(mux2Out), .ALUOut(ALUOut));
    
    
    exe_mem EXE2(.wregOUT(wregOUT), .m2regOUT(m2regOUT), .wmemOUT(wmemOUT), .mux1OutOUT(mux1OutOUT), .ALUOut(ALUOut), .qbOUT(qbOUT), .clk(clk),
    .mwreg(mwreg), .mm2reg(mm2reg), .mwmem(mwmem), .mmux1OutOUT(mmux1OutOUT), .ALUOutOUT(ALUOutOUT), .mqbOUT(mqbOUT));
    
    dataMem DMEM(.we(mwmem), .addr(ALUOutOUT), .datain(mqbOUT), .clk(clk), .doDataMem(doDataMem));
    
    mem_wb FF3(.mwreg(mwreg), .mm2reg(mm2reg), .mwmem(mwmem), .mmux1OutOUT(mmux1OutOUT), .ALUOutOUT(ALUOutOUT), .doDataMem(doDataMem), .clk(clk),
    .wwreg(wwreg), .wm2reg(wm2reg), .FMuxOUT(FMuxOUT), .wALUOutOUT(wALUOutOUT), .doDataMemOUT(doDataMemOUT));
    
    m2regMUX MUX3(.wm2reg(wm2reg), .wALUOutOUT(wALUOutOUT), .doDataMemOUT(doDataMemOUT), .m2regMUXOut(m2regMUXOut));
    
    regFile RF(.clk(clk), .we(wwreg), .rna(rs), .rnb(rt), .wn(FMuxOUT), .d(m2regMUXOut), .qa(qa), .qb(qb));
endmodule


module instMem(input [31:0] a, output reg [31:0] do);
reg [31:0] rom[1023:0];
integer i;
initial begin
    for (i=0; i<64; i=i+1)
        rom[i] = 0;
    rom[0] = 32'h34040050; // (00) main: ori $4, $zero, 0x50 no stall $4 = 0x0 | 0x50 = 0x50
    rom[4] = 32'h8c880000; // (04) lw $8, 0($4) no stall $8 = ram[$4] = ram[0x50] = 0xa3
    rom[8] = 32'h20840004; // (08) addi $4, $4, 4 no stall $4 = $4 + 4 = 0x54
    rom[12] = 32'h8c890000; // (0c) lw $9, 0($4) no stall $9 = ram[$4] = ram[0x54] = 0x27
    rom[16] = 32'h01094020; // (10) add $8, $8, $9 stall $8 = $8 + $9 = 0xa3 + 0x27 = 0xca
    rom[20] = 32'h20840004; // (14) addi $4, $4, 4 no stall $4 = $4 + 4 = 0x58
    rom[24] = 32'hac880000; // (18) sw $8, 0($4) no stall ram[$4] = $8 -> ram[0x58] = 0xca*/
end

always @(*) begin
    do = rom[a[7:2]];
end
endmodule


module program_counter(
input clk,
input wpcir,
output reg [31:0] pc);

parameter increment_amount = 32'd4;

initial 
    pc=0;
    
always @(posedge clk)
begin
    if (wpcir == 1)
        pc <= pc;
    else
        pc <= pc + increment_amount;
end
endmodule


module  ifid_ff(
input [31:0] do,
input clk,
input wpcir,
output reg [31:0] q);

always @(posedge clk)
begin
    if (wpcir == 1)
        q <= q;
    else
        q <= do;
end
endmodule


module regFile(
input clk,
input we,
input [4:0] rna,
input [4:0] rnb,
input [4:0] wn,
input [31:0] d,
output [31:0] qa,
output [31:0] qb);

reg [31:0] regfile [31:0];

integer i;
initial begin
    for (i=0; i<32; i=i+1)
        regfile[i] = 0;
end

assign qa = regfile[rna];
assign qb = regfile[rnb];

always @(negedge clk)
begin
    if (we)
        regfile[wn] = d;
end
endmodule   


module signExtender(
input [15:0] imm,
output [31:0] e);

assign e = {{16{imm[15]}}, imm};
endmodule

module controlUnit(op,func,wreg,m2reg,wmem,aluc,aluimm,regrt,
rs, rt, fwda, fwdb, wpcir, mrn, mm2reg, mwreg, ern, em2reg, ewreg);
input [5:0] op, func;
input [4:0] rs, rt;
input [4:0] mrn, ern;
input mm2reg, mwreg, em2reg, ewreg;
output reg wreg, m2reg, wmem, aluimm, regrt;
output reg [3:0] aluc;
output reg [1:0] fwda, fwdb;
output reg wpcir;

always@(*)
begin
    fwda = 2'b00;
    fwdb = 2'b00;
    
    //I-Type Forwarding
    if ((ewreg == 1) && (em2reg == 0) && (ern == rt)) begin
        fwda = 2'b01;
        wpcir = 0; 
    end
    else begin
        if ((ewreg == 1) && (em2reg == 1) && (ern == rt)) begin
            wpcir = 1;
        end
        else begin
            if ((mwreg == 1) && (mm2reg == 1) && (mrn == rt)) begin
                fwda = 2'b11;
                wpcir = 0;
            end
            else begin
                if ((mwreg == 1) && (mm2reg == 0) && (mrn == rt)) begin
                    fwda = 2'b10;
                    wpcir = 0;
                end
            end
        end
    end
    
    case(op)
        6'b000000: //Rtype
        begin
            //R-type forwarding
            if ((ewreg == 1) && (em2reg == 0) && (ern == rs)) begin
                fwdb = 2'b01;
                wpcir = 0;
            end
            else if ((ewreg == 1) && (em2reg == 1) && (ern == rs)) begin
                wpcir = 1;
            end
            else if ((mwreg == 1) && (mm2reg == 1) && (mrn == rs)) begin
                fwdb = 2'b11;
                wpcir = 0;
            end
            else if ((mwreg == 1) && (mm2reg == 0) && (mrn == rs)) begin
                fwdb = 2'b10;
                wpcir = 0;
            end
            else begin      
            case(func)
                6'b100000: //Add
                begin
                    wreg=1;
                    m2reg=0;
                    wmem=0;
                    aluimm=0;
                    regrt=0;
                    aluc=4'b0010;
                end
                6'b100010: //sub
                begin
                    wreg=1;
                    m2reg=0;
                    wmem=0;
                    aluimm=0;
                    regrt=0;
                    aluc=4'b0110;               
                end
                6'b100100: //and
                begin
                    wreg=1;
                    m2reg=0;
                    wmem=0;
                    aluimm=0;
                    regrt=0;
                    aluc=4'b0000;
                end
                6'b100101: //or
                begin
                    wreg=1;
                    m2reg=0;
                    wmem=0;
                    aluimm=0;
                    regrt=0;
                    aluc=4'b0001;
                end
                6'b100110: //xor
                begin
                    wreg=1;
                    m2reg=0;
                    wmem=0;
                    aluimm=0;
                    regrt=0;
                    aluc=4'b1100;
                end
            endcase
        end
        end
        6'b100011: //Lw
        begin
            wreg=1;
            m2reg=1;
            wmem=0;
            aluimm=1;
            regrt=1;
            aluc=4'b0010;
        end
        6'b101011: //Sw
        begin
            wreg=1;
            m2reg=1;
            wmem=0;
            aluimm=1;
            regrt=1;
            aluc=4'b0010;
        end
        6'b001000: //addi
        begin
           wreg=1;
           m2reg=0;
           wmem=0;
           aluimm=0;
           regrt=0;
           aluc=4'b0010;
        end
        6'b001100: //andi
        begin
            wreg=1;
            m2reg=0;
            wmem=0;
            aluimm=0;
            regrt=0;
            aluc=4'b0000;
        end
        6'b001101: //ori
        begin
            wreg=1;
            m2reg=0;
            wmem=0;
            aluimm=0;
            regrt=0;
            aluc=4'b0001;
        end
        6'b001110: //xori
        begin
            wreg=1;
            m2reg=0;
            wmem=0;
            aluimm=0;
            regrt=0;
            aluc=4'b0001;
        end
    endcase
end
endmodule

module mux_01(
input regrt,
input [31:0] instr,
output [4:0] out);

wire [4:0] rd;
wire [4:0] rt;
assign rd = instr[15:11];
assign rt = instr[20:16];
assign out = regrt ? rt : rd;
endmodule

module mux_41(
input [1:0] fwda,
input [31:0] qa,
input [31:0] ALUOut,
input [31:0] ALUOutOUT,
input [31:0] doDataMem,
output reg [31:0] qaOut);

always @ (qa or ALUOut or ALUOutOUT or doDataMem or fwda) begin
      case (fwda)
         2'b00 : qaOut = qa;
         2'b01 : qaOut = ALUOut;
         2'b10 : qaOut = ALUOutOUT;
         2'b11 : qaOut = doDataMem;
      endcase
   end
endmodule

module mux_42(
input [1:0] fwdb,
input [31:0] qb,
input [31:0] ALUOut,
input [31:0] ALUOutOUT,
input [31:0] doDataMem,
output reg [31:0] qbOut);

always @ (qb or ALUOut or ALUOutOUT or doDataMem or fwdb) begin
      case (fwdb)
         2'b00 : qbOut = qb;
         2'b01 : qbOut = ALUOut;
         2'b10 : qbOut = ALUOutOUT;
         2'b11 : qbOut = doDataMem;
      endcase
   end
endmodule

module id_exe(
input wreg,
input m2reg,
input wmem,
input [3:0] aluc,
input aluimm,
input [4:0] mux1Out,
input [31:0] qaOut,
input [31:0] qbOut,
input [31:0] e,
input clk,
input wpcir,
output reg wregOUT,
output reg m2regOUT,
output reg wmemOUT,
output reg [3:0] alucOUT,
output reg aluimmOUT,
output reg [4:0] mux1OutOUT,
output reg [31:0] qaOUT,
output reg [31:0] qbOUT,
output reg [31:0] eOUT);

always @(posedge clk)
begin
    if (wpcir == 1) begin
        wregOUT = 0;
        m2regOUT = 0;
        wmemOUT = 0;
        alucOUT = 0;
        aluimmOUT = 0;
        mux1OutOUT = 0;
        qaOUT = 0;
        qbOUT = 0;
        eOUT = 0;
    end
    else begin
    wregOUT = wreg;
    m2regOUT = m2reg;
    wmemOUT = wmem;
    alucOUT = aluc;
    aluimmOUT = aluimm;
    mux1OutOUT = mux1Out;
    qaOUT = qaOut;
    qbOUT = qbOut;
    eOUT = e;
    end
end
endmodule


// LAB 4 BEGINS HERE

//Green 2nd MUX
module mux2(
input [31:0] qbOUT,
input [31:0] eOUT,
input aluimmOUT,
output [31:0] mux2Out);

assign mux2Out = aluimmOUT ? eOUT : qbOUT;
endmodule

//Green ALU
module ALU(
input [3:0] alucOUT,
input [31:0] qaOUT,
input [31:0] mux2Out,
output reg [31:0] ALUOut
);

always@(*)
begin
    case(alucOUT)
    4'b0010:
        begin
            ALUOut = qaOUT + mux2Out;
        end
    4'b0001:
        begin
            ALUOut = qaOUT | mux2Out;
        end
    4'b0000:
        begin
            ALUOut = qaOUT & mux2Out;
        end
    4'b0110:
        begin
            ALUOut = qaOUT - mux2Out;
        end
    4'b0111:
        begin
            if (qaOUT < mux2Out)
                ALUOut = 1;
            else
                ALUOut = 0;
        end
    4'b1100:
        begin
            ALUOut = ~(qaOUT | mux2Out);
        end
    endcase
end
endmodule

//EXE/MEM Pipe
module exe_mem(
input wregOUT,
input m2regOUT,
input wmemOUT,
input [4:0] mux1OutOUT,
input [31:0] ALUOut,
input [31:0] qbOUT,
input clk,
output reg mwreg,
output reg mm2reg,
output reg mwmem,
output reg [4:0] mmux1OutOUT,
output reg [31:0] ALUOutOUT,
output reg [31:0] mqbOUT);

always @(posedge clk)
begin
    mwreg = wregOUT;
    mm2reg = m2regOUT;
    mwmem = wmemOUT;
    mmux1OutOUT = mux1OutOUT;
    ALUOutOUT = ALUOut;
    mqbOUT = qbOUT;
end
endmodule

//Blue DataMem
module dataMem(
input we,
input [31:0] addr,
input [31:0] datain,
input clk,
output [31:0] doDataMem);

reg[31:0]data_memory[127:0];
    initial begin
         data_memory[0] = 32'hA00000AA;
         data_memory[4] = 32'h10000011;
         data_memory[8] = 32'h20000022;
         data_memory[12] = 32'h30000033;
         data_memory[16] = 32'h40000044;
         data_memory[20] = 32'h50000055;
         data_memory[24] = 32'h60000066;
         data_memory[28] = 32'h70000077;
         data_memory[32] = 32'h80000088;
         data_memory[36] = 32'h90000099;
         data_memory[80] = 32'h000000a3;
         data_memory[84] = 32'h00000027;
         data_memory[88] = 32'h00000000;
     end
     
     always @(posedge clk)
     begin
        if (we)
            data_memory[addr] = datain;
     end    
     assign doDataMem = data_memory[addr];
endmodule

//Red MEM/WB
module mem_wb(
input mwreg,
input mm2reg,
input mwmem,
input [4:0] mmux1OutOUT,
input [31:0] ALUOutOUT,
input [31:0] doDataMem,
input clk,
output reg wwreg,
output reg wm2reg,
output reg [4:0] FMuxOUT,
output reg [31:0] wALUOutOUT,
output reg [31:0] doDataMemOUT);

always @(posedge clk)
begin
    wwreg = mwreg;
    wm2reg = mm2reg;
    FMuxOUT = mmux1OutOUT;
    wALUOutOUT = ALUOutOUT;
    doDataMemOUT = doDataMem;
end
endmodule

module m2regMUX(
input wm2reg,
input [31:0] wALUOutOUT,
input [31:0] doDataMemOUT,
output [31:0] m2regMUXOut
);

assign m2regMUXOut = wm2reg ? doDataMemOUT : wALUOutOUT;
endmodule