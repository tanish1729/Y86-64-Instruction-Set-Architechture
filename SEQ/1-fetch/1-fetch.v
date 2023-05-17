module SEQfetch(clk, PC, icode, ifun, rA, rB, valC, valP, hlt, mem_error, instr_valid);
    input clk;
    input [63:0] PC;
    output reg hlt, mem_error, instr_valid;
    output reg [3:0] icode, ifun, rA, rB;
    output reg [63:0] valC, valP;

    initial begin
        rA = 4'hF;
        rB = 4'hF;
        valC = 0;
        valP = 0;
        hlt = 0;
        mem_error = 0;
        instr_valid = 1;
    end

    reg [7:0] instr_memory [0:1023];
    initial begin
        instr_memory[0]  = 8'b00010000; // nop

        instr_memory[1]  = 8'b01100000; // OPq ADD
        instr_memory[2]  = 8'b00000001; // rA = 0, rB = 1

        instr_memory[3]  = 8'b00110000; // irmovq 
        instr_memory[4]  = 8'b11110010; // F, rB = 2;
        instr_memory[5]  = 8'b11111111; // 1st byte
        instr_memory[6]  = 8'b00000000; // 2nd byte
        instr_memory[7]  = 8'b00000000; // 3rd byte
        instr_memory[8]  = 8'b00000000; // 4th byte
        instr_memory[9]  = 8'b00000000; // 5th byte
        instr_memory[10] = 8'b00000000; // 6th byte
        instr_memory[11] = 8'b00000000; // 7th byte
        instr_memory[12] = 8'b00000000; // 8th byte

        instr_memory[13] = 8'b00110000; // irmovq instruction
        instr_memory[14] = 8'b11110011; // F, rB = 3;
        instr_memory[15] = 8'b00000101; // 1st byte
        instr_memory[16] = 8'b00000000; // 2nd byte
        instr_memory[17] = 8'b00000000; // 3rd byte
        instr_memory[18] = 8'b00000000; // 4th byte
        instr_memory[19] = 8'b00000000; // 5th byte
        instr_memory[20] = 8'b00000000; // 6th byte
        instr_memory[21] = 8'b00000000; // 7th byte
        instr_memory[22] = 8'b00000000; // 8th byte

        instr_memory[23] = 8'b00100000; // rrmovq
        instr_memory[24] = 8'b01000101; // rA = 4, rB = 5

        instr_memory[25] = 8'b01100010; // OPq AND 
        instr_memory[26] = 8'b00110100; // rA = 3, rB = 4

        instr_memory[27] = 8'b00100101; // cmovge
        instr_memory[28] = 8'b01010110; // rA = 5; rB = 6;

        instr_mem[29]=8'b10000000; //8 0
        instr_mem[30]=8'b00000000; //Dest
        instr_mem[31]=8'b00000000; //Dest
        instr_mem[32]=8'b00000000; //Dest
        instr_mem[33]=8'b00000000; //Dest
        instr_mem[34]=8'b00000000; //Dest
        instr_mem[35]=8'b00000000; //Dest
        instr_mem[36]=8'b00000000; //Dest
        instr_mem[37]=8'b00000001; //Dest

        instr_mem[38]=8'b10010000; // 9 0

        instr_memory[39] = 8'b00000000; // halt
    end

    reg [7:0] opcode, regids;
    
    always @(*) begin
        if (PC > 1023) begin
            mem_error = 1;
        end
        opcode = instr_memory[PC];
        icode = opcode[7:4];
        ifun = opcode[3:0];
        if (icode == 4'b0000) begin // halt
            hlt = 1;
            valP = PC+1;
        end
        else if (icode == 4'b0001) begin // nop
            valP = PC+1;
        end
        else if (icode == 4'b0010) begin // cmovXX
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b0011) begin // irmovq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {instr_memory[PC+9], instr_memory[PC+8], instr_memory[PC+7], instr_memory[PC+6], instr_memory[PC+5], instr_memory[PC+4], instr_memory[PC+3], instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0100) begin // rmmovq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {instr_memory[PC+9], instr_memory[PC+8], instr_memory[PC+7], instr_memory[PC+6], instr_memory[PC+5], instr_memory[PC+4], instr_memory[PC+3], instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0101) begin // mrmovq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {instr_memory[PC+9], instr_memory[PC+8], instr_memory[PC+7], instr_memory[PC+6], instr_memory[PC+5], instr_memory[PC+4], instr_memory[PC+3], instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0110) begin // OPq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b0111) begin // jXX
            valC = {instr_memory[PC+8], instr_memory[PC+7], instr_memory[PC+6], instr_memory[PC+5], instr_memory[PC+4], instr_memory[PC+3], instr_memory[PC+2], instr_memory[PC+1]};
            valP = PC+9;
        end
        else if (icode == 4'b1000) begin // call
            valC = {instr_memory[PC+8], instr_memory[PC+7], instr_memory[PC+6], instr_memory[PC+5], instr_memory[PC+4], instr_memory[PC+3], instr_memory[PC+2], instr_memory[PC+1]};
            valP = PC+9;
        end
        else if (icode == 4'b1001) begin // ret
            valP = PC+1;
        end
        else if (icode == 4'b1010) begin // pushq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b1011) begin // popq
            regids = instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else begin 
            instr_valid = 0;
        end
    end
endmodule