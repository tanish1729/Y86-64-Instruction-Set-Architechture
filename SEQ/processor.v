module processor;
    reg clk;
    reg [3:0] status_codes; // AOK, HLT, ADR, INS
    reg [63:0] PC;
    wire cnd, hlt, mem_error, instr_valid, ZF, SF, OF;
    wire [3:0] icode, ifun, rA, rB, dstE, dstM;
    wire [63:0] valA, valB, valC, valE, valM, valP, newPC; 
    reg [7:0] instr_memory[0:1023];
    reg [63:0] registers [0:14];
    reg [63:0] data_memory[2047:0];
    initial begin
        instr_memory[0]  = 8'b00010000; // nop

        instr_memory[1]  = 8'b00110000; // irmovq 
        instr_memory[2]  = 8'b11110010; // F, rB = 2;
        instr_memory[3]  = 8'b00000110; // 6
        instr_memory[4]  = 8'b00000000; 
        instr_memory[5]  = 8'b00000000; 
        instr_memory[6]  = 8'b00000000; 
        instr_memory[7]  = 8'b00000000; 
        instr_memory[8] = 8'b00000000; 
        instr_memory[9] = 8'b00000000; 
        instr_memory[10] = 8'b00000000;

        instr_memory[11]  = 8'b00110000; // irmovq 
        instr_memory[12]  = 8'b11110011; // F, rB = 3;
        instr_memory[13]  = 8'b00000101; // 5
        instr_memory[14]  = 8'b00000000; 
        instr_memory[15]  = 8'b00000000; 
        instr_memory[16]  = 8'b00000000; 
        instr_memory[17]  = 8'b00000000; 
        instr_memory[18] = 8'b00000000; 
        instr_memory[19] = 8'b00000000; 
        instr_memory[20] = 8'b00000000;

        instr_memory[21]  = 8'b00110000; // irmovq 
        instr_memory[22]  = 8'b11110111; // F, rB = 7;
        instr_memory[23]  = 8'b00001011; // 11
        instr_memory[24]  = 8'b00000000; 
        instr_memory[25]  = 8'b00000000; 
        instr_memory[26]  = 8'b00000000; 
        instr_memory[27]  = 8'b00000000; 
        instr_memory[28] = 8'b00000000; 
        instr_memory[29] = 8'b00000000; 
        instr_memory[30] = 8'b00000000;

        instr_memory[31]  = 8'b00110000; // irmovq 
        instr_memory[32]  = 8'b11110001; // F, rB = 1;
        instr_memory[33]  = 8'b00001010; // 10
        instr_memory[34]  = 8'b00000000; 
        instr_memory[35]  = 8'b00000000; 
        instr_memory[36]  = 8'b00000000; 
        instr_memory[37]  = 8'b00000000; 
        instr_memory[38] = 8'b00000000; 
        instr_memory[39] = 8'b00000000; 
        instr_memory[40] = 8'b00000000;      

        instr_memory[41] = 8'b00100000; // rrmovq
        instr_memory[42] = 8'b00010101; // rA = 1, rB = 5

        instr_memory[43]  = 8'b01000000; // rmmovq 
        instr_memory[44]  = 8'b01110010; // rA = 7, rB = 2;
        instr_memory[45]  = 8'b00000110; // 6
        instr_memory[46]  = 8'b00000000; 
        instr_memory[47]  = 8'b00000000; 
        instr_memory[48]  = 8'b00000000; 
        instr_memory[49]  = 8'b00000000; 
        instr_memory[50] = 8'b00000000; 
        instr_memory[51] = 8'b00000000; 
        instr_memory[52] = 8'b00000000; 

        instr_memory[53]  = 8'b01010000; // mrmovq 
        instr_memory[54]  = 8'b10010101; // rA = 9, rB = 5;
        instr_memory[55]  = 8'b00000010; // 2
        instr_memory[56]  = 8'b00000000; 
        instr_memory[57]  = 8'b00000000; 
        instr_memory[58]  = 8'b00000000; 
        instr_memory[59]  = 8'b00000000; 
        instr_memory[60] = 8'b00000000; 
        instr_memory[61] = 8'b00000000; 
        instr_memory[62] = 8'b00000000;   

        instr_memory[63] = 8'b01100000; // OPq ADD
        instr_memory[64] = 8'b00100011; // rA = 2, rB = 3;

        instr_memory[65] = 8'b01110000; // jXX
        instr_memory[66] = 8'b01001010; // 74
        instr_memory[67] = 8'b00000000; 
        instr_memory[68] = 8'b00000000; 
        instr_memory[69] = 8'b00000000; 
        instr_memory[70] = 8'b00000000; 
        instr_memory[71] = 8'b00000000; 
        instr_memory[72] = 8'b00000000; 
        instr_memory[73] = 8'b00000000; 

        instr_memory[74] = 8'b01100000; // OPq ADD
        instr_memory[75] = 8'b00010010; // rA = 1, rB = 2;

        instr_memory[76] = 8'b10100000; // pushq
        instr_memory[77] = 8'b10101111; // rA = 10, F;

        instr_memory[78] = 8'b10110000; // popq
        instr_memory[79] = 8'b01101111; // rA = 6, F;

        instr_memory[80] = 8'b10000000; // call
        instr_memory[81] = 8'b01011011; // 91
        instr_memory[82] = 8'b00000000;
        instr_memory[83] = 8'b00000000;
        instr_memory[84] = 8'b00000000;
        instr_memory[85] = 8'b00000000;
        instr_memory[86] = 8'b00000000;
        instr_memory[87] = 8'b00000000;
        instr_memory[88] = 8'b00000000;
        instr_memory[89] = 8'b00000000;

        instr_memory[90] = 8'b10010000; // ret

        instr_memory[91] = 8'b00010000; // nop
        instr_memory[92] = 8'b00000000; // halt

        // for invalid instruction, comment out the halt instruction and uncomment below line
        // instr_memory[92] = 8'b11110000;
        
    end
    integer i;

        initial begin
        for(i = 0; i <= 14; i=i+1) begin
            registers[i] = 10*i;
        end

        for(i = 0; i <= 50; i=i+1) begin
            data_memory[i] = 10*i;
        end
    end

    always #5 clk = ~clk;
    always @(posedge clk) begin
        PC = newPC;
    end

    SEQfetch proc_fetch(clk, PC, icode, ifun, rA, rB, valC, valP, hlt, mem_error, instr_valid);
    SEQdecode proc_decode(clk, icode, rA, rB, valA, valB);
    SEQexecute proc_execute(clk, icode, ifun, valA, valB, valC, valE, cnd, ZF, SF, OF);
    SEQmemory proc_memory(clk, icode, valA, valE, valP, valM);
    SEQwriteback proc_writeback(clk, icode, rA, rB, valA, valB, valE, valM, cnd, dstE, dstM);
    SEQPCupdate proc_PCupdate(clk, PC, icode, cnd, valC, valM, valP, newPC);

    initial begin
        $dumpfile("processor.vcd");
        $dumpvars(0, processor);
        clk = 0;
        PC = 0;
        status_codes[0] = 1;
        status_codes[1] = 0;
        status_codes[2] = 0;
        status_codes[3] = 0;
    end
    
    always @(negedge clk) begin
        if (instr_valid) begin
                status_codes[0] = 1;
                status_codes[1] = 0;
                status_codes[2] = 0;
                status_codes[3] = 0;
        end
        else if (mem_error) begin
            status_codes[0] = 0;
            status_codes[1] = 0;
            status_codes[2] = 1;
            status_codes[3] = 0;
            $display("Error accessing memory location!\n");
        end
        else if (hlt) begin
            status_codes[0] = 0;
            status_codes[1] = 1;
            status_codes[2] = 0;
            status_codes[3] = 0;
            $display("Halt Instruction encountered!\n");
        end
        else if (!instr_valid) begin
            status_codes[0] = 0;
            status_codes[1] = 0;
            status_codes[2] = 0;
            status_codes[3] = 1;
            $display("Invalid Instruction!\n");
        end

        if (status_codes[1] == 1 || status_codes[3] == 1) begin
            $finish;
        end
    end

    always @(*) begin
        $monitor("clk = %0d \tPC = %0d\nicode = %0b (%0d)\tifun = %0b (%0d)\nrA = %0d \trB = %0d \nvalA = %0d \tvalB = %0d \nvalC = %0d \tvalE = %0d \tvalM = %0d \tvalP = %0d\nAOK = %0d \tHLT = %0d \tADR = %0d \tINS = %0d \n", clk, PC, icode, icode, ifun, ifun, rA, rB, valA, valB, valC, valE, valM, valP,status_codes[0], status_codes[1],status_codes[2], status_codes[3]);
    end

endmodule

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

    reg [7:0] opcode, regids;
    
    always @(*) begin
        if (PC > 1023) begin
            mem_error = 1;
        end
        opcode = processor.instr_memory[PC];
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
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b0011) begin // irmovq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {processor.instr_memory[PC+9], processor.instr_memory[PC+8], processor.instr_memory[PC+7], processor.instr_memory[PC+6], processor.instr_memory[PC+5], processor.instr_memory[PC+4], processor.instr_memory[PC+3], processor.instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0100) begin // rmmovq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {processor.instr_memory[PC+9], processor.instr_memory[PC+8], processor.instr_memory[PC+7], processor.instr_memory[PC+6], processor.instr_memory[PC+5], processor.instr_memory[PC+4], processor.instr_memory[PC+3], processor.instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0101) begin // mrmovq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valC = {processor.instr_memory[PC+9], processor.instr_memory[PC+8], processor.instr_memory[PC+7], processor.instr_memory[PC+6], processor.instr_memory[PC+5], processor.instr_memory[PC+4], processor.instr_memory[PC+3], processor.instr_memory[PC+2]};
            valP = PC+10;
        end
        else if (icode == 4'b0110) begin // OPq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b0111) begin // jXX
            valC = {processor.instr_memory[PC+8], processor.instr_memory[PC+7], processor.instr_memory[PC+6], processor.instr_memory[PC+5], processor.instr_memory[PC+4], processor.instr_memory[PC+3], processor.instr_memory[PC+2], processor.instr_memory[PC+1]};
            valP = PC+9;
        end
        else if (icode == 4'b1000) begin // call
            valC = {processor.instr_memory[PC+8], processor.instr_memory[PC+7], processor.instr_memory[PC+6], processor.instr_memory[PC+5], processor.instr_memory[PC+4], processor.instr_memory[PC+3], processor.instr_memory[PC+2], processor.instr_memory[PC+1]};
            valP = PC+9;
        end
        else if (icode == 4'b1001) begin // ret
            valP = PC+1;
        end
        else if (icode == 4'b1010) begin // pushq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else if (icode == 4'b1011) begin // popq
            regids = processor.instr_memory[PC+1];
            rA = regids[7:4];
            rB = regids[3:0];
            valP = PC+2;
        end
        else begin 
            instr_valid = 0;
        end
    end
endmodule

module SEQdecode(clk, icode, rA, rB, valA, valB);
    input clk;
    input [3:0] icode, rA, rB;
    output reg [63:0] valA, valB;

    initial begin
        valA = 0;
        valB = 0;
    end

    always @(*) begin
        if (icode == 4'b0000) begin // halt
        end
        else if (icode == 4'b0001) begin // nop
        end
        else if (icode == 4'b0010) begin // cmovXX
            valA = processor.registers[rA];
        end
        else if (icode == 4'b0011) begin // irmovq
            valB = 0;
        end
        else if (icode == 4'b0100) begin // rmmovq
            valA = processor.registers[rA];
            valB = processor.registers[rB];
        end
        else if (icode == 4'b0101) begin // mrmovq
            valB = processor.registers[rB];
        end
        else if (icode == 4'b0110) begin // OPq
            valA = processor.registers[rA];
            valB = processor.registers[rB];
        end
        else if (icode == 4'b0111) begin // jXX
        end
        else if (icode == 4'b1000) begin // call
            valB = processor.registers[4];
        end
        else if (icode == 4'b1001) begin // ret
            valA = processor.registers[4];
            valB = processor.registers[4];
        end
        else if (icode == 4'b1010) begin // pushq
            valA = processor.registers[rA];
            valB = processor.registers[4];
        end
        else if (icode == 4'b1011) begin // popq
            valA = processor.registers[4];
            valB = processor.registers[4];
        end
    end
endmodule

module SEQexecute(clk, icode, ifun, valA, valB, valC, valE, cnd, ZF, SF, OF);
    input clk;
    input [3:0] icode, ifun;
    input [63:0] valA, valB, valC;
    output reg cnd, ZF, OF, SF;
    output reg [63:0] valE;

    initial begin
      cnd = 0;
      valE = 0;
      ZF = 0;
      OF = 0;
      SF = 0;
    end

    reg [1:0] control;
    reg signed [63:0] aluInputA, aluInputB;
    wire tempZF, tempSF, tempOF;
    wire overflow;
    wire signed [63:0] out;
    initial begin
      control = 0;
      aluInputA = 0;
      aluInputB = 0;
    end

    alu execute_ALU(aluInputA, aluInputB, control, out, overflow, tempZF, tempOF, tempSF);

    always @(*) begin
      // calculating values using ALU
      if (icode == 4'b0000) begin // halt
      end
      else if (icode == 4'b0001) begin // nop
      end
      else if (icode == 4'b0010) begin // cmovXX
          aluInputA = valA;
          aluInputB = 0;
          control = 2'b00;
          valE = out;
          if (ifun == 4'b0000) begin
            cnd = 1;
          end
          else if (ifun == 4'b0001) begin
            cnd = (SF ^ OF) | ZF;
          end
          else if (ifun == 4'b0010) begin
            cnd = SF^OF;
          end
          else if (ifun == 4'b0011) begin
            cnd = ZF;
          end
          else if (ifun == 4'b0100) begin
            cnd = ~ZF;
          end
          else if (ifun == 4'b0101) begin
            cnd = ~(SF^OF);
          end
          else if (ifun == 4'b0110) begin
            cnd = ~(SF^OF)&(~ZF);
          end
      end
      else if (icode == 4'b0011) begin // irmovq
          aluInputA = valC;
          aluInputB = 0;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b0100) begin // rmmovq
          aluInputA = valC;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b0101) begin // mrmovq
          aluInputA = valC;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b0110) begin // OPq
          aluInputA = valA;
          aluInputB = valB;
          if (ifun == 4'b0000) begin // ADD
            control = 2'b00;
          end
          else if (ifun == 4'b0001) begin // SUB
            control = 2'b01;
          end
          else if (ifun == 4'b0010) begin // AND
            control = 2'b10;
          end
          else if (ifun == 4'b0011) begin // XOR
            control = 2'b11;
          end
          valE = out;
      end
      else if (icode == 4'b0111) begin // jXX
          if (ifun == 4'b0000) begin
            cnd = 1;
          end
          else if (ifun == 4'b0001) begin
            cnd = (SF ^ OF) | ZF;
          end
          else if (ifun == 4'b0010) begin
            cnd = SF^OF;
          end
          else if (ifun == 4'b0011) begin
            cnd = ZF;
          end
          else if (ifun == 4'b0100) begin
            cnd = ~ZF;
          end
          else if (ifun == 4'b0101) begin
            cnd = ~(SF^OF);
          end
          else if (ifun == 4'b0110) begin
            cnd = ~(SF^OF)&(~ZF);
          end
      end
      else if (icode == 4'b1000) begin // call
          aluInputA = -4'b1000;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b1001) begin // ret
          aluInputA = 4'b1000;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b1010) begin // pushq
          aluInputA = -4'b1000;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      else if (icode == 4'b1011) begin // popq
          aluInputA = 4'b1000;
          aluInputB = valB;
          control = 2'b00;
          valE = out;
      end
      // setting flags
     assign ZF = tempZF;
      assign OF = tempOF;
      assign SF = tempSF;
    end

endmodule

module and64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        and G1(ans[i], a[i], b[i]);
    end
endmodule

module xor64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        xor G1(ans[i], a[i], b[i]);
    end
endmodule

module add1(input a, input b, input c_in, output sum, output c_out);
    xor G1(sum,a,b,c_in);
    and G2(t1,a,b);
    and G3(t2,a,c_in);
    and G4(t3,b,c_in);
    or G5(c_out, t1,t2,t3);
endmodule

module add64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans, output overflow);
    wire [64:0] carry;
    assign carry[0] = 1'b0;
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        add1 G1(a[i], b[i], carry[i], ans[i], carry[i+1]);
    end
    xor G2(overflow, carry[64], carry[63]);
endmodule

module not64(input signed [63:0]a, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        not G1 (ans[i], a[i]);
    end
endmodule

module sub64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans, output overflow);
    wire [63:0] complement1;
    not64 G1(b, complement1);
    wire [63:0] complement2;
    wire [63:0] temp1;
    assign temp1 = 64'b1;
    wire temp2;
    add64 G2(complement1, temp1, complement2, temp2);
    add64 G3(a, complement2, ans, overflow);
endmodule

module alu(input signed [63:0]a, input signed [63:0]b, input [1:0] control, output signed [63:0] ans, output overflow, output tempZF, output tempOF, output tempSF);
    wire signed [63:0] tempadd; wire overflowadd;
    wire signed [63:0] tempsub; wire overflowsub;
    wire signed [63:0] tempand;
    wire signed [63:0] tempxor;
    reg signed [63:0] tempans;
    reg tempoverflow;
    wire tempZF, tempSF, tempOF;
    add64 G1 (a,b,tempadd, overflowadd);
    sub64 G2 (a,b,tempsub, overflowsub);
    and64 G3 (a,b,tempand);
    xor64 G4 (a,b,tempxor);
    always @(*)
    begin
        case(control)
            2'b00: // ADD 
            begin
                tempans = tempadd;
                tempoverflow = overflowadd;
            end
            2'b01: // SUB
            begin
                tempans = tempsub;
                tempoverflow = overflowsub;
            end
            2'b10: // AND
            begin
                tempans = tempand;
                tempoverflow= 1'b0;
            end
            2'b11: // XOR
            begin
                tempans = tempxor;
                tempoverflow= 1'b0;
            end
        endcase
    end
    assign ans = tempans;
    assign overflow = tempoverflow;
    assign tempZF = (ans == 1'b0);
    assign tempOF = (overflow != 0);
    assign tempSF = (ans[63] == 1);
    
endmodule

module SEQmemory(clk, icode, valA, valE, valP, valM);
    input clk;
    input [3:0] icode;
    input [63:0] valA, valE, valP;
    output reg [63:0] valM;

    always @(*) begin
         if (icode == 4'b0101) begin // mrmovq
            valM = processor.data_memory[valE];
        end
        else if (icode == 4'b1001) begin // ret
            valM = processor.data_memory[valA];
        end
        else if (icode == 4'b1011) begin // popq
            valM = processor.data_memory[valA];
        end
    end
    always @(posedge clk) begin
       if (icode == 4'b0100) begin // rmmovq
            processor.data_memory[valE] = valA;
    end
        else if (icode == 4'b1000) begin // call
            processor.data_memory[valE] = valP;
        end
        else if (icode == 4'b1010) begin // pushq
            processor.data_memory[valE] = valA;
        end
    end
endmodule

module SEQwriteback(clk, icode, rA, rB, valA, valB, valE, valM, cnd, dstE, dstM);
    input clk, cnd;
    input [3:0] icode, rA, rB;
    input [63:0] valA, valB, valE, valM;
    output reg [3:0] dstE, dstM;
    
    initial begin
    dstE = 4'hF;
    dstM = 4'hF;
    end

    always @(posedge clk) begin
        if (icode == 4'b0000) begin // halt
        end
        else if (icode == 4'b0001) begin // nop
        end
        else if ((icode == 4'b0010) && (cnd == 1)) begin // cmovXX
            dstE = rB;
            processor.registers[rB] = valE;
        end
        else if (icode == 4'b0011) begin // irmovq
            dstE = rB;
            processor.registers[rB] = valE;
        end
        else if (icode == 4'b0100) begin // rmmovq
        end
        else if (icode == 4'b0101) begin // mrmovq
            dstM = rA;
            processor.registers[rA] = valM;
        end
        else if (icode == 4'b0110) begin // OPq
            dstE = rB;
            processor.registers[rB] = valE;
        end
        else if (icode == 4'b0111) begin // jXX
        end
        else if (icode == 4'b1000) begin // call
            dstE = 4;
            processor.registers[4] = valE;
        end
        else if (icode == 4'b1001) begin // ret
            dstE = 4;
            processor.registers[4] = valE;
        end
        else if (icode == 4'b1010) begin // pushq
            dstE = 4;
            processor.registers[4] = valE;
        end
        else if (icode == 4'b1011) begin // popq
            dstE = 4;
            dstM = rA;
            processor.registers[4] = valE;
            processor.registers[rA] = valM;
        end
    end
endmodule

module SEQPCupdate(clk, PC, icode, cnd, valC, valM, valP, newPC);
    input clk, cnd;
    input [3:0] icode;
    input [63:0] PC, valC, valM, valP;
    output reg [63:0] newPC;
    always @(*) begin
        if (icode == 4'b0111) begin // jXX
            newPC = (cnd == 1) ? valC : valP;
        end
        else if (icode == 4'b1000) begin // call
            newPC = valC;
        end
        else if (icode == 4'b1001) begin // ret
            newPC = valM;
        end
        else begin // all other instructions
            newPC = valP;
        end
    end
endmodule