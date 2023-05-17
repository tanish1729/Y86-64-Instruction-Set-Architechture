module PIPEfetch(clk, PC, icode, ifun, rA, rB, valC, valP, hlt, mem_error, instr_valid);
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

module PIPEPCupdate(clk, PC, icode, cnd, valC, valM, valP, newPC);
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

module PIPEdecode_writeback(clk, d_icode, d_rA, d_rB, d_valA, d_valB, d_Cnd, w_icode, w_rA, w_rB, w_valE, w_valM, w_Cnd);
    input clk, d_Cnd, w_Cnd;
    input [3:0] d_icode, d_rA, d_rB, w_icode, w_rA, w_rB;
    input [63:0] w_valE, w_valM;
    output reg [63:0] d_valA, d_valB;

    reg [63:0] registers[0:14];
    integer i;
    initial begin
        d_valA = 0;
        d_valB = 0;
        for(i = 0; i <= 14; i=i+1) begin
            registers[i] = 10*i;
        end
    end
    
        always @(*) begin
        if (d_icode == 4'b0000) begin // halt
        end
        else if (d_icode == 4'b0001) begin // nop
        end
        else if (d_icode == 4'b0010) begin // cmovXX
            d_valA = registers[d_rA];
        end
        else if (d_icode == 4'b0011) begin // irmovq
        end
        else if (d_icode == 4'b0100) begin // rmmovq
            d_valA = registers[d_rA];
            d_valB = registers[d_rB];
        end
        else if (d_icode == 4'b0101) begin // mrmovq
            d_valB = registers[d_rB];
        end
        else if (d_icode == 4'b0110) begin // OPq
            d_valA = registers[d_rA];
            d_valB = registers[d_rB];
        end
        else if (d_icode == 4'b0111) begin // jXX
        end
        else if (d_icode == 4'b1000) begin // call
            d_valB = registers[4];
        end
        else if (d_icode == 4'b1001) begin // ret
            d_valA = registers[4];
            d_valB = registers[4];
        end
        else if (d_icode == 4'b1010) begin // pushq
            d_valA = registers[rA];
            d_valB = registers[4];
        end
        else if (d_icode == 4'b1011) begin // popq
            d_valA = registers[4];
            d_valB = registers[4];
        end
    end

    always @(*) begin
        if (w_icode == 4'b0000) begin // halt
        end
        else if (w_icode == 4'b0001) begin // nop
        end
        else if ((w_icode == 4'b0010) && (w_cnd == 1)) begin // cmovXX
            registers[w_rB] = w_valE;
        end
        else if (w_icode == 4'b0011) begin // irmovq
            registers[w_rB] = w_valE;
        end
        else if (w_icode == 4'b0100) begin // rmmovq
        end
        else if (w_icode == 4'b0101) begin // mrmovq
            registers[w_rA] = w_valM;
        end
        else if (w_icode == 4'b0110) begin // OPq
            registers[w_rB] = w_valE;
        end
        else if (w_icode == 4'b0111) begin // jXX
        end
        else if (w_icode == 4'b1000) begin // call
            registers[4] = w_valE;
        end
        else if (w_icode == 4'b1001) begin // ret
            registers[4] = w_valE;
        end
        else if (w_icode == 4'b1010) begin // pushq
            registers[4] = w_valE;
        end
        else if (w_icode == 4'b1011) begin // popq
            registers[4] = w_valE;
            registers[w_rA] = w_valM;
        end
    end
endmodule

module PIPEexecute(clk, icode, ifun, valA, valB, valC, valE, cnd, ZF, SF, OF);
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

module PIPEmemory(clk, icode, valA, valE, valP, valM);
    input clk;
    input [3:0] icode;
    input [63:0] valA, valE, valP;
    output reg [63:0] valM;

    always @(*) begin
         if (icode == 4'b0101) begin // mrmovq
            valM = data_memory[valE];
        end
        else if (icode == 4'b1001) begin // ret
            valM = data_memory[valA];
        end
        else if (icode == 4'b1011) begin // popq
            valM = data_memory[valA];
        end
    end
    always @(posedge clk) begin
       if (icode == 4'b0100) begin // rmmovq
            data_memory[valE] = valA;
    end
        else if (icode == 4'b1000) begin // call
            data_memory[valE] = valP;
        end
        else if (icode == 4'b1010) begin // pushq
            data_memory[valE] = valA;
        end
    end
endmodule