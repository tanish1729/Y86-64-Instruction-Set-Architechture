`include "alu.v"
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