module SEQwriteback(clk, icode, rA, rB, valA, valB, valE, valM, cnd, dstE, dstM);
    input clk, cnd;
    input [3:0] icode, rA, rB;
    input [63:0] valA, valB, valE, valM;
    output reg [3:0] dstE, dstM;
    
    initial begin
    dstE = 4'hF;
    dstM = 4'hF;
    end

    reg [63:0] registers [0:14];
    integer i;
    initial begin
        for(i = 0; i <= 14; i=i+1) begin
            registers[i] = i;
        end
    end

    always @(posedge clk) begin
        if (icode == 4'b0000) begin // halt
        end
        else if (icode == 4'b0001) begin // nop
        end
        else if ((icode == 4'b0010) && (cnd == 1)) begin // cmovXX
            dstE = rB;
            registers[rB] = valE;
        end
        else if (icode == 4'b0011) begin // irmovq
            dstE = rB;
            registers[rB] = valE;
        end
        else if (icode == 4'b0100) begin // rmmovq
        end
        else if (icode == 4'b0101) begin // mrmovq
            dstM = rA;
            registers[rA] = valM;
        end
        else if (icode == 4'b0110) begin // OPq
            dstE = rB;
            registers[rB] = valE;
        end
        else if (icode == 4'b0111) begin // jXX
        end
        else if (icode == 4'b1000) begin // call
            dstE = 4;
            registers[4] = valE;
        end
        else if (icode == 4'b1001) begin // ret
            dstE = 4;
            registers[4] = valE;
        end
        else if (icode == 4'b1010) begin // pushq
            dstE = 4;
            registers[4] = valE;
        end
        else if (icode == 4'b1011) begin // popq
            dstE = 4;
            dstM = rA;
            registers[4] = valE;
            registers[rA] = valM;
        end
    end
endmodule