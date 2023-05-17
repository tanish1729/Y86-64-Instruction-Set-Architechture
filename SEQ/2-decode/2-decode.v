module SEQdecode(clk, icode, rA, rB, valA, valB);
    input clk;
    input [3:0] icode, rA, rB;
    output reg [63:0] valA, valB;

    initial begin
        valA = 0;
        valB = 0;
    end

    reg [63:0] registers [0:14];
    integer i;
    initial begin
        for(i = 0; i <= 14; i=i+1) begin
            registers[i] = i;
        end
    end

    always @(*) begin
        if (icode == 4'b0000) begin // halt
        end
        else if (icode == 4'b0001) begin // nop
        end
        else if (icode == 4'b0010) begin // cmovXX
            valA = registers[rA];
        end
        else if (icode == 4'b0011) begin // irmovq
            valB = registers[rB];
        end
        else if (icode == 4'b0100) begin // rmmovq
            valA = registers[rA];
            valB = registers[rB];
        end
        else if (icode == 4'b0101) begin // mrmovq
            valB = registers[rB];
        end
        else if (icode == 4'b0110) begin // OPq
            valA = registers[rA];
            valB = registers[rB];
        end
        else if (icode == 4'b0111) begin // jXX
        end
        else if (icode == 4'b1000) begin // call
            valB = registers[4];
        end
        else if (icode == 4'b1001) begin // ret
            valA = registers[4];
            valB = registers[4];
        end
        else if (icode == 4'b1010) begin // pushq
            valA = registers[rA];
            valB = registers[4];
        end
        else if (icode == 4'b1011) begin // popq
            valA = registers[4];
            valB = registers[4];
        end
    end
endmodule