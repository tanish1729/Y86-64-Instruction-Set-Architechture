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