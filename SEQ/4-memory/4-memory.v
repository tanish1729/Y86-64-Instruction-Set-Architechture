module SEQmemory(clk, icode, valA, valE, valP, valM);
    input clk;
    input [3:0] icode;
    input [63:0] valA, valE, valP;
    output reg [63:0] valM;

    reg [63:0] data_memory[2047:0];
    integer i;
    initial begin
        for(i = 0; i <= 50; i=i+1) begin
            data_memory[i] = i;
        end
    end

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
