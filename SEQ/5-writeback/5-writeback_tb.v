module writeback_tb;
    reg clk, cnd;
    reg [3:0] icode, rA, rB;
    reg [63:0] valA, valB, valE, valM;
    output [3:0] dstE, dstM;

    SEQwriteback DUT(clk, icode, rA, rB, valA, valB, valE, valM, cnd, dstE, dstM);
    initial begin
        $dumpfile("5-writeback.vcd");
        $dumpvars(0, writeback_tb);
        clk = 0;
        cnd = 1;

        icode = 4'b0010; rA = 4'b0100; rB = 4'b0010; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0011; rA = 4'b0101; rB = 4'b0110; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0101; rA = 4'b0010; rB = 4'b0111; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1010; rA = 4'b0110; rB = 4'b0010; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1011; rA = 4'b0111; rB = 4'b0100; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1000; rA = 4'b0010; rB = 4'b0101; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
    end

    initial
        	$monitor("clk = %0d \tdstE = %0g \tdstM = %0g\n",clk,dstE,dstM);
endmodule