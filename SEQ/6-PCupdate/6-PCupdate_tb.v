module PCupdate_tb;
    reg clk, cnd;
    reg [3:0] icode;
    reg [63:0] PC, valC, valM, valP;
    output [63:0] newPC;

    SEQPCupdate DUT(clk, PC, icode, cnd, valC, valM, valP, newPC);
    initial begin
        $dumpfile("6-PCupdate.vcd");
        $dumpvars(0, PCupdate_tb);
        clk = 0;
        PC = 0;

        icode = 4'b0011; valC = 10; valM = 20; valP = 30; cnd = 0;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0101; valC = 10; valM = 20; valP = 30; cnd = 0;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1000; valC = 10; valM = 20; valP = 30; cnd = 0;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1001; valC = 10; valM = 20; valP = 30; cnd = 0;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; valC = 10; valM = 20; valP = 30; cnd = 0;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; valC = 10; valM = 20; valP = 30; cnd = 1;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        $finish;
    end

    initial
        	$monitor("clk = %0d \tNew PC = %0b (%0d)\n",clk,newPC, newPC);
endmodule