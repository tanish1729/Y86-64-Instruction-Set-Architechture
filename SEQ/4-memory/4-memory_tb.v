module memory_tb;
    reg clk;
    reg [3:0] icode;
    reg [63:0] valA, valE, valP;
    output [63:0] valM;

    SEQmemory DUT(clk, icode, valA, valE, valP, valM);
    initial begin
        $dumpfile("4-memory.vcd");
        $dumpvars(0, memory_tb);
        clk = 0;

        icode = 4'b0101; valE = 35; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1001; valA = 36; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1011; valA = 37; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0100;valE = 15; valA = 69; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1000;valE = 16; valP = 420;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1010;valE = 17; valA = 500; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0101; valE = 15; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1001; valA = 16; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1011; valA = 17; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        
    end

    initial
        	$monitor("clk = %0d \tvalM = %0d \n",clk,valM);
endmodule