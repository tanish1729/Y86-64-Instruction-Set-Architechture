module decode_tb;
    reg clk;
    reg [3:0] icode, rA, rB;
    output [63:0] valA, valB;

    SEQdecode DUT(clk, icode, rA, rB, valA, valB);
    initial begin
        $dumpfile("2-decode.vcd");
        $dumpvars(0, decode_tb);
        clk = 0;

        icode = 4'b0010; rA = 0; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0011; rB = 14; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0100; rA = 1; rB = 13; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0101; rB = 12; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0110; rA = 2; rB = 11; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1000; rB = 10; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1001; rA = 3; rB = 9; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1010; rA = 4; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1011; rA = 5; rB = 8; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        
    end

    initial
        	$monitor("clk = %0d \tvalA = %0d \tvalB = %0d \n",clk,valA, valB);
endmodule