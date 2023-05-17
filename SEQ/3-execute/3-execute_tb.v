module execute_tb;
    reg clk;
    reg [3:0] icode, ifun;
    reg [63:0] valA, valB, valC;
    output cnd, ZF, OF, SF;
    output signed[63:0] valE;

    SEQexecute DUT(clk, icode, ifun, valA, valB, valC, valE, cnd, ZF, OF, SF);
    initial begin
        $dumpfile("3-execute.vcd");
        $dumpvars(0, execute_tb);
        clk = 0;

        icode = 4'b0110; ifun = 4'b0000; valA = 2'b10; valB = 2'b01; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0110; ifun = 4'b0001; valA = 4'b0111; valB = 4'b0001; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0110; ifun = 4'b0001; valA = 4'b0001; valB = 4'b1111; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0110; ifun = 4'b0010; valA = 6'b101010; valB = 6'b110011; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1011; valB = 4'b1000; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1010; valB = 4'b1000;
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1001; valB = 4'b0010; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b1000; valB = 4'b0101; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; ifun = 4'b0000; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; ifun = 4'b0010; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; ifun = 4'b0101; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0111; ifun = 4'b0110; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0100; valC = 4'b1001; valB = 4'b1001; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0100; valC = 4'b1101; valB = 4'b1001; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0010; ifun = 4'b0000; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0010; ifun = 4'b0011; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0010; ifun = 4'b0101; 
        #10 clk = ~clk; 
        #10 clk = ~clk;
        icode = 4'b0010; ifun = 4'b0000; 
        #10 clk = ~clk; 
        #10 clk = ~clk;

    end

    initial
        	$monitor("clk = %0d \ticode = %0b (%0d) \tvalA = %0d \tvalB = %0d \tvalE = %0d \t cnd = %0d \tZF = %0d \tSF = %0d \tOF = %0d \n",clk,icode, icode, valA, valB, valE, cnd, ZF, SF, OF);
endmodule