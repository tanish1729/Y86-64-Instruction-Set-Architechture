module fetch_tb;
    reg clk;
    reg [63:0] PC;
    wire hlt, mem_error, instr_valid;
    wire [3:0] icode, ifun, rA, rB;
    wire [63:0] valC, valP;

    SEQfetch DUT(clk, PC, icode, ifun, rA, rB, valC, valP, hlt, mem_error, instr_valid);
    initial begin
        $dumpfile("1-fetch.vcd");
        $dumpvars(0, fetch_tb);
        clk = 0;
        PC = 0;

        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        #10 clk=~clk;
        #10 clk=~clk;PC=valP;
        $finish;
    end

    initial
    		$monitor("clk = %0d \tPC = %0b (%0d)\ticode=%0b (%0d)\tifun=%0b (%0d)\nrA=%0b (%0d)\trB=%0b (%0d)\tvalC=%0d \tvalP=%0d\nhlt = %0d \tinstr_valid = %0d \tmem_error = %0d\n",clk,PC,PC,icode,icode,ifun,ifun,rA,rA,rB,rB,valC,valP,hlt,instr_valid,mem_error);
endmodule