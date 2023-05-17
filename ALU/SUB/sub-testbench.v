`timescale 10ns/1ns
module testbench;
    reg signed [63:0] a;
    reg signed [63:0] b;
    wire signed [63:0] ans;
    wire overflow;
    sub64 DUT(a,b,ans, overflow);
  initial
    begin
        $dumpfile("sub-dump.vcd");
        $dumpvars(0, testbench);
        a = 64'b0; b = 64'b0;
        #10;
        #2 a = 64'b110101010101001010101010; b = 64'b10100101010111010111010;
        #2 a = 64'b010101000011010111011001; b = 64'b1111000101011100000110;
        #2 a = 64'b111101001010001001011010; b = 64'b111100111011001101111011;
        #2 a = 587619328768; b = 9923145637281;
        #2 a = -64'h7FFFFFFFFFFFFFFE; b = 64'h1;
        #2 a = -64'h7FFFFFFFFFFFFFFE; b = 64'h2;
        #2 a = 64'h7FFFFFFFFFFFFFFE; b = -64'h2;
        #2 a = $random; b = $random;
        #2 a = $random; b = $random;
        #2 a = $random; b = $random;
        #2 a = $random; b = $random;
        #2 a = $random; b = $random;
        #2 a = $random; b = $random;
        #2 $finish;
    end
    initial
        $monitor ("A = %0d, B = %0d -> Ans = %0d, Overflow = %0d\nA   = %b\nB   = %b\nAns = %b\nOverflow = %0b\n", a,b,ans,overflow,a,b,ans,overflow);
endmodule