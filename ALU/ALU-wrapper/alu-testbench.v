`timescale 10ns/1ns
module testbench;
    reg signed [63:0] a;
    reg signed [63:0] b;
    reg signed [1:0] control;
    wire signed [63:0] ans;
    wire overflow;
    alu DUT(a,b,control, ans, overflow);
    initial begin
        $dumpfile("alu-dump.vcd");
        $dumpvars(0, testbench);
        a = 64'b0;
        b = 64'b0;
        control = 2'b00;
        #10;
        #2 control = 2'b00; a = $random; b = $random;
        #2 control = 2'b00; a = $random; b = $random;
        #2 control = 2'b00; a = $random; b = $random;
        #2 control = 2'b00; a = 64'h7FFFFFFFFFFFFFFE; b = 64'h2;
        #2 control = 2'b01; a = $random; b = $random;
        #2 control = 2'b01; a = $random; b = $random;
        #2 control = 2'b01; a = $random; b = $random;
        #2 control = 2'b01; a = 64'h7FFFFFFFFFFFFFFE; b = -64'h7;
        #2 control = 2'b10; a = $random; b = $random;
        #2 control = 2'b10; a = $random; b = $random;
        #2 control = 2'b10; a = $random; b = $random;
        #2 control = 2'b11; a = $random; b = $random;
        #2 control = 2'b11; a = $random; b = $random;
        #2 control = 2'b11; a = $random; b = $random;
        #2;
    end
    initial
        $monitor ($time, "  A = %d | B = %d | Control = %b || Ans = %d | Overflow = %d", a,b,control,ans, overflow);
endmodule
