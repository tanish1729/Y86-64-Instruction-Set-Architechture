`timescale 10ns/1ns
module not64(input signed [63:0]a, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        not G1 (ans[i], a[i]);
    end
endmodule