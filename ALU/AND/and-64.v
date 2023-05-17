`timescale 10ns/1ns
module and64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        and G1(ans[i], a[i], b[i]);
    end
endmodule
