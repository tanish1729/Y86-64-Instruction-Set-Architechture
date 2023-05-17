module and64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        and G1(ans[i], a[i], b[i]);
    end
endmodule

module xor64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        xor G1(ans[i], a[i], b[i]);
    end
endmodule

module add1(input a, input b, input c_in, output sum, output c_out);
    xor G1(sum,a,b,c_in);
    and G2(t1,a,b);
    and G3(t2,a,c_in);
    and G4(t3,b,c_in);
    or G5(c_out, t1,t2,t3);
endmodule

module add64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans, output overflow);
    wire [64:0] carry;
    assign carry[0] = 1'b0;
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        add1 G1(a[i], b[i], carry[i], ans[i], carry[i+1]);
    end
    xor G2(overflow, carry[64], carry[63]);
endmodule

module not64(input signed [63:0]a, output signed [63:0] ans);
    genvar i;
    for(i = 0; i < 64; i=i+1)
    begin
        not G1 (ans[i], a[i]);
    end
endmodule

module sub64(input signed [63:0]a, input signed [63:0]b, output signed [63:0] ans, output overflow);
    wire [63:0] complement1;
    not64 G1(b, complement1);
    wire [63:0] complement2;
    wire [63:0] temp1;
    assign temp1 = 64'b1;
    wire temp2;
    add64 G2(complement1, temp1, complement2, temp2);
    add64 G3(a, complement2, ans, overflow);
endmodule

module alu(input signed [63:0]a, input signed [63:0]b, input [1:0] control, output signed [63:0] ans, output overflow, output tempZF, output tempOF, output tempSF);
    wire signed [63:0] tempadd; wire overflowadd;
    wire signed [63:0] tempsub; wire overflowsub;
    wire signed [63:0] tempand;
    wire signed [63:0] tempxor;
    reg signed [63:0] tempans;
    reg tempoverflow;
    wire tempZF, tempSF, tempOF;
    add64 G1 (a,b,tempadd, overflowadd);
    sub64 G2 (a,b,tempsub, overflowsub);
    and64 G3 (a,b,tempand);
    xor64 G4 (a,b,tempxor);
    always @(*)
    begin
        case(control)
            2'b00: // ADD 
            begin
                tempans = tempadd;
                tempoverflow = overflowadd;
            end
            2'b01: // SUB
            begin
                tempans = tempsub;
                tempoverflow = overflowsub;
            end
            2'b10: // AND
            begin
                tempans = tempand;
                tempoverflow= 1'b0;
            end
            2'b11: // XOR
            begin
                tempans = tempxor;
                tempoverflow= 1'b0;
            end
        endcase
    end
    assign ans = tempans;
    assign overflow = tempoverflow;
    assign tempZF = (ans == 1'b0);
    assign tempOF = (overflow != 0);
    assign tempSF = (ans[63] == 1);
    
endmodule