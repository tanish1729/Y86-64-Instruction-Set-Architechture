`timescale 10ns/1ns
module alu(input signed [63:0]a, input signed [63:0]b, input [1:0] control, output signed [63:0] ans, output overflow);
    wire signed [63:0] tempadd; wire overflowadd;
    wire signed [63:0] tempsub; wire overflowsub;
    wire signed [63:0] tempand;
    wire signed [63:0] tempxor;
    reg signed [63:0] tempans;
    reg tempoverflow;
    add64 G1 (a,b,tempadd, overflowadd);
    sub64 G2 (a,b,tempsub, overflowsub);
    and64 G3 (a,b,tempand);
    xor64 G4 (a,b,tempxor);
    always @(*)
    begin
        case(control)
            2'b00: begin
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
endmodule