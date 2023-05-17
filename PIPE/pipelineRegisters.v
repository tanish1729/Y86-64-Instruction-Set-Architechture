module PIPEregF(clk, f_valP, f_PC);
    input clk;
    input [63:0] f_valP;
    output reg [63:0] f_PC;

    always @(posedge clk) begin
        f_PC <= f_valP;
    end
endmodule

module PIPEregD(clk, f_stat, f_icode, f_ifun, f_rA, f_rB, f_valC, f_valP, d_stat, d_icode, d_ifun, d_rA, d_rB, d_valC, d_valP);
    input clk;
    input [3:0] f_stat, f_icode, f_ifun, f_rA, f_rB;
    input [63:0] f_valC, f_valP;
    output reg [3:0] d_stat, d_icode, d_ifun, d_rA, d_rB;
    output reg [63:0] d_valC, d_valP;

    always @(posedge clk) begin
        d_stat <= f_stat;
        d_icode <= f_icode;
        d_ifun <= f_ifun;
        d_rA <= f_rA;
        d_rB <= f_rB;
        d_valC <= f_valC;
        d_valP <= f_valP;
    end
endmodule

module PIPEregE(clk, d_stat, d_icode, d_ifun, d_rA, d_rB, d_valA, d_valB, d_valC, d_valP,e_stat, e_icode, e_ifun, e_rA, e_rB, e_valA, e_valB, e_valC, e_valP);
    input clk;
    input [3:0] d_stat, d_icode, d_ifun, d_rA, d_rB;
    input [63:0] d_valA, d_valB, d_valC, d_valP;
    output reg [3:0] e_stat, e_icode, e_ifun, e_rA, e_rB;
    output reg [63:0] e_valA, e_valB, e_valC, e_valP;

    always @(posedge clk) begin
        e_stat <= d_stat;
        e_icode <= d_icode;
        e_ifun <= d_ifun;
        e_rA <= d_rA;
        e_rB <= d_rB;
        e_valA <= d_valA;
        e_valB <= d_valB;
        e_valC <= d_valC;
        e_valP <= d_valP;
    end
endmodule

module PIPEregM(clk, e_stat, e_icode, e_rA, e_rB, e_valA, e_valB, e_valC, e_valE, e_valP,e_Cnd, m_stat, m_icode, m_rA, m_rB, m_valA, m_valB, m_valC, m_valE, m_valP, m_Cnd);
    input clk;
    input e_Cnd;
    input [3:0] e_stat, e_icode, e_rA, e_rB;
    input [63:0] e_valA, e_valB, e_valC, e_valE, e_valP;
    output reg m_Cnd;
    output reg [3:0] m_stat, m_icode, m_rA, m_rB;
    output reg [63:0] m_valA, m_valB, m_valC, m_valE, m_valP;

    always @(posedge clk) begin
        m_Cnd <= e_Cnd;
        m_stat <= e_stat;
        m_icode <= e_icode;
        m_rA <= e_rA;
        m_rB <= e_rB;
        m_valA <= e_valA;
        m_valB <= e_valB;
        m_valC <= e_valC;
        m_valE <= e_valE;
        m_valP <= e_valP;
    end
endmodule

module PIPEregW(clk, m_stat, m_icode, m_rA, m_rB, m_valA, m_valB, m_valC, m_valE, m_valM, m_valP, m_Cnd, w_stat, w_icode, w_rA, w_rB, w_valA, w_valB, w_valC, w_valE, w_valM, w_valP, w_Cnd);
    input clk;
    input m_Cnd;
    input [3:0] m_stat, m_icode, m_rA, m_rB;
    input [63:0] m_valA, m_valB, m_valC, m_valE, m_valM, m_valP;
    output reg w_Cnd;
    output reg [3:0] w_stat, w_icode, w_rA, w_rB;
    output reg [63:0] w_valA, w_valB, w_valC, w_valE, w_valM, w_valP;

    always @(posedge clk) begin
        w_Cnd <= m_Cnd;
        w_stat <= m_stat;
        w_icode <= m_icode;
        w_rA <= m_rA;
        w_rB <= m_rB;
        w_valA <= m_valA;
        w_valB <= m_valB;
        w_valC <= m_valC;
        w_valE <= m_valE;
        w_valM <= m_valM;
        w_valP <= m_valP;
    end
endmodule 