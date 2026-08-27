`timescale 1ns/1ps

// Signed, two-stage MAC retained from the Part 1 architecture.
// A valid input is multiplied at one edge.  At the following edge the
// registered product is accumulated and acc_out/valid_out become valid.
//
// Fix log (vs original):
//  1. Removed dead `product_ext` wire.  It was defined as a sign-extension
//     of product_q to ACC_W bits but never read by any downstream logic;
//     it generated a synthesis warning and an unused net in the netlist.
module mac_unit #(
    parameter integer DW    = 8,
    parameter integer ACC_W = 32
) (
    input  wire clk,
    input  wire rst_n,
    input  wire signed [DW-1:0]     a,
    input  wire signed [DW-1:0]     b,
    input  wire                     valid_in,
    input  wire                     clear_acc,
    output reg  signed [ACC_W-1:0]  acc_out,
    output reg                      valid_out
);
    reg signed [(2*DW)-1:0] product_q;
    reg                     valid_q;
    reg                     clear_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            product_q <= '0;
            valid_q   <= 1'b0;
            clear_q   <= 1'b0;
            acc_out   <= '0;
            valid_out <= 1'b0;
        end else begin
            product_q <= $signed(a) * $signed(b);
            valid_q   <= valid_in;
            clear_q   <= clear_acc;
            valid_out <= valid_q;
            if (valid_q) begin
                if (clear_q)
                    acc_out <= {{(ACC_W-(2*DW)){product_q[(2*DW)-1]}}, product_q};
                else
                    acc_out <= acc_out +
                               {{(ACC_W-(2*DW)){product_q[(2*DW)-1]}}, product_q};
            end
        end
    end
endmodule
