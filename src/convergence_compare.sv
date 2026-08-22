`timescale 1ns/1ps

// Width-safe signed distance comparator.  Compare against the inclusive
// [old-tolerance, old+tolerance] interval.  Two bounds are evaluated in
// parallel, avoiding a subtract -> conditional negate -> compare chain.
module convergence_compare #(
    parameter integer WIDTH = 32
) (
    input  wire signed [WIDTH-1:0] new_value,
    input  wire signed [WIDTH-1:0] old_value,
    input  wire        [WIDTH-1:0] tolerance,
    output wire                    exceeds_tolerance
);
    // WIDTH+2 is required because signed MAX plus unsigned MAX exceeds the
    // positive range of a WIDTH+1-bit signed value.
    wire signed [WIDTH+1:0] new_ext =
        {{2{new_value[WIDTH-1]}}, new_value};
    wire signed [WIDTH+1:0] old_ext =
        {{2{old_value[WIDTH-1]}}, old_value};
    wire signed [WIDTH+1:0] tol_ext =
        $signed({2'b00, tolerance});
    wire signed [WIDTH+1:0] upper_bound = old_ext + tol_ext;
    wire signed [WIDTH+1:0] lower_bound = old_ext - tol_ext;

    assign exceeds_tolerance =
        (new_ext > upper_bound) || (new_ext < lower_bound);
endmodule
