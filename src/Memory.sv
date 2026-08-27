`timescale 1ns/1ps

module Memory #(

    parameter int DW = 8,
    parameter int DATA_D = 64,
    parameter int ADDR_W = $clog2(DATA_D)
) (
   
    input  logic clk,
    input  logic                  w_enable,
    input  logic [ADDR_W-1:0]     w_address,
    input  logic signed [DW-1:0] w_data,

   
    input  logic [ADDR_W-1:0]     r_address,
    output logic signed [DW-1:0] r_data
);

    
    logic signed [DW-1:0] memory [0:DATA_D-1];

  
    always_ff @(posedge clk) begin
        if (w_enable)
            memory[w_address] <= w_data;
    end

   
    always_comb begin
        r_data = memory[r_address];
    end


endmodule

