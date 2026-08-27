`default_nettype none
`timescale 1ns/1ps
//
// tt_um_OwenAReich_markov_chain.v
//
// Tiny Tapeout top module for the Markov-chain convergence ASIC.
//
// RENAME BEFORE SUBMITTING: Tiny Tapeout requires the top module name to
// start with tt_um_ and be unique on the shuttle -- the convention is
// tt_um_<your-github-username>_<project-name>. Replace YOURUSERNAME below
// (and in the filename, and in info.yaml's top_module field) with your own.
//
// -------------------------------------------------------------------------
// PIN MAP (this is a design choice -- change it if you want a different
// protocol, just keep info.yaml's pinout section in sync with whatever you
// pick)
// -------------------------------------------------------------------------
// ui_in[7:0]   : data byte in -- meaning depends on the strobe pulsed below
//
// uio_in[0]    : ld_byte_wr    -- pulse 1 cycle: commit ui_in into the
//                                  target selected by uio_in[2:1]; byte
//                                  pointer auto-increments and wraps per
//                                  target
// uio_in[2:1]  : ld_target_sel -- 00=probability_matrix (row-major, N*N
//                                  bytes), 01=initial_vector (N bytes),
//                                  10=tolerance (ACC_W/8 bytes, LSB first),
//                                  11=max_cycles (CYC_W/8 bytes, LSB first)
// uio_in[3]    : start         -- pulse 1 cycle to kick off a new chain once
//                                  matrix+vector+tolerance+max_cycles have
//                                  all been loaded
// uio_in[4]    : rd_addr_wr    -- pulse 1 cycle: latch ui_in[CNT_W-1:0] as
//                                  the state-vector read address, reset the
//                                  byte pointer for that word to 0
// uio_in[5]    : rd_byte_next  -- pulse 1 cycle: advance to the next byte of
//                                  the currently addressed ACC_W-bit result
//                                  (wraps 0 .. ACC_W/8-1)
// uio_in[7:6]  : unused, tie low on the host side
//
// uio_out[0]   : chain_done_latched -- sticky, set when the core finishes a
//                                       chain, cleared when you next pulse
//                                       start
// uio_out[1]   : converged_latched  -- sticky, valid once
//                                       chain_done_latched = 1
// uio_out[7:2] : 0
//
// uo_out[7:0]  : byte `rd_byte_idx` of rd_vec_data for the address latched
//                 via rd_addr_wr (ACC_W/8 bytes per element -- 4 bytes for
//                 the default ACC_W=32)
// -------------------------------------------------------------------------

module tt_um_YOURUSERNAME_markov_chain #(
    parameter N     = 8,
    parameter DW    = 8,
    parameter ACC_W = 32,
    parameter CYC_W = 16
) (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 1=output)
    input  wire       ena,      // always 1 when powered; safe to ignore here
    input  wire       clk,
    input  wire       rst_n
);
    localparam CNT_W  = (N <= 1) ? 1 : $clog2(N);
    localparam RD_BYTES = ACC_W/8;
    localparam RD_PTR_W = (RD_BYTES <= 1) ? 1 : $clog2(RD_BYTES);

    // ---- decode the control strobes off uio_in -----------------------
    wire        ld_byte_wr    = uio_in[0];
    wire [1:0]  ld_target_sel = uio_in[2:1];
    wire        start_pulse   = uio_in[3];
    wire        rd_addr_wr    = uio_in[4];
    wire        rd_byte_next  = uio_in[5];

    // bidir pins: 0,1 are outputs (status), 2-7 are inputs (control/data
    // handshake lines above)
    assign uio_oe = 8'b0000_0011;

    // ---- state-vector read address + byte pointer ---------------------
    reg [CNT_W-1:0]   rd_vec_addr_reg;
    reg [RD_PTR_W-1:0] rd_byte_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_vec_addr_reg <= '0;
            rd_byte_idx     <= '0;
        end
        else if (rd_addr_wr) begin
            rd_vec_addr_reg <= ui_in[CNT_W-1:0];
            rd_byte_idx     <= '0;
        end
        else if (rd_byte_next) begin
            rd_byte_idx <= (rd_byte_idx == RD_BYTES-1) ? '0 : rd_byte_idx + 1'b1;
        end
    end

    wire signed [ACC_W-1:0] rd_vec_data;
    assign uo_out = rd_vec_data[rd_byte_idx*8 +: 8];

    // ---- sticky status flags -------------------------------------------
    wire core_chain_done;
    wire core_converged;
    reg  chain_done_latched;
    reg  converged_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chain_done_latched <= 1'b0;
            converged_latched  <= 1'b0;
        end
        else begin
            if (start_pulse) begin
                chain_done_latched <= 1'b0;
                converged_latched  <= 1'b0;
            end
            if (core_chain_done) begin
                chain_done_latched <= 1'b1;
                converged_latched  <= core_converged;
            end
        end
    end

    assign uio_out = {6'b0, converged_latched, chain_done_latched};

    // ---- the core ---------------------------------------------------
    markov_core_tt #(
        .N     (N),
        .DW    (DW),
        .ACC_W (ACC_W),
        .CYC_W (CYC_W)
    ) core (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start_pulse),
        .ld_byte_wr    (ld_byte_wr),
        .ld_target_sel (ld_target_sel),
        .ld_byte_data  (ui_in),
        .chain_done    (core_chain_done),
        .converged     (core_converged),
        .rd_vec_addr   (rd_vec_addr_reg),
        .rd_vec_data   (rd_vec_data)
    );

    // ena is asserted whenever the design is powered; this design has no
    // low-power mode, so it is intentionally unused. Tie off to avoid an
    // "unused port" lint warning without affecting function.
    wire _unused_ena = ena;

endmodule
