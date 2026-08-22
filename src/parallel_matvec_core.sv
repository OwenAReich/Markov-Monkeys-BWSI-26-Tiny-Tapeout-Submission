`timescale 1ns/1ps

module parallel_matvec_core #(
    parameter integer N      = 8,
    parameter integer DW     = 8,
    parameter integer ACC_W  = 32,
    parameter integer ADDR_W = (N*N <= 1) ? 1 : $clog2(N*N),
    parameter integer CNT_W  = (N <= 1) ? 1 : $clog2(N)
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        start,
    output reg                         busy,
    output reg                         done,

    input  wire                        ld_en,
    input  wire                        ld_sel_ab,
    input  wire [ADDR_W-1:0]           ld_addr,
    input  wire signed [DW-1:0]        ld_data,

    input  wire                        rd_en,
    input  wire [ADDR_W-1:0]           rd_addr,
    output wire signed [ACC_W-1:0]     rd_data
);

    localparam integer X_ADDR_W = (N <= 1) ? 1 : $clog2(N);

    localparam [2:0] IDLE    = 3'd0;
    localparam [2:0] FEED    = 3'd1;
    localparam [2:0] DRAIN   = 3'd2;
    localparam [2:0] CAPTURE = 3'd3;

    reg [2:0] state;
    reg [CNT_W-1:0] k_count;
    // Width-matched constants. SV part-select casts explicitly truncate from
    // 32-bit parameter arithmetic to the target width, suppressing WIDTHTRUNC.
    // N_SQ is intentionally absent: when N is a power of 2, N*N == 2^ADDR_W
    // which overflows an ADDR_W-bit constant to 0 (always-true comparison).
    // The loader FSM guarantees ld_addr stays in [0, N*N-1] without a guard.
    localparam [CNT_W-1:0]  N_LAST = CNT_W'(N - 1);
    localparam [ADDR_W-1:0] N_VEC  = ADDR_W'(N);

    reg signed [DW-1:0] x_mem [0:N-1];

    // Retained because the parallel datapath requires N simultaneous reads.
    reg signed [DW-1:0] p_mem [0:N*N-1];

    reg signed [ACC_W-1:0] result_mem [0:N-1];
    reg results_valid;

    wire lane_valid_in;
    wire lane_clear;
    wire load_accept;

    reg final_pipe_0;
    reg final_pipe_1;

    wire signed [ACC_W-1:0] lane_acc [0:N-1];
    wire [N-1:0] lane_valid;

    assign lane_valid_in = (state == FEED);
    assign lane_clear    = lane_valid_in && (k_count == 0);
    assign load_accept   = ld_en && (state == IDLE);

    assign rd_data =
        (rd_en && results_valid && (rd_addr < N_VEC)) ? result_mem[rd_addr[X_ADDR_W-1:0]] : '0;

    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : GEN_MAC
            wire signed [DW-1:0] p_value;
            // A per-lane reset distribution register prevents the external
            // reset input from directly driving every product/accumulator bit.
            // The MAC reset is synchronous, so the one-cycle local delay is
            // harmless: operands are loaded for many cycles before start.
            (* keep = "true" *) reg lane_rst_n;
            // Replicate the feed index so one global counter does not drive
            // every operand mux in all N multiplier lanes.
            (* keep = "true" *) reg [CNT_W-1:0] lane_k_count;

            assign p_value = p_mem[(lane_k_count * N) + g];

            always @(posedge clk)
                lane_rst_n <= rst_n;

            always @(posedge clk) begin
                if (!rst_n)
                    lane_k_count <= '0;
                else if (state == IDLE && start)
                    lane_k_count <= '0;
                else if (state == FEED && lane_k_count != N_LAST)
                    lane_k_count <= lane_k_count + 1'b1;
            end

            mac_unit #(
                .DW    (DW),
                .ACC_W (ACC_W)
            ) u_mac (
                .clk       (clk),
                .rst_n     (lane_rst_n),
                .a         (x_mem[lane_k_count]),
                .b         (p_value),
                .valid_in  (lane_valid_in),
                .clear_acc (lane_clear),
                .acc_out   (lane_acc[g]),
                .valid_out (lane_valid[g])
            );
        end
    endgenerate

    always @(posedge clk) begin
        if (load_accept) begin
    
            if (ld_sel_ab) begin
                if (ld_addr < N_VEC)
                    x_mem[ld_addr[X_ADDR_W-1:0]] <= ld_data;
            end
       
            else begin
                // No explicit bounds guard: the loader FSM sequences ld_addr
                // through [0, N*N-1] only, so it is always in range. A guard
                // of the form `ld_addr < N*N` overflows an ADDR_W-bit constant
                // to 0 when N is a power of 2 (e.g. N=8 → N*N=64=2^ADDR_W).
                p_mem[ld_addr] <= ld_data;
            end
        end
    end

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            k_count      <= '0;
            final_pipe_0 <= 1'b0;
            final_pipe_1 <= 1'b0;
            results_valid <= 1'b0;

        end
        else begin
            done <= 1'b0;

            final_pipe_0 <= lane_valid_in && (k_count == N_LAST);
            final_pipe_1 <= final_pipe_0;

            case (state)
                IDLE: begin
                    if (start) begin
                        busy    <= 1'b1;
                        k_count <= '0;
                        results_valid <= 1'b0;
                        state   <= FEED;
                    end
                end

                FEED: begin
                    if (k_count == N_LAST)
                        state <= DRAIN;
                    else
                        k_count <= k_count + 1'b1;
                end

                // The final accumulator update occurs on the detection edge.
                DRAIN: begin
                    if (lane_valid[0] && final_pipe_1)
                        state <= CAPTURE;
                end

                // One full cycle later, all lane accumulators are stable.
                CAPTURE: begin
                    for (i = 0; i < N; i = i + 1)
                        result_mem[i] <= lane_acc[i];

                    busy  <= 1'b0;
                    done  <= 1'b1;
                    results_valid <= 1'b1;
                    state <= IDLE;
                end

                default: begin
                    busy  <= 1'b0;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
