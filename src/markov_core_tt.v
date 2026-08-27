`timescale 1ns/1ps
// markov_core_tt.v
//
// Tiny Tapeout adaptation of markov_top_converge.
//
// WHY THIS FILE EXISTS
// ---------------------
// The original markov_top_converge exposes probability_matrix (N*N*DW bits =
// 512 b for N=8/DW=8), initial_vector (N*DW = 64 b), tolerance (ACC_W = 32 b)
// and max_cycles (CYC_W = 16 b) as parallel top-level ports. Tiny Tapeout
// gives every digital project exactly 24 general-purpose pins no matter how
// many tiles you buy: 8 dedicated inputs (ui_in), 8 dedicated outputs
// (uo_out), 8 bidirectional (uio_*), plus clk/rst_n/ena. There is no way to
// route a 512-bit bus onto that pinout, so every wide input has to be loaded
// a byte at a time instead.
//
// The FSMs, matmul_top instance, Memory instance and convergence pipeline
// below are IDENTICAL to markov_top_converge — only the way prob_mat_q,
// init_vec_q, tolerance_q and the cycle-count register get their data has
// changed. Downstream logic still sees the exact same registers it did
// before, so the fix log from the original file (elements 1-6) still
// applies unchanged to everything after the loader.
//
// Original wide-bus ports removed:
//   input signed [N*N*DW-1:0] probability_matrix
//   input signed [N*DW-1:0]   initial_vector
//   input [CYC_W-1:0]         max_cycles
//   input [ACC_W-1:0]         tolerance
//
// Replaced with a 4-target byte-serial write port. The host writes
// MAT_BYTES = N*N bytes to target 2'b00 (row-major, matching the internal
// p_read_addr = k*N+j ordering), then VEC_BYTES = N bytes to target 2'b01,
// then TOL_BYTES = ACC_W/8 bytes (LSB first) to target 2'b10, then
// CYC_BYTES = CYC_W/8 bytes (LSB first) to target 2'b11. Each target has its
// own byte pointer that wraps to 0 after a full block, so as long as the
// host always writes complete blocks in order, no explicit pointer reset is
// needed between chains (this mirrors how the existing loader FSM always
// walks S_LOAD_P through exactly N*N counts and S_LOAD_X through exactly N).
//
// ASSUMPTIONS baked into the loader below:
//   - DW == 8, i.e. each probability/vector element is exactly one byte.
//     This matches the design's own default. If you ever change DW, you
//     must also change MAT_BYTES/VEC_BYTES to (N*N)*(DW/8) and (N)*(DW/8)
//     and write DW/8 bytes per element instead of 1.
//   - ACC_W and CYC_W are multiples of 8 (true for the defaults 32 and 16).
//     Those two are already handled generically via ACC_W/8 and CYC_W/8.

module markov_core_tt #(
    parameter N      = 8,
    parameter DW     = 8,
    parameter ACC_W  = 32,
    parameter ADDR_W = (N*N <= 1) ? 1 : $clog2(N*N),
    parameter CYC_W  = 16,
    parameter FRAC_W = 8,
    localparam CNT_W = (N <= 1) ? 1 : $clog2(N)
)(
    input clk,
    input rst_n,

    // Kicks off the whole chain
    input start,

    // Byte-serial loader port (replaces probability_matrix / initial_vector
    // / tolerance / max_cycles wide ports)
    input        ld_byte_wr,     // pulse 1 cycle to commit ld_byte_data
    input  [1:0] ld_target_sel,  // 00=matrix 01=vector 10=tolerance 11=max_cycles
    input  [7:0] ld_byte_data,

    output reg chain_done,   // pulses once the final vector is stored and safe to read
    output reg converged,    // high if stopped early due to convergence

    // External read port into the state-vector memory (unchanged: ACC_W is
    // wide, so the *wrapper* module serialises rd_vec_data to bytes — this
    // core still hands back the full word, same as the original design)
    input  [CNT_W-1:0]        rd_vec_addr,
    output signed [ACC_W-1:0] rd_vec_data
);

    // =========================================================================
    // Byte-serial loader registers
    //
    // These are the exact same storage the original design pipelined the
    // wide ports into (prob_mat_q / init_vec_q), plus two new registers for
    // tolerance/max_cycles that used to be sampled directly off top-level
    // ports. Nothing downstream cares how they got filled in.
    // =========================================================================
    localparam MAT_BYTES = N*N;                 // one byte per DW=8 matrix element
    localparam VEC_BYTES = N;
    localparam TOL_BYTES = ACC_W/8;
    localparam CYC_BYTES = CYC_W/8;

    localparam MAT_PTR_W = (MAT_BYTES <= 1) ? 1 : $clog2(MAT_BYTES);
    localparam VEC_PTR_W = (VEC_BYTES <= 1) ? 1 : $clog2(VEC_BYTES);
    localparam TOL_PTR_W = (TOL_BYTES <= 1) ? 1 : $clog2(TOL_BYTES);
    localparam CYC_PTR_W = (CYC_BYTES <= 1) ? 1 : $clog2(CYC_BYTES);

    (* keep = "true" *) reg signed [N*N*DW-1:0] prob_mat_q;
    (* keep = "true" *) reg signed [N*DW-1:0]   init_vec_q;
    reg [ACC_W-1:0] tolerance_reg;
    reg [CYC_W-1:0] max_cycles_reg;

    reg [MAT_PTR_W-1:0] mat_ptr;
    reg [VEC_PTR_W-1:0] vec_ptr;
    reg [TOL_PTR_W-1:0] tol_ptr;
    reg [CYC_PTR_W-1:0] cyc_ptr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mat_ptr <= '0;
            vec_ptr <= '0;
            tol_ptr <= '0;
            cyc_ptr <= '0;
        end
        else if (ld_byte_wr) begin
            case (ld_target_sel)
                2'b00: begin
                    prob_mat_q[mat_ptr*8 +: 8] <= ld_byte_data;
                    mat_ptr <= (mat_ptr == MAT_BYTES-1) ? '0 : mat_ptr + 1'b1;
                end
                2'b01: begin
                    init_vec_q[vec_ptr*8 +: 8] <= ld_byte_data;
                    vec_ptr <= (vec_ptr == VEC_BYTES-1) ? '0 : vec_ptr + 1'b1;
                end
                2'b10: begin
                    tolerance_reg[tol_ptr*8 +: 8] <= ld_byte_data;
                    tol_ptr <= (tol_ptr == TOL_BYTES-1) ? '0 : tol_ptr + 1'b1;
                end
                default: begin // 2'b11
                    max_cycles_reg[cyc_ptr*8 +: 8] <= ld_byte_data;
                    cyc_ptr <= (cyc_ptr == CYC_BYTES-1) ? '0 : cyc_ptr + 1'b1;
                end
            endcase
        end
    end

    // =========================================================================
    // Everything below this line is the original markov_top_converge design,
    // unchanged, except:
    //   - tolerance_q is loaded from tolerance_reg instead of a tolerance port
    //   - cycles_left is loaded from max_cycles_reg instead of a max_cycles port
    // =========================================================================
    reg  need_load;
    reg  load_ready;
    reg  ld_en, ld_sel_ab;
    reg  [ADDR_W-1:0] ld_addr;
    reg  signed [DW-1:0] ld_data;

    reg  mm_start;
    wire mm_done;
    reg  [ADDR_W-1:0] mm_rd_addr;
    wire signed [ACC_W-1:0] mm_rd_data;

    matmul_top #(
        .N(N),
        .DW(DW),
        .ACC_W(ACC_W),
        .FRAC_W(FRAC_W),
        .ADDR_W(ADDR_W),
        .CNT_W(CNT_W),
        .USE_PARALLEL(0)   // serial MAC engine: same math, ~1/8th the area of
                            // the 8-lane parallel core, at a cost of a few
                            // extra clock cycles per matmul. On a pin- and
                            // area-limited chip that trade is essentially
                            // free — set to 1 only if you have tiles to spare
                            // and want the throughput.
    ) mm (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (mm_start),
        .done     (mm_done),
        .ld_en    (ld_en),
        .ld_sel_ab(ld_sel_ab),
        .ld_addr  (ld_addr),
        .ld_data  (ld_data),
        .rd_en    (1'b1),
        .rd_addr  (mm_rd_addr),
        .rd_data  (mm_rd_data)
    );

    reg signed [ACC_W-1:0] vec_w_data;
    reg [CNT_W-1:0]        vec_w_addr;
    reg                    vec_w_en;

    wire [CNT_W-1:0]        state_mem_raddr;
    wire signed [ACC_W-1:0] state_mem_rdata;

    Memory #(
        .DW    (ACC_W),
        .DATA_D(N),
        .ADDR_W(CNT_W)
    ) state_mem (
        .clk      (clk),
        .w_data   (vec_w_data),
        .w_address(vec_w_addr),
        .w_enable (vec_w_en),
        .r_address(state_mem_raddr),
        .r_data   (state_mem_rdata)
    );

    assign rd_vec_data = state_mem_rdata;

    localparam IDLE     = 3'd0;
    localparam LOAD     = 3'd1;
    localparam WAIT_MM  = 3'd2;
    localparam READOUT  = 3'd3;
    localparam EVALUATE = 3'd4;

    (* keep = "true" *) reg [2:0] state;
    /* verilator lint_off WIDTHTRUNC */
    localparam [CNT_W-1:0] N_LAST = N - 1;
    /* verilator lint_on WIDTHTRUNC */

    reg [CNT_W-1:0] idx;
    reg [CYC_W-1:0] cycles_left;
    reg [ACC_W-1:0] tolerance_q;

    reg             is_first_iteration;
    reg             exceeded_tolerance;
    reg             compare_valid;
    reg signed [ACC_W-1:0] compare_new_q;
    reg signed [ACC_W-1:0] compare_old_q;

    wire is_readout_phase = (state == READOUT);

    wire current_exceeds;
    wire will_exceed = exceeded_tolerance | current_exceeds;

    convergence_compare #(
        .WIDTH(ACC_W)
    ) convergence_cmp (
        .new_value        (compare_new_q),
        .old_value        (compare_old_q),
        .tolerance        (tolerance_q),
        .exceeds_tolerance(current_exceeds)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= IDLE;
            mm_start           <= 0;
            chain_done         <= 0;
            converged          <= 0;
            need_load          <= 0;
            idx                <= 0;
            mm_rd_addr         <= 0;
            cycles_left        <= 0;
            tolerance_q        <= 0;
            vec_w_en           <= 0;
            is_first_iteration <= 1;
            exceeded_tolerance <= 0;
            compare_valid      <= 0;
            compare_new_q      <= 0;
            compare_old_q      <= 0;
        end
        else begin
            mm_start   <= 0;
            chain_done <= 0;
            vec_w_en   <= 0;

            case (state)

                IDLE: begin
                    if (start) begin
                        need_load          <= 1'b1;
                        cycles_left        <= (max_cycles_reg == 0) ? 0 : max_cycles_reg - 1'b1;
                        tolerance_q        <= tolerance_reg;
                        is_first_iteration <= 1'b1;
                        converged          <= 1'b0;
                        state              <= LOAD;
                    end
                end

                LOAD: begin
                    if (load_ready) begin
                        need_load <= 1'b0;
                        mm_start  <= 1'b1;
                        state     <= WAIT_MM;
                    end
                end

                WAIT_MM: begin
                    if (mm_done) begin
                        idx    <= 0;
                        mm_rd_addr <= 0;
                        state  <= READOUT;
                        exceeded_tolerance <= is_first_iteration ? 1'b1 : 1'b0;
                        compare_valid      <= 1'b0;
                    end
                end

                READOUT: begin
                    compare_new_q <= mm_rd_data;
                    compare_old_q <= state_mem_rdata;
                    compare_valid <= 1'b1;

                    vec_w_addr <= idx;
                    vec_w_data <= mm_rd_data;
                    vec_w_en   <= 1'b1;

                    if (idx == N_LAST) begin
                        is_first_iteration <= 1'b0;
                        if (compare_valid && current_exceeds)
                            exceeded_tolerance <= 1'b1;
                        state <= EVALUATE;
                    end
                    else begin
                        idx        <= idx + 1'b1;
                        mm_rd_addr <= mm_rd_addr + 1'b1;
                        if (compare_valid && current_exceeds)
                            exceeded_tolerance <= 1'b1;
                    end
                end

                EVALUATE: begin
                    if (cycles_left == 0 || !will_exceed) begin
                        chain_done <= 1'b1;
                        converged  <= !will_exceed;
                        state      <= IDLE;
                    end
                    else begin
                        cycles_left <= cycles_left - 1'b1;
                        need_load   <= 1'b1;
                        state       <= LOAD;
                    end
                end

                default: begin
                    state <= IDLE;
                end

            endcase
        end
    end

    localparam S_IDLE   = 2'b00;
    localparam S_LOAD_P = 2'b01;
    localparam S_LOAD_X = 2'b10;
    localparam S_DONE   = 2'b11;
    /* verilator lint_off WIDTHTRUNC */
    localparam [ADDR_W-1:0] N_VEC_LAST = N - 1;
    localparam [ADDR_W-1:0] N_SQ_LAST  = N*N - 1;
    /* verilator lint_on WIDTHTRUNC */

    wire new_chain_start = start && (state == IDLE);
    wire trigger         = new_chain_start | need_load;

    (* keep = "true" *) reg [1:0]      ldr_state;
    reg [ADDR_W-1:0] load_count;
    reg              first_pass;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ldr_state  <= S_IDLE;
            load_count <= 0;
            load_ready <= 1'b0;
            first_pass <= 1'b1;
        end
        else begin
            case (ldr_state)

                S_IDLE: begin
                    load_ready <= 1'b0;
                    if (new_chain_start) begin
                        first_pass <= 1'b1;
                        load_count <= 0;
                        ldr_state  <= S_LOAD_P;
                    end
                    else if (need_load) begin
                        load_count <= 0;
                        ldr_state  <= first_pass ? S_LOAD_P : S_LOAD_X;
                    end
                end

                S_LOAD_P: begin
                    if (load_count == N_SQ_LAST) begin
                        load_count <= 0;
                        ldr_state  <= S_LOAD_X;
                    end
                    else begin
                        load_count <= load_count + 1'b1;
                    end
                end

                S_LOAD_X: begin
                    if (load_count == N_VEC_LAST) begin
                        load_count <= 0;
                        load_ready <= 1'b1;
                        first_pass <= 1'b0;
                        ldr_state  <= S_DONE;
                    end
                    else begin
                        load_count <= load_count + 1'b1;
                    end
                end

                S_DONE: begin
                    load_ready <= 1'b1;
                    if (!trigger)
                        ldr_state <= S_IDLE;
                end

                default: begin
                    ldr_state  <= S_IDLE;
                    load_count <= 0;
                    load_ready <= 1'b0;
                end

            endcase
        end
    end

    wire loader_feeding_back = (ldr_state == S_LOAD_X) && !first_pass;

    assign state_mem_raddr = loader_feeding_back ? load_count[CNT_W-1:0] :
                             is_readout_phase    ? idx[CNT_W-1:0] :
                                                   rd_vec_addr;

    always @(*) begin
        ld_en     = 1'b0;
        ld_sel_ab = 1'b0;
        ld_addr   = 0;
        ld_data   = 0;

        case (ldr_state)

            S_LOAD_P: begin
                ld_en     = 1'b1;
                ld_sel_ab = 1'b0;
                ld_addr   = load_count;
                ld_data   = prob_mat_q[load_count*DW +: DW];
            end

            S_LOAD_X: begin
                ld_en     = 1'b1;
                ld_sel_ab = 1'b1;
                ld_addr   = load_count;
                ld_data   = first_pass ? init_vec_q[load_count*DW +: DW]
                                       : state_mem_rdata[DW-1:0];
            end

            default: ;

        endcase
    end

endmodule
