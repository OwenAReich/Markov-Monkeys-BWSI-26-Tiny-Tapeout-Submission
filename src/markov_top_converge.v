`timescale 1ns/1ps

module markov_top_converge #(
    parameter N      = 8,
    parameter DW     = 8,
    parameter ACC_W  = 32,
    // FIX 1: guard prevents $clog2(1)=0 zero-width bus when N=1
    parameter ADDR_W = (N*N <= 1) ? 1 : $clog2(N*N),
    parameter CYC_W  = 16,
    parameter FRAC_W = 8,
    localparam CNT_W = (N <= 1) ? 1 : $clog2(N)
)(
    input clk,
    input rst_n,

    // Kicks off the whole chain
    input start,
    input [CYC_W-1:0] max_cycles,
    input [ACC_W-1:0] tolerance,

    output reg chain_done,   // pulses once the final vector is stored and safe to read
    output reg converged,    // high if stopped early due to convergence

    input signed [N*N*DW-1:0] probability_matrix,
    input signed [N*DW-1:0]   initial_vector,

    // External read port into the state-vector memory
    input  [CNT_W-1:0]        rd_vec_addr,
    output signed [ACC_W-1:0] rd_vec_data
);

    (* keep = "true" *) reg signed [N*N*DW-1:0] prob_mat_q;
    (* keep = "true" *) reg signed [N*DW-1:0]   init_vec_q;

    always @(posedge clk) begin
        prob_mat_q <= probability_matrix;
        init_vec_q <= initial_vector;
    end

    reg  need_load;
    reg  load_ready;
    reg  ld_en, ld_sel_ab;
    reg  [ADDR_W-1:0] ld_addr;
    reg  signed [DW-1:0] ld_data;

    // Matrix multiplier interface
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
        .CNT_W(CNT_W)
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

    // State-vector memory: holds the current/previous probability vector
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
    // Width-matched constants. Truncation from 32-bit parameter arithmetic is
    // intentional; lint_off scopes suppress WIDTHTRUNC for these declarations only.
    /* verilator lint_off WIDTHTRUNC */
    localparam [CNT_W-1:0] N_LAST = N - 1;     // N-1 fits in CNT_W by construction
    /* verilator lint_on WIDTHTRUNC */

    reg [CNT_W-1:0] idx;
    reg [CYC_W-1:0] cycles_left;
    reg [ACC_W-1:0] tolerance_q;

    // Convergence-checking pipeline registers
    reg             is_first_iteration;
    reg             exceeded_tolerance;
    reg             compare_valid;
    reg signed [ACC_W-1:0] compare_new_q;
    reg signed [ACC_W-1:0] compare_old_q;

    wire is_readout_phase = (state == READOUT);

    wire current_exceeds;
    // Combinatorial OR: accumulated result from elements 0..N-2 (via
    // exceeded_tolerance) with element N-1 (via current_exceeds in EVALUATE).
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
                        cycles_left        <= (max_cycles == 0) ? 0 : max_cycles - 1'b1;
                        tolerance_q        <= tolerance;
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
                        // Force exceeded on the first pass so at least two
                        // iterations always occur (prevents spurious early
                        // convergence on uninitialised state memory).
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
                        mm_rd_addr <= mm_rd_addr + 1'b1; // self-increment: avoids CNT_W→ADDR_W width mismatch
                        // Capture element idx-1 (one-cycle pipelined).
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
    localparam [ADDR_W-1:0] N_VEC_LAST = N - 1;    // fits: N-1 < 2^ADDR_W
    localparam [ADDR_W-1:0] N_SQ_LAST  = N*N - 1;  // fits: N*N-1 < 2^ADDR_W (N*N == 2^ADDR_W)
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

    // Multi-way address mux for the state-vector memory.
    // Priority: loader feedback > READOUT index > external read port.
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
                ld_sel_ab = 1'b0;                                   // P -> B
                ld_addr   = load_count;
                ld_data   = prob_mat_q[load_count*DW +: DW];       // registered input
            end

            S_LOAD_X: begin
                ld_en     = 1'b1;
                ld_sel_ab = 1'b1;                                   // X -> A
                ld_addr   = load_count;
                ld_data   = first_pass ? init_vec_q[load_count*DW +: DW]  // registered input
                                       : state_mem_rdata[DW-1:0];
            end

            // S_IDLE and S_DONE: ld_en is already 0 from the defaults above
            // this case. Explicit arm suppresses Verilator CASEINCOMPLETE.
            default: ;

        endcase
    end

endmodule
