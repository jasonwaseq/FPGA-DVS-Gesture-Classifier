`timescale 1ns/1ps

// Parallel-input systolic matrix multiply for feature classification.
// Processes PARALLEL_INPUTS cells per cycle; total cycles = ceil(NUM_CELLS / PARALLEL_INPUTS).

module SystolicMatrixMultiply #(
    parameter NUM_CLASSES    = 4,
    parameter NUM_CELLS      = 1024,
    parameter VALUE_BITS     = 6,
    parameter WEIGHT_BITS    = 8,
    parameter ACC_BITS       = 24,
    parameter PARALLEL_INPUTS = 4
)(
    input  logic                                    clk,
    input  logic                                    rst,
    input  logic                                    start,
    input  logic [PARALLEL_INPUTS*VALUE_BITS-1:0]   feature_in,
    input  logic                                    feature_valid,
    output logic [PARALLEL_INPUTS*$clog2(NUM_CELLS)-1:0]            w_addr_flat,
    input  logic [PARALLEL_INPUTS*NUM_CLASSES*WEIGHT_BITS-1:0]      w_data_flat,
    output logic                                    result_valid,
    output logic [1:0]                              best_class,
    output logic [NUM_CLASSES*ACC_BITS-1:0]         scores_flat
);

    localparam CNT_BITS      = $clog2(NUM_CELLS + 4);
    localparam PARALLEL_BITS = $clog2(PARALLEL_INPUTS);
    localparam CYCLES_NEEDED = (NUM_CELLS + PARALLEL_INPUTS - 1) / PARALLEL_INPUTS;

    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_RUNNING = 2'd1,
        S_DRAIN   = 2'd2,
        S_ARGMAX  = 2'd3
    } state_t;

    state_t state;

    logic [CNT_BITS-1:0] cell_cnt;

    logic signed [ACC_BITS-1:0] acc   [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] acc_r [0:NUM_CLASSES-1];

    logic [PARALLEL_INPUTS*VALUE_BITS-1:0] feat_pipe_r;
    logic                                   pipe_valid_r;

    wire [$clog2(NUM_CELLS)-1:0] w_addr [0:PARALLEL_INPUTS-1];
    wire [NUM_CLASSES*WEIGHT_BITS-1:0] w_data [0:PARALLEL_INPUTS-1];

    generate
        genvar pa;
        for (pa = 0; pa < PARALLEL_INPUTS; pa = pa + 1) begin : gen_unpack_addr
            assign w_addr[pa] = w_addr_flat[(pa+1)*$clog2(NUM_CELLS)-1 : pa*$clog2(NUM_CELLS)];
            assign w_data[pa] = w_data_flat[(pa+1)*NUM_CLASSES*WEIGHT_BITS-1 : pa*NUM_CLASSES*WEIGHT_BITS];
        end
    endgenerate

    wire signed [WEIGHT_BITS-1:0] w [0:PARALLEL_INPUTS-1][0:NUM_CLASSES-1];
    generate
        genvar pi, gk;
        for (pi = 0; pi < PARALLEL_INPUTS; pi = pi + 1) begin : gen_parallel_weights
            for (gk = 0; gk < NUM_CLASSES; gk = gk + 1) begin : gen_w_unpack
                assign w[pi][gk] = $signed(w_data[pi][(gk+1)*WEIGHT_BITS-1 : gk*WEIGHT_BITS]);
            end
        end
    endgenerate

    wire signed [VALUE_BITS-1:0] feat_vals [0:PARALLEL_INPUTS-1];
    generate
        genvar fi;
        for (fi = 0; fi < PARALLEL_INPUTS; fi = fi + 1) begin : gen_feat_unpack
            assign feat_vals[fi] = $signed(feat_pipe_r[(fi+1)*VALUE_BITS-1 : fi*VALUE_BITS]);
        end
    endgenerate

    logic signed [ACC_BITS-1:0] max_score;
    integer k, i;

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            cell_cnt     <= '0;
            w_addr_flat  <= '0;
            feat_pipe_r  <= '0;
            pipe_valid_r <= 1'b0;
            result_valid <= 1'b0;
            best_class   <= '0;
            max_score    <= '0;
            for (k = 0; k < NUM_CLASSES; k = k + 1) begin
                acc  [k] <= '0;
                acc_r[k] <= '0;
            end
        end else begin
            result_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    pipe_valid_r <= 1'b0;
                    if (start) begin
                        for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= '0;
                        cell_cnt <= '0;
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1)
                            w_addr_flat[(p+1)*$clog2(NUM_CELLS)-1 : p*$clog2(NUM_CELLS)] <= p;
                        state <= S_RUNNING;
                    end
                end

                S_RUNNING: begin
                    feat_pipe_r <= feature_in;

                    if (pipe_valid_r) begin
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1) begin
                            if ((cell_cnt * PARALLEL_INPUTS + p) < NUM_CELLS) begin
                                for (k = 0; k < NUM_CLASSES; k = k + 1) begin
                                    acc[k] <= acc[k] +
                                        ACC_BITS'($signed({1'b0, feat_vals[p]}) * $signed(w[p][k]));
                                end
                            end
                        end
                    end

                    if (feature_valid) begin
                        pipe_valid_r <= 1'b1;
                        cell_cnt <= cell_cnt + 1'b1;
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1) begin
                            if ((cell_cnt * PARALLEL_INPUTS + p + PARALLEL_INPUTS) < NUM_CELLS) begin
                                w_addr_flat[(p+1)*$clog2(NUM_CELLS)-1 : p*$clog2(NUM_CELLS)] <= (cell_cnt + 1) * PARALLEL_INPUTS + p;
                            end
                        end

                        if (cell_cnt >= CNT_BITS'(CYCLES_NEEDED - 1))
                            state <= S_DRAIN;
                    end
                end

                S_DRAIN: begin
                    pipe_valid_r <= 1'b0;
                    state        <= S_ARGMAX;
                end

                S_ARGMAX: begin
                    for (k = 0; k < NUM_CLASSES; k = k + 1)
                        acc_r[k] <= acc[k];

                    max_score  <= acc[0];
                    best_class <= 2'd0;
                    for (i = 1; i < NUM_CLASSES; i = i + 1) begin
                        if ($signed(acc[i]) > $signed(max_score)) begin
                            max_score  <= acc[i];
                            best_class <= 2'(i);
                        end
                    end

                    result_valid <= 1'b1;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    generate
        genvar gs;
        for (gs = 0; gs < NUM_CLASSES; gs = gs + 1) begin : gen_scores
            assign scores_flat[(gs+1)*ACC_BITS-1 : gs*ACC_BITS] = acc_r[gs];
        end
    endgenerate

endmodule
