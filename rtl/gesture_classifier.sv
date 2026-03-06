// Gesture classifier over 4 class scores:
//   0: Down, 1: Left, 2: Right, 3: Up
// class_pass = (max_score - second_score) > PASS_MARGIN
// gesture_valid requires PERSISTENCE_COUNT consecutive passing windows with same class.

module gesture_classifier #(
    parameter int NUM_CLASSES        = 4,
    parameter int SCORE_BITS         = 32,
    parameter int PASS_MARGIN        = 64,
    parameter int PERSISTENCE_COUNT  = 2,
    parameter int CONF_BITS          = 8,
    parameter int CONF_SHIFT         = 4
)(
    input  logic                               clk,
    input  logic                               rst,
    input  logic [NUM_CLASSES*SCORE_BITS-1:0] scores_flat,
    input  logic                               scores_valid,
    output logic [1:0]                         class_gesture,
    output logic                               class_valid,
    output logic                               class_pass,
    output logic [1:0]                         gesture,
    output logic                               gesture_valid,
    output logic [CONF_BITS-1:0]               gesture_confidence,
    output logic [2:0]                         debug_state
);

    localparam int PERSIST_BITS = (PERSISTENCE_COUNT > 1) ? $clog2(PERSISTENCE_COUNT + 1) : 1;

    logic signed [SCORE_BITS-1:0] scores [0:NUM_CLASSES-1];
    logic signed [SCORE_BITS-1:0] max_score_c;
    logic signed [SCORE_BITS-1:0] second_score_c;
    logic [1:0]                   max_class_c;
    logic signed [SCORE_BITS:0]   margin_c;
    logic                         pass_c;

    logic [1:0] last_pass_class;
    logic [PERSIST_BITS-1:0] pass_streak;
    logic [PERSIST_BITS-1:0] next_streak;

    always_comb begin
        for (int i = 0; i < NUM_CLASSES; i = i + 1)
            scores[i] = $signed(scores_flat[i*SCORE_BITS +: SCORE_BITS]);
    end

    always_comb begin
        max_score_c    = scores[0];
        second_score_c = {1'b1, {(SCORE_BITS-1){1'b0}}};
        max_class_c    = 2'd0;

        for (int i = 1; i < NUM_CLASSES; i = i + 1) begin
            if (scores[i] > max_score_c) begin
                second_score_c = max_score_c;
                max_score_c    = scores[i];
                max_class_c    = i[1:0];
            end else if (scores[i] > second_score_c) begin
                second_score_c = scores[i];
            end
        end

        margin_c = max_score_c - second_score_c;
        pass_c   = (margin_c > PASS_MARGIN);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            class_gesture      <= 2'd0;
            class_valid        <= 1'b0;
            class_pass         <= 1'b0;
            gesture            <= 2'd0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
            last_pass_class    <= 2'd0;
            pass_streak        <= '0;
            debug_state        <= 3'd0;
        end else begin
            class_valid   <= 1'b0;
            class_pass    <= 1'b0;
            gesture_valid <= 1'b0;
            debug_state   <= 3'd0;

            if (scores_valid) begin
                class_gesture <= max_class_c;
                class_valid   <= 1'b1;
                class_pass    <= pass_c;

                if (pass_c) begin
                    if (max_class_c == last_pass_class) begin
                        if (pass_streak < PERSISTENCE_COUNT)
                            next_streak = pass_streak + 1'b1;
                        else
                            next_streak = pass_streak;
                    end else begin
                        next_streak = {{(PERSIST_BITS-1){1'b0}}, 1'b1};
                    end

                    last_pass_class <= max_class_c;
                    pass_streak     <= next_streak;

                    if (next_streak >= PERSISTENCE_COUNT) begin
                        gesture       <= max_class_c;
                        gesture_valid <= 1'b1;
                        debug_state   <= 3'd2;
                    end else begin
                        debug_state <= 3'd1;
                    end

                    if ((margin_c >>> CONF_SHIFT) > ((1 << CONF_BITS) - 1))
                        gesture_confidence <= {CONF_BITS{1'b1}};
                    else if (margin_c <= 0)
                        gesture_confidence <= '0;
                    else
                        gesture_confidence <= margin_c[CONF_SHIFT +: CONF_BITS];
                end else begin
                    pass_streak <= '0;
                    debug_state <= 3'd0;
                end
            end
        end
    end

endmodule
