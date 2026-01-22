// Gesture Classifier using Matrix Multiplication
// Multiplies movement vector [dx, dy] by 4x2 weight matrix
// Outputs: 4 gesture scores (UP, DOWN, LEFT, RIGHT)
// Gesture = argmax(scores)
// Uses SYNCHRONOUS reset (active-high) for iCE40 compatibility

module gesture_classifier #(
    parameter DATA_BITS = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic clk,
    input  logic rst,       // Synchronous, active-high reset
    
    // Input: movement delta
    input  logic delta_valid,
    input  logic signed [DATA_BITS-1:0] delta_x,
    input  logic signed [DATA_BITS-1:0] delta_y,
    
    // Output: gesture result
    output logic [1:0] gesture,      // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    output logic gesture_valid
);

    // Fixed weight matrix (4 gestures x 2 inputs)
    // Each row: [weight_x, weight_y]
    // UP:    dy > 0  -> weights [0, +64]
    // DOWN:  dy < 0  -> weights [0, -64]  
    // LEFT:  dx < 0  -> weights [-64, 0]
    // RIGHT: dx > 0  -> weights [+64, 0]
    
    localparam signed [WEIGHT_BITS-1:0] W_UP_X    = 8'sd0;
    localparam signed [WEIGHT_BITS-1:0] W_UP_Y    = 8'sd64;
    localparam signed [WEIGHT_BITS-1:0] W_DOWN_X  = 8'sd0;
    localparam signed [WEIGHT_BITS-1:0] W_DOWN_Y  = -8'sd64;
    localparam signed [WEIGHT_BITS-1:0] W_LEFT_X  = -8'sd64;
    localparam signed [WEIGHT_BITS-1:0] W_LEFT_Y  = 8'sd0;
    localparam signed [WEIGHT_BITS-1:0] W_RIGHT_X = 8'sd64;
    localparam signed [WEIGHT_BITS-1:0] W_RIGHT_Y = 8'sd0;

    // Score registers
    logic signed [DATA_BITS+WEIGHT_BITS:0] score_up;
    logic signed [DATA_BITS+WEIGHT_BITS:0] score_down;
    logic signed [DATA_BITS+WEIGHT_BITS:0] score_left;
    logic signed [DATA_BITS+WEIGHT_BITS:0] score_right;
    
    // Pipeline stage
    logic stage1_valid;
    
    // Stage 1: Matrix multiply (dot product)
    // score[i] = W[i][0] * dx + W[i][1] * dy
    always @(posedge clk) begin
        if (rst) begin
            score_up <= 25'sd0;
            score_down <= 25'sd0;
            score_left <= 25'sd0;
            score_right <= 25'sd0;
            stage1_valid <= 1'b0;
        end else begin
            stage1_valid <= delta_valid;
            
            if (delta_valid) begin
                score_up    <= (delta_x * W_UP_X)    + (delta_y * W_UP_Y);
                score_down  <= (delta_x * W_DOWN_X)  + (delta_y * W_DOWN_Y);
                score_left  <= (delta_x * W_LEFT_X)  + (delta_y * W_LEFT_Y);
                score_right <= (delta_x * W_RIGHT_X) + (delta_y * W_RIGHT_Y);
            end
        end
    end
    
    // Stage 2: Argmax - find gesture with highest score
    always @(posedge clk) begin
        if (rst) begin
            gesture <= 2'b00;
            gesture_valid <= 1'b0;
        end else begin
            gesture_valid <= stage1_valid;
            
            if (stage1_valid) begin
                // Compare scores to find max
                if (score_up >= score_down && score_up >= score_left && score_up >= score_right)
                    gesture <= 2'b00;  // UP
                else if (score_down >= score_left && score_down >= score_right)
                    gesture <= 2'b01;  // DOWN
                else if (score_left >= score_right)
                    gesture <= 2'b10;  // LEFT
                else
                    gesture <= 2'b11;  // RIGHT
            end
        end
    end

endmodule
