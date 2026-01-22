// DVS Gesture Accelerator Top Module
// Simple matrix multiplication accelerator for gesture recognition
// Input: DVS events (x, y, polarity, timestamp)
// Output: Gesture classification (UP, DOWN, LEFT, RIGHT)
// Uses SYNCHRONOUS reset (active-high) for iCE40 compatibility

module dvs_gesture_accel #(
    parameter X_BITS = 7,          // DVS128: 0-127
    parameter Y_BITS = 7,          // DVS128: 0-127
    parameter TS_BITS = 16,        // Timestamp bits
    parameter WINDOW_EVENTS = 100  // Events before classification
)(
    input  logic clk,
    input  logic rst,              // Synchronous, active-high reset
    
    // DVS Event Input Interface
    input  logic event_valid,
    input  logic [X_BITS-1:0] event_x,
    input  logic [Y_BITS-1:0] event_y,
    input  logic event_polarity,   // 1=ON, 0=OFF
    input  logic [TS_BITS-1:0] event_ts,
    
    // Gesture Output Interface
    output logic [1:0] gesture,    // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    output logic gesture_valid
);

    // Internal signals
    logic signed [15:0] delta_x, delta_y;
    logic delta_valid;
    logic compute_trigger;
    
    // Event counter for windowing
    logic [15:0] event_counter;
    
    // Generate classification trigger every N events
    always @(posedge clk) begin
        if (rst) begin
            event_counter <= 16'd0;
            compute_trigger <= 1'b0;
        end else begin
            compute_trigger <= 1'b0;
            
            if (event_valid) begin
                if (event_counter >= WINDOW_EVENTS - 1) begin
                    compute_trigger <= 1'b1;
                    event_counter <= 16'd0;
                end else begin
                    event_counter <= event_counter + 1'b1;
                end
            end
        end
    end
    
    // Event Accumulator: computes center-of-mass movement
    event_accumulator #(
        .X_BITS(X_BITS),
        .Y_BITS(Y_BITS),
        .ACC_BITS(16)
    ) u_accumulator (
        .clk(clk),
        .rst(rst),
        .event_valid(event_valid),
        .event_x(event_x),
        .event_y(event_y),
        .event_polarity(event_polarity),
        .compute_trigger(compute_trigger),
        .delta_x(delta_x),
        .delta_y(delta_y),
        .delta_valid(delta_valid)
    );
    
    // Gesture Classifier: matrix multiply + argmax
    gesture_classifier #(
        .DATA_BITS(16),
        .WEIGHT_BITS(8)
    ) u_classifier (
        .clk(clk),
        .rst(rst),
        .delta_valid(delta_valid),
        .delta_x(delta_x),
        .delta_y(delta_y),
        .gesture(gesture),
        .gesture_valid(gesture_valid)
    );

endmodule
