// Event Accumulator - Accumulates DVS events and computes center of mass
// Input: DVS events (x, y, polarity)
// Output: Delta movement (dx, dy) when triggered
// Uses SYNCHRONOUS reset (active-high) for iCE40 compatibility

module event_accumulator #(
    parameter X_BITS = 7,   // DVS128: 0-127
    parameter Y_BITS = 7,   // DVS128: 0-127
    parameter ACC_BITS = 16 // Accumulator width
)(
    input  logic clk,
    input  logic rst,       // Synchronous, active-high reset
    
    // Event input
    input  logic event_valid,
    input  logic [X_BITS-1:0] event_x,
    input  logic [Y_BITS-1:0] event_y,
    input  logic event_polarity,  // 1=ON (+1), 0=OFF (-1)
    
    // Control
    input  logic compute_trigger,  // Trigger to compute delta and reset
    
    // Output: movement delta (signed)
    output logic signed [ACC_BITS-1:0] delta_x,
    output logic signed [ACC_BITS-1:0] delta_y,
    output logic delta_valid
);

    // Accumulators for weighted sum of positions
    logic signed [ACC_BITS-1:0] sum_x;
    logic signed [ACC_BITS-1:0] sum_y;
    logic [ACC_BITS-1:0] event_count;
    
    // Previous center (for computing delta)
    logic signed [ACC_BITS-1:0] prev_center_x;
    logic signed [ACC_BITS-1:0] prev_center_y;
    
    // Polarity as signed value: +1 or -1
    wire signed [1:0] pol_signed = event_polarity ? 2'sd1 : -2'sd1;
    
    always @(posedge clk) begin
        if (rst) begin
            sum_x <= 16'sd0;
            sum_y <= 16'sd0;
            event_count <= 16'd0;
            prev_center_x <= 16'sd0;
            prev_center_y <= 16'sd0;
            delta_x <= 16'sd0;
            delta_y <= 16'sd0;
            delta_valid <= 1'b0;
        end else begin
            delta_valid <= 1'b0;
            
            if (compute_trigger) begin
                // Compute current center of mass (simple average)
                // delta = current_center - previous_center
                if (event_count > 0) begin
                    // Simple division by shift (approximate)
                    // Use sum directly as weighted center indicator
                    delta_x <= sum_x - prev_center_x;
                    delta_y <= sum_y - prev_center_y;
                    delta_valid <= 1'b1;
                    
                    // Save current as previous
                    prev_center_x <= sum_x;
                    prev_center_y <= sum_y;
                end
                
                // Reset accumulators for next window
                sum_x <= 16'sd0;
                sum_y <= 16'sd0;
                event_count <= 16'd0;
                
            end else if (event_valid) begin
                // Accumulate: weighted position
                sum_x <= sum_x + ({{(ACC_BITS-X_BITS){1'b0}}, event_x} * pol_signed);
                sum_y <= sum_y + ({{(ACC_BITS-Y_BITS){1'b0}}, event_y} * pol_signed);
                event_count <= event_count + 1'b1;
            end
        end
    end

endmodule
