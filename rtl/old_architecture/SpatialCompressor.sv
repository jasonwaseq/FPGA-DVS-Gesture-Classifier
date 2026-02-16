// =============================================================================
// Module: SpatialCompressor
// =============================================================================
// Maps 320×320 sensor coordinates to 16×16 grid using integer arithmetic
//
// Algorithm: grid = (sensor × 13) >> 8
//   - Approximates division by 20 (since 320/16 = 20)
//   - Multiplication by 13 and right shift by 8 avoids expensive divider
//   - Maximum error: ~0.6% vs true division
//
// Output is signed position relative to grid center:
//   - Grid cell 0 → position -8
//   - Grid cell 8 → position 0 (center)
//   - Grid cell 15 → position +7
//
// Coordinate System:
//   - Input: Unsigned sensor coordinates [0-319]
//   - Output: Signed grid-relative positions [-8 to +7]
//   - Center of grid (cell 8) corresponds to position 0
// =============================================================================

module SpatialCompressor #(
    parameter SENSOR_RES = 320,                  // Input resolution
    parameter GRID_SIZE  = 16,                   // Output grid size
    parameter GRID_BITS  = 4                     // Bits per grid coordinate
)(
    input  logic        clk,
    input  logic        rst,
    
    // Input event (from FIFO)
    input  logic        in_valid,                // Data available
    input  logic [8:0]  in_x,                    // X [0-319], unsigned
    input  logic [8:0]  in_y,                    // Y [0-319], unsigned
    input  logic        in_polarity,             // 1=ON, 0=OFF
    output logic        in_ready,                // Consumed this cycle
    
    // Output compressed coordinates
    output logic signed [GRID_BITS:0] out_x,     // X [-8 to +7], signed
    output logic signed [GRID_BITS:0] out_y,     // Y [-8 to +7], signed
    output logic        out_polarity,            // Polarity passthrough
    output logic        out_valid                // Output valid pulse
);

    // -------------------------------------------------------------------------
    // Multiplication constants for shift-based division
    // grid = (sensor × MULT_FACTOR) >> SHIFT_AMOUNT
    // For 320→16: factor=13, shift=8 gives grid = sensor/20 (approx)
    // -------------------------------------------------------------------------
    localparam integer MULT_FACTOR  = 13;
    localparam integer SHIFT_AMOUNT = 8;
    localparam integer CENTER       = GRID_SIZE / 2;  // 8 for 16×16
    
    // -------------------------------------------------------------------------
    // Combinational compression (single-cycle, event-driven)
    // -------------------------------------------------------------------------
    
    // Multiplication: 9-bit × 4-bit = 13-bit result
    wire [12:0] mult_x = in_x * MULT_FACTOR[4:0];
    wire [12:0] mult_y = in_y * MULT_FACTOR[4:0];
    
    // Shift and extract grid coordinate [0-15]
    // Take bits [11:8] after implicit >>8
    wire [GRID_BITS-1:0] grid_x_raw = mult_x[11:SHIFT_AMOUNT];
    wire [GRID_BITS-1:0] grid_y_raw = mult_y[11:SHIFT_AMOUNT];
    
    // Clamp to valid range [0, GRID_SIZE-1] for overflow protection
    wire [GRID_BITS-1:0] grid_x = (mult_x[12:SHIFT_AMOUNT] >= GRID_SIZE) ? 
                                   (GRID_SIZE - 1) : grid_x_raw;
    wire [GRID_BITS-1:0] grid_y = (mult_y[12:SHIFT_AMOUNT] >= GRID_SIZE) ? 
                                   (GRID_SIZE - 1) : grid_y_raw;
    
    // Convert to signed position relative to center
    // signed_pos = unsigned_grid - CENTER
    wire signed [GRID_BITS:0] pos_x = $signed({1'b0, grid_x}) - $signed(CENTER);
    wire signed [GRID_BITS:0] pos_y = $signed({1'b0, grid_y}) - $signed(CENTER);
    
    // -------------------------------------------------------------------------
    // Output registration (single-cycle latency)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            out_x        <= '0;
            out_y        <= '0;
            out_polarity <= '0;
            out_valid    <= '0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                out_x        <= pos_x;
                out_y        <= pos_y;
                out_polarity <= in_polarity;
            end
        end
    end
    
    // Combinational ready (always accepts when valid)
    assign in_ready = in_valid;

endmodule
