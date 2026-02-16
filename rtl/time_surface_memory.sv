`timescale 1ns/1ps

// =============================================================================
// Module: time_surface_memory
// =============================================================================
// Time-Surface Memory for DVS Event Processing
//
// Concept:
//   Stores the timestamp of the most recent event at each pixel location.
//   The "surface value" decays over time: S(p,t) = e^(-(t - t_last)/tau)
//   Hardware approximation: Value = MAX - (T_now - T_last), clamped to [0, MAX]
//
// Implementation:
//   - 16x16 grid = 256 cells
//   - Each cell stores a 16-bit timestamp
//   - Decay computed on read: Value = MAX - min(T_now - T_stored, MAX)
//   - Uses dual-port BRAM: Port A for writes, Port B for read scanning
//
// Benefits:
//   - No explicit decay counter/sweep needed
//   - Decay computed lazily on read
//   - Moving objects create "comet tails" of high values
//   - Sporadic noise decays quickly (natural filtering)
// =============================================================================

module time_surface_memory #(
    parameter GRID_SIZE   = 16,             // Grid dimension (16x16)
    parameter ADDR_BITS   = 8,              // log2(GRID_SIZE^2) = 8
    parameter TS_BITS     = 16,             // Timestamp bits
    parameter VALUE_BITS  = 8,              // Decay value output bits
    parameter MAX_VALUE   = 255,            // Maximum surface value
    parameter DECAY_SHIFT = 6               // Decay rate: larger = slower decay
)(
    input  logic                    clk,
    input  logic                    rst,
    
    // Global Timestamp (from system timer)
    input  logic [TS_BITS-1:0]      t_now,
    
    // Event Write Interface (from decoder)
    input  logic                    event_valid,
    input  logic [3:0]              event_x,        // Grid X [0-15]
    input  logic [3:0]              event_y,        // Grid Y [0-15]
    input  logic [TS_BITS-1:0]      event_ts,       // Event timestamp
    
    // Read Interface (for feature extraction)
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,      // Linear address [0-255]
    output logic [VALUE_BITS-1:0]   read_value,     // Decayed surface value
    output logic [TS_BITS-1:0]      read_ts_raw     // Raw timestamp (for debug)
);

    // -------------------------------------------------------------------------
    // Address Computation for Write
    // Linear address = y * GRID_SIZE + x = {y, x} for 16x16 grid
    // -------------------------------------------------------------------------
    wire [ADDR_BITS-1:0] write_addr = {event_y, event_x};
    
    // -------------------------------------------------------------------------
    // Dual-Port BRAM Instance
    // Port A: Write (event updates)
    // Port B: Read (feature extraction scanning)
    // -------------------------------------------------------------------------
    logic [TS_BITS-1:0] bram_dout_a;
    logic [TS_BITS-1:0] bram_dout_b;
    
    bram_256x16 u_bram (
        .clk     (clk),
        
        // Port A: Write
        .we_a    (event_valid),
        .addr_a  (write_addr),
        .din_a   (event_ts),
        .dout_a  (bram_dout_a),
        
        // Port B: Read
        .addr_b  (read_addr),
        .dout_b  (bram_dout_b)
    );
    
    // -------------------------------------------------------------------------
    // Raw Timestamp Output
    // -------------------------------------------------------------------------
    assign read_ts_raw = bram_dout_b;
    
    // -------------------------------------------------------------------------
    // Decay Computation (on read)
    // 
    // Time difference: delta_t = T_now - T_stored
    // Decayed value: Value = MAX - (delta_t >> DECAY_SHIFT)
    // Clamped to [0, MAX_VALUE]
    //
    // DECAY_SHIFT controls decay rate:
    //   - Larger shift = slower decay
    //   - At 12MHz, shift=6 gives ~5ms to decay from max to zero
    // -------------------------------------------------------------------------
    logic [TS_BITS-1:0] delta_t;
    logic [TS_BITS-1:0] decay_amount;
    logic [VALUE_BITS:0] value_calc;  // Extra bit for overflow detection
    
    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else if (read_enable) begin
            // Compute time difference (handles wrap-around naturally)
            delta_t = t_now - bram_dout_b;
            
            // Scale decay by shift (larger shift = slower decay)
            decay_amount = delta_t >> DECAY_SHIFT;
            
            // Compute value with saturation
            if (decay_amount >= MAX_VALUE) begin
                read_value <= 8'd0;
            end else begin
                read_value <= MAX_VALUE - decay_amount[VALUE_BITS-1:0];
            end
        end
    end

endmodule
