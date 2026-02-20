`timescale 1ns/1ps

// =============================================================================
// Module: time_surface_encoder
// =============================================================================
// Exponential-Decay Time Surface for DVS Event Processing
//
// Concept:
//   Stores the timestamp of the most recent event at each grid cell.
//   The "surface value" decays exponentially over time:
//       S(x,y,t) = 255 · 2^(–Δt / τ)
//   where Δt = t_now – t_last_event and τ = 2^DECAY_SHIFT clock ticks.
//
// Hardware Implementation:
//   - On read: decay_steps = Δt >> DECAY_SHIFT  (number of half-lives elapsed)
//   - Decayed value  = MAX_VALUE >> decay_steps  (right shift = multiply by 0.5^n)
//   - If decay_steps >= VALUE_BITS, output = 0   (fully decayed)
//
// This gives exactly S = 255 · (0.5)^n where n is the number of half-lives,
// matching an exponential with time constant τ = 2^DECAY_SHIFT / ln(2).
//
// Interface:
//   Identical to time_surface_memory so it can be used as a drop-in replacement.
//   Internally instantiates time_surface_memory for BRAM storage, and overrides
//   the decay computation with the exponential version.
//
// Parameters:
//   GRID_SIZE    - Grid dimension (16 → 16×16 = 256 cells for ice40up5k)
//   ADDR_BITS    - log2(GRID_SIZE^2) = 8
//   TS_BITS      - Timestamp bits (16)
//   VALUE_BITS   - Output decay value bits (8)
//   MAX_VALUE    - Maximum surface value (255)
//   DECAY_SHIFT  - log2(half-life in clocks); e.g. 6 → τ ≈ 5 μs at 12 MHz
// =============================================================================

module time_surface_encoder #(
    parameter GRID_SIZE   = 16,
    parameter ADDR_BITS   = 8,
    parameter TS_BITS     = 16,
    parameter VALUE_BITS  = 8,
    parameter MAX_VALUE   = 255,
    parameter DECAY_SHIFT = 6       // Half-life = 2^DECAY_SHIFT clock ticks
)(
    input  logic                    clk,
    input  logic                    rst,

    // Global Timestamp
    input  logic [TS_BITS-1:0]      t_now,

    // Event Write Interface (from decoder)
    input  logic                              event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_x,   // Grid X coordinate
    input  logic [$clog2(GRID_SIZE)-1:0]      event_y,   // Grid Y coordinate
    input  logic [TS_BITS-1:0]                event_ts,

    // Read Interface (for flatten / feature extraction)
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,
    output logic [VALUE_BITS-1:0]   read_value,    // Exponentially decayed surface value
    output logic [TS_BITS-1:0]      read_ts_raw    // Raw stored timestamp (debug)
);

    // -------------------------------------------------------------------------
    // Raw timestamp BRAM (reuse existing time_surface_memory for storage)
    // We only use its raw timestamp output; we replace the decay computation.
    // -------------------------------------------------------------------------
    logic [TS_BITS-1:0]  raw_ts;
    logic [VALUE_BITS-1:0] ts_mem_value;  // Linear decay output (not used)

    time_surface_memory #(
        .GRID_SIZE  (GRID_SIZE),
        .ADDR_BITS  (ADDR_BITS),
        .TS_BITS    (TS_BITS),
        .VALUE_BITS (VALUE_BITS),
        .MAX_VALUE  (MAX_VALUE),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_ts_mem (
        .clk         (clk),
        .rst         (rst),
        .t_now       (t_now),
        .event_valid (event_valid),
        .event_x     (event_x),
        .event_y     (event_y),
        .event_ts    (event_ts),
        .read_enable (read_enable),
        .read_addr   (read_addr),
        .read_value  (ts_mem_value),  // Discard; we recompute below
        .read_ts_raw (raw_ts)
    );

    assign read_ts_raw = raw_ts;

    // -------------------------------------------------------------------------
    // Exponential Decay Computation (registered, matches BRAM pipeline latency)
    //
    // Pipeline stage 1 (combinatorial): compute Δt and decay_steps
    // Pipeline stage 2 (registered):    shift MAX_VALUE right by decay_steps
    //
    // Two registered stages maintain the same 2-cycle latency as
    // time_surface_memory so feature-scan pipeline timing is unchanged.
    // -------------------------------------------------------------------------

    // Stage 1 registers (capture BRAM output on same cycle it becomes valid)
    logic [TS_BITS-1:0]   delta_t_r1;
    logic [TS_BITS-1:0]   decay_steps_r1;

    always_ff @(posedge clk) begin
        if (rst) begin
            delta_t_r1    <= '0;
            decay_steps_r1 <= '0;
        end else if (read_enable) begin
            // Δt handles 16-bit wrap-around naturally via unsigned subtraction
            delta_t_r1    <= t_now - raw_ts;
            decay_steps_r1 <= (t_now - raw_ts) >> DECAY_SHIFT;
        end
    end

    // Stage 2: apply exponential decay via arithmetic right-shift
    // Value = 255 >> decay_steps  (255 when decay_steps=0, 127 after 1 half-life, …)
    // If decay_steps >= VALUE_BITS the value underflows to 0.
    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else begin
            if (decay_steps_r1 >= TS_BITS'(VALUE_BITS)) begin
                read_value <= '0;
            end else begin
                // right-shift by decay_steps (number of half-lives elapsed)
                read_value <= VALUE_BITS'(MAX_VALUE) >> decay_steps_r1[3:0];
            end
        end
    end

endmodule
