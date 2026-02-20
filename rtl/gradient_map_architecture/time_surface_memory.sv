`timescale 1ns/1ps

// =============================================================================
// Module: time_surface_memory
// =============================================================================
// Time-Surface Memory for DVS Event Processing
//
// Stores the timestamp of the most recent event at each grid cell.
// Decay computed lazily on read: Value = MAX >> (Δt >> DECAY_SHIFT)
// (True binary-exponential decay via shift approximation)
//
// A per-cell `cell_valid` flag prevents unwritten cells (or cells cleared by
// reset) from producing phantom non-zero values when the 16-bit timestamp
// wraps around.
//
// Storage: Inlined logic array (no external BRAM module dependency).
// Dual-port semantics:
//   Port A: Synchronous write (event updates) — 1-cycle write
//   Port B: Synchronous read (scan) — 1-cycle read latency
//
// Parameterized for any GRID_SIZE (default 16×16 = 256 cells for ice40up5k).
// =============================================================================

module time_surface_memory #(
    parameter GRID_SIZE   = 16,             // Grid dimension (16×16)
    parameter ADDR_BITS   = 8,              // log2(GRID_SIZE^2) = 8 for 256 cells
    parameter TS_BITS     = 16,             // Timestamp bits
    parameter VALUE_BITS  = 8,              // Decay value output bits
    parameter MAX_VALUE   = 255,            // Maximum surface value
    parameter DECAY_SHIFT = 6               // Half-life = 2^DECAY_SHIFT ticks
)(
    input  logic                    clk,
    input  logic                    rst,

    // Global Timestamp (from system timer)
    input  logic [TS_BITS-1:0]      t_now,

    // Event Write Interface (from decoder)
    input  logic                    event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0] event_x,   // Grid X [0..GRID_SIZE-1]
    input  logic [$clog2(GRID_SIZE)-1:0] event_y,   // Grid Y [0..GRID_SIZE-1]
    input  logic [TS_BITS-1:0]      event_ts,       // Event timestamp

    // Read Interface (for feature extraction)
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,      // Linear address [0..GRID_SIZE²-1]
    output logic [VALUE_BITS-1:0]   read_value,     // Decayed surface value
    output logic [TS_BITS-1:0]      read_ts_raw,    // Raw timestamp (for debug)
    output logic                    read_cell_valid // Cell has been written at least once
);

    localparam NUM_CELLS = GRID_SIZE * GRID_SIZE;

    // -------------------------------------------------------------------------
    // Timestamp Memory + Per-Cell Valid Flags
    // -------------------------------------------------------------------------
    logic [TS_BITS-1:0] mem [0:NUM_CELLS-1];
    logic               cell_valid [0:NUM_CELLS-1];

    // Initialize all cells as invalid so they read as zero until written.
    integer init_i;
    initial begin
        for (init_i = 0; init_i < NUM_CELLS; init_i = init_i + 1) begin
            mem[init_i]        = '0;
            cell_valid[init_i] = 1'b0;
        end
    end

    // Linear write address = y * GRID_SIZE + x
    wire [ADDR_BITS-1:0] write_addr = ADDR_BITS'(event_y) * ADDR_BITS'(GRID_SIZE) +
                                       ADDR_BITS'(event_x);

    // Port A: Synchronous write — mark cell as valid
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int wi = 0; wi < NUM_CELLS; wi = wi + 1)
                cell_valid[wi] <= 1'b0;
        end else if (event_valid) begin
            mem[write_addr]        <= event_ts;
            cell_valid[write_addr] <= 1'b1;
        end
    end

    // Port B: Synchronous read (1-cycle latency)
    logic [TS_BITS-1:0] bram_dout;
    logic               bram_valid;
    always_ff @(posedge clk) begin
        bram_dout  <= mem[read_addr];
        bram_valid <= cell_valid[read_addr];
    end

    // -------------------------------------------------------------------------
    // Raw Timestamp Output (registered)
    // -------------------------------------------------------------------------
    assign read_ts_raw    = bram_dout;
    assign read_cell_valid = bram_valid;

    // -------------------------------------------------------------------------
    // Decay Computation (registered — second pipeline stage)
    // Total read latency: 1 (BRAM) + 1 (decay) = 2 cycles
    //
    // Binary exponential decay: value = MAX_VALUE >> (Δt >> DECAY_SHIFT)
    //   e.g. DECAY_SHIFT=6 → half-life = 64 ticks = ~5.3 µs @ 12 MHz
    //
    // Cells not yet written always produce 0 (no timestamp wraparound
    // artifacts from uninitialized or cleared cells).
    // -------------------------------------------------------------------------
    logic [TS_BITS-1:0]  delta_t;
    logic [7:0]          decay_steps;   // Number of half-lives elapsed

    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else if (read_enable) begin
            if (!bram_valid) begin
                // Cell never written — output zero
                read_value <= '0;
            end else begin
                delta_t    = t_now - bram_dout;
                decay_steps = TS_BITS'(delta_t >> DECAY_SHIFT);

                // Saturate: after 8 half-lives value is effectively 0
                if (decay_steps >= 8)
                    read_value <= 8'd0;
                else
                    read_value <= VALUE_BITS'(MAX_VALUE >> decay_steps);
            end
        end
    end

endmodule

