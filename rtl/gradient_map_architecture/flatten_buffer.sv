`timescale 1ns/1ps

// =============================================================================
// Module: flatten_buffer
// =============================================================================
// Spatio-Temporal Surface Flattener
//
// On a `start` pulse, this module scans all NUM_CELLS time-surface cells in
// row-major order (addr = y*GRID_SIZE + x), reads the exponentially-decayed
// 8-bit value for each cell, and fills a parallel 1-D vector register.
//
// The read pipeline has a 2-cycle latency (BRAM + decay register), so the
// address issued to the time-surface is pre-fetched 2 cycles ahead, matching
// the same strategy used in feature_extractor.sv.
//
// Output:
//   flat_valid     - one-cycle pulse when all NUM_CELLS bytes are captured
//   flat_data[i]   - decayed value for cell i (i = y*GRID_SIZE + x)
//
// Parameters:
//   GRID_SIZE  - Grid dimension (16)
//   VALUE_BITS - Bits per cell value (8)
//   NUM_CELLS  - Total cells = GRID_SIZE^2 (256)
//   PIPE_DEPTH - Read pipeline latency in cycles (2)
// =============================================================================

module flatten_buffer #(
    parameter GRID_SIZE  = 16,
    parameter VALUE_BITS = 8,
    parameter NUM_CELLS  = 256,   // GRID_SIZE * GRID_SIZE
    parameter PIPE_DEPTH = 2      // BRAM + decay register latency
)(
    input  logic                          clk,
    input  logic                          rst,

    // Trigger
    input  logic                          start,   // One-cycle pulse to begin scan

    // Time-Surface Read Interface
    output logic [$clog2(NUM_CELLS)-1:0]  ts_addr,
    output logic                          ts_en,
    input  logic [VALUE_BITS-1:0]         ts_val,

    // Output Vector
    output logic                          flat_valid,
    output logic [NUM_CELLS*VALUE_BITS-1:0] flat_data
);

    // Internal unpacked array
    logic [VALUE_BITS-1:0] flat_data_int [0:NUM_CELLS-1];

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE   = 2'd0,
        S_FILL   = 2'd1,   // Pipeline fill (PIPE_DEPTH cycles, no write-back)
        S_SCAN   = 2'd2,   // Full scan + capture
        S_DONE   = 2'd3
    } state_t;

    state_t state;

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    // issue_addr: the address currently being driven to ts_addr (pre-fetched)
    // capture_idx: the flat_data index being written this cycle
    localparam CNT_BITS = $clog2(NUM_CELLS + PIPE_DEPTH + 2);

    logic [CNT_BITS-1:0] issue_cnt;    // How many addresses have been issued
    logic [CNT_BITS-1:0] capture_cnt;  // How many values have been captured

    // -------------------------------------------------------------------------
    // FSM + Data Capture
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            issue_cnt   <= '0;
            capture_cnt <= '0;
            ts_addr     <= '0;
            ts_en       <= 1'b0;
            flat_valid  <= 1'b0;
        end else begin
            flat_valid <= 1'b0;  // Default: deassert

            case (state)
                // -----------------------------------------------------------
                // IDLE: wait for start pulse, pre-issue first address
                // -----------------------------------------------------------
                S_IDLE: begin
                    if (start) begin
                        issue_cnt   <= CNT_BITS'(1);
                        capture_cnt <= '0;
                        ts_addr     <= '0;
                        ts_en       <= 1'b1;
                        state       <= S_FILL;
                    end
                end

                // -----------------------------------------------------------
                // FILL: issue addresses 1..(PIPE_DEPTH-1) to prime pipeline
                //       No data is valid yet
                // -----------------------------------------------------------
                S_FILL: begin
                    if (issue_cnt < CNT_BITS'(NUM_CELLS)) begin
                        ts_addr   <= $clog2(NUM_CELLS)'(issue_cnt);
                        ts_en     <= 1'b1;
                        issue_cnt <= issue_cnt + 1'b1;
                    end else begin
                        ts_en <= 1'b0;
                    end

                    // After PIPE_DEPTH cycles the first value is valid
                    // (capture_cnt is used as a fill counter here)
                    capture_cnt <= capture_cnt + 1'b1;
                    if (capture_cnt == CNT_BITS'(PIPE_DEPTH - 1)) begin
                        state <= S_SCAN;
                        capture_cnt <= '0;
                    end
                end

                // -----------------------------------------------------------
                // SCAN: each cycle, value for capture_cnt is valid on ts_val
                //       Continue pre-issuing addresses until all issued
                // -----------------------------------------------------------
                S_SCAN: begin
                    // Capture current valid value
                    flat_data_int[capture_cnt[$clog2(NUM_CELLS)-1:0]] <= ts_val;
                    capture_cnt <= capture_cnt + 1'b1;

                    // Continue issuing addresses ahead (capped at NUM_CELLS-1)
                    if (issue_cnt < CNT_BITS'(NUM_CELLS)) begin
                        ts_addr   <= $clog2(NUM_CELLS)'(issue_cnt);
                        ts_en     <= 1'b1;
                        issue_cnt <= issue_cnt + 1'b1;
                    end else begin
                        ts_en   <= 1'b0;
                        ts_addr <= '0;
                    end

                    // Done when all NUM_CELLS captured
                    if (capture_cnt == CNT_BITS'(NUM_CELLS - 1)) begin
                        state <= S_DONE;
                    end
                end

                // -----------------------------------------------------------
                // DONE: assert flat_valid for one cycle, return to IDLE
                // -----------------------------------------------------------
                S_DONE: begin
                    flat_valid <= 1'b1;
                    ts_en      <= 1'b0;
                    issue_cnt  <= '0;
                    capture_cnt <= '0;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Convert unpacked array to packed array for output
    genvar i;
    generate
        for (i = 0; i < NUM_CELLS; i = i + 1) begin : gen_pack
            assign flat_data[i*VALUE_BITS +: VALUE_BITS] = flat_data_int[i];
        end
    endgenerate

endmodule
