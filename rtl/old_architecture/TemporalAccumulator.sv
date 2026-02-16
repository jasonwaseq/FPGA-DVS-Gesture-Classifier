// =============================================================================
// Module: TemporalAccumulator
// =============================================================================
// Maintains dual sliding window accumulators for centroid computation
//
// Architecture:
//   - Two accumulator sets: "early" and "late" window halves
//   - Each tracks: Σx, Σy, count (total and per-polarity)
//   - Window slides at configurable rate (default 200ms per half)
//
// Accumulator Arithmetic (all integer):
//   - sum_x/sum_y: Signed 18-bit, holds sum of positions [-8,+7] × count
//   - count: Unsigned 12-bit, max 4095 events per half-window
//
// Polarity Handling:
//   - Separate accumulators for ON (polarity=1) and OFF (polarity=0) events
//   - Combined totals also maintained for overall centroid
//
// Window Sliding Mechanism:
//   - Timer counts cycles for each half-window (200ms default)
//   - At half-window boundary: phase toggles
//   - At full window boundary: early ← late, late ← 0
//   - This implements continuous sliding observation window
//
// Fixed-Point Arithmetic:
//   - Position values: Q0.4 (4 fractional bits implied by grid mapping)
//   - Sum values: Q0.4 accumulated (actual centroid = sum/count)
// =============================================================================

module TemporalAccumulator #(
    parameter CLK_FREQ_HZ     = 12_000_000,       // System clock frequency
    parameter HALF_WINDOW_MS  = 200,              // Half-window duration (ms)
    parameter CYCLES_PER_HALF = 2_400_000,        // Cycles per half-window
    parameter TIMER_BITS      = 22,               // Timer bit width
    parameter GRID_BITS       = 4,                // Grid coordinate bits
    parameter ACC_SUM_BITS    = 18,               // Accumulator sum bits (signed)
    parameter ACC_COUNT_BITS  = 12                // Accumulator count bits (unsigned)
)(
    input  logic        clk,
    input  logic        rst,
    
    // Event input (from SpatialCompressor)
    input  logic        event_valid,
    input  logic signed [GRID_BITS:0] event_x,   // [-8 to +7], signed
    input  logic signed [GRID_BITS:0] event_y,   // [-8 to +7], signed
    input  logic        event_polarity,          // 1=ON, 0=OFF
    
    // Window timing outputs
    output logic        window_phase,            // 0=early half, 1=late half
    output logic        window_complete,         // Pulse at full window end
    
    // Accumulator snapshots (captured at window_complete)
    output logic signed [ACC_SUM_BITS-1:0] early_sum_x,   // Σx for early window
    output logic signed [ACC_SUM_BITS-1:0] early_sum_y,   // Σy for early window
    output logic [ACC_COUNT_BITS-1:0]      early_count,   // Event count, early
    output logic signed [ACC_SUM_BITS-1:0] late_sum_x,    // Σx for late window
    output logic signed [ACC_SUM_BITS-1:0] late_sum_y,    // Σy for late window
    output logic [ACC_COUNT_BITS-1:0]      late_count,    // Event count, late
    output logic                           accum_valid,   // Snapshot ready pulse
    
    // Debug
    output logic [7:0]  debug_event_count                 // Events in current window
);

    // -------------------------------------------------------------------------
    // Window Timer
    // -------------------------------------------------------------------------
    logic [TIMER_BITS-1:0] timer;
    logic                  phase_advance;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            timer           <= '0;
            window_phase    <= 1'b0;
            phase_advance   <= 1'b0;
            window_complete <= 1'b0;
        end else begin
            phase_advance   <= 1'b0;
            window_complete <= 1'b0;
            
            if (timer >= CYCLES_PER_HALF - 1) begin
                timer         <= '0;
                phase_advance <= 1'b1;
                
                if (window_phase == 1'b1) begin
                    // Completed late phase -> full window done
                    window_complete <= 1'b1;
                    window_phase    <= 1'b0;
                end else begin
                    // Completed early phase -> move to late
                    window_phase <= 1'b1;
                end
            end else begin
                timer <= timer + 1'b1;
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Live Accumulators (continuously updated)
    // -------------------------------------------------------------------------
    
    // Early window (combined polarity)
    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x, live_early_sum_y;
    logic [ACC_COUNT_BITS-1:0]      live_early_count;
    
    // Early window (ON polarity only)
    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x_on, live_early_sum_y_on;
    logic [ACC_COUNT_BITS-1:0]      live_early_count_on;
    
    // Early window (OFF polarity only)
    logic signed [ACC_SUM_BITS-1:0] live_early_sum_x_off, live_early_sum_y_off;
    logic [ACC_COUNT_BITS-1:0]      live_early_count_off;
    
    // Late window (combined polarity)
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x, live_late_sum_y;
    logic [ACC_COUNT_BITS-1:0]      live_late_count;
    
    // Late window (ON polarity only)
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x_on, live_late_sum_y_on;
    logic [ACC_COUNT_BITS-1:0]      live_late_count_on;
    
    // Late window (OFF polarity only)
    logic signed [ACC_SUM_BITS-1:0] live_late_sum_x_off, live_late_sum_y_off;
    logic [ACC_COUNT_BITS-1:0]      live_late_count_off;
    
    // Event counter for debug
    logic [7:0] event_cnt;
    assign debug_event_count = event_cnt;
    
    // Sign-extended position for accumulation
    // Extend from (GRID_BITS+1) bits to ACC_SUM_BITS bits, preserving sign
    wire signed [ACC_SUM_BITS-1:0] ext_x = {{(ACC_SUM_BITS-GRID_BITS-1){event_x[GRID_BITS]}}, event_x};
    wire signed [ACC_SUM_BITS-1:0] ext_y = {{(ACC_SUM_BITS-GRID_BITS-1){event_y[GRID_BITS]}}, event_y};
    
    // -------------------------------------------------------------------------
    // Accumulation Logic (event-driven updates)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            // Reset all live accumulators
            live_early_sum_x     <= '0; live_early_sum_y     <= '0; live_early_count     <= '0;
            live_early_sum_x_on  <= '0; live_early_sum_y_on  <= '0; live_early_count_on  <= '0;
            live_early_sum_x_off <= '0; live_early_sum_y_off <= '0; live_early_count_off <= '0;
            live_late_sum_x      <= '0; live_late_sum_y      <= '0; live_late_count      <= '0;
            live_late_sum_x_on   <= '0; live_late_sum_y_on   <= '0; live_late_count_on   <= '0;
            live_late_sum_x_off  <= '0; live_late_sum_y_off  <= '0; live_late_count_off  <= '0;
            event_cnt            <= '0;
        end else begin
            // Window slide: late → early, clear late
            if (window_complete) begin
                live_early_sum_x     <= live_late_sum_x;
                live_early_sum_y     <= live_late_sum_y;
                live_early_count     <= live_late_count;
                live_early_sum_x_on  <= live_late_sum_x_on;
                live_early_sum_y_on  <= live_late_sum_y_on;
                live_early_count_on  <= live_late_count_on;
                live_early_sum_x_off <= live_late_sum_x_off;
                live_early_sum_y_off <= live_late_sum_y_off;
                live_early_count_off <= live_late_count_off;
                
                live_late_sum_x      <= '0;
                live_late_sum_y      <= '0;
                live_late_count      <= '0;
                live_late_sum_x_on   <= '0;
                live_late_sum_y_on   <= '0;
                live_late_count_on   <= '0;
                live_late_sum_x_off  <= '0;
                live_late_sum_y_off  <= '0;
                live_late_count_off  <= '0;
                
                event_cnt <= '0;
            end
            // Event accumulation (asynchronous to window timing)
            else if (event_valid) begin
                if (event_cnt < 8'hFF) event_cnt <= event_cnt + 1'b1;
                
                if (window_phase == 1'b0) begin
                    // Early phase: accumulate into early window
                    live_early_sum_x <= live_early_sum_x + ext_x;
                    live_early_sum_y <= live_early_sum_y + ext_y;
                    live_early_count <= live_early_count + 1'b1;
                    
                    if (event_polarity) begin
                        live_early_sum_x_on  <= live_early_sum_x_on + ext_x;
                        live_early_sum_y_on  <= live_early_sum_y_on + ext_y;
                        live_early_count_on  <= live_early_count_on + 1'b1;
                    end else begin
                        live_early_sum_x_off <= live_early_sum_x_off + ext_x;
                        live_early_sum_y_off <= live_early_sum_y_off + ext_y;
                        live_early_count_off <= live_early_count_off + 1'b1;
                    end
                end else begin
                    // Late phase: accumulate into late window
                    live_late_sum_x <= live_late_sum_x + ext_x;
                    live_late_sum_y <= live_late_sum_y + ext_y;
                    live_late_count <= live_late_count + 1'b1;
                    
                    if (event_polarity) begin
                        live_late_sum_x_on  <= live_late_sum_x_on + ext_x;
                        live_late_sum_y_on  <= live_late_sum_y_on + ext_y;
                        live_late_count_on  <= live_late_count_on + 1'b1;
                    end else begin
                        live_late_sum_x_off <= live_late_sum_x_off + ext_x;
                        live_late_sum_y_off <= live_late_sum_y_off + ext_y;
                        live_late_count_off <= live_late_count_off + 1'b1;
                    end
                end
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Snapshot Logic (capture at window completion)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            early_sum_x  <= '0; early_sum_y  <= '0; early_count  <= '0;
            late_sum_x   <= '0; late_sum_y   <= '0; late_count   <= '0;
            accum_valid  <= 1'b0;
        end else begin
            accum_valid <= 1'b0;
            
            if (window_complete) begin
                // Capture current values before sliding
                early_sum_x <= live_early_sum_x;
                early_sum_y <= live_early_sum_y;
                early_count <= live_early_count;
                late_sum_x  <= live_late_sum_x;
                late_sum_y  <= live_late_sum_y;
                late_count  <= live_late_count;
                accum_valid <= 1'b1;
            end
        end
    end

endmodule
