// =============================================================================
// Module: InputFIFO
// =============================================================================
// Asynchronous-style shallow FIFO for DVS event buffering
// Handles bursty event arrivals with minimal latency
//
// Features:
//   - Event-driven push (combinational ready signal)
//   - Single-cycle pop latency
//   - Power-of-2 depth for efficient pointer wrapping
//
// Data Format (35 bits per entry):
//   [34:26] = x[8:0]      - X coordinate, unsigned, range [0-319]
//   [25:17] = y[8:0]      - Y coordinate, unsigned, range [0-319]
//   [16]    = polarity    - 1=ON (brightness increase), 0=OFF (decrease)
//   [15:0]  = timestamp   - Optional timing information
// =============================================================================

module InputFIFO #(
    parameter DEPTH    = 16,                     // FIFO depth (must be power of 2)
    parameter PTR_BITS = 4                       // Pointer width = log2(DEPTH)
)(
    input  logic        clk,
    input  logic        rst,
    
    // Push interface (DVS event input)
    input  logic        push_valid,              // Event available
    input  logic [8:0]  push_x,                  // X coordinate [0-319]
    input  logic [8:0]  push_y,                  // Y coordinate [0-319]
    input  logic        push_polarity,           // 1=ON, 0=OFF
    input  logic [15:0] push_ts,                 // Timestamp
    output logic        push_ready,              // Can accept (combinational)
    
    // Pop interface (to processing pipeline)
    input  logic        pop_req,                 // Pop request
    output logic [8:0]  pop_x,                   // X coordinate out
    output logic [8:0]  pop_y,                   // Y coordinate out
    output logic        pop_polarity,            // Polarity out
    output logic [15:0] pop_ts,                  // Timestamp out
    
    // Status flags
    output logic        empty,                   // FIFO empty
    output logic        full                     // FIFO full
);

    // -------------------------------------------------------------------------
    // Storage: Packed vector (35 bits per entry)
    //   [34:26] = x[8:0]      - X coordinate
    //   [25:17] = y[8:0]      - Y coordinate
    //   [16]    = polarity    - ON/OFF flag
    //   [15:0]  = ts[15:0]    - Timestamp
    // -------------------------------------------------------------------------
    logic [34:0] fifo_mem [0:DEPTH-1];           // Storage array
    
    // -------------------------------------------------------------------------
    // Pointers and count (extra bit for full/empty disambiguation)
    // -------------------------------------------------------------------------
    logic [PTR_BITS:0] wr_ptr;                   // Write pointer [0 to 2*DEPTH-1]
    logic [PTR_BITS:0] rd_ptr;                   // Read pointer [0 to 2*DEPTH-1]
    logic [PTR_BITS:0] count;                    // Entry count [0 to DEPTH]
    
    // -------------------------------------------------------------------------
    // Combinational status (asynchronous response)
    // -------------------------------------------------------------------------
    assign empty      = (count == 0);
    assign full       = (count == DEPTH);
    assign push_ready = !full;                   // Accept if not full
    
    // -------------------------------------------------------------------------
    // Output: Combinational read from memory (bit slicing)
    // -------------------------------------------------------------------------
    wire [PTR_BITS-1:0] rd_addr = rd_ptr[PTR_BITS-1:0];
    assign pop_x        = fifo_mem[rd_addr][34:26];      // x[8:0]
    assign pop_y        = fifo_mem[rd_addr][25:17];      // y[8:0]
    assign pop_polarity = fifo_mem[rd_addr][16];         // polarity
    assign pop_ts       = fifo_mem[rd_addr][15:0];       // ts[15:0]
    
    // -------------------------------------------------------------------------
    // Push/Pop control
    // -------------------------------------------------------------------------
    wire do_push = push_valid && !full;
    wire do_pop  = pop_req && !empty;
    
    // -------------------------------------------------------------------------
    // Write logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= '0;
        end else if (do_push) begin
            // Pack fields into 35-bit vector: {x[8:0], y[8:0], polarity, ts[15:0]}
            fifo_mem[wr_ptr[PTR_BITS-1:0]] <= {push_x, push_y, push_polarity, push_ts};
            wr_ptr <= wr_ptr + 1'b1;
        end
    end
    
    // -------------------------------------------------------------------------
    // Read pointer update
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            rd_ptr <= '0;
        end else if (do_pop) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end
    
    // -------------------------------------------------------------------------
    // Count tracking
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            count <= '0;
        end else begin
            case ({do_push, do_pop})
                2'b10:   count <= count + 1'b1;  // Push only
                2'b01:   count <= count - 1'b1;  // Pop only
                default: count <= count;          // Both or neither
            endcase
        end
    end

endmodule
