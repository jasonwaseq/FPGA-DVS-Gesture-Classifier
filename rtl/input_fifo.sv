`timescale 1ns/1ps

// =============================================================================
// Module: input_fifo
// =============================================================================
// Synchronous Ring Buffer FIFO for EVT 2.0 input buffering
//
// Specifications:
//   - Depth: 256 words (configurable)
//   - Width: 32 bits (EVT 2.0 word size)
//   - Memory: Infers iCE40 Block RAM for storage
//   - Interface: Standard wr_en, rd_en, full, empty flags
//
// Purpose:
//   Buffers bursty EVT 2.0 data from the sensor interface, providing
//   smooth data flow to the downstream decoder.
// =============================================================================

module input_fifo #(
    parameter DEPTH     = 256,          // FIFO depth (power of 2)
    parameter PTR_BITS  = 8,            // log2(DEPTH)
    parameter DATA_WIDTH = 32           // EVT 2.0 word width
)(
    input  logic                    clk,
    input  logic                    rst,
    
    // Write Interface (from sensor)
    input  logic                    wr_en,          // Write enable
    input  logic [DATA_WIDTH-1:0]   wr_data,        // EVT 2.0 word
    
    // Read Interface (to decoder)
    input  logic                    rd_en,          // Read enable
    output logic [DATA_WIDTH-1:0]   rd_data,        // EVT 2.0 word
    
    // Status Flags
    output logic                    empty,          // FIFO empty
    output logic                    full,           // FIFO full
    output logic [PTR_BITS:0]       count           // Number of entries
);

    // -------------------------------------------------------------------------
    // Memory Array (infers BRAM on iCE40)
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    
    // -------------------------------------------------------------------------
    // Pointers (extra MSB for wrap-around detection)
    // -------------------------------------------------------------------------
    logic [PTR_BITS:0] wr_ptr;          // Write pointer
    logic [PTR_BITS:0] rd_ptr;          // Read pointer
    
    // -------------------------------------------------------------------------
    // Pointer addresses (LSBs only for memory access)
    // -------------------------------------------------------------------------
    wire [PTR_BITS-1:0] wr_addr = wr_ptr[PTR_BITS-1:0];
    wire [PTR_BITS-1:0] rd_addr = rd_ptr[PTR_BITS-1:0];
    
    // -------------------------------------------------------------------------
    // Status Logic
    // -------------------------------------------------------------------------
    // Full: pointers differ only in MSB (wrapped)
    // Empty: pointers are identical
    assign full  = (wr_ptr[PTR_BITS] != rd_ptr[PTR_BITS]) && 
                   (wr_ptr[PTR_BITS-1:0] == rd_ptr[PTR_BITS-1:0]);
    assign empty = (wr_ptr == rd_ptr);
    assign count = wr_ptr - rd_ptr;
    
    // -------------------------------------------------------------------------
    // Write Control
    // -------------------------------------------------------------------------
    wire do_write = wr_en && !full;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= '0;
        end else if (do_write) begin
            mem[wr_addr] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end
    
    // -------------------------------------------------------------------------
    // Read Control (synchronous read for BRAM)
    // -------------------------------------------------------------------------
    wire do_read = rd_en && !empty;
    
    // Registered read data output (BRAM inference requirement)
    always_ff @(posedge clk) begin
        if (rst) begin
            rd_ptr <= '0;
            rd_data <= '0;
        end else begin
            // Output data from current read address
            rd_data <= mem[rd_addr];
            
            if (do_read) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end

endmodule
