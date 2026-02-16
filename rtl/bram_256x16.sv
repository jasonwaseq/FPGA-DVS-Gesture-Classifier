`timescale 1ns/1ps

// =============================================================================
// Module: bram_256x16
// =============================================================================
// Inferrable Dual-Port Block RAM for iCE40 FPGA
// 
// Specifications:
//   - Depth: 256 words
//   - Width: 16 bits per word
//   - Total: 4 kbits (fits single iCE40 BRAM block)
//   - Port A: Read/Write (for event timestamp updates)
//   - Port B: Read-only (for feature extraction scanning)
//
// Memory Inference:
//   - Uses synchronous read/write for BRAM inference
//   - No reset on memory contents (BRAM limitation)
//   - Read-first behavior on Port A
// =============================================================================

module bram_256x16 (
    input  logic        clk,
    
    // Port A: Read/Write (Event Update)
    input  logic        we_a,           // Write enable
    input  logic [7:0]  addr_a,         // Address [0-255]
    input  logic [15:0] din_a,          // Write data (timestamp)
    output logic [15:0] dout_a,         // Read data
    
    // Port B: Read-Only (Feature Extraction)
    input  logic [7:0]  addr_b,         // Address [0-255]
    output logic [15:0] dout_b          // Read data (timestamp)
);

    // -------------------------------------------------------------------------
    // Memory Array (infers BRAM on iCE40)
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) logic [15:0] mem [0:255];

`ifndef SYNTHESIS
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 256; init_i = init_i + 1) begin
            mem[init_i] = 16'h8000;
        end
    end
`endif
    
    // -------------------------------------------------------------------------
    // Port A: Synchronous Read/Write
    // Write-first behavior: new data available on next cycle after write
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
        dout_a <= mem[addr_a];
    end
    
    // -------------------------------------------------------------------------
    // Port B: Synchronous Read-Only
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        dout_b <= mem[addr_b];
    end

endmodule
