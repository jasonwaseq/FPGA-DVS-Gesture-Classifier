`timescale 1ns/1ps

// Synchronous ring-buffer FIFO for EVT2.0 32-bit words.

module input_fifo #(
    parameter [31:0] width_p = 32,
    parameter [31:0] depth_log2_p = 8
)(  input  [0:0]         clk_i,
    input  [0:0]         reset_i,
    input  [width_p-1:0] data_i,
    input  [0:0]         ready_i,
    input  [0:0]         valid_i,
    output [0:0]         ready_o, 
    output [0:0]         valid_o,
    output [width_p-1:0] data_o 
);

    localparam int depth_p = (1 << depth_log2_p);

    logic [width_p-1:0] mem [0:depth_p-1];
    logic [depth_log2_p:0] wr_ptr;
    logic [depth_log2_p:0] rd_ptr;

    logic [0:0] full;
    logic [0:0] empty;
    logic [0:0] wr_en;
    logic [0:0] rd_en;

    integer i;
    initial begin
        for (i = 0; i < depth_p; i = i + 1)
            mem[i] = '0;
    end

    assign full  = (wr_ptr[depth_log2_p] ^ rd_ptr[depth_log2_p]) &&
                   (wr_ptr[depth_log2_p-1:0] == rd_ptr[depth_log2_p-1:0]);
    assign empty = (wr_ptr == rd_ptr);

    assign ready_o = ~full;
    assign valid_o = ~empty;

    assign wr_en = valid_i & ready_o;
    assign rd_en = valid_o & ready_i;

    // First-word-fall-through view of the current read pointer.
    assign data_o = mem[rd_ptr[depth_log2_p-1:0]];

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (wr_en) begin
                mem[wr_ptr[depth_log2_p-1:0]] <= data_i;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (rd_en) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end

endmodule
