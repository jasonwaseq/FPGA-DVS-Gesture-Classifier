module cocotb_iverilog_dump();
initial begin
    $dumpfile("sim_build_uart_rx/uart_rx.fst");
    $dumpvars(0, uart_rx);
end
endmodule
