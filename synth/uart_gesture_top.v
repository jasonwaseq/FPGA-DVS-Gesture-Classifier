module uart_rx (
	clk,
	rst,
	rx,
	data,
	valid
);
	parameter CLKS_PER_BIT = 104;
	input wire clk;
	input wire rst;
	input wire rx;
	output reg [7:0] data;
	output reg valid;
	localparam IDLE = 2'd0;
	localparam START = 2'd1;
	localparam DATA = 2'd2;
	localparam STOP = 2'd3;
	reg [1:0] state;
	reg [7:0] clk_cnt;
	reg [2:0] bit_idx;
	reg [7:0] rx_data;
	reg rx_sync;
	reg rx_d;
	always @(posedge clk)
		if (rst) begin
			rx_sync <= 1'b1;
			rx_d <= 1'b1;
		end
		else begin
			rx_sync <= rx;
			rx_d <= rx_sync;
		end
	always @(posedge clk)
		if (rst) begin
			state <= IDLE;
			clk_cnt <= 8'd0;
			bit_idx <= 3'd0;
			rx_data <= 8'd0;
			data <= 8'd0;
			valid <= 1'b0;
		end
		else begin
			valid <= 1'b0;
			case (state)
				IDLE: begin
					clk_cnt <= 8'd0;
					bit_idx <= 3'd0;
					if (rx_d == 1'b0)
						state <= START;
				end
				START:
					if (clk_cnt == ((CLKS_PER_BIT - 1) / 2)) begin
						if (rx_d == 1'b0) begin
							clk_cnt <= 8'd0;
							state <= DATA;
						end
						else
							state <= IDLE;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				DATA:
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						rx_data[bit_idx] <= rx_d;
						if (bit_idx == 3'd7) begin
							bit_idx <= 3'd0;
							state <= STOP;
						end
						else
							bit_idx <= bit_idx + 1'b1;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				STOP:
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= IDLE;
						if (rx_d == 1'b1) begin
							data <= rx_data;
							valid <= 1'b1;
						end
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				default: state <= IDLE;
			endcase
		end
endmodule
module uart_tx (
	clk,
	rst,
	data,
	valid,
	tx,
	busy
);
	parameter CLKS_PER_BIT = 104;
	input wire clk;
	input wire rst;
	input wire [7:0] data;
	input wire valid;
	output reg tx;
	output reg busy;
	localparam IDLE = 2'd0;
	localparam START = 2'd1;
	localparam DATA = 2'd2;
	localparam STOP = 2'd3;
	reg [1:0] state;
	reg [7:0] clk_cnt;
	reg [2:0] bit_idx;
	reg [7:0] tx_data;
	always @(posedge clk)
		if (rst) begin
			state <= IDLE;
			clk_cnt <= 8'd0;
			bit_idx <= 3'd0;
			tx_data <= 8'd0;
			tx <= 1'b1;
			busy <= 1'b0;
		end
		else
			case (state)
				IDLE: begin
					tx <= 1'b1;
					clk_cnt <= 8'd0;
					bit_idx <= 3'd0;
					busy <= 1'b0;
					if (valid) begin
						tx_data <= data;
						busy <= 1'b1;
						state <= START;
					end
				end
				START: begin
					tx <= 1'b0;
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= DATA;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				DATA: begin
					tx <= tx_data[bit_idx];
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						if (bit_idx == 3'd7) begin
							bit_idx <= 3'd0;
							state <= STOP;
						end
						else
							bit_idx <= bit_idx + 1'b1;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				STOP: begin
					tx <= 1'b1;
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= IDLE;
						busy <= 1'b0;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				default: state <= IDLE;
			endcase
endmodule
module event_accumulator (
	clk,
	rst,
	event_valid,
	event_x,
	event_y,
	event_polarity,
	compute_trigger,
	delta_x,
	delta_y,
	delta_valid
);
	parameter X_BITS = 7;
	parameter Y_BITS = 7;
	parameter ACC_BITS = 16;
	input wire clk;
	input wire rst;
	input wire event_valid;
	input wire [X_BITS - 1:0] event_x;
	input wire [Y_BITS - 1:0] event_y;
	input wire event_polarity;
	input wire compute_trigger;
	output reg signed [ACC_BITS - 1:0] delta_x;
	output reg signed [ACC_BITS - 1:0] delta_y;
	output reg delta_valid;
	reg signed [ACC_BITS - 1:0] sum_x;
	reg signed [ACC_BITS - 1:0] sum_y;
	reg [ACC_BITS - 1:0] event_count;
	reg signed [ACC_BITS - 1:0] prev_center_x;
	reg signed [ACC_BITS - 1:0] prev_center_y;
	wire signed [1:0] pol_signed = (event_polarity ? 2'sd1 : -2'sd1);
	always @(posedge clk)
		if (rst) begin
			sum_x <= 16'sd0;
			sum_y <= 16'sd0;
			event_count <= 16'd0;
			prev_center_x <= 16'sd0;
			prev_center_y <= 16'sd0;
			delta_x <= 16'sd0;
			delta_y <= 16'sd0;
			delta_valid <= 1'b0;
		end
		else begin
			delta_valid <= 1'b0;
			if (compute_trigger) begin
				if (event_count > 0) begin
					delta_x <= sum_x - prev_center_x;
					delta_y <= sum_y - prev_center_y;
					delta_valid <= 1'b1;
					prev_center_x <= sum_x;
					prev_center_y <= sum_y;
				end
				sum_x <= 16'sd0;
				sum_y <= 16'sd0;
				event_count <= 16'd0;
			end
			else if (event_valid) begin
				sum_x <= sum_x + ({{ACC_BITS - X_BITS {1'b0}}, event_x} * pol_signed);
				sum_y <= sum_y + ({{ACC_BITS - Y_BITS {1'b0}}, event_y} * pol_signed);
				event_count <= event_count + 1'b1;
			end
		end
endmodule
module gesture_classifier (
	clk,
	rst,
	delta_valid,
	delta_x,
	delta_y,
	gesture,
	gesture_valid
);
	parameter DATA_BITS = 16;
	parameter WEIGHT_BITS = 8;
	input wire clk;
	input wire rst;
	input wire delta_valid;
	input wire signed [DATA_BITS - 1:0] delta_x;
	input wire signed [DATA_BITS - 1:0] delta_y;
	output reg [1:0] gesture;
	output reg gesture_valid;
	localparam signed [WEIGHT_BITS - 1:0] W_UP_X = 8'sd0;
	localparam signed [WEIGHT_BITS - 1:0] W_UP_Y = 8'sd64;
	localparam signed [WEIGHT_BITS - 1:0] W_DOWN_X = 8'sd0;
	localparam signed [WEIGHT_BITS - 1:0] W_DOWN_Y = -8'sd64;
	localparam signed [WEIGHT_BITS - 1:0] W_LEFT_X = -8'sd64;
	localparam signed [WEIGHT_BITS - 1:0] W_LEFT_Y = 8'sd0;
	localparam signed [WEIGHT_BITS - 1:0] W_RIGHT_X = 8'sd64;
	localparam signed [WEIGHT_BITS - 1:0] W_RIGHT_Y = 8'sd0;
	reg signed [DATA_BITS + WEIGHT_BITS:0] score_up;
	reg signed [DATA_BITS + WEIGHT_BITS:0] score_down;
	reg signed [DATA_BITS + WEIGHT_BITS:0] score_left;
	reg signed [DATA_BITS + WEIGHT_BITS:0] score_right;
	reg stage1_valid;
	always @(posedge clk)
		if (rst) begin
			score_up <= 25'sd0;
			score_down <= 25'sd0;
			score_left <= 25'sd0;
			score_right <= 25'sd0;
			stage1_valid <= 1'b0;
		end
		else begin
			stage1_valid <= delta_valid;
			if (delta_valid) begin
				score_up <= (delta_x * W_UP_X) + (delta_y * W_UP_Y);
				score_down <= (delta_x * W_DOWN_X) + (delta_y * W_DOWN_Y);
				score_left <= (delta_x * W_LEFT_X) + (delta_y * W_LEFT_Y);
				score_right <= (delta_x * W_RIGHT_X) + (delta_y * W_RIGHT_Y);
			end
		end
	always @(posedge clk)
		if (rst) begin
			gesture <= 2'b00;
			gesture_valid <= 1'b0;
		end
		else begin
			gesture_valid <= stage1_valid;
			if (stage1_valid) begin
				if (((score_up >= score_down) && (score_up >= score_left)) && (score_up >= score_right))
					gesture <= 2'b00;
				else if ((score_down >= score_left) && (score_down >= score_right))
					gesture <= 2'b01;
				else if (score_left >= score_right)
					gesture <= 2'b10;
				else
					gesture <= 2'b11;
			end
		end
endmodule
module dvs_gesture_accel (
	clk,
	rst,
	event_valid,
	event_x,
	event_y,
	event_polarity,
	event_ts,
	gesture,
	gesture_valid
);
	parameter X_BITS = 7;
	parameter Y_BITS = 7;
	parameter TS_BITS = 16;
	parameter WINDOW_EVENTS = 100;
	input wire clk;
	input wire rst;
	input wire event_valid;
	input wire [X_BITS - 1:0] event_x;
	input wire [Y_BITS - 1:0] event_y;
	input wire event_polarity;
	input wire [TS_BITS - 1:0] event_ts;
	output wire [1:0] gesture;
	output wire gesture_valid;
	wire signed [15:0] delta_x;
	wire signed [15:0] delta_y;
	wire delta_valid;
	reg compute_trigger;
	reg [15:0] event_counter;
	always @(posedge clk)
		if (rst) begin
			event_counter <= 16'd0;
			compute_trigger <= 1'b0;
		end
		else begin
			compute_trigger <= 1'b0;
			if (event_valid) begin
				if (event_counter >= (WINDOW_EVENTS - 1)) begin
					compute_trigger <= 1'b1;
					event_counter <= 16'd0;
				end
				else
					event_counter <= event_counter + 1'b1;
			end
		end
	event_accumulator #(
		.X_BITS(X_BITS),
		.Y_BITS(Y_BITS),
		.ACC_BITS(16)
	) u_accumulator(
		.clk(clk),
		.rst(rst),
		.event_valid(event_valid),
		.event_x(event_x),
		.event_y(event_y),
		.event_polarity(event_polarity),
		.compute_trigger(compute_trigger),
		.delta_x(delta_x),
		.delta_y(delta_y),
		.delta_valid(delta_valid)
	);
	gesture_classifier #(
		.DATA_BITS(16),
		.WEIGHT_BITS(8)
	) u_classifier(
		.clk(clk),
		.rst(rst),
		.delta_valid(delta_valid),
		.delta_x(delta_x),
		.delta_y(delta_y),
		.gesture(gesture),
		.gesture_valid(gesture_valid)
	);
endmodule
module uart_gesture_top (
	clk,
	uart_rx,
	uart_tx,
	led_heartbeat
);
	parameter CLK_FREQ = 12000000;
	parameter BAUD_RATE = 115200;
	parameter WINDOW_EVENTS = 100;
	input wire clk;
	input wire uart_rx;
	output wire uart_tx;
	output wire led_heartbeat;
	localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
	reg [3:0] por_cnt = 4'd0;
	wire rst;
	always @(posedge clk)
		if (&por_cnt)
			por_cnt <= por_cnt;
		else
			por_cnt <= por_cnt + 1'b1;
	assign rst = ~&por_cnt;
	wire [7:0] rx_data;
	wire rx_valid;
	reg [7:0] tx_data;
	reg tx_valid;
	wire tx_busy;
	wire [1:0] gesture;
	wire gesture_valid;
	reg [23:0] heartbeat_cnt;
	always @(posedge clk)
		if (rst)
			heartbeat_cnt <= 24'd0;
		else
			heartbeat_cnt <= heartbeat_cnt + 1'b1;
	assign led_heartbeat = ~heartbeat_cnt[22];
	reg [1:0] tx_state;
	reg [1:0] pending_gesture;
	reg [1:0] byte_cnt;
	reg [6:0] saved_x;
	reg [6:0] saved_y;
	reg saved_pol;
	reg event_valid;
	reg [6:0] event_x;
	reg [6:0] event_y;
	reg event_polarity;
	uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_rx(
		.clk(clk),
		.rst(rst),
		.rx(uart_rx),
		.data(rx_data),
		.valid(rx_valid)
	);
	uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_tx(
		.clk(clk),
		.rst(rst),
		.data(tx_data),
		.valid(tx_valid),
		.tx(uart_tx),
		.busy(tx_busy)
	);
	dvs_gesture_accel #(
		.X_BITS(7),
		.Y_BITS(7),
		.TS_BITS(16),
		.WINDOW_EVENTS(WINDOW_EVENTS)
	) u_accel(
		.clk(clk),
		.rst(rst),
		.event_valid(event_valid),
		.event_x(event_x),
		.event_y(event_y),
		.event_polarity(event_polarity),
		.event_ts(16'd0),
		.gesture(gesture),
		.gesture_valid(gesture_valid)
	);
	always @(posedge clk)
		if (rst) begin
			byte_cnt <= 2'd0;
			saved_x <= 7'd0;
			saved_y <= 7'd0;
			saved_pol <= 1'b0;
			event_valid <= 1'b0;
			event_x <= 7'd0;
			event_y <= 7'd0;
			event_polarity <= 1'b0;
			tx_state <= 2'd0;
			tx_data <= 8'd0;
			tx_valid <= 1'b0;
			pending_gesture <= 2'd0;
		end
		else begin
			event_valid <= 1'b0;
			tx_valid <= 1'b0;
			if (rx_valid) begin
				if ((byte_cnt == 2'd0) && (rx_data == 8'hff)) begin
					if (tx_state == 2'd0)
						tx_state <= 2'd1;
				end
				else
					case (byte_cnt)
						2'd0: begin
							saved_x <= rx_data[6:0];
							byte_cnt <= 2'd1;
						end
						2'd1: begin
							saved_y <= rx_data[6:0];
							byte_cnt <= 2'd2;
						end
						2'd2: begin
							saved_pol <= rx_data[0];
							byte_cnt <= 2'd3;
						end
						2'd3: begin
							event_x <= saved_x;
							event_y <= saved_y;
							event_polarity <= saved_pol;
							event_valid <= 1'b1;
							byte_cnt <= 2'd0;
						end
					endcase
			end
			if (gesture_valid && (tx_state == 2'd0)) begin
				pending_gesture <= gesture;
				tx_state <= 2'd2;
			end
			case (tx_state)
				2'd0:
					;
				2'd1:
					if (!tx_busy) begin
						tx_data <= 8'h55;
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				2'd2:
					if (!tx_busy) begin
						tx_data <= {6'h28, pending_gesture};
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				default: tx_state <= 2'd0;
			endcase
		end
endmodule