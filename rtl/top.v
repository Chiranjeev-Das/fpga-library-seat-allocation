// FPGA-Based Library Seat Allocation System
// Description: 32-seat multi-bank library seat allocation system using Verilog
Simple synchronous debouncer (edge detect). Assumes clk ~50 MHz
module debounce(
  input clk,
  input rst_n,
  input btn_in,
  output reg btn_out // one-cycle pulse on press
);
  reg [19:0] cnt;
  reg btn_sync, btn_sync2, btn_stable;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      btn_sync <= 1'b1;
      btn_sync2 <= 1'b1;
      btn_stable <= 1'b1;
      cnt <= 0;
      btn_out <= 0;
    end else begin
      btn_sync <= btn_in;
      btn_sync2 <= btn_sync;
      if (btn_sync2 == btn_stable) begin
        cnt <= 0;
        btn_out <= 0;
      end else begin
        cnt <= cnt + 1;
        if (cnt == 20'd1_000_000) begin // ~20ms at 50MHz
          btn_stable <= btn_sync2;
          if (btn_stable == 1'b1 && btn_sync2 == 1'b0) begin
            // pressed (active low assumed); generate pulse
            btn_out <= 1;
          end else begin
            btn_out <= 0;
          end
        end else begin
          btn_out <= 0;
        end
      end
    end
  end
endmodule

module popcount8 (
  input [7:0] seat_bits, // 1 = occupied, 0 = free
  output [3:0] free_count // 0..8
);
  wire [7:0] free_bits = ~seat_bits; // free = 1
  wire [3:0] sum1 = free_bits[0] + free_bits[1] + free_bits[2] + free_bits[3];
  wire [3:0] sum2 = free_bits[4] + free_bits[5] + free_bits[6] + free_bits[7];
  assign free_count = sum1 + sum2;
endmodule

module priority_encoder_8(
  input [7:0] seat_bits, // 1 = occupied, 0 = free
  output reg valid, // 1 if a free seat exists
  output reg [2:0] index // index of first free seat (0..7)
);
  integer i;
  reg found; // flag to stop further updates

  always @(*) begin
    valid = 0;
    index = 3'b000;
    found = 0;

    for (i = 0; i < 8; i = i + 1) begin
      if (!found && seat_bits[i] == 1'b0) begin
        valid = 1;
        index = i[2:0]; // store first free seat index
        found = 1; // prevent overwriting
      end
    end
  end

endmodule

module seat_manager(
  input clk,
  input rst_n,
  input load_from_sw_pulse, // load initial occupancy from switches (active high pulse)
  input book_pulse, // book suggested seat (pulse)
  input [7:0] sw_in, // raw switches (user sensors)
  input [2:0] suggested_idx, // seat index inside the bank (0..7)
  input [1:0] bank_select, // NEW → selects which seat group (0..3)
  output reg [31:0] seat_reg_out // now 32 seats total
);

  wire [4:0] target_index;
  assign target_index = {bank_select, suggested_idx}; // = (bank * 8) + seat

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      seat_reg_out <= 32'b0; // all free on reset
    end
    else begin
      if (load_from_sw_pulse) begin
        // Load ONLY the selected bank from SW switches
        seat_reg_out[ (bank_select*8) +: 8 ] <= sw_in;
      end
      else if (book_pulse) begin
        // Book only selected bank seat
        seat_reg_out[target_index] <= 1'b1;
      end
    end
  end

endmodule

module sevenseg_decoder(
  input [3:0] digit, // 0..9
  output reg [6:0] seg // a..g active high for typical wiring (adjust if active low)
);
  always @(*) begin
    case (digit)
      4'd0: seg = 7'b1000000; // 0
      4'd1: seg = 7'b1111001; // 1
      4'd2: seg = 7'b0100100; // 2
      4'd3: seg = 7'b0110000; // 3
      4'd4: seg = 7'b0011001; // 4
      4'd5: seg = 7'b0010010; // 5
      4'd6: seg = 7'b0000010; // 6
      4'd7: seg = 7'b1111000; // 7
      4'd8: seg = 7'b0000000; // 8
      4'd9: seg = 7'b0010000; // 9
      default: seg = 7'b1111111;
    endcase
  end
endmodule

module top(
    input CLOCK_50, // 50 MHz board clock
    input [9:0] SW, // SW[7:0] = seat load, SW[9:8] = bank select
    input [2:0] KEY, // KEY[0] = Request, KEY[1] = Book, KEY[2] = Load
    output [7:0] LEDG, // Seat occupancy LEDs (current bank only)
    output [6:0] HEX0, // Free seat count (current bank)
    output [6:0] HEX1, // Suggested seat index (1..8)
    output [6:0] HEX2 // Bank number (1..4)
);

    // Bank select from switches (0–3)
    wire [1:0] bank_select = SW[9:8];

    // Buttons are active LOW, so invert them
    wire btn_request = ~KEY[0];
    wire btn_book = ~KEY[1];
    wire btn_load = ~KEY[2];

    wire req_pulse, book_pulse, load_pulse;
    debounce db_req (.clk(CLOCK_50), .rst_n(1'b1), .btn_in(btn_request), .btn_out(req_pulse));
    debounce db_book (.clk(CLOCK_50), .rst_n(1'b1), .btn_in(btn_book), .btn_out(book_pulse));
    debounce db_load (.clk(CLOCK_50), .rst_n(1'b1), .btn_in(btn_load), .btn_out(load_pulse));

    // Full 32 seats (4 banks × 8 seats)
    wire [31:0] seat_reg;

    // Extract the currently selected bank (8 seats)
    wire [7:0] seat_bank = seat_reg[(bank_select * 8) +: 8];

    // Priority encoder for current bank
    wire pe_valid;
    wire [2:0] pe_index;
    priority_encoder_8 pe_inst (
        .seat_bits(seat_bank),
        .valid(pe_valid),
        .index(pe_index)
    );

    // Seat manager handles full 32 seats
    seat_manager sm(
        .clk(CLOCK_50),
        .rst_n(1'b1),
        .load_from_sw_pulse(load_pulse),
        .book_pulse(book_pulse),
        .sw_in(SW[7:0]),
        .suggested_idx(pe_index),
        .bank_select(bank_select),
        .seat_reg_out(seat_reg)
    );

    // Count free seats in selected bank
    wire [3:0] free_count;
    popcount8 pc(
        .seat_bits(seat_bank),
        .free_count(free_count)
    );

    // Display free seat count (HEX0)
    sevenseg_decoder s0(.digit(free_count), .seg(HEX0));

    // Display suggested seat index (1..8), or 0 if none (HEX1)
    wire [3:0] sug_digit = pe_valid ? (pe_index + 4'd1) : 4'd0;
    sevenseg_decoder s1(.digit(sug_digit), .seg(HEX1));

    // Display bank number as 1–4 (HEX2)
    sevenseg_decoder s2(.digit(bank_select + 4'd1), .seg(HEX2));

    // LED shows seat occupancy for selected bank
    assign LEDG = seat_bank;

endmodule
