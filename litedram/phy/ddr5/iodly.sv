`timescale 1fs/1fs

module iodly #(
  parameter NUM_TAPS  = 1,
  parameter TAP_DELAY = 1,
  parameter FIXUP     = 1
)(
  input C,
  input INC,
  input RST,
  input INPUT,
  output OUTPUT
);
  wire [0:0] taps [NUM_TAPS];

  localparam SEL_WIDTH = NUM_TAPS > 1 ? $clog2(NUM_TAPS) : 1;

  reg  [SEL_WIDTH-1:0] sel = {SEL_WIDTH{1'b0}};
  assign taps[0] = INPUT;

  generate
    for(genvar i = 1; i < NUM_TAPS; i++) begin
      assign #TAP_DELAY taps[i] = taps[i-1];
    end
  endgenerate

  always @(posedge C) begin
    if(INC) begin
      sel <= sel + 1'b1;
    end else if(RST) begin
      sel <= {SEL_WIDTH{1'b0}};
    end
  end

  assign OUTPUT = taps[sel];

endmodule
