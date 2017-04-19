
module fastfilter (
	input recoveryclock,
	input d,
	output reg q
);

// filter incoming signal. the 8 bit gate delay is ~2.5ns in total
wire [5:0] in_d = { in_d[4:0], d } /* synthesis keep */;
wire filtered = (q && in_d != 6'b000000) || (!q && in_d == 6'b111111);

reg d1;
reg d2;

// Should double-sync this but it'll probably be too slow.
always @(posedge recoveryclock)
	q<=filtered;

endmodule
