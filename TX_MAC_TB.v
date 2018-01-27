`timescale 1ns / 100ps

module TX_MAC_TB();

reg SYS_CLK;
reg SYS_RST;
reg TX_START;

wire TX_BUSY;
wire [7:0] GMII_TXD;
wire GMII_TX_EN;
reg [7:0] CONTROL_IN;

TX_MAC inst_TX_MAC(
	.TX_CLK(SYS_CLK),
	.SYS_RST(SYS_RST),
	.TX_START(TX_START),
	.TX_BUSY(TX_BUSY),
	.GMII_TXD(GMII_TXD),
	.GMII_TX_EN(GMII_TX_EN),
	.CONTROL_IN(CONTROL_IN)
);


initial begin
SYS_CLK = 1'b0;
forever #4 SYS_CLK = ~SYS_CLK;
end

initial begin
$dumpfile("TX_MAC.vcd");
$dumpvars(0, TX_MAC_TB);
end

initial begin
SYS_RST = 1'b1;
TX_START = 1'b0;
CONTROL_IN = 8'd00;

#1000 SYS_RST = 1'b0;

wait(TX_BUSY == 1'b0);
#4 TX_START = 1'b1;
#12 TX_START = 1'b0;

wait(TX_BUSY == 1'b1);
$display("TX MAC Started");
CONTROL_IN = 8'd60;
wait(TX_BUSY == 1'b0);
$display("TX MAC Operation Finished!");

#100 $finish;
end

endmodule

