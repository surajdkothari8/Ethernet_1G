
/////////////////////////////////////////////////////////////////////////////////////////
// Module Name	: TX_MAC
// Author	: Suraj Kothari
// Data Created : 1/27/2018
// Revision	: 1.0
// Description	: This module functions as MAC module to support 1G Ethernet Protocol 
// Interface	: 8-bit GMII I/F with 1-bit GMII_EN
/////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 100ps

`include "crc.v"
module TX_MAC (
	TX_CLK,
	SYS_RST,
	TX_START,
	TX_BUSY,
	GMII_TXD,
	GMII_TX_EN,
	CONTROL_IN
);

// System Clock and System Reset
input wire TX_CLK;
input wire SYS_RST;
input wire TX_START;
output reg TX_BUSY;

// GMII Interface between MAC - PHY
output reg [7:0] GMII_TXD;
output reg GMII_TX_EN;

// Control and Data Information
input wire [7:0] CONTROL_IN;

reg tx_start_d = 1'b0;
reg tx_start_dd = 1'b0;
reg [7:0] control_in_d = 8'h00;


parameter ENABLE_PADDING = 1'b1;
parameter MIN_FRAME_LENGTH = 64;
parameter [2:0] IDLE 		= 3'b000,	// 3'd0
		 PREAMBLE 	= 3'b001,	// 3'd1
		 CNTRL 		= 3'b010,	// 3'd2 
		 PAYLOAD	= 3'b011,	// 3'd3
	         PAD		= 3'b100,	// 3'd4
		 FCS		= 3'b101,	// 3'd5
		 IFG		= 3'b110;	// 3'd6

reg [2:0] state = IDLE;

// Internal Counters and Registers
reg [15:0] frame_count = 16'd0; 
reg crc_update = 1'b0;
reg crc_reset = 1'b0;
wire [31:0] crc_out;
reg [7:0] data_count = 8'h00;
reg [7:0] data_out_count = 8'h00;
reg [7:0] IFG_delay_count = 8'h00;

// CRC Logic Calculation 

crc inst_crc(
  .data_in(GMII_TXD),
  .crc_en(crc_update),
  .crc_out(crc_out),
  .rst(crc_reset),
  .clk(TX_CLK)
);


// Flop the start condition
always @(posedge TX_CLK) begin
	if(SYS_RST) begin
		tx_start_d <= 1'b0;
		tx_start_dd <= 1'b0;
	end
	else begin
		tx_start_d <= TX_START;
		tx_start_dd <= tx_start_d;		
	end
end

// Flop the Control and Data Inputs
always @(posedge TX_CLK) begin
	if(SYS_RST) begin
		control_in_d <= 8'h00;
	end
	else begin
		if(TX_BUSY == 1'b1) begin
			control_in_d <= CONTROL_IN;	
		end	
		else begin
			control_in_d <= 8'h00;	
		end
	end
end

// TX MAC Finite State Machine
always @(posedge TX_CLK) begin
	if(SYS_RST) begin
		state <= IDLE;
		GMII_TXD <= 8'h00;
		GMII_TX_EN <= 1'b0;
		TX_BUSY	<= 1'b0;
		frame_count <= 16'd0;
		data_count <= 8'h00;
		data_out_count <= 8'h00;
		crc_update <= 1'b0;
		crc_reset <= 1'b1;
		IFG_delay_count <= 8'h00;
	end
	else begin
	case(state)
	// Wait for Start Condition
	IDLE:		begin  
			if(tx_start_d == 1'b1 && tx_start_dd == 1'b0) begin
				TX_BUSY <= 1'b1;
				frame_count <= 16'd0;
				data_count <= 8'h00;
				data_out_count <= 8'h00;
				GMII_TXD <= 8'h00;
				GMII_TX_EN <= 1'b0;
				crc_update <= 1'b0;
				crc_reset <= 1'b1;
				IFG_delay_count <= 8'h0C;
				state <= PREAMBLE;
			end			
			end
	
	// PREAMBLE and SFD
	// PREAMBLE = 8'h55. 56-bit/7-byte of pattern of 1s and 0s for network devices to automatically synchronize their Receiver Clocks
	// SFD = 8'hD5 is for bit level synchronization and marks the starting of a new incoming frame
	PREAMBLE:	begin
			frame_count <= frame_count + 1'b1;
			crc_reset <= 1'b1;
			crc_update <= 1'b0;
			GMII_TXD <= 8'h55;
			GMII_TX_EN <= 1'b1;
			if(frame_count == 16'd7) begin
				GMII_TXD <= 8'hD5;
				state	 <= CNTRL;
				crc_reset <= 1'b0;
				crc_update <= 1'b1;

			end		
			end
	
	// CNTRL stage decodes how many bytes are to be transferred as a Payload on the GMII IF 
	CNTRL:		begin
			frame_count <= frame_count + 1'b1;
			data_count <= control_in_d[7:0];
			GMII_TXD <= control_in_d[7:0];
			state <= PAYLOAD; 						
			end
	
	// PAYLOAD represents actual data to be transferred over the network. This data is used by user to perform further operations
	// Background: Any Ethernet Frame received or sent has to be of certain minimum length. 
	// Any Ethernet Frame less than 64-bytes is illegal over the network
	// To avoid communication descrepancies, we need to send atleast 64-bytes (8-Byte PREAMBLE/SFD+ 4-Byte CRC + 52-Bytes of Data).
	 
	PAYLOAD:	begin
			frame_count <= frame_count + 1'b1;
			GMII_TXD <= data_out_count[7:0];
			data_out_count <= data_out_count + 1'b1;
			if(data_out_count == data_count[7:0]) begin
				if(ENABLE_PADDING && (frame_count < MIN_FRAME_LENGTH-5)) begin
					state <= PAD;	
				end
				else begin								
				state <= FCS;
				data_out_count <= 8'h00;
				frame_count <= 16'd0;
				end
			end			
			end		
	// Pad Field Counter in Ethernet Frame
	PAD:		begin
			frame_count <= frame_count + 1'b1;
			GMII_TXD <= 8'h00;
			if(frame_count == (MIN_FRAME_LENGTH -5)) begin
				frame_count <= 16'd0;
				state <= FCS;
			end
			end

	// Frame Check Sequence is a 4-octet CRC that allows detection of corrupted data within the received frame. 
	// It is calculated over the entire Ethernet Frame including Address/Command/Control + Payload + Padding
	FCS:		begin
			frame_count <= frame_count + 1'b1;
			case(frame_count)
				16'd0: begin GMII_TXD <= crc_out[7:0]; end
				16'd1: begin GMII_TXD <= crc_out[15:8]; end
				16'd2: begin GMII_TXD <= crc_out[23:16]; end
				16'd3: begin GMII_TXD <= crc_out[31:24]; end			
			endcase
			if(frame_count == 16'd3) begin
				frame_count <= 16'd0;				
				IFG_delay_count <= 8'h0C;
				crc_reset <= 1'b1;
				crc_update <= 1'b0;			
				state <= IFG;
			end
			end
	
	// Inter Frame Gap(IFG) is Idle Time between packets
	// After Ethernet Frame Sent, Transmitters are required to transmit atleast 96-bytes (12-octets) of idle data(8'h00 in this case) 
	// Before the next Ethernet Frame transfer is initiated

	IFG:		begin
			frame_count <= frame_count + 1'b1;
			GMII_TXD <= 8'h00;
			if(frame_count == (IFG_delay_count[7:0])) begin
				TX_BUSY <= 1'b0;
				frame_count <= 16'd0;
				GMII_TX_EN <= 1'b0;
				state <= IDLE;
						
			end			
			end
	endcase	
	end
end

endmodule



