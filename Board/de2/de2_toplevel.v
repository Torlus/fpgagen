/********************************************/
/* minimig_de2_top.v                        */
/* Altera DE2 FPGA Top File                 */
/*                                          */
/* 2012, rok.krajnc@gmail.com               */
/********************************************/


module de2_toplevel (
  // clock inputs
  input wire 		CLOCK_27, // 27 MHz
  input wire 		CLOCK_50, // 50 MHz
  input wire 		EXT_CLOCK, // External Clock
  // USB JTAG Link
  input wire 		TDI, // CPLD -> FPGA (data in)
  input wire 		TCK, // CPLD -> FPGA (clk)
  input wire 		TCS, // CPLD -> FPGA (CS)
  output wire 		TDO, // FPGA -> CPLD (data out)
  // push button inputs
  input wire [ 4-1:0] 	KEY, // Pushbutton[3:0]
  // switch inputs
  input wire [ 10-1:0] 	SW, // Toggle Switch[9:0]
  // 7-seg display outputs
  output wire [ 7-1:0] 	HEX0, // Seven Segment Digit 0
  output wire [ 7-1:0] 	HEX1, // Seven Segment Digit 1
  output wire [ 7-1:0] 	HEX2, // Seven Segment Digit 2
  output wire [ 7-1:0] 	HEX3, // Seven Segment Digit 3
  // LED outputs
  output wire [ 8-1:0] 	LEDG, // LED Green[7:0]
  output wire [ 10-1:0] LEDR, // LED Red[9:0]
  // UART
  output wire 		UART_TXD, // UART Transmitter
  input wire 		UART_RXD, // UART Receiver
  // I2C
  inout wire 		I2C_SDAT, // I2C Data
  output wire 		I2C_SCLK, // I2C Clock
  // PS2
  inout wire 		PS2_DAT, // PS2 Keyboard Data
  inout wire 		PS2_CLK, // PS2 Keyboard Clock
  inout wire 		PS2_MDAT, // PS2 Mouse Data
  inout wire 		PS2_MCLK, // PS2 Mouse Clock
  // VGA
  output wire 		VGA_HS, // VGA H_SYNC
  output wire 		VGA_VS, // VGA V_SYNC
  output wire [ 10-1:0] VGA_R, // VGA Red[3:0]
  output wire [ 10-1:0] VGA_G, // VGA Green[3:0]
  output wire [ 10-1:0] VGA_B, // VGA Blue[3:0]
  output wire           VGA_SYNC,
  output wire           VGA_BLANK,
  output wire           VGA_CLK,
			
  // Audio CODEC
  inout wire 		AUD_ADCLRCK, // Audio CODEC ADC LR Clock
  input wire 		AUD_ADCDAT, // Audio CODEC ADC Data
  inout wire 		AUD_DACLRCK, // Audio CODEC DAC LR Clock
  output wire 		AUD_DACDAT, // Audio CODEC DAC Data
  inout wire 		AUD_BCLK, // Audio CODEC Bit-Stream Clock
  output wire 		AUD_XCK, // Audio CODEC Chip Clock
  // SD Card
  input wire 		SD_DAT, // SD Card Data            - spi MISO
  output wire 		SD_DAT3, // SD Card Data 3          - spi CS
  output wire 		SD_CMD, // SD Card Command Signal  - spi MOSI
  output wire 		SD_CLK, // SD Card Clock           - spi CLK
  // SRAM
  inout wire [ 16-1:0] 	SRAM_DQ, // SRAM Data bus 16 Bits
  output wire [ 18-1:0] SRAM_ADDR, // SRAM Address bus 18 Bits
  output wire 		SRAM_UB_N, // SRAM High-byte Data Mask
  output wire 		SRAM_LB_N, // SRAM Low-byte Data Mask
  output wire 		SRAM_WE_N, // SRAM Write Enable
  output wire 		SRAM_CE_N, // SRAM Chip Enable
  output wire 		SRAM_OE_N, // SRAM Output Enable
  // SDRAM
  inout wire [ 16-1:0] 	DRAM_DQ, // SDRAM Data bus 16 Bits
  output wire [ 12-1:0] DRAM_ADDR, // SDRAM Address bus 12 Bits
  output wire 		DRAM_LDQM, // SDRAM Low-byte Data Mask
  output wire 		DRAM_UDQM, // SDRAM High-byte Data Mask
  output wire 		DRAM_WE_N, // SDRAM Write Enable
  output wire 		DRAM_CAS_N, // SDRAM Column Address Strobe
  output wire 		DRAM_RAS_N, // SDRAM Row Address Strobe
  output wire 		DRAM_CS_N, // SDRAM Chip Select
  output wire 		DRAM_BA_0, // SDRAM Bank Address 0
  output wire 		DRAM_BA_1, // SDRAM Bank Address 1
  output wire 		DRAM_CLK, // SDRAM Clock
  output wire 		DRAM_CKE, // SDRAM Clock Enable
  // FLASH
  inout wire [ 8-1:0] 	FL_DQ, // FLASH Data bus 8 Bits
  output wire [ 22-1:0] FL_ADDR, // FLASH Address bus 22 Bits
  output wire 		FL_WE_N, // FLASH Write Enable
  output wire 		FL_RST_N, // FLASH Reset
  output wire 		FL_OE_N, // FLASH Output Enable
  output wire 		FL_CE_N, // FLASH Chip Enable
  // MINIMIG specific
  input wire [ 6-1:0] 	Joya, // joystick port A
  input wire [ 6-1:0] 	Joyb, // joystick port B
  output wire 		AUDIOLEFT, // sigma-delta DAC output left
  output wire 		AUDIORIGHT    // sigma-delta DAC output right
);



////////////////////////////////////////
// internal signals                   //
////////////////////////////////////////

// clock
wire           pll_in_clk;
wire           clk_114;
wire           clk_28;
wire           pll_locked;
wire           clk_7;
wire           c1;
wire           c3;
wire           cck;
wire [ 10-1:0] eclk;
wire           clk_50;

// reset
wire           pll_rst;
wire           sdctl_rst;
wire           rst_50;
wire           rst_minimig;

// ctrl
wire           rom_status;
wire           ram_status;
wire           reg_status;

// tg68
wire           tg68_rst;
wire [ 16-1:0] tg68_dat_in;
wire [ 16-1:0] tg68_dat_out;
wire [ 32-1:0] tg68_adr;
wire [  3-1:0] tg68_IPL;
wire           tg68_dtack;
wire           tg68_as;
wire           tg68_uds;
wire           tg68_lds;
wire           tg68_rw;
wire           tg68_ena7RD;
wire           tg68_ena7WR;
wire           tg68_enaWR;
wire [ 16-1:0] tg68_cout;
wire           tg68_cpuena;
wire [  2-1:0] cpu_config;
wire [  6-1:0] memcfg;
wire [ 32-1:0] tg68_cad;
wire [  6-1:0] tg68_cpustate;
wire           tg68_cdma;
wire           tg68_clds;
wire           tg68_cuds;

// minimig
wire [ 16-1:0] ram_data;      // sram data bus
wire [ 16-1:0] ramdata_in;    // sram data bus in
wire [ 22-1:1] ram_address;   // sram address bus
wire           _ram_bhe;      // sram upper byte select
wire           _ram_ble;      // sram lower byte select
wire           _ram_we;       // sram write enable
wire           _ram_oe;       // sram output enable
wire           _15khz;        // scandoubler disable
wire           joy_emu_en;    // joystick emulation enable
wire           sdo;           // SPI data output
wire [ 15-1:0] ldata;         // left DAC data
wire [ 15-1:0] rdata;         // right DAC data
wire [15:0]    audio_left;
wire [15:0]    audio_right;
wire           floppy_fwr;
wire           floppy_frd;
wire           hd_fwr;
wire           hd_frd;

// sdram
wire           reset_out;
wire [  4-1:0] sdram_cs;
wire [  2-1:0] sdram_dqm;
wire [  2-1:0] sdram_ba;

// audio
wire           audio_lr_switch;
wire           audio_lr_mix;

// ctrl
wire [ 16-1:0] SRAM_DAT_W;
wire [ 16-1:0] SRAM_DAT_R;
wire [  8-1:0] FL_DAT_W;
wire [  8-1:0] FL_DAT_R;
wire [  4-1:0] SPI_CS_N;
wire           SPI_DI;
wire           rst_ext;
wire           boot_sel;
wire [  4-1:0] ctrl_cfg;
wire [  4-1:0] ctrl_status;

// indicators
wire [  8-1:0] track;

////////////////////////////////////////
// toplevel assignments               //
////////////////////////////////////////

// PS/2 keyboard
wire PS2K_DAT_IN=PS2_DAT;
wire PS2K_DAT_OUT;
assign PS2_DAT = (PS2K_DAT_OUT == 1'b0) ? 1'b0 : 1'bz;
wire PS2K_CLK_IN=PS2_CLK;
wire PS2K_CLK_OUT;
assign PS2_CLK = (PS2K_CLK_OUT == 1'b0) ? 1'b0 : 1'bz;

// PS/2 Mouse
wire PS2M_DAT_IN=PS2_MDAT;
wire PS2M_DAT_OUT;
assign PS2_MDAT = (PS2M_DAT_OUT == 1'b0) ? 1'b0 : 1'bz;
wire PS2M_CLK_IN=PS2_MCLK;
wire PS2M_CLK_OUT;
assign PS2_MCLK = (PS2M_CLK_OUT == 1'b0) ? 1'b0 : 1'bz;


// assign unused outputs
assign TDO              = 1'b1;

// SD card
// assign SD_DAT3          = SPI_CS_N[0];

// SRAM
assign SRAM_DQ          = SRAM_OE_N ? SRAM_DAT_W : 16'bzzzzzzzzzzzzzzzz;
assign SRAM_DAT_R       = SRAM_DQ;


// FLASH
assign FL_DQ            = FL_OE_N   ? FL_DAT_W   : 8'bzzzzzzzz;
assign FL_DAT_R         = FL_DQ;

// AUDIO
assign AUDIOLEFT        = audio_left;
assign AUDIORIGHT       = audio_right;
//assign AUDIOLEFT        = 1'b0;
//assign AUDIORIGHT       = 1'b0;

// ctrl
assign SPI_DI           = !SPI_CS_N[0] ? SD_DAT : sdo;

// clock
assign pll_in_clk       = CLOCK_27;

// reset
assign pll_rst          = !SW[0];
assign sdctl_rst        = pll_locked & SW[0];

wire [7:0] vga_red;
wire [7:0] vga_green;
wire [7:0] vga_blue;

// DE2 specific VGA wiring
assign VGA_R = {vga_red[7:4], vga_red[7:4], vga_red[7:6]};
assign VGA_G = {vga_green[7:4], vga_green[7:4], vga_green[7:6]};
assign VGA_B = {vga_blue[7:4], vga_blue[7:4], vga_blue[7:6]};
assign VGA_BLANK = 1'b1; // (VGA_HS && VGA_VS);
assign VGA_SYNC = 0;
assign VGA_CLK = memclk; //DRAM_CLK;
 

//// generated clocks ////

pll mypll
(
	.inclk0(CLOCK_27),
	.c0(sysclk),
	.c1(memclk),
	.c2(DRAM_CLK)
);


// 7MHz
reg [2-1:0] clk7_cnt;
reg         clk7_en_reg;
always @ (posedge clk_28, negedge pll_locked) begin
  if (!pll_locked) begin
    clk7_cnt <= 2'b10;
    clk7_en_reg <= #1 1'b1;
  end else begin
    clk7_cnt <= clk7_cnt + 2'b01;
    clk7_en_reg <= #1 ~|clk7_cnt;
  end
end

assign clk_7 = clk7_cnt[1];
assign clk7_en = clk7_en_reg;
 
assign audio_lr_switch = 1'b0;
assign audio_lr_mix = 1'b0;

//// audio ////
audio_top audio_top (
  .clk          (sysclk           ),  // 28MHz input clock
  .rst_n        (SW[0]^!KEY[0]    ),  // active low reset (from sdram controller)
  // config
  .exchan       (audio_lr_switch  ),  // switch audio left / right channel
  .mix          (audio_lr_mix     ),  // normal / centered mix (play some left channel on the right channel and vise-versa)
  // audio shifter
  .rdata        (audio_right      ),  // right channel sample data
  .ldata        (audio_left       ),  // left channel sample data
  .aud_bclk     (AUD_BCLK         ),  // CODEC data clock
  .aud_daclrck  (AUD_DACLRCK      ),  // CODEC data clock
  .aud_dacdat   (AUD_DACDAT       ),  // CODEC data
  .aud_xck      (AUD_XCK          ),  // CODEC data clock
  // I2C audio config
  .i2c_sclk     (I2C_SCLK         ),  // CODEC config clock
  .i2c_sdat     (I2C_SDAT         )   // CODEC config data
);



defparam myvt.rowAddrBits = 12;
defparam myvt.colAddrBits = 8;

Virtual_Toplevel myvt
(
	.reset(SW[0]^!KEY[0]),
	.MCLK(sysclk),
	.SDR_CLK(memclk),

	.DRAM_ADDR(DRAM_ADDR),
	.DRAM_DQ(DRAM_DQ),
	.DRAM_BA_1(DRAM_BA_1),
	.DRAM_BA_0(DRAM_BA_0),
	.DRAM_CKE(DRAM_CKE),
	.DRAM_UDQM(DRAM_UDQM),
	.DRAM_LDQM(DRAM_LDQM),
	.DRAM_CS_N(DRAM_CS_N),
	.DRAM_WE_N(DRAM_WE_N),
	.DRAM_CAS_N(DRAM_CAS_N),
	.DRAM_RAS_N(DRAM_RAS_N),
	
	.DAC_LDATA(audio_left),
	.DAC_RDATA(audio_right),
	
	.VGA_R(vga_red),
	.VGA_G(vga_green),
	.VGA_B(vga_blue),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),

	.RS232_RXD(UART_RXD),
	.RS232_TXD(UART_TXD),
	
	// PS/2
	.ps2k_clk_in(PS2K_CLK_IN),
	.ps2k_dat_in(PS2K_DAT_IN),
	.ps2k_clk_out(PS2K_CLK_OUT),
	.ps2k_dat_out(PS2K_DAT_OUT),
	
	// Joystick
	.joya({2'b11,Joya[5:4],Joya[0],Joya[1],Joya[2],Joya[3]}),
	.joyb({2'b11,Joyb[5:4],Joyb[0],Joyb[1],Joyb[2],Joyb[3]}),

	// SD card
	.spi_cs(SD_DAT3),
	.spi_miso(SD_DAT),
	.spi_mosi(SD_CMD),
	.spi_clk(SD_CLK),
);

endmodule

