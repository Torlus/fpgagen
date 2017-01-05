library STD;
use STD.TEXTIO.ALL;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
-- use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
use IEEE.NUMERIC_STD.ALL;


entity Virtual_Toplevel is
	generic
	(
		--colAddrBits : integer := 8;
		colAddrBits : integer := 9;
		--rowAddrBits : integer := 12
		rowAddrBits : integer := 13
	);
	port(
		reset : in std_logic;
		CLK : in std_logic;
		SDR_CLK : in std_logic;

		DRAM_ADDR	: out std_logic_vector(rowAddrBits-1 downto 0);
		DRAM_BA_0	: out std_logic;
		DRAM_BA_1	: out std_logic;
		DRAM_CAS_N	: out std_logic;
		DRAM_CKE	: out std_logic;
		DRAM_CS_N	: out std_logic;
		DRAM_DQ		: inout std_logic_vector(15 downto 0);
		DRAM_LDQM	: out std_logic;
		DRAM_RAS_N	: out std_logic;
		DRAM_UDQM	: out std_logic;
		DRAM_WE_N	: out std_logic;
		
		DAC_LDATA : out std_logic_vector(15 downto 0);
		DAC_RDATA : out std_logic_vector(15 downto 0);
		
		VGA_R		: out std_logic_vector(7 downto 0);
		VGA_G		: out std_logic_vector(7 downto 0);
		VGA_B		: out std_logic_vector(7 downto 0);
		VGA_VS		: out std_logic;
		VGA_HS		: out std_logic;

		RS232_RXD : in std_logic;
		RS232_TXD : out std_logic;

		ps2k_clk_out : out std_logic;
		ps2k_dat_out : out std_logic;
		ps2k_clk_in : in std_logic;
		ps2k_dat_in : in std_logic;
		
		joya : in std_logic_vector(7 downto 0) := (others =>'1');
		joyb : in std_logic_vector(7 downto 0) := (others =>'1');
		joyc : in std_logic_vector(7 downto 0) := (others =>'1');
		joyd : in std_logic_vector(7 downto 0) := (others =>'1');
		joye : in std_logic_vector(7 downto 0) := (others =>'1');

		spi_miso		: in std_logic := '1';
		spi_mosi		: out std_logic;
		spi_clk		: out std_logic;
		spi_cs 		: out std_logic
	);
end entity;

architecture rtl of Virtual_Toplevel is

constant addrwidth : integer := rowAddrBits+colAddrBits+2;

-- "FLASH"
signal romrd_req : std_logic := '0';
signal romrd_ack : std_logic;
signal romrd_a : std_logic_vector((13+9+2) downto 3);
signal romrd_q : std_logic_vector(63 downto 0);
signal romrd_a_cached : std_logic_vector((13+9+2) downto 3);
signal romrd_q_cached : std_logic_vector(63 downto 0);
type fc_t is ( FC_IDLE, 
	FC_TG68_RD,
	FC_DMA_RD,
	FC_T80_RD
);
signal FC : fc_t;

-- 68000 RAM
signal ram68k_req : std_logic;
signal ram68k_ack : std_logic;
signal ram68k_we : std_logic;
signal ram68k_a : std_logic_vector((13+9+2) downto 1);
signal ram68k_d : std_logic_vector(15 downto 0);
signal ram68k_q : std_logic_vector(15 downto 0);
signal ram68k_l_n : std_logic;
signal ram68k_u_n : std_logic;

-- VRAM
signal vram_req : std_logic;
signal vram_ack : std_logic;
signal vram_we : std_logic;
signal vram_a : std_logic_vector((13+9+2) downto 1);
signal vram_d : std_logic_vector(15 downto 0);
signal vram_q : std_logic_vector(15 downto 0);
signal vram_l_n : std_logic;
signal vram_u_n : std_logic;


type sdrc_t is ( SDRC_IDLE,
	SDRC_TG68,
	SDRC_DMA, 
	SDRC_T80);
signal SDRC : sdrc_t;

-- Z80 RAM

signal zram_a : std_logic_vector(12 downto 0);
signal zram_d : std_logic_vector(7 downto 0);
signal zram_q : std_logic_vector(7 downto 0);
signal zram_we : std_logic;

signal TG68_ZRAM_SEL		: std_logic;
signal TG68_ZRAM_D			: std_logic_vector(15 downto 0);
signal TG68_ZRAM_DTACK_N	: std_logic;

signal T80_ZRAM_SEL		: std_logic;
signal T80_ZRAM_D			: std_logic_vector(7 downto 0);
signal T80_ZRAM_DTACK_N	: std_logic;

type zrc_t is ( ZRC_IDLE,
	ZRC_ACC1, ZRC_ACC2, ZRC_ACC3
);
signal ZRC : zrc_t;

type zrcp_t is ( ZRCP_T80, ZRCP_TG68 );
signal ZRCP : zrcp_t;

constant useCache : boolean := false;

-- Genesis core
signal NO_DATA		: std_logic_vector(15 downto 0) := x"4E71";	-- SYNTHESIS gp/m68k.c line 12

signal MCLK			: std_logic;
signal MRST_N		: std_logic;

-- 68K
signal TG68_CLK		: std_logic;
signal TG68_RES_N	: std_logic;
signal TG68_CLKE	: std_logic;
signal TG68_DI		: std_logic_vector(15 downto 0);
signal TG68_IPL_N	: std_logic_vector(2 downto 0);
signal TG68_DTACK_N	: std_logic;
signal TG68_A		: std_logic_vector(31 downto 0);
signal TG68_DO		: std_logic_vector(15 downto 0);
signal TG68_AS_N		: std_logic;
signal TG68_UDS_N	: std_logic;
signal TG68_LDS_N	: std_logic;
signal TG68_RNW		: std_logic;
signal TG68_INTACK	: std_logic;

signal TG68_ENARDREG	: std_logic;
signal TG68_ENAWRREG	: std_logic;

-- Z80
signal T80_RESET_N	: std_logic;
signal T80_CLK_N	: std_logic;
signal T80_CLKEN	: std_logic;
signal T80_WAIT_N	: std_logic;
signal T80_INT_N           : std_logic;
signal T80_NMI_N           : std_logic;
signal T80_BUSRQ_N         : std_logic;
signal T80_M1_N            : std_logic;
signal T80_MREQ_N          : std_logic;
signal T80_IORQ_N          : std_logic;
signal T80_RD_N            : std_logic;
signal T80_WR_N            : std_logic;
signal T80_RFSH_N          : std_logic;
signal T80_HALT_N          : std_logic;
signal T80_BUSAK_N         : std_logic;
signal T80_A               : std_logic_vector(15 downto 0);
signal T80_DI              : std_logic_vector(7 downto 0);
signal T80_DO              : std_logic_vector(7 downto 0);

-- CLOCK GENERATION
signal VCLK			: std_logic;
signal VCLKCNT		: std_logic_vector(2 downto 0);
-- signal VCLKCNT		: unsigned(2 downto 0);
signal ZCLK			: std_logic := '0';

signal ZCLKCNT		: std_logic_vector(3 downto 0);

-- FLASH CONTROL
signal TG68_FLASH_SEL		: std_logic;
signal TG68_FLASH_D			: std_logic_vector(15 downto 0);
signal TG68_FLASH_DTACK_N	: std_logic;

signal T80_FLASH_SEL		: std_logic;
signal T80_FLASH_D			: std_logic_vector(7 downto 0);
signal T80_FLASH_DTACK_N	: std_logic;

signal DMA_FLASH_SEL		: std_logic;
signal DMA_FLASH_D			: std_logic_vector(15 downto 0);
signal DMA_FLASH_DTACK_N	: std_logic;

-- SDRAM CONTROL
signal TG68_SDRAM_SEL		: std_logic;
signal TG68_SDRAM_D			: std_logic_vector(15 downto 0);
signal TG68_SDRAM_DTACK_N	: std_logic;

signal T80_SDRAM_SEL		: std_logic;
signal T80_SDRAM_D			: std_logic_vector(7 downto 0);
signal T80_SDRAM_DTACK_N	: std_logic;

signal DMA_SDRAM_SEL		: std_logic;
signal DMA_SDRAM_D			: std_logic_vector(15 downto 0);
signal DMA_SDRAM_DTACK_N	: std_logic;

-- OPERATING SYSTEM ROM
signal TG68_OS_SEL			: std_logic;
signal TG68_OS_D			: std_logic_vector(15 downto 0);
signal TG68_OS_DTACK_N		: std_logic;
signal OS_OEn				: std_logic;

-- CONTROL AREA
signal ZBUSREQ				: std_logic;
signal ZRESET_N				: std_logic;
signal ZBUSACK_N				: std_logic;
signal CART_EN				: std_logic;

signal TG68_CTRL_SEL		: std_logic;
signal TG68_CTRL_D			: std_logic_vector(15 downto 0);
signal TG68_CTRL_DTACK_N		: std_logic;

signal T80_CTRL_SEL		: std_logic;
signal T80_CTRL_D			: std_logic_vector(7 downto 0);
signal T80_CTRL_DTACK_N		: std_logic;

-- I/O AREA
signal IO_SEL				: std_logic;
signal IO_A 				: std_logic_vector(4 downto 0);
signal IO_RNW				: std_logic;
signal IO_UDS_N				: std_logic;
signal IO_LDS_N				: std_logic;
signal IO_DI				: std_logic_vector(15 downto 0);
signal IO_DO				: std_logic_vector(15 downto 0);
signal IO_DTACK_N			: std_logic;

signal TG68_IO_SEL		: std_logic;
signal TG68_IO_D			: std_logic_vector(15 downto 0);
signal TG68_IO_DTACK_N		: std_logic;

signal T80_IO_SEL		: std_logic;
signal T80_IO_D			: std_logic_vector(7 downto 0);
signal T80_IO_DTACK_N		: std_logic;

type ioc_t is ( IOC_IDLE, IOC_TG68_ACC, IOC_T80_ACC, IOC_DESEL );
signal IOC : ioc_t;

-- VDP AREA
signal VDP_SEL				: std_logic;
signal VDP_A 				: std_logic_vector(4 downto 0);
signal VDP_RNW				: std_logic;
signal VDP_UDS_N			: std_logic;
signal VDP_LDS_N			: std_logic;
signal VDP_DI				: std_logic_vector(15 downto 0);
signal VDP_DO				: std_logic_vector(15 downto 0);
signal VDP_DTACK_N			: std_logic;

signal TG68_VDP_SEL		: std_logic;
signal TG68_VDP_D			: std_logic_vector(15 downto 0);
signal TG68_VDP_DTACK_N		: std_logic;

signal T80_VDP_SEL		: std_logic;
signal T80_VDP_D			: std_logic_vector(7 downto 0);
signal T80_VDP_DTACK_N		: std_logic;

type vdpc_t is ( VDPC_IDLE, VDPC_TG68_ACC, VDPC_T80_ACC, VDPC_DESEL );
signal VDPC : vdpc_t;

signal INTERLACE	: std_logic;

-- FM AREA
signal FM_SEL				: std_logic;
signal FM_A 				: std_logic_vector(1 downto 0);
signal FM_RNW				: std_logic;
signal FM_UDS_N				: std_logic;
signal FM_LDS_N				: std_logic;
signal FM_DI				: std_logic_vector(15 downto 0);
signal FM_DO				: std_logic_vector(15 downto 0);
signal FM_DTACK_N			: std_logic;

signal TG68_FM_SEL		: std_logic;
signal TG68_FM_D			: std_logic_vector(15 downto 0);
signal TG68_FM_DTACK_N		: std_logic;

signal T80_FM_SEL		: std_logic;
signal T80_FM_D			: std_logic_vector(7 downto 0);
signal T80_FM_DTACK_N		: std_logic;

type fmc_t is ( FMC_IDLE, FMC_TG68_ACC, FMC_T80_ACC, FMC_DESEL );
signal FMC : fmc_t;


-- BANK ADDRESS REGISTER
signal BAR 					: std_logic_vector(23 downto 15);
signal TG68_BAR_SEL			: std_logic;
signal TG68_BAR_D			: std_logic_vector(15 downto 0);
signal TG68_BAR_DTACK_N		: std_logic;
signal T80_BAR_SEL			: std_logic;
signal T80_BAR_D			: std_logic_vector(7 downto 0);
signal T80_BAR_DTACK_N		: std_logic;

-- INTERRUPTS
signal HINT		: std_logic;
signal HINT_ACK	: std_logic;
signal VINT_TG68	: std_logic;
signal VINT_T80		: std_logic;
signal VINT_TG68_ACK	: std_logic;
signal VINT_T80_ACK	: std_logic;

-- VDP VBUS DMA
signal VBUS_ADDR	: std_logic_vector(23 downto 0);
signal VBUS_UDS_N	: std_logic;
signal VBUS_LDS_N	: std_logic;
signal VBUS_DATA	: std_logic_vector(15 downto 0);		
signal VBUS_SEL		: std_logic;
signal VBUS_DTACK_N	: std_logic;

-- DEBUG
signal HEXVALUE			: std_logic_vector(15 downto 0);

-- signal GPIO_CLKCNT	: std_logic_vector(15 downto 0);
signal GPIO_CLKCNT	: unsigned(15 downto 0);

signal GPIO_SEL		: std_logic;

signal PRE_RESET_N	: std_logic;
signal RSTCNT		: std_logic_vector(15 downto 0);
signal ROM_RESET_N	: std_logic := '0';
signal RESET_N		: std_logic := '0';

-- CPU signals
signal CPU_NMI_N	: std_logic;
signal CPU_IRQ1_N	: std_logic;
signal CPU_IRQ2_N	: std_logic;
signal CPU_RD_N		: std_logic;
signal CPU_WR_N		: std_logic;
signal CPU_DI		: std_logic_vector(7 downto 0);
signal CPU_DO		: std_logic_vector(7 downto 0);
signal CPU_A		: std_logic_vector(20 downto 0);
signal CPU_HSM		: std_logic;

signal CPU_CLKOUT	: std_logic;
signal CPU_CLKEN	: std_logic;
signal CPU_CLKRST	: std_logic;
signal CPU_RDY		: std_logic;

signal CPU_VCE_SEL_N	: std_logic;
signal CPU_VDC_SEL_N	: std_logic;
signal CPU_RAM_SEL_N	: std_logic;

signal CPU_IO_DI		: std_logic_vector(7 downto 0);  -- bit 6: country, bits 3-0: joypad data
signal CPU_IO_DO		: std_logic_vector(7 downto 0);  -- bit 1: clr, bit 0: sel

-- RAM signals
signal RAM_A		: std_logic_vector(12 downto 0);
signal RAM_DI		: std_logic_vector(7 downto 0);
signal RAM_WE		: std_logic;
signal RAM_DO		: std_logic_vector(7 downto 0);

-- ROM signals
signal HEADER		: std_logic;
signal SPLIT		: std_logic;
signal BITFLIP		: std_logic;

signal FL_RST_N_FF	: std_logic := '1';

-- VCE signals
signal VCE_DO		: std_logic_vector(7 downto 0);

-- VDC signals
signal VDC_DO		: std_logic_vector(7 downto 0);
signal VDC_BUSY_N	: std_logic;
signal VDC_IRQ_N	: std_logic;

-- VDP Video Output
signal VDP_RED		: std_logic_vector(3 downto 0);
signal VDP_GREEN	: std_logic_vector(3 downto 0);
signal VDP_BLUE	: std_logic_vector(3 downto 0);
signal VDP_VS_N	: std_logic;
signal VDP_HS_N	: std_logic;

-- NTSC/RGB Video Output
signal RED			: std_logic_vector(7 downto 0);
signal GREEN			: std_logic_vector(7 downto 0);
signal BLUE			: std_logic_vector(7 downto 0);		
signal VS_N			: std_logic;
signal HS_N			: std_logic;

-- VGA Video Output
signal VGA_RED			: std_logic_vector(7 downto 0);
signal VGA_GREEN			: std_logic_vector(7 downto 0);
signal VGA_BLUE			: std_logic_vector(7 downto 0);		
signal VGA_VS_N			: std_logic;
signal VGA_HS_N			: std_logic;

-- current video signal (switchable between TV and VGA)
signal vga_red_i : std_logic_vector(7 downto 0);
signal vga_green_i : std_logic_vector(7 downto 0);
signal vga_blue_i	: std_logic_vector(7 downto 0);		
signal vga_vsync_i : std_logic;
signal vga_hsync_i : std_logic;

-- VDC signals
signal VDC_COLNO	: std_logic_vector(8 downto 0);
signal VDC_CLKEN	: std_logic;


signal VDCBG_RAM_A	: std_logic_vector(15 downto 0);		
signal VDCBG_RAM_DO	: std_logic_vector(15 downto 0);
signal VDCBG_RAM_REQ	: std_logic;
signal VDCBG_RAM_ACK	: std_logic;
		
signal VDCSP_RAM_A	: std_logic_vector(15 downto 0);
signal VDCSP_RAM_DO	: std_logic_vector(15 downto 0);
signal VDCSP_RAM_REQ	: std_logic;
signal VDCSP_RAM_ACK	: std_logic;

signal VDCCPU_RAM_REQ	: std_logic;
signal VDCCPU_RAM_A	: std_logic_vector(15 downto 0);
signal VDCCPU_RAM_DO	: std_logic_vector(15 downto 0); -- Output from RAM
signal VDCCPU_RAM_DI	: std_logic_vector(15 downto 0);
signal VDCCPU_RAM_WE	: std_logic;
signal VDCCPU_RAM_ACK	: std_logic;

signal VDCDMA_RAM_REQ	: std_logic;
signal VDCDMA_RAM_A	: std_logic_vector(15 downto 0);
signal VDCDMA_RAM_DO	: std_logic_vector(15 downto 0); -- Output from RAM
signal VDCDMA_RAM_DI	: std_logic_vector(15 downto 0);
signal VDCDMA_RAM_WE	: std_logic;
signal VDCDMA_RAM_ACK	: std_logic;

signal VDCDMAS_RAM_REQ	: std_logic;
signal VDCDMAS_RAM_A		: std_logic_vector(15 downto 0);
signal VDCDMAS_RAM_DO		: std_logic_vector(15 downto 0); -- Output from RAM
signal VDCDMAS_RAM_ACK	: std_logic;


signal SDR_INIT_DONE	: std_logic;

type bootStates is (BOOT_READ_1, BOOT_READ_2, BOOT_WRITE_1, BOOT_WRITE_2, BOOT_WRITE_3, BOOT_WRITE_4, BOOT_REL, BOOT_DONE);
signal bootState : bootStates := BOOT_READ_1;
signal bootTimer : integer range 0 to 32767;

signal boot_a		: std_logic_vector(21 downto 0);
signal boot_oe_n	: std_logic;

signal romwr_req : std_logic := '0';
signal romwr_ack : std_logic;
signal romwr_we  : std_logic := '1';
signal romwr_a : unsigned(addrwidth downto 1);
signal romwr_d : std_logic_vector(15 downto 0);
signal romwr_q : std_logic_vector(15 downto 0);

--signal romrd_req : std_logic := '0';
--signal romrd_ack : std_logic;
--signal romrd_a : std_logic_vector(addrwidth downto 3);
--signal romrd_q : std_logic_vector(63 downto 0);
--signal romrd_a_cached : std_logic_vector(addrwidth downto 3);
--signal romrd_q_cached : std_logic_vector(63 downto 0);

signal host_reset_n : std_logic;
signal host_bootdone : std_logic;
signal rommap : std_logic_vector(1 downto 0);

signal boot_req : std_logic;
signal boot_ack : std_logic;
signal boot_data : std_logic_vector(15 downto 0);
signal FL_DQ : std_logic_vector(15 downto 0);

signal osd_window : std_logic;
signal osd_pixel : std_logic;

type romStates is (ROM_IDLE, ROM_READ);
signal romState : romStates := ROM_IDLE;

signal CPU_A_PREV : std_logic_vector(20 downto 0);
signal ROM_RDY	: std_logic;
signal ROM_DO	: std_logic_vector(7 downto 0);

signal SW : std_logic_vector(11 downto 0);
signal KEY : std_logic_vector(3 downto 0);

signal gp1emu : std_logic_vector(7 downto 0);
signal gp2emu : std_logic_vector(7 downto 0);

signal joya_merged : std_logic_vector(7 downto 0);
signal joyb_merged : std_logic_vector(7 downto 0);

signal gamepad_port : unsigned(2 downto 0);
signal multitap : std_logic :='1';
signal prev_sel : std_logic;

begin

-- Reset
PRE_RESET_N <= reset and SDR_INIT_DONE and host_reset_n;

-- Bit flipping switch
BITFLIP <= SW(2);
-- ROM splitting switch
SPLIT <= rommap(1);
multitap <= SW(4);

-- I/O
-- GPIO_1 <= (others => 'Z');


--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

-- CPU
--CPU : entity work.huc6280 port map(
--	CLK 	=> CLK,
--	RESET_N	=> RESET_N,
--	
--	NMI_N	=> CPU_NMI_N,
--	IRQ1_N	=> CPU_IRQ1_N,
--	IRQ2_N	=> CPU_IRQ2_N,

--	DI		=> CPU_DI,
--	DO 		=> CPU_DO,
	
--	HSM		=> CPU_HSM,
	
--	A 		=> CPU_A,
--	WR_N 	=> CPU_WR_N,
--	RD_N	=> CPU_RD_N,
	
--	CLKOUT	=> CPU_CLKOUT,
--	CLKRST	=> CPU_CLKRST,
--	RDY		=> CPU_RDY,
--	ROM_RDY	=> ROM_RDY,
	
--	CEK_N	=> CPU_VCE_SEL_N,
--	CE7_N	=> CPU_VDC_SEL_N,
--	CER_N	=> CPU_RAM_SEL_N,
	
--	K		=> CPU_IO_DI,
--	O		=> CPU_IO_DO,
	
--	DAC_LDATA => DAC_LDATA,
--	DAC_RDATA => DAC_RDATA
--);

-- RAM
--RAM : entity work.ram port map(
--	address	=> RAM_A,
--	clock	=> CLK,
--	data	=> RAM_DI,
--	wren	=> RAM_WE,
--	q		=> RAM_DO
--);

--VCE : entity work.huc6260 port map(
--	CLK 		=> CLK,
--	RESET_N		=> RESET_N,  -- Allow 6260 to emit sync signals even during reset.

	-- CPU Interface
--	A			=> CPU_A(2 downto 0),
--	CE_N		=> CPU_VCE_SEL_N,
--	WR_N		=> CPU_WR_N,
--	RD_N		=> CPU_RD_N,
--	DI			=> CPU_DO,
--	DO 			=> VCE_DO,
		
	-- VDC Interface
--	COLNO		=> VDC_COLNO,
--	CLKEN		=> VDC_CLKEN,
		
	-- NTSC/RGB Video Output
--	R			=> RED,
--	G			=> GREEN,
--	B			=> BLUE,
--	VS_N		=> VS_N,
--	HS_N		=> HS_N,
		
	-- VGA Video Output (Scandoubler)
--	VGA_R		=> VGA_RED,
--	VGA_G		=> VGA_GREEN,
--	VGA_B		=> VGA_BLUE,
--	VGA_VS_N	=> VGA_VS_N,
--	VGA_HS_N	=> VGA_HS_N
--);


--VDC : entity work.huc6270 port map(
--	CLK 		=> CLK,
--	RESET_N		=> RESET_N,

	-- CPU Interface
--	A			=> CPU_A(1 downto 0),
--	CE_N		=> CPU_VDC_SEL_N,
--	WR_N		=> CPU_WR_N,
--	RD_N		=> CPU_RD_N,
--	DI			=> CPU_DO,
--	DO 			=> VDC_DO,
--	BUSY_N		=> VDC_BUSY_N,
--	IRQ_N		=> VDC_IRQ_N,
	
--	BG_RAM_A	=> VDCBG_RAM_A,
--	BG_RAM_DO	=> VDCBG_RAM_DO,
--	BG_RAM_REQ	=> VDCBG_RAM_REQ,
--	BG_RAM_ACK	=> VDCBG_RAM_ACK,
	
--	SP_RAM_A	=> VDCSP_RAM_A,
--	SP_RAM_DO	=> VDCSP_RAM_DO,
--	SP_RAM_REQ	=> VDCSP_RAM_REQ,
--	SP_RAM_ACK	=> VDCSP_RAM_ACK,
	
--	CPU_RAM_REQ	=> VDCCPU_RAM_REQ,
--	CPU_RAM_A	=> VDCCPU_RAM_A,
--	CPU_RAM_DO	=> VDCCPU_RAM_DO,
--	CPU_RAM_DI	=> VDCCPU_RAM_DI,
--	CPU_RAM_WE	=> VDCCPU_RAM_WE,
--	CPU_RAM_ACK	=> VDCCPU_RAM_ACK,
	
--	DMA_RAM_REQ => VDCDMA_RAM_REQ,
--	DMA_RAM_A	=> VDCDMA_RAM_A,
--	DMA_RAM_DO	=> VDCDMA_RAM_DO,
--	DMA_RAM_DI	=> VDCDMA_RAM_DI,
--	DMA_RAM_WE	=> VDCDMA_RAM_WE,
--	DMA_RAM_ACK	=> VDCDMA_RAM_ACK,
	
--	DMAS_RAM_REQ	=> VDCDMAS_RAM_REQ,
--	DMAS_RAM_A		=> VDCDMAS_RAM_A,
--	DMAS_RAM_DO		=> VDCDMAS_RAM_DO,
--	DMAS_RAM_ACK	=> VDCDMAS_RAM_ACK,
	
	-- VCE Interface
--	COLNO		=> VDC_COLNO,
--	CLKEN		=> VDC_CLKEN,
--	HS_N		=> HS_N,
--	VS_N		=> VS_N

--);
-- VDC_RAM_A_FULL <= "00" & "1000" & VDC_RAM_A;


--SDRC : entity work.sdram_controller
--	generic map(
--		colAddrBits => colAddrBits,
--		rowAddrBits => rowAddrBits
--	)
--	port map(
--	clk			=> SDR_CLK,
	
--	std_logic_vector(sd_data) => DRAM_DQ,
--	std_logic_vector(sd_addr) => DRAM_ADDR,
--	sd_we_n		=> DRAM_WE_N,
--	sd_ras_n	=> DRAM_RAS_N,
--	sd_cas_n	=> DRAM_CAS_N,
--	sd_ba_0		=> DRAM_BA_0,
--	sd_ba_1		=> DRAM_BA_1,
--	sd_ldqm		=> DRAM_LDQM,
--	sd_udqm		=> DRAM_UDQM,
	
--	vdccpu_req		=> VDCCPU_RAM_REQ,
--	vdccpu_ack		=> VDCCPU_RAM_ACK,
--	vdccpu_we		=> VDCCPU_RAM_WE,
--	vdccpu_a		=> VDCCPU_RAM_A,
--	vdccpu_d		=> VDCCPU_RAM_DI,
--	vdccpu_q		=> VDCCPU_RAM_DO,

--	vdcbg_a	=> VDCBG_RAM_A,
--	vdcbg_q	=> VDCBG_RAM_DO,
--	vdcbg_req	=> VDCBG_RAM_REQ,
--	vdcbg_ack	=> VDCBG_RAM_ACK,
	
--	vdcsp_a	=> VDCSP_RAM_A,
--	vdcsp_q	=> VDCSP_RAM_DO,
--	vdcsp_req	=> VDCSP_RAM_REQ,
--	vdcsp_ack	=> VDCSP_RAM_ACK,
	
--	vdcdma_req => VDCDMA_RAM_REQ,
--	vdcdma_a	=> VDCDMA_RAM_A,
--	vdcdma_q	=> VDCDMA_RAM_DO,
--	vdcdma_d	=> VDCDMA_RAM_DI,
--	vdcdma_we	=> VDCDMA_RAM_WE,
--	vdcdma_ack	=> VDCDMA_RAM_ACK,
	
--	vdcdmas_req	=> VDCDMAS_RAM_REQ,
--	vdcdmas_a		=> VDCDMAS_RAM_A,
--	vdcdmas_q		=> VDCDMAS_RAM_DO,
--	vdcdmas_ack	=> VDCDMAS_RAM_ACK,
	
--	romwr_req	=> romwr_req,
--	romwr_ack	=> romwr_ack,
--	romwr_a		=> romwr_a,
--	romwr_d		=> romwr_d,
	
--	romrd_req	=> romrd_req,
--	romrd_ack	=> romrd_ack,
--	romrd_a		=> romrd_a,
--	romrd_q		=> romrd_q,
	
--	initDone 	=> SDR_INIT_DONE
--);

-- -----------------------------------------------------------------------
-- Clocks and PLL
-- -----------------------------------------------------------------------
MCLK <= CLK;
MRST_N <= reset;

-- -----------------------------------------------------------------------
-- SDRAM Controller
-- -----------------------------------------------------------------------		
sdc : entity work.sdram_controller port map(
	clk			=> SDR_CLK,
	
	std_logic_vector(sd_data)	=> DRAM_DQ,
	std_logic_vector(sd_addr)	=> DRAM_ADDR,
	sd_we_n							=> DRAM_WE_N,
	sd_ras_n							=> DRAM_RAS_N,
	sd_cas_n							=> DRAM_CAS_N,
	sd_ba_0							=> DRAM_BA_0,
	sd_ba_1							=> DRAM_BA_1,
	sd_ldqm							=> DRAM_LDQM,
	sd_udqm							=> DRAM_UDQM,
		
	romwr_req	=> romwr_req,
	romwr_ack	=> romwr_ack,
	romwr_we 	=> romwr_we,
	romwr_a		=> std_logic_vector(romwr_a),
	romwr_d		=> romwr_d,
	romwr_q		=> romwr_q,
	
	romrd_req	=> romrd_req,
	romrd_ack	=> romrd_ack,
	romrd_a		=> romrd_a,
	romrd_q		=> romrd_q,

	ram68k_req	=> ram68k_req,
	ram68k_ack	=> ram68k_ack,
	ram68k_we	=> ram68k_we,
	ram68k_a		=> ram68k_a,
	ram68k_d		=> ram68k_d,
	ram68k_q		=> ram68k_q,
	ram68k_u_n	=> ram68k_u_n,
	ram68k_l_n	=> ram68k_l_n,

	vram_req	=> vram_req,
	vram_ack => vram_ack,
	vram_we	=> vram_we,
	vram_a	=> vram_a,
	vram_d	=> vram_d,
	vram_q	=> vram_q,
	vram_u_n => vram_u_n,
	vram_l_n => vram_l_n,
	
	initDone 	=> SDR_INIT_DONE
);

-- -----------------------------------------------------------------------
-- Z80 RAM
-- -----------------------------------------------------------------------
zr : entity work.zram port map (
	address	=> zram_a,
	clock		=> MCLK,
	data		=> zram_d,
	wren		=> zram_we,
	q			=> zram_q
);

-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- Genesis Core
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------

-- 68K
tg68 : entity work.TG68 
port map(
	-- clk			=> TG68_CLK,
	clk			=> MCLK,
	reset			=> TG68_RES_N,
	clkena_in	=> TG68_CLKE,
	data_in		=> TG68_DI,
	IPL			=> TG68_IPL_N,
	dtack			=> TG68_DTACK_N,
	addr			=> TG68_A,
	data_out		=> TG68_DO,
	as				=> TG68_AS_N,
	uds			=> TG68_UDS_N,
	lds			=> TG68_LDS_N,
	rw				=> TG68_RNW,
	enaRDreg		=> TG68_ENARDREG,
	enaWRreg		=> TG68_ENAWRREG,
	intack		=> TG68_INTACK
);

-- Z80
t80 : entity work.t80se
port map(
	RESET_n		=> T80_RESET_N,
	CLK_n		=> T80_CLK_N,
	CLKEN		=> T80_CLKEN,
	WAIT_n	=> T80_WAIT_N,
	INT_n		=> T80_INT_N,
	NMI_n		=> T80_NMI_N,
	BUSRQ_n	=> T80_BUSRQ_N,
	M1_n		=> T80_M1_N,
	MREQ_n	=> T80_MREQ_N,
	IORQ_n	=> T80_IORQ_N,
	RD_n		=> T80_RD_N,
	WR_n		=> T80_WR_N,
	RFSH_n	=> T80_RFSH_N,
	HALT_n	=> T80_HALT_N,
	BUSAK_n	=> T80_BUSAK_N,
	A			=> T80_A,
	DI			=> T80_DI,
	DO			=> T80_DO
);

-- OS ROM
os : entity work.os_rom
port map(
	A			=> TG68_A(8 downto 1),
	OEn		=> OS_OEn,
	D			=> TG68_OS_D
);

-- VDP
vdp : entity work.vdp
port map(
	RST_N		=> MRST_N,
	CLK		=> MCLK,
		
	SEL		=> VDP_SEL,
	A			=> VDP_A,
	RNW		=> VDP_RNW,
	UDS_N		=> VDP_UDS_N,
	LDS_N		=> VDP_LDS_N,
	DI			=> VDP_DI,
	DO			=> VDP_DO,
	DTACK_N		=> VDP_DTACK_N,

	vram_req => vram_req,
	vram_ack => vram_ack,
	vram_we	=> vram_we,
	vram_a	=> vram_a,
	vram_d	=> vram_d,
	vram_q	=> vram_q,
	vram_u_n	=> vram_u_n,
	vram_l_n	=> vram_l_n,
	
	INTERLACE	=> INTERLACE,

	HINT			=> HINT,
	HINT_ACK		=> HINT_ACK,

	VINT_TG68		=> VINT_TG68,
	VINT_T80			=> VINT_T80,
	VINT_TG68_ACK	=> VINT_TG68_ACK,
	VINT_T80_ACK	=> VINT_T80_ACK,
		
	VBUS_ADDR		=> VBUS_ADDR,
	VBUS_UDS_N		=> VBUS_UDS_N,
	VBUS_LDS_N		=> VBUS_LDS_N,
	VBUS_DATA		=> VBUS_DATA,
		
	VBUS_SEL			=> VBUS_SEL,
	VBUS_DTACK_N	=> VBUS_DTACK_N,
	
	VGA_R				=> VDP_RED,
	VGA_G				=> VDP_GREEN,
	VGA_B				=> VDP_BLUE,
	VGA_HS			=> VDP_HS_N,
	VGA_VS			=> VDP_VS_N
);

-- FM
fm : entity work.gen_fm
port map(
	RST_N		=> T80_RESET_N,	-- gen-hw.txt line 328
	CLK		=> VCLK,
		
	SEL		=> FM_SEL,
	A			=> FM_A,
	RNW		=> FM_RNW,
	UDS_N		=> FM_UDS_N,
	LDS_N		=> FM_LDS_N,
	DI			=> FM_DI,
	DO			=> FM_DO,
	DTACK_N	=> FM_DTACK_N
);

----------------------------------------------------------------
-- INTERRUPTS CONTROL
----------------------------------------------------------------

-- HINT_ACK <= HINT;
-- VINT_TG68_ACK <= VINT_TG68;
-- VINT_T80_ACK <= VINT_T80;

-- TG68_IPL_N <= "111";
process(MRST_N, MCLK)
begin
	if MRST_N = '0' then
		TG68_IPL_N <= "111";
		T80_INT_N <= '1';
		
		HINT_ACK <= '0';
		VINT_TG68_ACK <= '0';
		VINT_T80_ACK <= '0';
	elsif rising_edge( MCLK ) then
		if HINT = '0' then
			HINT_ACK <= '0';
		end if;
		if VINT_TG68 = '0' then
			VINT_TG68_ACK <= '0';
		end if;
		if VINT_T80 = '0' then
			VINT_T80_ACK <= '0';
		end if;
		if TG68_INTACK = '1' then
			VINT_TG68_ACK <= '1';
		end if;				
		if TG68_INTACK = '1' then
			HINT_ACK <= '1';
		end if;

		if VCLKCNT = "110" then
			if VINT_TG68 = '1' and VINT_TG68_ACK = '0' then
				TG68_IPL_N <= "001";	-- IPL Level 6
				-- if TG68_INTACK = '1' then
					-- VINT_TG68_ACK <= '1';
				-- end if;				
			elsif HINT = '1' and HINT_ACK = '0' then
				TG68_IPL_N <= "011";	-- IPL Level 4
				-- if TG68_INTACK = '1' then
					-- HINT_ACK <= '1';
				-- end if;
			else
				TG68_IPL_N <= "111";
			end if;
			
			if ZCLK = '0' then
				if VINT_T80 = '1' and VINT_T80_ACK = '0' then
					T80_INT_N <= '0';
					if T80_M1_N = '0' and T80_IORQ_N = '0' then
						VINT_T80_ACK <= '1';
					end if;
				else
					T80_INT_N <= '1';
				end if;
			end if;
			
		end if;
			
	end if;
end process;

----------------------------------------------------------------
-- SWITCHES CONTROL
----------------------------------------------------------------
INTERLACE <= '0';


-- #############################################################################
-- #############################################################################
-- #############################################################################

-- CLOCK GENERATION
process( MRST_N, MCLK, VCLKCNT )
begin
	if MRST_N = '0' then
		VCLK <= '1';
		ZCLK <= '0';
		VCLKCNT <= "001"; -- important for SDRAM controller (EDIT: not needed anymore)
		TG68_ENARDREG <= '0';
		TG68_ENAWRREG <= '0';
	elsif rising_edge(MCLK) then
		VCLKCNT <= VCLKCNT + 1;
		if VCLKCNT = "000" then
			ZCLK <= not ZCLK;
		end if;
		if VCLKCNT = "110" then
			VCLKCNT <= "000";
		end if;
		if VCLKCNT <= "011" then
			VCLK <= '1';
		else
			VCLK <= '0';
		end if;
		
		if VCLKCNT = "110" then
			TG68_ENAWRREG <= '1';
		else
			TG68_ENAWRREG <= '0';
		end if;
		
		if VCLKCNT = "011" then
			TG68_ENARDREG <= '1';
		else
			TG68_ENARDREG <= '0';
		end if;
		
	end if;
end process;

process( MRST_N, ZCLK )
begin
	if MRST_N = '0' then
		T80_CLKEN <= '1';
		ZCLKCNT <= (others => '0');
	elsif falling_edge( ZCLK ) then
		ZCLKCNT <= ZCLKCNT + 1;
		T80_CLKEN <= '1';
		if ZCLKCNT = "1110" then
			ZCLKCNT <= (others => '0');
			T80_CLKEN <= '0';
		end if;
	end if;
end process;

-- DMA VBUS
VBUS_DTACK_N <= DMA_FLASH_DTACK_N when DMA_FLASH_SEL = '1'
	else DMA_SDRAM_DTACK_N when DMA_SDRAM_SEL = '1'
	else '0';
VBUS_DATA <= DMA_FLASH_D when DMA_FLASH_SEL = '1'
	else DMA_SDRAM_D when DMA_SDRAM_SEL = '1'
	else x"FFFF";

-- 68K INPUTS
TG68_RES_N <= MRST_N;
TG68_CLK <= VCLK;
TG68_CLKE <= '1';

TG68_DTACK_N <= TG68_FLASH_DTACK_N when TG68_FLASH_SEL = '1'
	else TG68_SDRAM_DTACK_N when TG68_SDRAM_SEL = '1' 
	else TG68_ZRAM_DTACK_N when TG68_ZRAM_SEL = '1' 
	else TG68_CTRL_DTACK_N when TG68_CTRL_SEL = '1' 
	else TG68_OS_DTACK_N when TG68_OS_SEL = '1' 
	else TG68_IO_DTACK_N when TG68_IO_SEL = '1' 
	else TG68_BAR_DTACK_N when TG68_BAR_SEL = '1' 
	else TG68_VDP_DTACK_N when TG68_VDP_SEL = '1' 
	else TG68_FM_DTACK_N when TG68_FM_SEL = '1' 
	else '0';
TG68_DI(15 downto 8) <= TG68_FLASH_D(15 downto 8) when TG68_FLASH_SEL = '1' and TG68_UDS_N = '0'
	else TG68_SDRAM_D(15 downto 8) when TG68_SDRAM_SEL = '1' and TG68_UDS_N = '0'
	else TG68_ZRAM_D(15 downto 8) when TG68_ZRAM_SEL = '1' and TG68_UDS_N = '0'
	else TG68_CTRL_D(15 downto 8) when TG68_CTRL_SEL = '1' and TG68_UDS_N = '0'
	else TG68_OS_D(15 downto 8) when TG68_OS_SEL = '1' and TG68_UDS_N = '0'
	else TG68_IO_D(15 downto 8) when TG68_IO_SEL = '1' and TG68_UDS_N = '0'
	else TG68_BAR_D(15 downto 8) when TG68_BAR_SEL = '1' and TG68_UDS_N = '0'
	else TG68_VDP_D(15 downto 8) when TG68_VDP_SEL = '1' and TG68_UDS_N = '0'
	else TG68_FM_D(15 downto 8) when TG68_FM_SEL = '1' and TG68_UDS_N = '0'
	else NO_DATA(15 downto 8);
TG68_DI(7 downto 0) <= TG68_FLASH_D(7 downto 0) when TG68_FLASH_SEL = '1' and TG68_LDS_N = '0'
	else TG68_SDRAM_D(7 downto 0) when TG68_SDRAM_SEL = '1' and TG68_LDS_N = '0'
	else TG68_ZRAM_D(7 downto 0) when TG68_ZRAM_SEL = '1' and TG68_LDS_N = '0'
	else TG68_CTRL_D(7 downto 0) when TG68_CTRL_SEL = '1' and TG68_LDS_N = '0'
	else TG68_OS_D(7 downto 0) when TG68_OS_SEL = '1' and TG68_LDS_N = '0'
	else TG68_IO_D(7 downto 0) when TG68_IO_SEL = '1' and TG68_LDS_N = '0'
	else TG68_BAR_D(7 downto 0) when TG68_BAR_SEL = '1' and TG68_LDS_N = '0'
	else TG68_VDP_D(7 downto 0) when TG68_VDP_SEL = '1' and TG68_LDS_N = '0'
	else TG68_FM_D(7 downto 0) when TG68_FM_SEL = '1' and TG68_LDS_N = '0'
	else NO_DATA(7 downto 0);

-- Z80 INPUTS
process(MRST_N, MCLK, ZRESET_N, ZBUSREQ)
begin
	if MRST_N = '0' then
		T80_RESET_N <= '0';
	elsif rising_edge(MCLK) then
		if T80_RESET_N = '0' then
			if ZBUSREQ = '0' and ZRESET_N = '1' then
				T80_RESET_N <= '1';
			end if;
			ZBUSACK_N <= not ZBUSREQ;
		else
			if ZRESET_N = '0' then
				T80_RESET_N <= '0';
			end if;
			ZBUSACK_N <= T80_BUSAK_N;
		end if;
	end if;
end process;

T80_CLK_N <= ZCLK;
--T80_INT_N <= '1';
T80_NMI_N <= '1';
T80_BUSRQ_N <= not ZBUSREQ;

T80_WAIT_N <= not T80_SDRAM_DTACK_N when T80_SDRAM_SEL = '1'
	else not T80_ZRAM_DTACK_N when T80_ZRAM_SEL = '1'
	else not T80_FLASH_DTACK_N when T80_FLASH_SEL = '1'
	else not T80_CTRL_DTACK_N when T80_CTRL_SEL = '1' 
	else not T80_IO_DTACK_N when T80_IO_SEL = '1' 
	else not T80_BAR_DTACK_N when T80_BAR_SEL = '1'
	else not T80_VDP_DTACK_N when T80_VDP_SEL = '1'
	else not T80_FM_DTACK_N when T80_FM_SEL = '1'
	else '1';
T80_DI <= T80_SDRAM_D when T80_SDRAM_SEL = '1'
	else T80_ZRAM_D when T80_ZRAM_SEL = '1'
	else T80_FLASH_D when T80_FLASH_SEL = '1'
	else T80_CTRL_D when T80_CTRL_SEL = '1'
	else T80_IO_D when T80_IO_SEL = '1'
	else T80_BAR_D when T80_BAR_SEL = '1'
	else T80_VDP_D when T80_VDP_SEL = '1'
	else T80_FM_D when T80_FM_SEL = '1'
	else x"FF";

-- OPERATING SYSTEM ROM
TG68_OS_DTACK_N <= '0';
OS_OEn <= '0';
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N )
begin

	if TG68_A(23 downto 22) = "00" 
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
		and TG68_RNW = '1' 
		and CART_EN = '0'
	then
		TG68_OS_SEL <= '1';
	else
		TG68_OS_SEL <= '0';
	end if;

end process;


-- CONTROL AREA
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N)
begin
	if (TG68_A(23 downto 12) = x"A11" or TG68_A(23 downto 12) = x"A14")
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_CTRL_SEL <= '1';
	else
		TG68_CTRL_SEL <= '0';
	end if;

	if T80_A(15) = '1' and (BAR(23 downto 15) & T80_A(14 downto 12) = x"A11" or BAR(23 downto 15) & T80_A(14 downto 12) = x"A14")
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_CTRL_SEL <= '1';		
	else
		T80_CTRL_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_CTRL_DTACK_N <= '1';	
		T80_CTRL_DTACK_N <= '1';	
		
		ZBUSREQ <= '0';
		ZRESET_N <= '0';
		CART_EN <= '0';
		
	elsif rising_edge(MCLK) then
		if TG68_CTRL_SEL = '0' then 
			TG68_CTRL_DTACK_N <= '1';
		end if;
		if T80_CTRL_SEL = '0' then 
			T80_CTRL_DTACK_N <= '1';
		end if;
		
		if TG68_CTRL_SEL = '1' and TG68_CTRL_DTACK_N = '1' then
			TG68_CTRL_DTACK_N <= '0';
			if TG68_RNW = '0' then
				-- Write
				if TG68_A(15 downto 8) = x"11" then
					-- ZBUSREQ
					if TG68_UDS_N = '0' then
						ZBUSREQ <= TG68_DO(8);
					end if;
				elsif TG68_A(15 downto 8) = x"12" then
					-- ZRESET_N
					if TG68_UDS_N = '0' then
						ZRESET_N <= TG68_DO(8);
					end if;			
				elsif TG68_A(15 downto 8) = x"41" then
					-- Cartridge Control Register
					if TG68_LDS_N = '0' then
						CART_EN <= TG68_DO(0);
					end if;								
				end if;
			else
				-- Read
				TG68_CTRL_D <= NO_DATA;
				if TG68_A(15 downto 8) = x"11" then
					-- ZBUSACK_N
					TG68_CTRL_D(8) <= ZBUSACK_N;
					TG68_CTRL_D(0) <= ZBUSACK_N;
				end if;
			end if;		
		elsif T80_CTRL_SEL = '1' and T80_CTRL_DTACK_N = '1' then
			T80_CTRL_DTACK_N <= '0';
			if T80_WR_N = '0' then
				-- Write
				if BAR(15) & T80_A(14 downto 8) = x"11" then
					-- ZBUSREQ
					if T80_A(0) = '0' then
						ZBUSREQ <= T80_DO(0);
					end if;
				elsif BAR(15) & T80_A(14 downto 8) = x"12" then
					-- ZRESET_N
					if T80_A(0) = '0' then
						ZRESET_N <= T80_DO(0);
					end if;			
				elsif BAR(15) & T80_A(14 downto 8) = x"41" then
					-- Cartridge Control Register
					if T80_A(0) = '1' then
						CART_EN <= T80_DO(0);
					end if;								
				end if;
			else
				-- Read
				T80_CTRL_D <= x"FF";
				if BAR(15) & T80_A(14 downto 8) = x"11" and T80_A(0) = '0' then
					-- ZBUSACK_N
					T80_CTRL_D(0) <= ZBUSACK_N;
				end if;
			end if;			
		end if;
		
	end if;
	
end process;

-- I/O AREA
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N )
begin
	if TG68_A(23 downto 5) = x"A100" & "000"
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_IO_SEL <= '1';		
	else
		TG68_IO_SEL <= '0';
	end if;

	if T80_A(15) = '1' and BAR & T80_A(14 downto 5) = x"A100" & "000"
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_IO_SEL <= '1';		
	else
		T80_IO_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_IO_DTACK_N <= '1';	
		T80_IO_DTACK_N <= '1';	
		
		IO_SEL <= '0';
		IO_RNW <= '1';
		IO_UDS_N <= '1';
		IO_LDS_N <= '1';
		IO_A <= (others => 'Z');

		IOC <= IOC_IDLE;
		
	elsif rising_edge(MCLK) then
		if TG68_IO_SEL = '0' then 
			TG68_IO_DTACK_N <= '1';
		end if;
		if T80_IO_SEL = '0' then 
			T80_IO_DTACK_N <= '1';
		end if;

		case IOC is
		when IOC_IDLE =>
			if TG68_IO_SEL = '1' and TG68_IO_DTACK_N = '1' then
				IO_SEL <= '1';
				IO_A <= TG68_A(4 downto 0);
				IO_RNW <= TG68_RNW;
				IO_UDS_N <= TG68_UDS_N;
				IO_LDS_N <= TG68_LDS_N;
				IO_DI <= TG68_DO;
				IOC <= IOC_TG68_ACC;
			elsif T80_IO_SEL = '1' and T80_IO_DTACK_N = '1' then
				IO_SEL <= '1';
				IO_A <= T80_A(4 downto 0);
				IO_RNW <= T80_WR_N;
				if T80_A(0) = '0' then
					IO_UDS_N <= '0';
					IO_LDS_N <= '1';
				else
					IO_UDS_N <= '1';
					IO_LDS_N <= '0';				
				end if;
				IO_DI <= T80_DO & T80_DO;
				IOC <= IOC_T80_ACC;			
			end if;

		when IOC_TG68_ACC =>
			if IO_DTACK_N = '0' then
				IO_SEL <= '0';
				TG68_IO_D <= IO_DO;
				TG68_IO_DTACK_N <= '0';
				IOC <= IOC_DESEL;
			end if;

		when IOC_T80_ACC =>
			if IO_DTACK_N = '0' then
				IO_SEL <= '0';
				if T80_A(0) = '0' then
					T80_IO_D <= IO_DO(15 downto 8);
				else
					T80_IO_D <= IO_DO(7 downto 0);
				end if;
				T80_IO_DTACK_N <= '0';
				IOC <= IOC_DESEL;
			end if;
		
		when IOC_DESEL =>
			if IO_DTACK_N = '1' then
				IO_RNW <= '1';
				IO_UDS_N <= '1';
				IO_LDS_N <= '1';
				IO_A <= (others => 'Z');

				IOC <= IOC_IDLE;
			end if;
		
		when others => null;
		end case;
	end if;
	
end process;


-- VDP in Z80 address space :
-- Z80:
-- 7F = 01111111 000
-- 68000:
-- 7F = 01111111 000
-- FF = 11111111 000
-- VDP AREA
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N )
begin
	if TG68_A(23 downto 21) = "110" and TG68_A(18 downto 16) = "000"
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_VDP_SEL <= '1';		
	elsif TG68_A(23 downto 16) = x"A0" and TG68_A(14 downto 5) = "1111111" & "000" -- Z80 Address space
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then
		TG68_VDP_SEL <= '1';
	else
		TG68_VDP_SEL <= '0';
	end if;

	if T80_A(15 downto 5) = x"7F" & "000"
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_VDP_SEL <= '1';			
	elsif T80_A(15) = '1' and BAR(23 downto 21) = "110" and BAR(18 downto 16) = "000" -- 68000 Address space
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_VDP_SEL <= '1';		
	else
		T80_VDP_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_VDP_DTACK_N <= '1';	
		T80_VDP_DTACK_N <= '1';	
		
		VDP_SEL <= '0';
		VDP_RNW <= '1';
		VDP_UDS_N <= '1';
		VDP_LDS_N <= '1';
		VDP_A <= (others => 'Z');

		VDPC <= VDPC_IDLE;

		HEXVALUE <= x"0000";
		
	elsif rising_edge(MCLK) then
		if TG68_VDP_SEL = '0' then 
			TG68_VDP_DTACK_N <= '1';
		end if;
		if T80_VDP_SEL = '0' then 
			T80_VDP_DTACK_N <= '1';
		end if;

		case VDPC is
		when VDPC_IDLE =>
			if TG68_VDP_SEL = '1' and TG68_VDP_DTACK_N = '1' then
				if TG68_A(4) = '1' then 
					-- PSG (used for debug)
					if TG68_A(3 downto 1) = "000" and TG68_LDS_N = '0' and TG68_RNW = '0' then
						HEXVALUE(15 downto 8) <= TG68_DO(7 downto 0);
					end if;
					if TG68_A(3 downto 1) = "001" and TG68_LDS_N = '0' and TG68_RNW = '0' then
						HEXVALUE(7 downto 0) <= TG68_DO(7 downto 0);
					end if;					
					TG68_VDP_D <= x"FFFF";
					TG68_VDP_DTACK_N <= '0';
				else
					-- VDP
					VDP_SEL <= '1';
					VDP_A <= TG68_A(4 downto 0);
					VDP_RNW <= TG68_RNW;
					VDP_UDS_N <= TG68_UDS_N;
					VDP_LDS_N <= TG68_LDS_N;
					VDP_DI <= TG68_DO;
					VDPC <= VDPC_TG68_ACC;
				end if;				
			elsif T80_VDP_SEL = '1' and T80_VDP_DTACK_N = '1' then
				if T80_A(4) = '1' then
					-- PSG (used for debug)
					if T80_A(3 downto 0) = "0001" and T80_WR_N = '0' then
						HEXVALUE(15 downto 8) <= T80_DO;
					end if;
					if T80_A(3 downto 0) = "0011" and T80_WR_N = '0' then
						HEXVALUE(7 downto 0) <= T80_DO;
					end if;					
					T80_VDP_D <= x"FF";
					T80_VDP_DTACK_N <= '0';
				else
					VDP_SEL <= '1';
					VDP_A <= T80_A(4 downto 0);
					VDP_RNW <= T80_WR_N;
					if T80_A(0) = '0' then
						VDP_UDS_N <= '0';
						VDP_LDS_N <= '1';
					else
						VDP_UDS_N <= '1';
						VDP_LDS_N <= '0';				
					end if;
					VDP_DI <= T80_DO & T80_DO;
					VDPC <= VDPC_T80_ACC;			
				end if;
			end if;

		when VDPC_TG68_ACC =>
			if VDP_DTACK_N = '0' then
				VDP_SEL <= '0';
				TG68_VDP_D <= VDP_DO;
				TG68_VDP_DTACK_N <= '0';
				VDPC <= VDPC_DESEL;
			end if;

		when VDPC_T80_ACC =>
			if VDP_DTACK_N = '0' then
				VDP_SEL <= '0';
				if T80_A(0) = '0' then
					T80_VDP_D <= VDP_DO(15 downto 8);
				else
					T80_VDP_D <= VDP_DO(7 downto 0);
				end if;
				T80_VDP_DTACK_N <= '0';
				VDPC <= VDPC_DESEL;
			end if;

		when VDPC_DESEL =>
			if VDP_DTACK_N = '1' then
				VDP_RNW <= '1';
				VDP_UDS_N <= '1';
				VDP_LDS_N <= '1';
				VDP_A <= (others => 'Z');

				VDPC <= VDPC_IDLE;
			end if;
			
		when others => null;
		end case;
	end if;
	
end process;

-- Z80:
-- 40 = 01000000
-- 5F = 01011111
-- 68000:
-- 40 = 01000000
-- 5F = 01011111
-- C0 = 11000000
-- DF = 11011111
-- FM AREA
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N )
begin
	if TG68_A(23 downto 16) = x"A0" and TG68_A(14 downto 13) = "10"
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_FM_SEL <= '1';		
	else
		TG68_FM_SEL <= '0';
	end if;

	if T80_A(15 downto 13) = "010"
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_FM_SEL <= '1';			
	else
		T80_FM_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_FM_DTACK_N <= '1';	
		T80_FM_DTACK_N <= '1';	
		
		FM_SEL <= '0';
		FM_RNW <= '1';
		FM_UDS_N <= '1';
		FM_LDS_N <= '1';
		FM_A <= (others => 'Z');
		
		FMC <= FMC_IDLE;
		
	elsif rising_edge(MCLK) then
		if TG68_FM_SEL = '0' then 
			TG68_FM_DTACK_N <= '1';
		end if;
		if T80_FM_SEL = '0' then 
			T80_FM_DTACK_N <= '1';
		end if;

		case FMC is
		when FMC_IDLE =>
			if TG68_FM_SEL = '1' and TG68_FM_DTACK_N = '1' then
				FM_SEL <= '1';
				FM_A <= TG68_A(1 downto 0);
				FM_RNW <= TG68_RNW;
				FM_UDS_N <= TG68_UDS_N;
				FM_LDS_N <= TG68_LDS_N;
				FM_DI <= TG68_DO;
				FMC <= FMC_TG68_ACC;
			elsif T80_FM_SEL = '1' and T80_FM_DTACK_N = '1' then
				FM_SEL <= '1';
				FM_A <= T80_A(1 downto 0);
				FM_RNW <= T80_WR_N;
				if T80_A(0) = '0' then
					FM_UDS_N <= '0';
					FM_LDS_N <= '1';
				else
					FM_UDS_N <= '1';
					FM_LDS_N <= '0';				
				end if;
				FM_DI <= T80_DO & T80_DO;
				FMC <= FMC_T80_ACC;			
			end if;

		when FMC_TG68_ACC =>
			if FM_DTACK_N = '0' then
				FM_SEL <= '0';
				TG68_FM_D <= FM_DO;
				TG68_FM_DTACK_N <= '0';
				FMC <= FMC_DESEL;
			end if;

		when FMC_T80_ACC =>
			if FM_DTACK_N = '0' then
				FM_SEL <= '0';
				if T80_A(0) = '0' then
					T80_FM_D <= FM_DO(15 downto 8);
				else
					T80_FM_D <= FM_DO(7 downto 0);
				end if;
				T80_FM_DTACK_N <= '0';
				FMC <= FMC_DESEL;
			end if;

		when FMC_DESEL =>
			if FM_DTACK_N = '1' then
				FM_RNW <= '1';
				FM_UDS_N <= '1';
				FM_LDS_N <= '1';
				FM_A <= (others => 'Z');

				FMC <= FMC_IDLE;
			end if;
			
		when others => null;
		end case;
	end if;
	
end process;


-- Z80:
-- 60 = 01100000
-- 7E = 01111110
-- 68000:
-- 60 = 01100000
-- 7E = 01111110
-- E0 = 11100000
-- FE = 11111110
-- BANK ADDRESS REGISTER AND UNUSED AREA IN Z80 ADDRESS SPACE
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N )
begin

	if (TG68_A(23 downto 16) = x"A0" and TG68_A(14 downto 13) = "11" and TG68_A(12 downto 8) /= "11111")
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_BAR_SEL <= '1';		
	else
		TG68_BAR_SEL <= '0';
	end if;

	if (T80_A(15 downto 13) = "011" and T80_A(12 downto 8) /= "11111")
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_BAR_SEL <= '1';
	else
		T80_BAR_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_BAR_DTACK_N <= '1';	
		T80_BAR_DTACK_N <= '1';
		
		BAR <= (others => '0');
		
	elsif rising_edge(MCLK) then
		if TG68_BAR_SEL = '0' then 
			TG68_BAR_DTACK_N <= '1';
		end if;
		if T80_BAR_SEL = '0' then 
			T80_BAR_DTACK_N <= '1';
		end if;

		if TG68_BAR_SEL = '1' and TG68_BAR_DTACK_N = '1' then
			if TG68_RNW = '0' then
				if TG68_A(23 downto 16) = x"A0" and TG68_A(14 downto 8) = "1100000" and TG68_UDS_N = '0' then
					BAR <= TG68_DO(8) & BAR(23 downto 16);
				end if;
			else
				TG68_BAR_D <= x"FFFF";
			end if;
			TG68_BAR_DTACK_N <= '0';
		elsif T80_BAR_SEL = '1' and T80_BAR_DTACK_N = '1' then
			if T80_WR_N = '0' then
				if T80_A(15 downto 8) = x"60" then
					BAR <= T80_DO(0) & BAR(23 downto 16);
				end if;
			else
				T80_BAR_D <= x"FF";
			end if;
			T80_BAR_DTACK_N <= '0';
		end if;
	end if;
end process;


-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- MiST Memory Handling
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------
-- -----------------------------------------------------------------------

-- FLASH (SDRAM) CONTROL
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N,
	VBUS_SEL, VBUS_ADDR	)
begin

	if TG68_A(23 downto 22) = "00" 
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
		and TG68_RNW = '1' 
		and CART_EN = '1'
	then
		TG68_FLASH_SEL <= '1';
	else
		TG68_FLASH_SEL <= '0';
	end if;

	if T80_A(15) = '1' and BAR(23 downto 22) = "00"
		and T80_MREQ_N = '0' and T80_RD_N = '0' 
	then
		T80_FLASH_SEL <= '1';
	else
		T80_FLASH_SEL <= '0';
	end if;	

	if VBUS_ADDR(23 downto 22) = "00" 
		and VBUS_SEL = '1'
	then
		DMA_FLASH_SEL <= '1';
	else
		DMA_FLASH_SEL <= '0';
	end if;

	if MRST_N = '0' then
		FC <= FC_IDLE;
		
		TG68_FLASH_DTACK_N <= '1';
		T80_FLASH_DTACK_N <= '1';
		DMA_FLASH_DTACK_N <= '1';

		romrd_req <= '0';
		romrd_a_cached <= (others => '1');
		romrd_q_cached <= (others => '0');
		
	elsif rising_edge( MCLK ) then
		if TG68_FLASH_SEL = '0' then 
			TG68_FLASH_DTACK_N <= '1';
		end if;
		if T80_FLASH_SEL = '0' then 
			T80_FLASH_DTACK_N <= '1';
		end if;
		if DMA_FLASH_SEL = '0' then 
			DMA_FLASH_DTACK_N <= '1';
		end if;

		case FC is
		when FC_IDLE =>			
			if VCLKCNT = "001" then
				if TG68_FLASH_SEL = '1' and TG68_FLASH_DTACK_N = '1' then
					-- FF_FL_ADDR <= TG68_A(21 downto 0);
					if useCache and (TG68_A(21 downto 3) = romrd_a_cached(21 downto 3)) then
						case TG68_A(2 downto 1) is
						when "00" =>
							if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q_cached(15 downto 8); end if;
							if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q_cached(7 downto 0); end if;

						when "01" =>
							if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q_cached(31 downto 24); end if;
							if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q_cached(23 downto 16); end if;

						when "10" =>
							if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q_cached(47 downto 40); end if;
							if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q_cached(39 downto 32); end if;

						when "11" =>
							if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q_cached(63 downto 56); end if;
							if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q_cached(55 downto 48); end if;

						when others => null;
						end case;
						TG68_FLASH_DTACK_N <= '0';
					else
						romrd_req <= not romrd_req;
						romrd_a <= "000" & TG68_A(21 downto 3);
						romrd_a_cached <= "000" & TG68_A(21 downto 3);
						FC <= FC_TG68_RD;
					end if;
				elsif T80_FLASH_SEL = '1' and T80_FLASH_DTACK_N = '1' then
					-- FF_FL_ADDR <= BAR(21 downto 15) & T80_A(14 downto 0);
					if useCache and (BAR(21 downto 15) & T80_A(14 downto 3) = romrd_a_cached(21 downto 3)) then
-- /!\
						case T80_A(2 downto 0) is
						when "001" =>
							T80_FLASH_D <= romrd_q_cached(7 downto 0);
						when "000" =>
							T80_FLASH_D <= romrd_q_cached(15 downto 8);
						when "011" =>
							T80_FLASH_D <= romrd_q_cached(23 downto 16);
						when "010" =>
							T80_FLASH_D <= romrd_q_cached(31 downto 24);
						when "101" =>
							T80_FLASH_D <= romrd_q_cached(39 downto 32);
						when "100" =>
							T80_FLASH_D <= romrd_q_cached(47 downto 40);
						when "111" =>
							T80_FLASH_D <= romrd_q_cached(55 downto 48);
						when "110" =>
							T80_FLASH_D <= romrd_q_cached(63 downto 56);
						when others => null;
						end case;
						T80_FLASH_DTACK_N <= '0';
					else
						romrd_req <= not romrd_req;
						romrd_a <= "000" & BAR(21 downto 15) & T80_A(14 downto 3);
						romrd_a_cached <= "000" & BAR(21 downto 15) & T80_A(14 downto 3);		
						FC <= FC_T80_RD;
					end if;
				elsif DMA_FLASH_SEL = '1' and DMA_FLASH_DTACK_N = '1' then
					-- FF_FL_ADDR <= VBUS_ADDR(21 downto 0);
					if useCache and (VBUS_ADDR(21 downto 3) = romrd_a_cached(21 downto 3)) then
						case VBUS_ADDR(2 downto 1) is
						when "00" =>
							DMA_FLASH_D <= romrd_q_cached(15 downto 0);
						when "01" =>
							DMA_FLASH_D <= romrd_q_cached(31 downto 16);
						when "10" =>
							DMA_FLASH_D <= romrd_q_cached(47 downto 32);
						when "11" =>
							DMA_FLASH_D <= romrd_q_cached(63 downto 48);
						when others => null;
						end case;
						DMA_FLASH_DTACK_N <= '0';
					else
						romrd_req <= not romrd_req;
						romrd_a <= "000" & VBUS_ADDR(21 downto 3);
						romrd_a_cached <= "000" & VBUS_ADDR(21 downto 3);
						FC <= FC_DMA_RD;
					end if;					
				end if;				
			end if;
		
		when FC_TG68_RD =>
			if romrd_req = romrd_ack then
				romrd_q_cached <= romrd_q;
				case TG68_A(2 downto 1) is
				when "00" =>
					if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q(15 downto 8); end if;
					if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q(7 downto 0); end if;

				when "01" =>
					if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q(31 downto 24); end if;
					if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q(23 downto 16); end if;

				when "10" =>
					if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q(47 downto 40); end if;
					if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q(39 downto 32); end if;

				when "11" =>
					if TG68_UDS_N = '0' then TG68_FLASH_D(15 downto 8) <= romrd_q(63 downto 56); end if;
					if TG68_LDS_N = '0' then TG68_FLASH_D(7 downto 0) <= romrd_q(55 downto 48); end if;

				when others => null;
				end case;				
				TG68_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			end if;

		when FC_T80_RD =>
			if romrd_req = romrd_ack then
				romrd_q_cached <= romrd_q;
-- /!\
				case T80_A(2 downto 0) is
				when "001" =>
					T80_FLASH_D <= romrd_q(7 downto 0);
				when "000" =>
					T80_FLASH_D <= romrd_q(15 downto 8);
				when "011" =>
					T80_FLASH_D <= romrd_q(23 downto 16);
				when "010" =>
					T80_FLASH_D <= romrd_q(31 downto 24);
				when "101" =>
					T80_FLASH_D <= romrd_q(39 downto 32);
				when "100" =>
					T80_FLASH_D <= romrd_q(47 downto 40);
				when "111" =>
					T80_FLASH_D <= romrd_q(55 downto 48);
				when "110" =>
					T80_FLASH_D <= romrd_q(63 downto 56);
				when others => null;
				end case;
				T80_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			end if;
		
		when FC_DMA_RD =>
			if romrd_req = romrd_ack then
				romrd_q_cached <= romrd_q;
				case VBUS_ADDR(2 downto 1) is
				when "00" =>
					DMA_FLASH_D <= romrd_q(15 downto 0);
				when "01" =>
					DMA_FLASH_D <= romrd_q(31 downto 16);
				when "10" =>
					DMA_FLASH_D <= romrd_q(47 downto 32);
				when "11" =>
					DMA_FLASH_D <= romrd_q(63 downto 48);
				when others => null;
				end case;
				DMA_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			end if;
				
		when others => null;
		end case;
	
	end if;

end process;





-- SDRAM (68K RAM) CONTROL
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N,
	VBUS_SEL, VBUS_ADDR)
begin
	if TG68_A(23 downto 21) = "111" -- 68000 RAM
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_SDRAM_SEL <= '1';
	else
		TG68_SDRAM_SEL <= '0';
	end if;

	if T80_A(15) = '1' and BAR(23 downto 21) = "111" -- 68000 RAM
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_SDRAM_SEL <= '1';
	else
		T80_SDRAM_SEL <= '0';
	end if;

	if VBUS_ADDR(23 downto 21) = "111" -- 68000 RAM
		and VBUS_SEL = '1' 
	then
		DMA_SDRAM_SEL <= '1';
	else
		DMA_SDRAM_SEL <= '0';
	end if;

	if MRST_N = '0' then
		TG68_SDRAM_DTACK_N <= '1';
		T80_SDRAM_DTACK_N <= '1';
		DMA_SDRAM_DTACK_N <= '1';

		ram68k_req <= '0';
		
		SDRC <= SDRC_IDLE;
		
	elsif rising_edge(MCLK) then
		if TG68_SDRAM_SEL = '0' then 
			TG68_SDRAM_DTACK_N <= '1';
		end if;	
		if T80_SDRAM_SEL = '0' then 
			T80_SDRAM_DTACK_N <= '1';
		end if;	
		if DMA_SDRAM_SEL = '0' then 
			DMA_SDRAM_DTACK_N <= '1';
		end if;	

		case SDRC is
		when SDRC_IDLE =>
			if VCLKCNT = "001" then
				if TG68_SDRAM_SEL = '1' and TG68_SDRAM_DTACK_N = '1' then
					ram68k_req <= not ram68k_req;
					ram68k_a <= "00" & "1000000" & TG68_A(15 downto 1);
					ram68k_d <= TG68_DO;
					ram68k_we <= not TG68_RNW;
					ram68k_u_n <= TG68_UDS_N;
					ram68k_l_n <= TG68_LDS_N;
					SDRC <= SDRC_TG68;
				elsif T80_SDRAM_SEL = '1' and T80_SDRAM_DTACK_N = '1' then
					ram68k_req <= not ram68k_req;
					ram68k_a <= "00" & "1000000" & BAR(15) & T80_A(14 downto 1);
					ram68k_d <= T80_DO & T80_DO;
					ram68k_we <= not T80_WR_N;
					ram68k_u_n <= T80_A(0);
					ram68k_l_n <= not T80_A(0);
					SDRC <= SDRC_T80;
				elsif DMA_SDRAM_SEL = '1' and DMA_SDRAM_DTACK_N = '1' then
					ram68k_req <= not ram68k_req;
					ram68k_a <= "00" & "1000000" & VBUS_ADDR(15 downto 1);
					ram68k_we <= '0';
					ram68k_u_n <= '0';
					ram68k_l_n <= '0';					
					SDRC <= SDRC_DMA;
				end if;
			end if;

		when SDRC_TG68 =>
			if ram68k_req = ram68k_ack then
				TG68_SDRAM_D <= ram68k_q;
				TG68_SDRAM_DTACK_N <= '0';
				SDRC <= SDRC_IDLE;
			end if;
		
		when SDRC_T80 =>
			if ram68k_req = ram68k_ack then
				if T80_A(0) = '0' then
					T80_SDRAM_D <= ram68k_q(15 downto 8);
				else
					T80_SDRAM_D <= ram68k_q(7 downto 0);
				end if;
				T80_SDRAM_DTACK_N <= '0';
				SDRC <= SDRC_IDLE;
			end if;

		when SDRC_DMA =>
			if ram68k_req = ram68k_ack then
				DMA_SDRAM_D <= ram68k_q;
				DMA_SDRAM_DTACK_N <= '0';
				SDRC <= SDRC_IDLE;
			end if;
		
		when others => null;
		end case;
		
	end if;

end process;





-- Z80 RAM CONTROL
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N,
	VBUS_SEL, VBUS_ADDR)
begin
	if TG68_A(23 downto 16) = x"A0" -- Z80 Address Space
		and TG68_A(14) = '0' -- Z80 RAM (gen-hw.txt lines 89 and 272-273)
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_ZRAM_SEL <= '1';
	else
		TG68_ZRAM_SEL <= '0';
	end if;

	if T80_A(15 downto 14) = "00" -- Z80 RAM
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_ZRAM_SEL <= '1';
	else
		T80_ZRAM_SEL <= '0';
	end if;
	
	if MRST_N = '0' then
		TG68_ZRAM_DTACK_N <= '1';
		T80_ZRAM_DTACK_N <= '1';
	
		zram_we <= '0';
		zram_a <= (others => '0');
	
		ZRC <= ZRC_IDLE;
	
	elsif rising_edge(MCLK) then
		if TG68_ZRAM_SEL = '0' then 
			TG68_ZRAM_DTACK_N <= '1';
		end if;	
		if T80_ZRAM_SEL = '0' then 
			T80_ZRAM_DTACK_N <= '1';
		end if;	

		case ZRC is
		when ZRC_IDLE =>
			if VCLKCNT = "001" then
				if TG68_ZRAM_SEL = '1' and TG68_ZRAM_DTACK_N = '1' then
					if TG68_UDS_N = '0' then
						zram_a <= TG68_A(12 downto 1) & "0";
						zram_d <= TG68_DO(15 downto 8);
					else
						zram_a <= TG68_A(12 downto 1) & "1";
						zram_d <= TG68_DO(7 downto 0);
					end if;
					zram_we <= not TG68_RNW;
					ZRCP <= ZRCP_TG68;
					ZRC <= ZRC_ACC1;
				elsif T80_ZRAM_SEL = '1' and T80_ZRAM_DTACK_N = '1' then
					zram_a <= T80_A(12 downto 0);
					zram_d <= T80_DO;
					zram_we <= not T80_WR_N;
					ZRCP <= ZRCP_T80;
					ZRC <= ZRC_ACC1;
				end if;
			end if;
		when ZRC_ACC1 =>
			zram_we <= '0';
			ZRC <= ZRC_ACC2;
		when ZRC_ACC2 =>
			ZRC <= ZRC_ACC3;
		when ZRC_ACC3 =>
			case ZRCP is
			when ZRCP_TG68 =>
				TG68_ZRAM_D <= zram_q & zram_q;
				TG68_ZRAM_DTACK_N <= '0';
			when ZRCP_T80 =>
				T80_ZRAM_D <= zram_q;
				T80_ZRAM_DTACK_N <= '0';				
			end case;
			ZRC <= ZRC_IDLE;
		when others => null;
		end case;
	end if;

end process;



-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################

-- DEBUG

-- synthesis translate_off
process( MCLK )
	file F		: text open write_mode is "gen.out";
	variable L	: line;
	variable rom_q : std_logic_vector(15 downto 0);
begin
	if rising_edge( MCLK ) then

		-- ROM ACCESS
		if FC = FC_TG68_RD and romrd_req = romrd_ack then
			write(L, string'("68K "));
			write(L, string'("RD"));
			write(L, string'(" ROM     ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			rom_q := x"FFFF";
			case TG68_A(2 downto 1) is
			when "00" =>
				if TG68_UDS_N = '0' then rom_q(15 downto 8) := romrd_q(15 downto 8); end if;
				if TG68_LDS_N = '0' then rom_q(7 downto 0) := romrd_q(7 downto 0); end if;

			when "01" =>
				if TG68_UDS_N = '0' then rom_q(15 downto 8) := romrd_q(31 downto 24); end if;
				if TG68_LDS_N = '0' then rom_q(7 downto 0) := romrd_q(23 downto 16); end if;

			when "10" =>
				if TG68_UDS_N = '0' then rom_q(15 downto 8) := romrd_q(47 downto 40); end if;
				if TG68_LDS_N = '0' then rom_q(7 downto 0) := romrd_q(39 downto 32); end if;

			when "11" =>
				if TG68_UDS_N = '0' then rom_q(15 downto 8) := romrd_q(63 downto 56); end if;
				if TG68_LDS_N = '0' then rom_q(7 downto 0) := romrd_q(55 downto 48); end if;

			when others => null;
			end case;				
			if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
				hwrite(L, rom_q);
			elsif TG68_UDS_N = '0' then
				hwrite(L, rom_q(15 downto 8));
				write(L, string'("  "));
			else
				write(L, string'("  "));
				hwrite(L, rom_q(7 downto 0));
			end if;								
			write(L, string'("]"));
			writeline(F,L);			
		end if;		

	
		-- 68K RAM ACCESS
		if SDRC = SDRC_TG68 and ram68k_req = ram68k_ack then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			write(L, string'(" RAM-68K ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			if TG68_RNW = '0' then
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, TG68_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, TG68_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, TG68_DO(7 downto 0));
				end if;				
			else
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, ram68k_q);
				elsif TG68_UDS_N = '0' then
					hwrite(L, ram68k_q(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, ram68k_q(7 downto 0));
				end if;								
			end if;
			write(L, string'("]"));
			writeline(F,L);			
		end if;		

		
		-- Z80 RAM ACCESS
		if ZRC = ZRC_ACC3 and ZRCP = ZRCP_TG68 then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			write(L, string'(" RAM-Z80 ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			if TG68_RNW = '0' then
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, TG68_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, TG68_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, TG68_DO(7 downto 0));
				end if;				
			else
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, zram_q & zram_q);
				elsif TG68_UDS_N = '0' then
					hwrite(L, zram_q);
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, zram_q);
				end if;				
			end if;
			write(L, string'("]"));
			writeline(F,L);			
		end if;		

		
		-- 68K CTRL ACCESS
		if TG68_CTRL_SEL = '1' and TG68_CTRL_DTACK_N = '1' then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			write(L, string'("    CTRL ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			if TG68_RNW = '0' then
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, TG68_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, TG68_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, TG68_DO(7 downto 0));
				end if;				
			else
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					write(L, string'("????"));
				elsif TG68_UDS_N = '0' then
					write(L, string'("??"));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					write(L, string'("??"));
				end if;								
			end if;
			write(L, string'("]"));
			writeline(F,L);							
		end if;

		-- 68K I/O ACCESS
		if IOC = IOC_TG68_ACC and IO_DTACK_N = '0' then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			write(L, string'("     I/O ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			if TG68_RNW = '0' then
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, TG68_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, TG68_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, TG68_DO(7 downto 0));
				end if;				
			else
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, IO_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, IO_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, IO_DO(7 downto 0));
				end if;								
			end if;
			write(L, string'("]"));
			writeline(F,L);					
		end if;
		
		-- 68K VDP ACCESS
		if VDPC = VDPC_TG68_ACC and VDP_DTACK_N = '0' then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			write(L, string'("     VDP ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			if TG68_RNW = '0' then
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, TG68_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, TG68_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, TG68_DO(7 downto 0));
				end if;				
			else
				if TG68_UDS_N = '0' and TG68_LDS_N = '0' then
					hwrite(L, VDP_DO);
				elsif TG68_UDS_N = '0' then
					hwrite(L, VDP_DO(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, VDP_DO(7 downto 0));
				end if;								
			end if;
			write(L, string'("]"));
			writeline(F,L);					
		end if;
		
	end if;
end process;
-- synthesis translate_on

------------------------ END ----------------------

-- COMMENTED START
--DRAM_CKE <= '1';
--DRAM_CS_N <= '0';
--
---- Interrupt signals
--CPU_NMI_N <= '1';
--CPU_IRQ1_N <= VDC_IRQ_N;
--CPU_IRQ2_N <= '1';
--CPU_RDY <= VDC_BUSY_N and ROM_RDY;
--
---- CPU data bus
--CPU_DI <= RAM_DO when CPU_RD_N = '0' and CPU_RAM_SEL_N = '0' 
--	else ROM_DO when CPU_RD_N = '0' and CPU_A(20) = '0'
--	else VCE_DO when CPU_RD_N = '0' and CPU_VCE_SEL_N = '0'
--	else VDC_DO when CPU_RD_N = '0' and CPU_VDC_SEL_N = '0'
--	else "ZZZZZZZZ";
-- COMMENTED END


	
	
-- ROM_RDY <= '1' when romrd_req = romrd_ack else '0';


-- COMMENTED START
--process( MCLK )
--begin
--	if rising_edge( MCLK ) then
--		if ROM_RESET_N = '0' then
--			RESET_N <= '0';
--			romrd_req <= '0';
--			romrd_a_cached <= (others => '0');
--			romrd_q_cached <= (others => '0');
--			ROM_RDY <= '0';
--			CPU_A_PREV <= (others => '0');
--		elsif ROM_RESET_N = '1' and RESET_N = '0' then
--			if CPU_CLKRST = '1' then
--				romrd_req <= not romrd_req;
--				romrd_a<=(others=>'0');
--				romrd_a(19 downto 3) <= CPU_A(19 downto 3);
--				romrd_a_cached<=(others=>'0');
--				romrd_a_cached(19 downto 3) <= CPU_A(19 downto 3);
--				ROM_RDY <= '0';
--				romState <= ROM_READ;				
--				RESET_N <= '1';
--			end if;
--		else
--			case romState is
--			when ROM_IDLE =>
--				if CPU_CLKOUT = '1' then
--					if CPU_RD_N = '0' or CPU_WR_N = '0' then
--						CPU_A_PREV <= CPU_A;
--					else 
--						CPU_A_PREV <= (others => '1');
--					end if;
--					if CPU_A(20) = '0' and CPU_RD_N = '0' and CPU_A /= CPU_A_PREV then
--						if CPU_A(19 downto 3) = romrd_a_cached(19 downto 3) then
--							case CPU_A(2 downto 0) is
--								when "000" =>
--									ROM_DO <= romrd_q_cached(7 downto 0);
--								when "001" =>
--									ROM_DO <= romrd_q_cached(15 downto 8);
--								when "010" =>
--									ROM_DO <= romrd_q_cached(23 downto 16);
--								when "011" =>
--									ROM_DO <= romrd_q_cached(31 downto 24);
--								when "100" =>
--									ROM_DO <= romrd_q_cached(39 downto 32);
--								when "101" =>
--									ROM_DO <= romrd_q_cached(47 downto 40);
--								when "110" =>
--									ROM_DO <= romrd_q_cached(55 downto 48);
--								when "111" =>
--									ROM_DO <= romrd_q_cached(63 downto 56);
--								when others => null;
--							end case;						
--						else
--							romrd_req <= not romrd_req;
--							romrd_a<=(others=>'0');
--							romrd_a(19 downto 3) <= CPU_A(19 downto 3);
--							-- Perform address mangling to mimic HuCard chip mapping.
--							-- rommap => 00  -  Straight mapping
--							-- rommap => 01  -  384K ROM, split in 3, mapped ABABCCCC
--							-- rommap => 10  -  768K ROM, straight mapping for now
--							-- rommap => 11  -  not yet defined.
--							
--							if rommap="01" then                       -- bits 19 downto 16
--								-- 00000 -> 20000  => 00000 -> 20000		0000 -> 0000
--								-- 20000 -> 40000  => 20000 -> 40000		0010 -> 0010
--								-- 40000 -> 60000  => 00000 -> 20000		0100 -> 0000
--								-- 60000 -> 80000  => 20000 -> 40000		0110 -> 0010
--								-- 80000 -> A0000  => 40000 -> 60000		1000 -> 0100
--								-- A0000 -> C0000  => 40000 -> 60000		1010 -> 0100
--								-- C0000 -> E0000  => 40000 -> 60000		1100 -> 0100
--								-- E0000 ->100000  => 40000 -> 60000		1110 -> 0100
--								romrd_a(19)<='0';
--								romrd_a(18)<=CPU_A(19);
--								romrd_a(17)<=CPU_A(17) and not CPU_A(19);
--							end if;
--								
--
--							romrd_a_cached<=(others=>'0');
--							romrd_a_cached(19 downto 3) <= CPU_A(19 downto 3);
--							ROM_RDY <= '0';
--							romState <= ROM_READ;
--						end if;
--					end if;
--				end if;
--			when ROM_READ =>
--				if romrd_req = romrd_ack then
--					ROM_RDY <= '1';
--					romrd_q_cached <= romrd_q;
--					case CPU_A(2 downto 0) is
--						when "000" =>
--							ROM_DO <= romrd_q(7 downto 0);
--						when "001" =>
--							ROM_DO <= romrd_q(15 downto 8);
--						when "010" =>
--							ROM_DO <= romrd_q(23 downto 16);
--						when "011" =>
--							ROM_DO <= romrd_q(31 downto 24);
--						when "100" =>
--							ROM_DO <= romrd_q(39 downto 32);
--						when "101" =>
--							ROM_DO <= romrd_q(47 downto 40);
--						when "110" =>
--							ROM_DO <= romrd_q(55 downto 48);
--						when "111" =>
--							ROM_DO <= romrd_q(63 downto 56);
--						when others => null;
--					end case;
--					romState <= ROM_IDLE;
--				end if;
--			when others => null;
--			end case;
--		end if;
--	end if;
--end process;
-- COMMENTED END


-- Boot process

FL_DQ<=boot_data;

ROM_RESET_N <= host_bootdone and host_reset_n;

process( SDR_CLK )
begin
	if rising_edge( SDR_CLK ) then
		if PRE_RESET_N = '0' then
				
			boot_req <='0';
			
			romwr_req <= '0';
			romwr_a <= to_unsigned(0, addrwidth);
			bootState<=BOOT_READ_1;
			
		else
			case bootState is 
				when BOOT_READ_1 =>
					boot_req<='1';
					if boot_ack='1' then
						boot_req<='0';
						bootState <= BOOT_WRITE_1;
					end if;
					if host_bootdone='1' then
						boot_req<='0';
						bootState <= BOOT_DONE;
					end if;
				when BOOT_WRITE_1 =>
					if BITFLIP = '1' then
						romwr_d <=
							FL_DQ(8)
							& FL_DQ(9)
							& FL_DQ(10)
							& FL_DQ(11)
							& FL_DQ(12)
							& FL_DQ(13)
							& FL_DQ(14)
							& FL_DQ(15)
							& FL_DQ(0)
							& FL_DQ(1)
							& FL_DQ(2)
							& FL_DQ(3)
							& FL_DQ(4)
							& FL_DQ(5)
							& FL_DQ(6)
							& FL_DQ(7);
					else
						romwr_d <= FL_DQ;
					end if;
					
					romwr_req <= not romwr_req;
					bootState <= BOOT_WRITE_2;
				when BOOT_WRITE_2 =>
					if romwr_req = romwr_ack then
						romwr_a <= romwr_a + 1;
						bootState <= BOOT_READ_1;
					end if;
				when BOOT_REL =>
					if CPU_CLKRST = '1' then
						bootState <= BOOT_DONE;
					end if;
				when others => null;
			end case;	
		end if;
	end if;
end process;



-- Block RAM
RAM_A <= CPU_A(12 downto 0);
RAM_DI <= CPU_DO;
process( MCLK )
begin
	if rising_edge( MCLK ) then
		RAM_WE <= '0';
		if CPU_CLKOUT = '1' and CPU_RAM_SEL_N = '0' and CPU_WR_N = '0' then
			RAM_WE <= '1';
		end if;
	end if;
end process;

joya_merged <= joya and gp1emu;
joyb_merged <= joyb and gp2emu;



-- I/O Port
CPU_IO_DI(7 downto 4) <= "1011"; -- No CD-Rom unit, TGFX-16
CPU_IO_DI(3 downto 0) <=
	joya_merged(7) & joya_merged(6) & joya_merged(4) & joya_merged(5)
		when CPU_IO_DO(1 downto 0) = "00" and gamepad_port = "000"
	else joya_merged(2) & joya_merged(1) & joya_merged(3) & joya_merged(0)
		when CPU_IO_DO(1 downto 0) = "01" and gamepad_port = "000"
		
	else joyb_merged(7) & joyb_merged(6) & joyb_merged(4) & joyb_merged(5)
		when CPU_IO_DO(1 downto 0) = "00" and gamepad_port = "001"
	else joyb_merged(2) & joyb_merged(1) & joyb_merged(3) & joyb_merged(0)
		when CPU_IO_DO(1 downto 0) = "01" and gamepad_port = "001"
		
	else joyc(7) & joyc(6) & joyc(4) & joyc(5)
		when CPU_IO_DO(1 downto 0) = "00" and gamepad_port = "010"
	else joyc(2) & joyc(1) & joyc(3) & joyc(0)
		when CPU_IO_DO(1 downto 0) = "01" and gamepad_port = "010"
		
	else joyd(7) & joyd(6) & joyd(4) & joyd(5)
		when CPU_IO_DO(1 downto 0) = "00" and gamepad_port = "011"
	else joyd(2) & joyd(1) & joyd(3) & joyd(0)
		when CPU_IO_DO(1 downto 0) = "01" and gamepad_port = "011"

	else joye(7) & joye(6) & joye(4) & joye(5)
		when CPU_IO_DO(1 downto 0) = "00" and gamepad_port = "100"
	else joye(2) & joye(1) & joye(3) & joye(0)
		when CPU_IO_DO(1 downto 0) = "01" and gamepad_port = "100"
		
	else "1111";

process(clk)
begin
	if rising_edge(clk) then
		if CPU_IO_DO(1)='1' then -- reset pad
			gamepad_port<=(others => '0');
		elsif prev_sel='0' and CPU_IO_DO(0)='1' and multitap='1' then -- Rising edge of select bit
			gamepad_port<=gamepad_port+1;
		end if;
		prev_sel<=CPU_IO_DO(0);
	end if;

end process;
	
-- Control module:

mycontrolmodule : entity work.CtrlModule
	generic map (
		sysclk_frequency => 1270 -- Sysclk frequency * 10
	)
	port map (
		clk => SDR_CLK,
		reset_n => reset,

		-- SPI signals
		spi_miso	=> spi_miso,
		spi_mosi => spi_mosi,
		spi_clk => spi_clk,
		spi_cs => spi_cs,
		
		-- UART
		rxd => RS232_RXD,
		txd => RS232_TXD,
		
		-- DIP switches
		dipswitches => SW,

		-- PS2 keyboard
		ps2k_clk_in => ps2k_clk_in,
		ps2k_dat_in => ps2k_dat_in,
		ps2k_clk_out => ps2k_clk_out,
		ps2k_dat_out => ps2k_dat_out,
		
		-- Host control
		host_reset_n => host_reset_n,
		host_bootdone => host_bootdone,
		
		-- Host boot data
		host_bootdata => boot_data,
		host_bootdata_req => boot_req,
		host_bootdata_ack => boot_ack,
		rommap => rommap,
		
		-- Video signals for OSD
		vga_hsync => vga_hsync_i,
		vga_vsync => vga_vsync_i,
		osd_window => osd_window,
		osd_pixel => osd_pixel,
		
		-- Gamepad emulation
		gp1emu => gp1emu,
		gp2emu => gp2emu
);


overlay : entity work.OSD_Overlay
	port map
	(
		clk => SDR_CLK,
		red_in => vga_red_i,
		green_in => vga_green_i,
		blue_in => vga_blue_i,
		window_in => '1',
		osd_window_in => osd_window,
		osd_pixel_in => osd_pixel,
		hsync_in => vga_hsync_i,
		red_out => VGA_R,
		green_out => VGA_G,
		blue_out => VGA_B,
		window_out => open,
		scanline_ena => SW(1)
	);

-- Route VDP signals to outputs
VGA_RED <= VDP_RED & VDP_RED;
VGA_GREEN <= VDP_GREEN & VDP_GREEN;
VGA_BLUE <= VDP_BLUE & VDP_BLUE;
VGA_HS_N <= VDP_HS_N;
VGA_VS_N <= VDP_VS_N;

-- Select between VGA and TV output	
vga_red_i <= RED when SW(0)='1' else VGA_RED;
vga_green_i <= GREEN when SW(0)='1' else VGA_GREEN;
vga_blue_i <= BLUE when SW(0)='1' else VGA_BLUE;
vga_hsync_i <= HS_N when SW(0)='1' else VGA_HS_N;
vga_vsync_i <= VS_N when SW(0)='1' else VGA_VS_N;
VGA_HS <= vga_hsync_i;
VGA_VS <= vga_vsync_i;

end rtl;
