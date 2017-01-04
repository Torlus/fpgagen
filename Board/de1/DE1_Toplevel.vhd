-- Copyright (c) 2010 Gregory Estrade (greg@torlus.com)
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Please report bugs to the author, but before you do so, please
-- make sure that this is not a derivative work and that
-- you have the latest version of this file.

library STD;
use STD.TEXTIO.ALL;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;

entity DE1_Toplevel is
	port(
		SW			: in std_logic_vector(9 downto 0);
		
		HEX0		: out std_logic_vector(6 downto 0);
		HEX1		: out std_logic_vector(6 downto 0);
		HEX2		: out std_logic_vector(6 downto 0);
		HEX3		: out std_logic_vector(6 downto 0);
		
		KEY			: in std_logic_vector(3 downto 0);
				
		LEDR		: out std_logic_vector(9 downto 0);
		LEDG		: out std_logic_vector(7 downto 0);
		
		CLOCK_27	: in std_logic_vector(1 downto 0);
		
		DRAM_ADDR	: out std_logic_vector(11 downto 0);
		DRAM_BA_0	: out std_logic;
		DRAM_BA_1	: out std_logic;
		DRAM_CAS_N	: out std_logic;
		DRAM_CKE	: out std_logic;
		DRAM_CLK	: out std_logic;
		DRAM_CS_N	: out std_logic;
		DRAM_DQ		: inout std_logic_vector(15 downto 0);
		DRAM_LDQM	: out std_logic;
		DRAM_RAS_N	: out std_logic;
		DRAM_UDQM	: out std_logic;
		DRAM_WE_N	: out std_logic;
		
		FL_ADDR		: out std_logic_vector(21 downto 0);
		FL_DQ		: inout std_logic_vector(7 downto 0);
		FL_OE_N		: out std_logic;
		FL_RST_N	: out std_logic;
		FL_WE_N		: out std_logic;
		
		SRAM_ADDR	: out std_logic_vector(17 downto 0);
		SRAM_CE_N	: out std_logic;
		SRAM_DQ		: inout std_logic_vector(15 downto 0);
		SRAM_LB_N	: out std_logic;
		SRAM_OE_N	: out std_logic;
		SRAM_UB_N	: out std_logic;
		SRAM_WE_N	: out std_logic;

		VGA_R		: out std_logic_vector(3 downto 0);
		VGA_G		: out std_logic_vector(3 downto 0);
		VGA_B		: out std_logic_vector(3 downto 0);
		VGA_VS		: out std_logic;
		VGA_HS		: out std_logic				
	);
end DE1_Toplevel;

architecture rtl of DE1_Toplevel is

signal NO_DATA		: std_logic_vector(15 downto 0) := x"4E71";	-- SYNTHESIS gp/m68k.c line 12

signal MCLK			: std_logic;
signal CLK_LOCKED	: std_logic;

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

-- RESET
signal RESET		: std_logic;
signal MRST_N		: std_logic := '0';
signal RSTCNT		: std_logic_vector(31 downto 0) := (others => '0');
signal FF_FL_RST_N	: std_logic := '0';
type rstseq_t is ( RS_INIT,
	RS_SDR_IDLE, RS_SDR_PALL, RS_SDR_RFSH, RS_SDR_MODE,
	RS_MEM_FILL, RS_DONE );
signal RSTSEQ		: rstseq_t	:= RS_INIT;
	
-- CLOCK GENERATION
signal VCLK			: std_logic;
signal VCLKCNT		: std_logic_vector(2 downto 0);
signal ZCLK			: std_logic := '0';

signal ZCLKCNT		: std_logic_vector(3 downto 0);

-- FLASH CONTROL
signal FF_FL_OE_N			: std_logic;
signal FF_FL_ADDR			: std_logic_vector(21 downto 0);

signal TG68_FLASH_SEL		: std_logic;
signal TG68_FLASH_D			: std_logic_vector(15 downto 0);
signal TG68_FLASH_DTACK_N	: std_logic;

signal T80_FLASH_SEL		: std_logic;
signal T80_FLASH_D			: std_logic_vector(7 downto 0);
signal T80_FLASH_DTACK_N	: std_logic;

signal DMA_FLASH_SEL		: std_logic;
signal DMA_FLASH_D			: std_logic_vector(15 downto 0);
signal DMA_FLASH_DTACK_N	: std_logic;

type fc_t is ( FC_IDLE, 
	FC_TG68_RDU,  FC_TG68_RDL,
	FC_DMA_RDU, FC_DMA_RDL,
	FC_T80_RD
);
signal FC : fc_t;
constant FC_WS_MAX	: std_logic_vector(2 downto 0) := "011";
-- constant FC_WS_MAX	: std_logic_vector(2 downto 0) := "110";
signal FC_WS_CNT	: std_logic_vector(2 downto 0);

-- SRAM CONTROL
signal VRAM_ADDR	: std_logic_vector(14 downto 0);
signal VRAM_CE_N	: std_logic;
signal VRAM_UB_N	: std_logic;
signal VRAM_LB_N	: std_logic;
signal VRAM_DO		: std_logic_vector(15 downto 0);
signal VRAM_DI		: std_logic_vector(15 downto 0);
signal VRAM_OE_N	: std_logic;
signal VRAM_WE_N	: std_logic;
signal VRAM_SEL		: std_logic;
signal VRAM_DTACK_N	: std_logic;

signal VRAM_ADDR_INIT	: std_logic_vector(14 downto 0);
signal VRAM_CE_N_INIT	: std_logic;
signal VRAM_UB_N_INIT	: std_logic;
signal VRAM_LB_N_INIT	: std_logic;
signal VRAM_DO_INIT		: std_logic_vector(15 downto 0);
signal VRAM_DI_INIT		: std_logic_vector(15 downto 0) := (others => '0');
signal VRAM_OE_N_INIT	: std_logic;
signal VRAM_WE_N_INIT	: std_logic;

signal W_SRAM_CE_N		: std_logic;
signal W_SRAM_OE_N		: std_logic;
signal W_SRAM_WE_N		: std_logic;
signal W_SRAM_UB_N		: std_logic;
signal W_SRAM_LB_N		: std_logic;


-- SDRAM CONTROL
SIGNAL SDRCMD			: STD_LOGIC_VECTOR(  3 DOWNTO 0 );
SIGNAL SDRBA			: STD_LOGIC_VECTOR(  1 DOWNTO 0 );
SIGNAL SDRUDQ			: STD_LOGIC;
SIGNAL SDRLDQ			: STD_LOGIC;
SIGNAL SDRADR			: STD_LOGIC_VECTOR( 11 DOWNTO 0 );
SIGNAL SDRDAT			: STD_LOGIC_VECTOR( 15 DOWNTO 0 );

-- SDRAM INIT
SIGNAL SDRCMD_INIT		: STD_LOGIC_VECTOR(  3 DOWNTO 0 );
SIGNAL SDRADR_INIT		: STD_LOGIC_VECTOR( 11 DOWNTO 0 );
--SIGNAL SDRBA_INIT		: STD_LOGIC_VECTOR(  1 DOWNTO 0 );
SIGNAL SDRUDQ_INIT		: STD_LOGIC := '0';
SIGNAL SDRLDQ_INIT		: STD_LOGIC := '0';
SIGNAL SDRDAT_INIT		: STD_LOGIC_VECTOR( 15 DOWNTO 0 ) := (others => '0');
SIGNAL SDRCKE_INIT		: STD_LOGIC := '0';

CONSTANT SDRCMD_DE		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "1111"; -- DESELECT
CONSTANT SDRCMD_PR		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0010"; -- PRECHARGE ALL
CONSTANT SDRCMD_RE		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0001"; -- REFRESH
CONSTANT SDRCMD_MS		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0000"; -- MODE REGISER SET
CONSTANT SDRCMD_XX		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0111"; -- NO OPERATION
CONSTANT SDRCMD_AC		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0011"; -- ACTIVATE
CONSTANT SDRCMD_RD		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0101"; -- READ
CONSTANT SDRCMD_WR		: STD_LOGIC_VECTOR(  3 DOWNTO 0 ) := "0100"; -- WRITE

signal TG68_SDRAM_SEL		: std_logic;
signal TG68_SDRAM_D			: std_logic_vector(15 downto 0);
signal TG68_SDRAM_DTACK_N	: std_logic;

signal T80_SDRAM_SEL		: std_logic;
signal T80_SDRAM_D			: std_logic_vector(7 downto 0);
signal T80_SDRAM_DTACK_N	: std_logic;

signal DMA_SDRAM_SEL		: std_logic;
signal DMA_SDRAM_D			: std_logic_vector(15 downto 0);
signal DMA_SDRAM_DTACK_N	: std_logic;

signal SDRSEQ			: std_logic_vector(2 downto 0);
signal SDRAM_DO			: std_logic_vector(15 downto 0);

type sdrc_t is ( SDRC_IDLE, SDRC_RFSH,
	SDRC_TG68_ACC, SDRC_TG68_RD, SDRC_TG68_WR,
	SDRC_DMA_RD, 
	SDRC_T80_ACC, SDRC_T80_RD, SDRC_T80_WR,
	SDRC_VDP_ACC );
signal SDRC : sdrc_t;

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
signal VBUS_DMA_REQ	: std_logic;
signal VBUS_DMA_ACK	: std_logic;
signal VBUS_ADDR	: std_logic_vector(23 downto 0);
signal VBUS_UDS_N	: std_logic;
signal VBUS_LDS_N	: std_logic;
signal VBUS_DATA	: std_logic_vector(15 downto 0);		
signal VBUS_SEL		: std_logic;
signal VBUS_DTACK_N	: std_logic;


signal P1_UP		: std_logic;
signal P1_DOWN		: std_logic;
signal P1_LEFT		: std_logic;
signal P1_RIGHT		: std_logic;
signal P1_A			: std_logic;
signal P1_B			: std_logic;
signal P1_C			: std_logic;
signal P1_START		: std_logic;		

-- DEBUG
signal HEXVALUE			: std_logic_vector(15 downto 0);



begin

-- synthesis translate_off
-- NO_DATA	<= "ZZZZZZZZZZZZZZZZ";	-- SIMULATION 
-- synthesis translate_on

-- PLL
pll : entity work.pll
port map(
	inclk0	=> CLOCK_27(0),
	c0			=> MCLK,
	c1			=> DRAM_CLK,
	c2			=> open,
	locked	=> CLK_LOCKED
);

-- DEBUG
hexd3 : entity work.hex
port map(
	DIGIT	=> HEXVALUE(15 downto 12),
	SEG		=> HEX3
);
hexd2 : entity work.hex
port map(
	DIGIT	=> HEXVALUE(11 downto 8),
	SEG		=> HEX2
);
hexd1 : entity work.hex
port map(
	DIGIT	=> HEXVALUE(7 downto 4),
	SEG		=> HEX1
);
hexd0 : entity work.hex
port map(
	DIGIT	=> HEXVALUE(3 downto 0),
	SEG		=> HEX0
);

-- 68K
tg68 : entity work.TG68 
port map(
	-- clk			=> TG68_CLK,
	clk			=> MCLK,
	reset		=> TG68_RES_N,
	clkena_in	=> TG68_CLKE,
	data_in		=> TG68_DI,
	IPL			=> TG68_IPL_N,
	dtack		=> TG68_DTACK_N,
	addr		=> TG68_A,
	data_out	=> TG68_DO,
	as			=> TG68_AS_N,
	uds			=> TG68_UDS_N,
	lds			=> TG68_LDS_N,
	rw			=> TG68_RNW,
	enaRDreg	=> TG68_ENARDREG,
	enaWRreg	=> TG68_ENAWRREG,
	intack		=> TG68_INTACK
);

-- Z80
t80 : entity work.t80se
port map(
	RESET_n		=> T80_RESET_N,
	CLK_n		=> T80_CLK_N,
	CLKEN		=> T80_CLKEN,
	WAIT_n		=> T80_WAIT_N,
	INT_n		=> T80_INT_N,
	NMI_n		=> T80_NMI_N,
	BUSRQ_n		=> T80_BUSRQ_N,
	M1_n		=> T80_M1_N,
	MREQ_n		=> T80_MREQ_N,
	IORQ_n		=> T80_IORQ_N,
	RD_n		=> T80_RD_N,
	WR_n		=> T80_WR_N,
	RFSH_n		=> T80_RFSH_N,
	HALT_n		=> T80_HALT_N,
	BUSAK_n		=> T80_BUSAK_N,
	A			=> T80_A,
	DI			=> T80_DI,
	DO			=> T80_DO
);

-- OS ROM
os : entity work.os_rom
port map(
	A			=> TG68_A(8 downto 1),
	OEn			=> OS_OEn,
	D			=> TG68_OS_D
);

-- I/O
io : entity work.gen_io
port map(
	RST_N		=> MRST_N,
	CLK			=> VCLK,

	P1_UP		=> P1_UP,
	P1_DOWN		=> P1_DOWN,
	P1_LEFT		=> P1_LEFT,
	P1_RIGHT	=> P1_RIGHT,
	P1_A		=> P1_A,
	P1_B		=> P1_B,
	P1_C		=> P1_C,
	P1_START	=> P1_START,
		
	SEL			=> IO_SEL,
	A			=> IO_A,
	RNW			=> IO_RNW,
	UDS_N		=> IO_UDS_N,
	LDS_N		=> IO_LDS_N,
	DI			=> IO_DI,
	DO			=> IO_DO,
	DTACK_N		=> IO_DTACK_N
);

-- VDP
vdp : entity work.vdp
port map(
	RST_N		=> MRST_N,
	CLK			=> MCLK,
		
	SEL			=> VDP_SEL,
	A			=> VDP_A,
	RNW			=> VDP_RNW,
	UDS_N		=> VDP_UDS_N,
	LDS_N		=> VDP_LDS_N,
	DI			=> VDP_DI,
	DO			=> VDP_DO,
	DTACK_N		=> VDP_DTACK_N,
	
	VRAM_ADDR	=> VRAM_ADDR,
	VRAM_CE_N	=> VRAM_CE_N,
	VRAM_UB_N	=> VRAM_UB_N,
	VRAM_LB_N	=> VRAM_LB_N,
	VRAM_DO		=> VRAM_DO,
	VRAM_DI		=> VRAM_DI,
	VRAM_OE_N	=> VRAM_OE_N,
	VRAM_WE_N	=> VRAM_WE_N,
	
	VRAM_SEL	=> VRAM_SEL,
	VRAM_DTACK_N => VRAM_DTACK_N,
	
	INTERLACE	=> INTERLACE,

	HINT		=> HINT,
	HINT_ACK	=> HINT_ACK,

	VINT_TG68	=> VINT_TG68,
	VINT_T80	=> VINT_T80,
	VINT_TG68_ACK	=> VINT_TG68_ACK,
	VINT_T80_ACK	=> VINT_T80_ACK,

	VBUS_DMA_REQ	=> VBUS_DMA_REQ,
	VBUS_DMA_ACK	=> VBUS_DMA_ACK,
		
	VBUS_ADDR		=> VBUS_ADDR,
	VBUS_UDS_N		=> VBUS_UDS_N,
	VBUS_LDS_N		=> VBUS_LDS_N,
	VBUS_DATA		=> VBUS_DATA,
		
	VBUS_SEL		=> VBUS_SEL,
	VBUS_DTACK_N	=> VBUS_DTACK_N,
	
	VGA_R		=> VGA_R,
	VGA_G		=> VGA_G,
	VGA_B		=> VGA_B,
	VGA_HS		=> VGA_HS,
	VGA_VS		=> VGA_VS
);

-- FM
fm : entity work.gen_fm
port map(
	RST_N		=> T80_RESET_N,	-- gen-hw.txt line 328
	CLK			=> VCLK,
		
	SEL			=> FM_SEL,
	A			=> FM_A,
	RNW			=> FM_RNW,
	UDS_N		=> FM_UDS_N,
	LDS_N		=> FM_LDS_N,
	DI			=> FM_DI,
	DO			=> FM_DO,
	DTACK_N		=> FM_DTACK_N
);

-- #############################################################################
-- #############################################################################
-- #############################################################################

-- UNUSED SIGNALS
LEDR <= "1111001111";
-- LEDG <= "11101110";

VBUS_DMA_ACK <= '0';
VRAM_DTACK_N <= '0';
-- VBUS_DATA <= (others => '0');
-- VBUS_DTACK_N <= '1';


-- HEXVALUE <= TG68_A(15 downto 0);


----------------------------------------------------------------
-- INPUTS
----------------------------------------------------------------

P1_UP		<= not SW(9);
P1_DOWN		<= not SW(8);
P1_LEFT		<= not SW(7);
P1_RIGHT	<= not SW(6);

P1_A		<= KEY(3);
P1_B		<= KEY(2);
P1_C		<= KEY(1);
P1_START	<= KEY(0);

LEDG <= P1_UP & P1_DOWN & P1_LEFT & P1_RIGHT & P1_A & P1_B & P1_C & P1_START;

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
RESET <= SW(0);
INTERLACE <= SW(1);



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

-- process( VCLK )
-- begin
	-- if rising_edge(VCLK) then
		-- ZCLK <= not ZCLK;
	-- end if;
-- end process;

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
	else TG68_CTRL_DTACK_N when TG68_CTRL_SEL = '1' 
	else TG68_OS_DTACK_N when TG68_OS_SEL = '1' 
	else TG68_IO_DTACK_N when TG68_IO_SEL = '1' 
	else TG68_BAR_DTACK_N when TG68_BAR_SEL = '1' 
	else TG68_VDP_DTACK_N when TG68_VDP_SEL = '1' 
	else TG68_FM_DTACK_N when TG68_FM_SEL = '1' 
	else '0';
TG68_DI(15 downto 8) <= TG68_FLASH_D(15 downto 8) when TG68_FLASH_SEL = '1' and TG68_UDS_N = '0'
	else TG68_SDRAM_D(15 downto 8) when TG68_SDRAM_SEL = '1' and TG68_UDS_N = '0'
	else TG68_CTRL_D(15 downto 8) when TG68_CTRL_SEL = '1' and TG68_UDS_N = '0'
	else TG68_OS_D(15 downto 8) when TG68_OS_SEL = '1' and TG68_UDS_N = '0'
	else TG68_IO_D(15 downto 8) when TG68_IO_SEL = '1' and TG68_UDS_N = '0'
	else TG68_BAR_D(15 downto 8) when TG68_BAR_SEL = '1' and TG68_UDS_N = '0'
	else TG68_VDP_D(15 downto 8) when TG68_VDP_SEL = '1' and TG68_UDS_N = '0'
	else TG68_FM_D(15 downto 8) when TG68_FM_SEL = '1' and TG68_UDS_N = '0'
	else NO_DATA(15 downto 8);
TG68_DI(7 downto 0) <= TG68_FLASH_D(7 downto 0) when TG68_FLASH_SEL = '1' and TG68_LDS_N = '0'
	else TG68_SDRAM_D(7 downto 0) when TG68_SDRAM_SEL = '1' and TG68_LDS_N = '0'
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
	else not T80_FLASH_DTACK_N when T80_FLASH_SEL = '1'
	else not T80_CTRL_DTACK_N when T80_CTRL_SEL = '1' 
	else not T80_IO_DTACK_N when T80_IO_SEL = '1' 
	else not T80_BAR_DTACK_N when T80_BAR_SEL = '1'
	else not T80_VDP_DTACK_N when T80_VDP_SEL = '1'
	else not T80_FM_DTACK_N when T80_FM_SEL = '1'
	else '1';
T80_DI <= T80_SDRAM_D when T80_SDRAM_SEL = '1'
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

-- FLASH CONTROL
FL_RST_N <= FF_FL_RST_N;
FL_WE_N <= '1';
FL_DQ <= "ZZZZZZZZ";
FL_OE_N <= FF_FL_OE_N;
FL_ADDR <= FF_FL_ADDR;
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
		FC_WS_CNT <= "000";

		TG68_FLASH_DTACK_N <= '1';
		T80_FLASH_DTACK_N <= '1';
		DMA_FLASH_DTACK_N <= '1';

		FF_FL_OE_N <= '1';
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
			FF_FL_OE_N <= '1';
			FC_WS_CNT <= "000";
			if VCLKCNT = "001" then
				if TG68_FLASH_SEL = '1' and TG68_FLASH_DTACK_N = '1' then
					if TG68_UDS_N = '0' then
						FF_FL_ADDR <= TG68_A(21 downto 0);
						FC <= FC_TG68_RDU;
					else
						FF_FL_ADDR <= TG68_A(21 downto 1) & '1';
						FC <= FC_TG68_RDL;				
					end if;
				elsif T80_FLASH_SEL = '1' and T80_FLASH_DTACK_N = '1' then
					FF_FL_ADDR <= BAR(21 downto 15) & T80_A(14 downto 0);
					FC <= FC_T80_RD;
				elsif DMA_FLASH_SEL = '1' and DMA_FLASH_DTACK_N = '1' then
					FF_FL_ADDR <= VBUS_ADDR(21 downto 0);
					FC <= FC_DMA_RDU;
				end if;
				
			end if;
		when FC_TG68_RDU =>
			FF_FL_OE_N <= '0';
			if FC_WS_CNT = FC_WS_MAX then
				TG68_FLASH_D(15 downto 8) <= FL_DQ;
				FF_FL_OE_N <= '1';
				FC_WS_CNT <= "000";
				if TG68_LDS_N = '0' then
					FF_FL_ADDR <= TG68_A(21 downto 1) & '1';
					FC <= FC_TG68_RDL;
				else
					TG68_FLASH_DTACK_N <= '0';
					FC <= FC_IDLE;
				end if;
			else
				FC_WS_CNT <= FC_WS_CNT + 1;
			end if;
		when FC_TG68_RDL =>
			FF_FL_OE_N <= '0';
			if FC_WS_CNT = FC_WS_MAX then
				TG68_FLASH_D(7 downto 0) <= FL_DQ;
				FF_FL_OE_N <= '1';
				FC_WS_CNT <= "000";
				TG68_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			else
				FC_WS_CNT <= FC_WS_CNT + 1;
			end if;		

		when FC_T80_RD =>
			FF_FL_OE_N <= '0';
			if FC_WS_CNT = FC_WS_MAX then
				T80_FLASH_D <= FL_DQ;
				FF_FL_OE_N <= '1';
				FC_WS_CNT <= "000";
				T80_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			else
				FC_WS_CNT <= FC_WS_CNT + 1;
			end if;		

		when FC_DMA_RDU =>
			FF_FL_OE_N <= '0';
			if FC_WS_CNT = FC_WS_MAX then
				DMA_FLASH_D(15 downto 8) <= FL_DQ;
				FF_FL_OE_N <= '1';
				FC_WS_CNT <= "000";
				FF_FL_ADDR <= VBUS_ADDR(21 downto 1) & '1';
				FC <= FC_DMA_RDL;
			else
				FC_WS_CNT <= FC_WS_CNT + 1;
			end if;
			
		when FC_DMA_RDL =>
			FF_FL_OE_N <= '0';
			if FC_WS_CNT = FC_WS_MAX then
				DMA_FLASH_D(7 downto 0) <= FL_DQ;
				FF_FL_OE_N <= '1';
				FC_WS_CNT <= "000";
				DMA_FLASH_DTACK_N <= '0';
				FC <= FC_IDLE;
			else
				FC_WS_CNT <= FC_WS_CNT + 1;
			end if;		

		when others => null;
		end case;
	end if;
end process;

-- Z80 RAM (from 68000)
-- A00000-A01FFF
-- A02000-A03FFF
-- A08000-A09FFF
-- A0A000-A0BFFF
-- 10100000 0 0 0 0000000000000
-- 10100000 0 0 0 1111111111111
-- 10100000 0 0 1 0000000000000
-- 10100000 0 0 1 1111111111111
-- 10100000 1 0 0 0000000000000
-- 10100000 1 0 0 1111111111111
-- 10100000 1 0 1 0000000000000
-- 10100000 1 0 1 1111111111111

-- SDRAM CONTROL
DRAM_ADDR <= SDRADR when MRST_N = '1' else SDRADR_INIT;
DRAM_DQ(7 downto 0) <= SDRDAT(7 downto 0) when MRST_N = '1' and SDRCMD = SDRCMD_WR and SDRLDQ = '0'
	else SDRDAT_INIT(7 downto 0) when MRST_N = '0' and SDRCMD_INIT = SDRCMD_WR and SDRLDQ_INIT = '0'
	else "ZZZZZZZZ";
DRAM_DQ(15 downto 8) <= SDRDAT(15 downto 8) when MRST_N = '1' and SDRCMD = SDRCMD_WR and SDRUDQ = '0'
	else SDRDAT_INIT(15 downto 8) when MRST_N = '0' and SDRCMD_INIT = SDRCMD_WR and SDRUDQ_INIT = '0'
	else "ZZZZZZZZ";

DRAM_LDQM <= SDRLDQ when MRST_N = '1' else SDRLDQ_INIT;
DRAM_UDQM <= SDRUDQ when MRST_N = '1' else SDRUDQ_INIT;
DRAM_CS_N	<= SDRCMD(3) when MRST_N = '1' else SDRCMD_INIT(3);
DRAM_RAS_N <= SDRCMD(2) when MRST_N = '1' else SDRCMD_INIT(2);
DRAM_CAS_N	<= SDRCMD(1) when MRST_N = '1' else SDRCMD_INIT(1);
DRAM_WE_N	<= SDRCMD(0) when MRST_N = '1' else SDRCMD_INIT(0);
DRAM_CKE <= '1' when MRST_N = '1' else SDRCKE_INIT;
DRAM_BA_0 <= SDRBA(0);
DRAM_BA_1 <= SDRBA(1);
SDRBA <= "00";
process( MRST_N, MCLK, TG68_AS_N, TG68_RNW,
	TG68_A, TG68_DO, TG68_UDS_N, TG68_LDS_N,
	BAR, T80_A, T80_MREQ_N, T80_RD_N, T80_WR_N,
	VBUS_SEL, VBUS_ADDR)
begin
	if TG68_A(23 downto 21) = "111" -- 68000 RAM
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_SDRAM_SEL <= '1';
	elsif TG68_A(23 downto 16) = x"A0" -- Z80 Address Space
		and TG68_A(14) = '0' -- Z80 RAM (gen-hw.txt lines 89 and 272-273)
		and TG68_AS_N = '0' and (TG68_UDS_N = '0' or TG68_LDS_N = '0') 
	then	
		TG68_SDRAM_SEL <= '1';
	else
		TG68_SDRAM_SEL <= '0';
	end if;

	if T80_A(15 downto 14) = "00" -- Z80 RAM
		and T80_MREQ_N = '0' and (T80_RD_N = '0' or T80_WR_N = '0')
	then
		T80_SDRAM_SEL <= '1';
	elsif T80_A(15) = '1' and BAR(23 downto 21) = "111" -- 68000 RAM
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
	
		SDRCMD <= SDRCMD_XX;
		SDRDAT <= (others => 'Z');
		SDRADR <= (others => 'Z');
		SDRLDQ <= '1';
		SDRUDQ <= '1';
		SDRSEQ <= "000";
		
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
			SDRCMD <= SDRCMD_XX;
			SDRDAT <= (others => 'Z');
			SDRADR <= (others => 'Z');
			SDRLDQ <= '1';
			SDRUDQ <= '1';
			SDRSEQ <= "000";

			if VCLKCNT = "001" then
				if TG68_SDRAM_SEL = '1' and TG68_SDRAM_DTACK_N = '1' then
					if TG68_RNW = '0' then -- write
						SDRC <= SDRC_TG68_WR;
						SDRAM_DO <= TG68_DO;
					else
						SDRC <= SDRC_TG68_RD;
					end if;
				elsif T80_SDRAM_SEL = '1' and T80_SDRAM_DTACK_N = '1' then
					if T80_WR_N = '0' then -- write
						SDRC <= SDRC_T80_WR;
						SDRAM_DO <= T80_DO & T80_DO;
					else
						SDRC <= SDRC_T80_RD;
					end if;			
				elsif DMA_SDRAM_SEL = '1' and DMA_SDRAM_DTACK_N = '1' then
					SDRC <= SDRC_DMA_RD;
				else
					SDRC <= SDRC_RFSH;
				end if;
			end if;
		
		when SDRC_RFSH =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_RE;
			
				when "001" =>
					SDRCMD <= SDRCMD_XX;
			
				when "101" =>
					SDRC <= SDRC_IDLE;
				
				when others => null;
			end case;
		
		when SDRC_TG68_WR =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_AC;
					SDRADR <= TG68_A(12 downto 1);
					
				when "001" =>
					SDRCMD <= SDRCMD_XX;
					TG68_SDRAM_DTACK_N <= '0'; -- Warning					
				
				when "010" =>
					SDRADR(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					if TG68_A(23 downto 21) = "111" then -- 68000 RAM
						SDRADR(7 downto 0) <= "00000" & TG68_A(15 downto 13);
					else -- Z80 RAM
						SDRADR(7 downto 0) <= "10000" & "000";
					end if;
					SDRCMD <= SDRCMD_WR;
					SDRDAT <= SDRAM_DO; -- could be "secured" with a FF
					SDRLDQ <= TG68_LDS_N;
					SDRUDQ <= TG68_UDS_N;
				
				when "011" =>
					SDRCMD <= SDRCMD_XX;
					SDRDAT <= (others => 'Z');
					SDRLDQ <= '1';
					SDRUDQ <= '1';
					
				
				when "101" =>
					SDRC <= SDRC_IDLE;
					
				when others => null;
			end case;
		
		when SDRC_TG68_RD =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_AC;
					SDRADR <= TG68_A(12 downto 1);				
			
				when "001" =>
					SDRCMD <= SDRCMD_XX;

				when "010" =>
					SDRADR(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					if TG68_A(23 downto 21) = "111" then -- 68000 RAM
						SDRADR(7 downto 0) <= "00000" & TG68_A(15 downto 13);
					else -- Z80 RAM
						SDRADR(7 downto 0) <= "10000" & "000";
					end if;
					SDRCMD <= SDRCMD_RD;
					SDRLDQ <= TG68_LDS_N;
					SDRUDQ <= TG68_UDS_N;

				when "011" =>
					SDRCMD <= SDRCMD_XX;
					SDRDAT <= (others => 'Z');
					SDRLDQ <= '1';
					SDRUDQ <= '1';
					
				when "101" =>
					TG68_SDRAM_D <= DRAM_DQ;
					SDRC <= SDRC_IDLE;
					TG68_SDRAM_DTACK_N <= '0';
				
				when others => null;
			end case;


		when SDRC_T80_WR =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_AC;
					SDRADR <= T80_A(12 downto 1);
					
				when "001" =>
					SDRCMD <= SDRCMD_XX;
					T80_SDRAM_DTACK_N <= '0';	-- Warning		
				
				when "010" =>
					SDRADR(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					if T80_A(15) = '1' then
						SDRADR(7 downto 0) <= "00000" & BAR(15) & T80_A(14 downto 13); -- 68000 RAM
					else
						SDRADR(7 downto 0) <= "10000" & "000"; -- Z80 RAM
					end if;
					
					SDRCMD <= SDRCMD_WR;
					SDRDAT <= SDRAM_DO;
					if T80_A(0) = '0' then
						SDRUDQ <= '0';
						SDRLDQ <= '1';
					else
						SDRUDQ <= '1';
						SDRLDQ <= '0';					
					end if;
				
				when "011" =>
					SDRCMD <= SDRCMD_XX;
					SDRDAT <= (others => 'Z');
					SDRLDQ <= '1';
					SDRUDQ <= '1';
				
				when "101" =>
					SDRC <= SDRC_IDLE;
					
				when others => null;
			end case;

		when SDRC_T80_RD =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_AC;
					SDRADR <= T80_A(12 downto 1);				
			
				when "001" =>
					SDRCMD <= SDRCMD_XX;

				when "010" =>
					SDRADR(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					if T80_A(15) = '1' then
						SDRADR(7 downto 0) <= "00000" & BAR(15) & T80_A(14 downto 13); -- 68000 RAM
					else
						SDRADR(7 downto 0) <= "10000" & "000"; -- Z80 RAM
					end if;
					SDRCMD <= SDRCMD_RD;
					if T80_A(0) = '0' then
						SDRUDQ <= '0';
						SDRLDQ <= '1';
					else
						SDRUDQ <= '1';
						SDRLDQ <= '0';					
					end if;

				when "011" =>
					SDRCMD <= SDRCMD_XX;
					SDRDAT <= (others => 'Z');
					SDRLDQ <= '1';
					SDRUDQ <= '1';
					
				when "101" =>
					if T80_A(0) = '0' then
						T80_SDRAM_D <= DRAM_DQ(15 downto 8);
					else
						T80_SDRAM_D <= DRAM_DQ(7 downto 0);
					end if;
					SDRC <= SDRC_IDLE;
					T80_SDRAM_DTACK_N <= '0';
				
				when others => null;
			end case;

		when SDRC_DMA_RD =>
			SDRSEQ <= SDRSEQ + 1;
			case SDRSEQ is
				when "000" =>
					SDRCMD <= SDRCMD_AC;
					SDRADR <= VBUS_ADDR(12 downto 1);				
			
				when "001" =>
					SDRCMD <= SDRCMD_XX;

				when "010" =>
					SDRADR(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					SDRADR(7 downto 0) <= "00000" & VBUS_ADDR(15 downto 13);
					SDRCMD <= SDRCMD_RD;
					SDRLDQ <= VBUS_LDS_N;
					SDRUDQ <= VBUS_UDS_N;

				when "011" =>
					SDRCMD <= SDRCMD_XX;
					SDRDAT <= (others => 'Z');
					SDRLDQ <= '1';
					SDRUDQ <= '1';
					
				when "101" =>
					DMA_SDRAM_D <= DRAM_DQ;
					SDRC <= SDRC_IDLE;
					DMA_SDRAM_DTACK_N <= '0';
				
				when others => null;
			end case;
			
		when others => null;
		end case;

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


-- SRAM CONTROL
SRAM_ADDR <= "000" & VRAM_ADDR_INIT when MRST_N = '0' else "000" & VRAM_ADDR(14 downto 0);
W_SRAM_CE_N <= VRAM_CE_N_INIT when MRST_N = '0' else VRAM_CE_N;
W_SRAM_OE_N <= VRAM_OE_N_INIT when MRST_N = '0' else VRAM_OE_N;
W_SRAM_WE_N <= VRAM_WE_N_INIT when MRST_N = '0' else VRAM_WE_N;
W_SRAM_UB_N <= VRAM_UB_N_INIT when MRST_N = '0' else VRAM_UB_N;
W_SRAM_LB_N <= VRAM_LB_N_INIT when MRST_N = '0' else VRAM_LB_N;

SRAM_DQ(15 downto 8) <= VRAM_DI_INIT(15 downto 8) when MRST_N = '0' and W_SRAM_CE_N = '0' and W_SRAM_WE_N = '0' and W_SRAM_UB_N = '0'
	else VRAM_DI(15 downto 8) when MRST_N = '1' and W_SRAM_CE_N = '0' and W_SRAM_WE_N = '0' and W_SRAM_UB_N = '0'
	else "ZZZZZZZZ";
SRAM_DQ(7 downto 0) <= VRAM_DI_INIT(7 downto 0) when MRST_N = '0' and W_SRAM_CE_N = '0' and W_SRAM_WE_N = '0' and W_SRAM_LB_N = '0'
	else VRAM_DI(7 downto 0) when MRST_N = '1' and W_SRAM_CE_N = '0' and W_SRAM_WE_N = '0' and W_SRAM_LB_N = '0'
	else "ZZZZZZZZ";

VRAM_DO_INIT <= SRAM_DQ;
VRAM_DO <= SRAM_DQ;
	
SRAM_CE_N <= W_SRAM_CE_N;
SRAM_OE_N <= W_SRAM_OE_N;
SRAM_WE_N <= W_SRAM_WE_N;
SRAM_UB_N <= W_SRAM_UB_N;
SRAM_LB_N <= W_SRAM_LB_N;


-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################
-- #############################################################################

-- RESET SEQUENCE
process( RESET, MCLK )
begin
	if RESET = '1' then
		MRST_N <= '0';
		RSTCNT <= (others => '0');
		
		VRAM_CE_N_INIT <= '1';
		VRAM_OE_N_INIT <= '1';
		VRAM_WE_N_INIT <= '1';
		VRAM_UB_N_INIT <= '1';
		VRAM_LB_N_INIT <= '1';
		VRAM_ADDR_INIT <= (others => '0');
		VRAM_DI_INIT <= (others => '0');
		
		RSTSEQ <= RS_INIT;
	elsif rising_edge( MCLK ) then
		case RSTSEQ is
		when RS_INIT =>
			-- 50 us Flash Reset
			-- FF_FL_RST_N	<= '0';
FF_FL_RST_N	<= '1';
			-- CLK, CKE, CSn, DQM, DQ low until power stabilizes
			SDRCKE_INIT <= '0';
			SDRCMD_INIT <= SDRCMD_XX; -- CSn low
			SDRLDQ_INIT <= '0';
			SDRUDQ_INIT <= '0';
			SDRDAT_INIT <= (others => '0');
			SDRADR_INIT <= (others => 'Z');

			RSTCNT <= RSTCNT + 1;
			
			if RSTCNT(12) = '1' -- SYNTHESIS
-- synthesis translate_off
			or RSTCNT(4) = '1' -- SIMULATION
-- synthesis translate_on			
			then
				RSTSEQ <= RS_SDR_IDLE;
				RSTCNT <= (others => '0');
			end if;
		when RS_SDR_IDLE =>
			-- Flash reset done
			FF_FL_RST_N	<= '1';
			-- 200 (+ 50 us) Idle commands
			SDRCKE_INIT <= '1';
			SDRCMD_INIT <= SDRCMD_XX;
			SDRLDQ_INIT <= '1';
			SDRUDQ_INIT <= '1';
			SDRDAT_INIT <= (others => 'Z');
			SDRADR_INIT <= (others => 'Z');

			RSTCNT <= RSTCNT + 1;
			
			if RSTCNT(14) = '1' -- SYNTHESIS
-- synthesis translate_off
			or RSTCNT(4) = '1' -- SIMULATION
-- synthesis translate_on
			then 
				RSTSEQ <= RS_SDR_PALL;
				RSTCNT <= (others => '0');
			end if;
		when RS_SDR_PALL =>
			SDRADR_INIT <= "010000000000"; -- Precharge all banks
			-- Precharge all banks
			if RSTCNT(2 downto 0) = "000" then
				SDRCMD_INIT <= SDRCMD_PR;
			else
				SDRCMD_INIT <= SDRCMD_XX;
			end if;
			
			RSTCNT <= RSTCNT + 1;
			if RSTCNT(3 downto 0) = "1111" then
				RSTSEQ <= RS_SDR_RFSH;
				RSTCNT <= (others => '0');
			end if;
		when RS_SDR_RFSH =>
			-- 8 or more Auto-Refresh cycles
			if RSTCNT(2 downto 0) = "000" then
				SDRCMD_INIT <= SDRCMD_RE;
			else
				SDRCMD_INIT <= SDRCMD_XX;
			end if;
			
			RSTCNT <= RSTCNT + 1;
			if RSTCNT(7 downto 0) = "11111111" then
				RSTSEQ <= RS_SDR_MODE;
				RSTCNT <= (others => '0');
			end if;
		when RS_SDR_MODE =>
			-- Mode register set
			--	   SINGLE   CL=2 WT=0(SEQ) BL=1
			SDRADR_INIT <= "00100" & "010" & "0" & "000";
			if RSTCNT(2 downto 0) = "000" then
				SDRCMD_INIT <= SDRCMD_MS;
			else
				SDRCMD_INIT <= SDRCMD_XX;
			end if;

			RSTCNT <= RSTCNT + 1;
			if RSTCNT(3 downto 0) = "1111" then
				RSTSEQ <= RS_MEM_FILL;
				RSTCNT <= (others => '0');
			end if;

		when RS_MEM_FILL =>
			-- Fills 68000 RAM in SDRAM with 1s
			-- Fills VRAM with 0s
			-- Z80 RAM isn't initialized (doesn't seem useful)
			case RSTCNT(2 downto 0) is
				when "001" =>
					SDRCMD_INIT <= SDRCMD_AC;
					SDRADR_INIT <= RSTCNT(14 downto 3);
					
					VRAM_CE_N_INIT <= '1';
					VRAM_WE_N_INIT <= '1';
					VRAM_UB_N_INIT <= '1';
					VRAM_LB_N_INIT <= '1';
					VRAM_ADDR_INIT <= RSTCNT(17 downto 3);
					
				when "010" =>
					SDRCMD_INIT <= SDRCMD_XX;
					
					VRAM_CE_N_INIT <= '0';
					VRAM_WE_N_INIT <= '0';
					VRAM_UB_N_INIT <= '0';
					VRAM_LB_N_INIT <= '0';
				
				when "011" =>
					SDRADR_INIT(11 DOWNTO 8) <= "0100";					-- A10=1 => ENABLE AUTO PRECHARGE
					SDRADR_INIT(7 downto 0) <= "00000" & RSTCNT(17 downto 15);
					SDRCMD_INIT <= SDRCMD_WR;
					SDRDAT_INIT <= x"FFFF";
					SDRLDQ_INIT <= '0';
					SDRUDQ_INIT <= '0';
				
				when "100" =>
					SDRCMD_INIT <= SDRCMD_XX;
					SDRDAT_INIT <= (others => 'Z');
					SDRLDQ_INIT <= '1';
					SDRUDQ_INIT <= '1';					
					
					VRAM_CE_N_INIT <= '1';
					VRAM_WE_N_INIT <= '1';
					VRAM_UB_N_INIT <= '1';
					VRAM_LB_N_INIT <= '1';
					
				when others => null;
			end case;


			RSTCNT <= RSTCNT + 1;
			
			if RSTCNT(18) = '1' -- SYNTHESIS
-- synthesis translate_off
			or RSTCNT(4) = '1' -- SIMULATION
-- synthesis translate_on
			then 
				RSTSEQ <= RS_DONE;
				RSTCNT <= (others => '0');
			end if;
			
		when others =>	--- DONE
			MRST_N <= '1';
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
begin
	if rising_edge( MCLK ) then
		-- 68K FLASH READ 8-bits HI
		if FC = FC_TG68_RDU and FC_WS_CNT = FC_WS_MAX and TG68_UDS_N = '0' and TG68_UDS_N = '1' then
			write(L, string'("68K RD ROM     ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			hwrite(L, FL_DQ);
			write(L, string'("  ]"));
			writeline(F,L);
		end if;
		-- 68K FLASH READ 8-bits LO
		if FC = FC_TG68_RDL and FC_WS_CNT = FC_WS_MAX and TG68_UDS_N = '1' and TG68_UDS_N = '0' then
			write(L, string'("68K RD ROM     ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = [  "));
			hwrite(L, FL_DQ);
			write(L, string'("]"));
			writeline(F,L);
		end if;
		-- 68K FLASH READ 16-bits
		if FC = FC_TG68_RDL and FC_WS_CNT = FC_WS_MAX and TG68_UDS_N = '0' and TG68_UDS_N = '0' then
			write(L, string'("68K RD ROM     ["));
			hwrite(L, TG68_A(23 downto 0));
			write(L, string'("] = ["));
			hwrite(L, TG68_FLASH_D(15 downto 8) & FL_DQ);
			write(L, string'("]"));
			writeline(F,L);
		end if;

		-- 68K RAM ACCESS
		if (SDRC = SDRC_TG68_RD or SDRC = SDRC_TG68_WR) and SDRSEQ = "101" then
			write(L, string'("68K "));
			if TG68_RNW = '0' then
				write(L, string'("WR"));
			else
				write(L, string'("RD"));
			end if;
			if TG68_A(23 downto 21) = "111" then
				write(L, string'(" RAM-68K ["));
			else
				write(L, string'(" RAM-Z80 ["));
			end if;
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
					hwrite(L, DRAM_DQ);
				elsif TG68_UDS_N = '0' then
					hwrite(L, DRAM_DQ(15 downto 8));
					write(L, string'("  "));
				else
					write(L, string'("  "));
					hwrite(L, DRAM_DQ(7 downto 0));
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

end rtl;
