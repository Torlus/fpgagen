library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
library STD;
use STD.TEXTIO.ALL;
use work.vdp_common.all;

entity video_sync is
	port(
		RST_N		: in std_logic;
		CLK		: in std_logic;
		VGA_HS	: out std_logic;
		VGA_VS	: out std_logic
	);
end video_sync;

architecture rtl of video_sync is


----------------------------------------------------------------
-- VIDEO OUTPUT
----------------------------------------------------------------
signal FF_VGA_HS		: std_logic;
signal FF_VGA_VS		: std_logic;

----------------------------------------------------------------
-- VIDEO COUNTING
----------------------------------------------------------------
signal H_CNT		: std_logic_vector(11 downto 0);
signal H_VGA_CNT	: std_logic_vector(10 downto 0);
signal V_CNT		: std_logic_vector(9 downto 0);

begin

----------------------------------------------------------------
-- VIDEO COUNTING
----------------------------------------------------------------
process( RST_N, CLK )
begin
	if RST_N = '0' then
		H_CNT <= (others => '0');
		H_VGA_CNT <= (others => '0');
		V_CNT <= (others => '0');
	elsif rising_edge(CLK) then
		H_CNT <= H_CNT + 1;
		H_VGA_CNT <= H_VGA_CNT + 1;
		if H_VGA_CNT = (CLOCKS_PER_LINE/2)-1 then
			H_VGA_CNT <= (others => '0');
			V_CNT <= V_CNT + 1;			
			if V_CNT = (NTSC_LINES*2)-1 then
				V_CNT <= (others => '0');
				--FIELD <= not FIELD;
			end if;
		end if;
	end if;
end process;

-- VERTICAL SYNC
process( RST_N, CLK )
begin
	if RST_N = '0' then
		FF_VGA_VS <= '1';
	elsif rising_edge(CLK) then
		if V_CNT = 0 then
			FF_VGA_VS <= '0';
		elsif V_CNT = (VGA_VS_LINES*2) then
			FF_VGA_VS <= '1';
		end if;
	end if;
end process;

-- HORIZONTAL SYNC
process( RST_N, CLK )
begin
	if RST_N = '0' then
		FF_VGA_HS <= '1';
	elsif rising_edge(CLK) then
		if H_VGA_CNT = 0 then
			FF_VGA_HS <= '0';
		elsif H_VGA_CNT = VGA_HS_CLOCKS then
			FF_VGA_HS <= '1';
		end if;
	end if;
end process;

VGA_HS <= FF_VGA_HS;
VGA_VS <= FF_VGA_VS;

end rtl;
