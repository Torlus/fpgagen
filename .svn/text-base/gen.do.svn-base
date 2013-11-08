vlib work
vcom -explicit  -93 "src/TG68_fast.vhd"
vcom -explicit  -93 "src/TG68.vhd"
vcom -explicit  -93 "src/T80/T80_Pack.vhd"
vcom -explicit  -93 "src/T80/T80.vhd"
vcom -explicit  -93 "src/T80/T80_ALU.vhd"
vcom -explicit  -93 "src/T80/T80_MCode.vhd"
vcom -explicit  -93 "src/T80/T80_Reg.vhd"
vcom -explicit  -93 "src/T80/T80se.vhd"
vcom -explicit  -93 "src/pll.vhd"
vcom -explicit  -93 "src/hex.vhd"
vcom -explicit  -93 "src/os_rom.vhd"
vcom -explicit  -93 "src/flash_sim.vhd"
vcom -explicit  -93 "src/sram_sim.vhd"
vcom -explicit  -93 "src/sdram_sim.vhd"
vcom -explicit  -93 "src/gen_io.vhd"
vcom -explicit  -93 "src/gen_fm.vhd"
vcom -explicit  -93 "src/vdp_common.vhd"
vcom -explicit  -93 "src/vdp_colinfo.vhd"
vcom -explicit  -93 "src/vdp_objinfo.vhd"
vcom -explicit  -93 "src/vdp.vhd"
vcom -explicit  -93 "src/gen_top.vhd"
vcom -explicit  -93 "src/gen_top_tb.vhd"
vsim -t 1ps -lib work gen_tb
view wave
delete wave *
view structure
view signals
onerror {resume}
quietly WaveActivateNextPane {} 0

#add wave -noupdate -format Logic /gen_tb/reset
add wave -noupdate -format Logic /gen_tb/gen/mclk

#add wave -noupdate -divider "INIT"
#add wave -noupdate -format Literal /gen_tb/gen/rstseq
#add wave -noupdate -format Logic /gen_tb/gen/mrst_n

add wave -noupdate -divider "68000"
add wave -noupdate -format Logic /gen_tb/gen/vclk
add wave -noupdate -format Literal /gen_tb/gen/vclkcnt
add wave -noupdate -format Logic /gen_tb/gen/tg68_enardreg
add wave -noupdate -format Logic /gen_tb/gen/tg68_enawrreg
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/tg68_a
add wave -noupdate -format Logic /gen_tb/gen/tg68_as_n
add wave -noupdate -format Logic /gen_tb/gen/tg68_uds_n
add wave -noupdate -format Logic /gen_tb/gen/tg68_lds_n
add wave -noupdate -format Logic /gen_tb/gen/tg68_rnw
add wave -noupdate -format Logic /gen_tb/gen/tg68_dtack_n
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/tg68_di
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/tg68_do

#add wave -noupdate -divider "Z80"
#add wave -noupdate -format Logic /gen_tb/gen/zclk
#add wave -noupdate -format Logic /gen_tb/gen/t80_clken
#add wave -noupdate -format Logic /gen_tb/gen/zbusreq
#add wave -noupdate -format Logic /gen_tb/gen/zbusack_n
#add wave -noupdate -format Logic /gen_tb/gen/zreset_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_busrq_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_busak_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_reset_n
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/t80_a
#add wave -noupdate -format Logic /gen_tb/gen/t80_mreq_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_rd_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_wr_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_wait_n
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/t80_di
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/t80_do
#add wave -noupdate -format Logic /gen_tb/gen/t80_int_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_m1_n
#add wave -noupdate -format Logic /gen_tb/gen/t80_iorq_n

#add wave -noupdate -divider "SDRAM"
#add wave -noupdate -format Logic /gen_tb/gen/mclk
#add wave -noupdate -format Logic /gen_tb/gen/vclk
#add wave -noupdate -format Literal /gen_tb/gen/vclkcnt
#add wave -noupdate -format Logic /gen_tb/gen/zclk
#add wave -noupdate -format Literal /gen_tb/gen/sdrc
#add wave -noupdate -format Logic /gen_tb/gen/dram_ras_n
#add wave -noupdate -format Logic /gen_tb/gen/dram_cas_n
#add wave -noupdate -format Logic /gen_tb/gen/dram_we_n
#add wave -noupdate -format Logic /gen_tb/gen/dram_udqm
#add wave -noupdate -format Logic /gen_tb/gen/dram_ldqm
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/dram_addr
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/dram_dq

#add wave -noupdate -divider "FLASH"
#add wave -noupdate -format Logic /gen_tb/gen/mclk
#add wave -noupdate -format Logic /gen_tb/gen/vclk
#add wave -noupdate -format Literal /gen_tb/gen/vclkcnt
#add wave -noupdate -format Logic /gen_tb/gen/zclk
#add wave -noupdate -format Literal /gen_tb/gen/fc
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fl_addr
#add wave -noupdate -format Logic /gen_tb/gen/fl_oe_n
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fl_dq

#add wave -noupdate -divider "INTERRUPTS"
#add wave -noupdate -format Logic /gen_tb/gen/hint
#add wave -noupdate -format Logic /gen_tb/gen/hint_ack
#add wave -noupdate -format Logic /gen_tb/gen/vint_tg68
#add wave -noupdate -format Logic /gen_tb/gen/vint_tg68_ack
#add wave -noupdate -format Literal /gen_tb/gen/tg68_ipl_n
#add wave -noupdate -format Logic /gen_tb/gen/tg68_intack
#add wave -noupdate -format Logic /gen_tb/gen/vint_t80
#add wave -noupdate -format Logic /gen_tb/gen/vint_t80_ack

#add wave -noupdate -divider "I/O"
#add wave -noupdate -format Literal /gen_tb/gen/ioc
#add wave -noupdate -format Logic /gen_tb/gen/io_sel
#add wave -noupdate -format Logic /gen_tb/gen/io_rnw
#add wave -noupdate -format Logic /gen_tb/gen/io_dtack_n
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/io_a
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/io_di
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/io_do

#add wave -noupdate -divider "FM"
#add wave -noupdate -format Literal /gen_tb/gen/fmc
#add wave -noupdate -format Logic /gen_tb/gen/fm_sel
#add wave -noupdate -format Logic /gen_tb/gen/fm_rnw
#add wave -noupdate -format Logic /gen_tb/gen/fm_dtack_n
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm_a
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm_di
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm_do
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm/status
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm/ta_base
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm/ta_div
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/fm/ta_value

add wave -noupdate -divider "VDP"
add wave -noupdate -format Literal /gen_tb/gen/vdpc
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp_a
add wave -noupdate -format Logic /gen_tb/gen/vdp_sel
add wave -noupdate -format Logic /gen_tb/gen/vdp_rnw
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp_di
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp_do
add wave -noupdate -format Logic /gen_tb/gen/vdp_dtack_n
add wave -noupdate -format Literal /gen_tb/gen/vdp/dtc
add wave -noupdate -format Logic /gen_tb/gen/vdp/pending
add wave -noupdate -format Literal /gen_tb/gen/vdp/code
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/addr
add wave -noupdate -format Logic /gen_tb/gen/vdp/dma_fill_pre
add wave -noupdate -format Logic /gen_tb/gen/vdp/dma_fill
add wave -noupdate -format Logic /gen_tb/gen/vdp/dma_copy
add wave -noupdate -format Logic /gen_tb/gen/vdp/dma_vbus

#add wave -noupdate -divider "PSG - BAR"
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/hexvalue
#add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/bar

add wave -noupdate -divider "VDP - SRAM"
add wave -noupdate -format Literal /gen_tb/gen/vdp/vmc
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/addr
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/addr_latch
add wave -noupdate -format Logic /gen_tb/gen/vdp/addr_set_req
add wave -noupdate -format Logic /gen_tb/gen/vdp/addr_set_ack
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/reg_latch
add wave -noupdate -format Logic /gen_tb/gen/vdp/reg_set_req
add wave -noupdate -format Logic /gen_tb/gen/vdp/reg_set_ack
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/vram_addr
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/vram_di
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/vram_do
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_ce_n
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_oe_n
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_we_n
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_ub_n
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_lb_n
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_sel
add wave -noupdate -format Logic /gen_tb/gen/vdp/vram_dtack_n

add wave -noupdate -divider "VDP - VBUS"
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vbus_addr
add wave -noupdate -format Logic /gen_tb/gen/vbus_sel
add wave -noupdate -format Logic /gen_tb/gen/vbus_dtack_n
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vbus_data
add wave -noupdate -format Literal -radix hexadecimal /gen_tb/gen/vdp/dt_dmav_data

TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
configure wave -namecolwidth 311
configure wave -valuecolwidth 60
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
