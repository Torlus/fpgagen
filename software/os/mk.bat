PATH=..\..\..\Genesis_misc\soft\GenesisDev04\bin;C:\ActivePerl\Perl\bin
..\..\..\Genesis_misc\soft\GenesisDev04\bin\gcc -Wa,-a=os.lst -m68000 -Wall -O1 -fomit-frame-pointer -c os.s
..\..\..\Genesis_misc\soft\GenesisDev04\bin\gcc -T md.ld -nostdlib os.o -o os.out
..\..\..\Genesis_misc\soft\GenesisDev04\bin\objcopy -O binary os.out os.bin
perl rom2vhdl.pl os.bin > ..\..\src\os_rom.vhd
