$ARGC = $#ARGV+1;
$ARGC == 1 or die("usage : rom2vhdl rom.bin");

$start = 0;

open(TEMPLATE,"<rom_template.vhd") or die;
while(<TEMPLATE>) {
	chop;
	$line = $_;	
	if ($line =~ m/XXX/g) {
		last;
	} else {
		print "$line\n";
	}
}

open(ROMFILE, "<".$ARGV[0]) or die;
binmode( ROMFILE );
while(sysread(ROMFILE, $buf,2)) {
	$buf = unpack("H4",$buf);
	print "when x\"";
	printf("%02X", ($start++) & 0xff);
	printf "\" => D <= x\"$buf\";\n";
}
close(ROMFILE);

while(<TEMPLATE>) {
	chop;
	$line = $_;	
	print "$line\n";
}
close(TEMPLATE);

