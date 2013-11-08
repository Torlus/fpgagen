$ARGC = $#ARGV+1;
$ARGC >= 1 or die("usage : bin2txt source.bin");

open(SOURCE,"<".$ARGV[0]) or die;
binmode(SOURCE);
while(sysread(SOURCE, $buf,1)) {
	$buf = unpack("H2",$buf);
	print("$buf\n");
}
close(SOURCE);
