#include "uart.h"

#ifndef DISABLE_UART_TX
__inline int putchar(int c)
{
	while(!(HW_UART(REG_UART)&(1<<REG_UART_TXREADY)))
		;
	HW_UART(REG_UART)=c;
	return(c);
}


int puts(const char *msg)
{
	int c;
	int result=0;
	// Because we haven't implemented loadb from ROM yet, we can't use *<char*>++.
	// Therefore we read the source data in 32-bit chunks and shift-and-split accordingly.
	int *s2=(int*)msg;

	do
	{
		int i;
		int cs=*s2++;
		for(i=0;i<4;++i)
		{
			c=(cs>>24)&0xff;
			cs<<=8;
			if(c==0)
				return(result);
			putchar(c);
			++result;
		}
	}
	while(c);
	return(result);
}
#endif

#ifndef DISABLE_UART_RX
char getserial()
{
	int r=0;
	while(!(r&(1<<REG_UART_RXINT)))
		r=HW_UART(REG_UART);
	return(r);
}
#endif

