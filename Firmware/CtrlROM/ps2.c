/* Cut-down, read-only PS/2 handler for OSD code. */

#include "small_printf.h"

#include "ps2.h"
#include "interrupts.h"
#include "keyboard.h"
#include "host.h"

void ps2_ringbuffer_init(struct ps2_ringbuffer *r)
{
	r->in_hw=0;
	r->in_cpu=0;
}


int ps2_ringbuffer_read(struct ps2_ringbuffer *r)
{
	unsigned char result;
	if(r->in_hw==r->in_cpu)
		return(-1);	// No characters ready
	DisableInterrupts();
	result=r->inbuf[r->in_cpu];
	r->in_cpu=(r->in_cpu+1) & (PS2_RINGBUFFER_SIZE-1);
	EnableInterrupts();
	return(result);
}

int ps2_ringbuffer_count(struct ps2_ringbuffer *r)
{
	if(r->in_hw>=r->in_cpu)
		return(r->in_hw-r->in_cpu);
	return(r->in_hw+PS2_RINGBUFFER_SIZE-r->in_cpu);
}

struct ps2_ringbuffer kbbuffer;

static volatile int intflag;

#define PS2_TIMEOUT 5

void PS2Handler()
{
	int kbd;
	int mouse;

	DisableInterrupts();
	
	kbd=HW_PS2(REG_PS2_KEYBOARD);

	if(kbd & (1<<BIT_PS2_RECV))
	{
		kbbuffer.inbuf[kbbuffer.in_hw]=kbd&0xff;
		kbbuffer.in_hw=(kbbuffer.in_hw+1) & (PS2_RINGBUFFER_SIZE-1);
	}

	intflag=0;
	GetInterrupts();	// Clear interrupt bit
	EnableInterrupts();
}


void PS2Wait()
{
	DisableInterrupts();
	intflag=1;
	EnableInterrupts();
	while(intflag)
		;
}


void PS2Init()
{
	ps2_ringbuffer_init(&kbbuffer);
	SetIntHandler(&PS2Handler);
	ClearKeyboard();
}

