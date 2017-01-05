#ifndef PS2_H
#define PS2_H

#define PS2BASE 0xffffffe0
#define HW_PS2(x) *(volatile unsigned int *)(PS2BASE+x)

#define REG_PS2_KEYBOARD 0

#define BIT_PS2_RECV 11
#define BIT_PS2_CTS 10

// Private
#define PS2_RINGBUFFER_SIZE 8  // ringbuffer size in bytes 
struct ps2_ringbuffer
{
	volatile int in_hw;
	volatile int in_cpu;
	volatile unsigned int inbuf[PS2_RINGBUFFER_SIZE]; // Int is much easier than char for ZPU to deal with
};

void ps2_ringbuffer_init(struct ps2_ringbuffer *r);
int ps2_ringbuffer_read(struct ps2_ringbuffer *r);
int ps2_ringbuffer_count(struct ps2_ringbuffer *r);
extern struct ps2_ringbuffer kbbuffer;


void PS2Handler();

// Public interface

void PS2Init();

void PS2Wait();

#define PS2KeyboardRead(x) ps2_ringbuffer_read(&kbbuffer)
#define PS2KeyboardBytesReady(x) ps2_ringbuffer_count(&kbbuffer)
#define PS2KeyboardWrite(x) ps2_ringbuffer_write(&kbbuffer,x);

#define PS2_INT 4

#endif
