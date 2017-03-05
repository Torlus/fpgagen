/*	Firmware for loading files from SD card.
	Part of the ZPUTest project by Alastair M. Robinson.
	SPI and FAT code borrowed from the Minimig project.
*/


#include "stdarg.h"

#include "uart.h"
#include "spi.h"
#include "minfat.h"
#include "small_printf.h"
#include "host.h"
#include "ps2.h"
#include "keyboard.h"
#include "hexdump.h"
#include "osd.h"
#include "menu.h"

fileTYPE file;
static struct menu_entry topmenu[];
int dipswitch;
int bootstatus=0;

#define DEFAULT_DIPSWITCH_SETTINGS 0x00

void SetVolumes(int v);
int GetDIPSwitch();


void OSD_Puts(char *str)
{
	int c;
	while((c=*str++))
		OSD_Putchar(c);
}


void WaitEnter()
{
	while(1)
	{
		HandlePS2RawCodes();
		if(TestKey(KEY_ENTER)&2)
			return;
	}
}


void LoadSettings()
{
	if(FileOpen(&file,"FPGAGEN CFG"))	// Do we have a configuration file?
	{
		FileRead(&file,sector_buffer);
		dipswitch=*(int *)(&sector_buffer[0]);
		HW_HOST(HW_HOST_SW)=dipswitch;
		SetDIPSwitch(dipswitch);
	}
}


void SaveSettings(int row)
{
	ChangeDirectory(0);
	if(FileOpen(&file,"FPGAGEN CFG"))	// Do we have a configuration file?
	{
		int i;
		int *p=(int *)sector_buffer;
		*p++=dipswitch;
		for(i=0;i<126;++i)	// Clear remainder of buffer
			*p++=0;
		FileWrite(&file,sector_buffer);
	}
}


static int LoadROM(const char *filename)
{
	int opened;

	bootstatus=0;
	HW_HOST(HW_HOST_SW)=dipswitch;
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_RESET;	// Put core into Reset
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_SDCARD;	// Release reset but steal SD card

	if((opened=FileOpen(&file,filename)))
	{
		int result;
		int filesize=file.size;
		unsigned int c=0;
		int bits;

		bits=0;
		c=filesize-1;
		while(c)
		{
			++bits;
			c>>=1;
		}
		bits-=9;

#if 0
		if((filesize&0xfff)) // Do we have a header?
		{
			filesize-=512;
			FileNextSector(&file);	// Skip the header		
		}
#endif

		result=1;

		while(filesize>0)
		{
			OSD_ProgressBar(c,bits);
			if(FileRead(&file,sector_buffer))
			{
				int i;
				int *p=(int *)&sector_buffer;
				for(i=0;i<512;i+=4)
				{
					unsigned int t=*p++;  // AABBCCDD
					unsigned int t1=t&0xffff;  // CCDD  // ((t&0xff00)>>8)|((t&0xff)<<8); // DDCC
					unsigned int t2=(t>>16); // AABB  // ((t&0xff000000)>>24)|((t&0xff0000)>>8);  // BBAA
					HW_HOST(HW_HOST_BOOTDATA)=t2; // AABB  // BBAA
					HW_HOST(HW_HOST_BOOTDATA)=t1; // CCDD  // DDCC
				}
			}
			else
			{
				result=0;
				filesize=512;
			}
			FileNextSector(&file);
			filesize-=512;
			++c;
		}
		if(result)
			bootstatus=HW_HOST_CTRLF_BOOTDONE;
		return(result);
	}
	return(0);
}


static void reset(int row)
{
	Menu_Hide();
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_RESET;	// Put core into Reset
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_BOOTDONE;	// release keyboard
}



static int romindex=0;
static int romcount;


static void listroms();
static void selectrom(int row);
static void scrollroms(int row);

static char romfilenames[13][30];

static struct menu_entry rommenu[]=
{
	{MENU_ENTRY_CALLBACK,romfilenames[0],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[1],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[2],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[3],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[4],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[5],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[6],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[7],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[8],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[9],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[10],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[11],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_CALLBACK,romfilenames[12],MENU_ACTION(&selectrom)},
	{MENU_ENTRY_SUBMENU,"Back",MENU_ACTION(topmenu)},
	{MENU_ENTRY_NULL,0,MENU_ACTION(scrollroms)}
};


static struct menu_entry loaderror[]=
{
	{MENU_ENTRY_SUBMENU,"Load Error!",MENU_ACTION(topmenu)},
	{MENU_ENTRY_SUBMENU,"",MENU_ACTION(topmenu)},
	{MENU_ENTRY_SUBMENU,"OK",MENU_ACTION(topmenu)},
	{MENU_ENTRY_NULL,0,0}
};


static void copyname(char *dst,const unsigned char *src,int l)
{
	int i;
	for(i=0;i<l;++i)
		*dst++=*src++;
	*dst++=0;
}


static DIRENTRY *nthfile(int n)
{
	int i,j=0;
	DIRENTRY *p;
	for(i=0;(j<=n) && (i<dir_entries);++i)
	{
		p=NextDirEntry(i);
		if(p)
			++j;
	}
	return(p);
}


static void selectrom(int row)
{
	DIRENTRY *p=nthfile(romindex+row);
	int success=0;
	if(p)
	{
		copyname(longfilename,p->Name,11); // Make use of the long filename buffer to store a temporary copy of the filename,
		success=LoadROM(longfilename);	// since loading it by name will overwrite the sector buffer which currently contains it!
	}
	if(success)
	{
		Menu_Set(topmenu);
		Menu_Hide();
		OSD_Show(0);
	}
	else
	{
		Menu_Set(loaderror);
		Menu_Show();
		OSD_Show(1);
	}
}


static void selectdir(int row)
{
	DIRENTRY *p=nthfile(romindex+row);
	if(p)
		ChangeDirectory(p);
	romindex=0;
	listroms();
	Menu_Draw();
}


static void scrollroms(int row)
{
	switch(row)
	{
		case ROW_LINEUP:
			if(romindex)
				--romindex;
			break;
		case ROW_PAGEUP:
			romindex-=16;
			if(romindex<0)
				romindex=0;
			break;
		case ROW_LINEDOWN:
			++romindex;
			break;
		case ROW_PAGEDOWN:
			romindex+=16;
			break;
	}
	listroms();
	Menu_Draw();
}


static void listroms()
{
	int i,j;
	j=0;
	for(i=0;(j<romindex) && (i<dir_entries);++i)
	{
		DIRENTRY *p=NextDirEntry(i);
		if(p)
			++j;
	}

	for(j=0;(j<12) && (i<dir_entries);++i)
	{
		DIRENTRY *p=NextDirEntry(i);
		if(p)
		{
			// FIXME declare a global long file name buffer.
			if(p->Attributes&ATTR_DIRECTORY)
			{
				rommenu[j].action=MENU_ACTION(&selectdir);
				romfilenames[j][0]=16; // Right arrow
				romfilenames[j][1]=' ';
				if(longfilename[0])
					copyname(romfilenames[j++]+2,longfilename,28);
				else
					copyname(romfilenames[j++]+2,p->Name,11);
			}
			else
			{
				rommenu[j].action=MENU_ACTION(&selectrom);
				if(longfilename[0])
					copyname(romfilenames[j++],longfilename,28);
				else
					copyname(romfilenames[j++],p->Name,11);
			}
		}
		else
			romfilenames[j][0]=0;
	}
	for(;j<12;++j)
		romfilenames[j][0]=0;
}


static void showrommenu(int row)
{
	romindex=0;
	listroms();
	Menu_Set(rommenu);
//	Menu_SetHotKeys(hotkeys);
}


static struct menu_entry topmenu[];

static char *video_labels[]=
{
	"VGA - 31KHz",
	"TV - 15KHz"
};


static char *joyswap_labels[]=
{
	"Joystick swap",
	"Joystick normal"
};


static char *psg_labels[]=
{
	"PSG disable",
	"PSG enable"
};


static char *fm_labels[]=
{
	"FM disable",
	"FM enable"
};


static char *model_labels[]=
{
	"Megadrive model 1",
	"Megadrive model 2"
};

static struct menu_entry topmenu[]=
{
	{MENU_ENTRY_CALLBACK,"Reset",MENU_ACTION(&reset)},
	{MENU_ENTRY_CALLBACK,"Save settings",MENU_ACTION(&SaveSettings)},
	{MENU_ENTRY_CYCLE,(char *)video_labels,2},
	{MENU_ENTRY_TOGGLE,"Scanlines",HW_HOST_SWB_SCANLINES},
	{MENU_ENTRY_CYCLE,(char *)joyswap_labels,2},
	{MENU_ENTRY_SLIDER,"Audio Volume",7},
	{MENU_ENTRY_CYCLE,(char *)psg_labels,2},
	{MENU_ENTRY_CYCLE,(char *)fm_labels,2},
	{MENU_ENTRY_CYCLE,(char *)model_labels,2},
	{MENU_ENTRY_CALLBACK,"Load ROM \x10",MENU_ACTION(&showrommenu)},
	{MENU_ENTRY_CALLBACK,"Exit",MENU_ACTION(&Menu_Hide)},
	{MENU_ENTRY_NULL,0,0}
};


int GetVolume()
{
	struct menu_entry *m=&topmenu[5];
	int result;
	result=MENU_SLIDER_VALUE(m);
	return(result);
}


void SetVolume(int v)
{
	struct menu_entry *m=&topmenu[5];
	MENU_SLIDER_VALUE(m)=v&7;
}

#define DIPSWITCHMASK (HW_HOST_SWF_SCANLINES | ((HW_HOST_SWF_MUTE6<<1) - HW_HOST_SWF_MUTE0)) // Scanlines and mute signals

int SetDIPSwitch(int d)
{
	struct menu_entry *m;
	MENU_TOGGLE_VALUES=d&DIPSWITCHMASK; // Scanlines and mute signals
	m=&topmenu[2]; MENU_CYCLE_VALUE(m)=d&HW_HOST_SWF_VIDEOMODE; // Video mode
	m=&topmenu[4]; MENU_CYCLE_VALUE(m)=(d&HW_HOST_SWF_JOYSTICKSWAP ? 1 : 0); // Joystick swap
	m=&topmenu[6]; MENU_CYCLE_VALUE(m)=(d&HW_HOST_SWF_PSGENABLE ? 1 : 0); // PSG disable
	m=&topmenu[7]; MENU_CYCLE_VALUE(m)=(d&HW_HOST_SWF_FMENABLE ? 1 : 0); // FM disable
	m=&topmenu[8]; MENU_CYCLE_VALUE(m)=(d&HW_HOST_SWF_MODEL2 ? 1 : 0); // Megadrive model
}


int GetDIPSwitch()
{
	struct menu_entry *m;
	int result=MENU_TOGGLE_VALUES&DIPSWITCHMASK; // Scanline
	int t;
	m=&topmenu[2];
	 	if(MENU_CYCLE_VALUE(m))
			result|=HW_HOST_SWF_VIDEOMODE;	// Video mode
	m=&topmenu[4];
	 	if(MENU_CYCLE_VALUE(m))
			result|=HW_HOST_SWF_JOYSTICKSWAP;	// Joystick swap
	m=&topmenu[6];
	 	if(MENU_CYCLE_VALUE(m))
			result|=HW_HOST_SWF_PSGENABLE;	// PSG disable
	m=&topmenu[7];
	 	if(MENU_CYCLE_VALUE(m))
			result|=HW_HOST_SWF_FMENABLE;	// FM disable
	m=&topmenu[8];
	 	if(MENU_CYCLE_VALUE(m))
			result|=HW_HOST_SWF_MODEL2;	// Megadrive model

	return(result);
}


int mutechannelkeys[]=
{
	KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7
};


int main(int argc,char **argv)
{
	int i;
	bootstatus=0;
	int vol=7;
	SetDIPSwitch(DEFAULT_DIPSWITCH_SETTINGS);
	SetVolume(vol);
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_RESET;	// Reset then release, so that we get video output.
	HW_HOST(HW_HOST_CTRL)=HW_HOST_CTRLF_SDCARD; // Steal the SD card too.

	HW_HOST(HW_HOST_MOUSEBUTTONS)=3;

	OSD_Puts("Initializing SD card\n");
	i=5;
	while(--i>0)
	{
		spi_init();
		if(FindDrive())
			i=-1;
	}
	if(!i)	// Did we escape the loop?
	{
		OSD_Puts("Card init failed\n");
		return(0);
	}

	PS2Init();
	EnableInterrupts();
	PS2Wait();
	PS2Wait();
	OSD_Clear();
	OSD_Show(1);	// Figure out sync polarity
	PS2Wait();
	PS2Wait();
	OSD_Show(1);	// OSD should now show correctly.

	LoadSettings();

	if(LoadROM("BOOT    GEN"))
	{
		Menu_Set(topmenu);
		OSD_Show(0);
		bootstatus=HW_HOST_CTRLF_BOOTDONE;
	}
	else	// If we couldn't load boot.gen then we drop into the file selector...
	{
		showrommenu(0);
		Menu_Show();
	}

	while(1)
	{
		int visible;
		int channelmask;
		int update;

		HandlePS2RawCodes();
		visible=Menu_Run();
		
		HW_HOST(HW_HOST_GAMEPAD)=joya<<8|joyb;

		// Check for DIPSwitch changes from OSD and update the core if necessary.
		if(GetDIPSwitch()!=dipswitch)
		{
			int i;
			dipswitch=GetDIPSwitch();
			HW_HOST(HW_HOST_SW)=dipswitch;
			for(i=0;i<5;++i)
			{
				OSD_Show(visible);	// Refresh OSD position - needed if video mode changed.
				PS2Wait();
				PS2Wait();
			}
		}

		// Selectively mute FM channels
		// This must happen after the check for changes to the dipswitch variable.
		channelmask=HW_HOST_SWF_MUTE0;
		update=0;
		for(i=0;i<sizeof(mutechannelkeys)/sizeof(int);++i)
		{
			if(TestKey(mutechannelkeys[i])&TESTKEY_NEWPRESS_F)
			{
				dipswitch^=channelmask;
				update=1;
			}
			channelmask<<=1;
		}

		if(update)
		{
			HW_HOST(HW_HOST_SW)=dipswitch;
			SetDIPSwitch(dipswitch);
		}

		if(TestKey(KEY_F1)&TESTKEY_NEWPRESS_F) // Newly pressed since last test?
		{
			--vol;
			if(vol<0)
				vol=0;
			SetVolume(vol);
			Menu_Draw();
		}
		if(TestKey(KEY_F2)&TESTKEY_NEWPRESS_F)
		{
			++vol;
			if(vol>7)
				vol=7;
			SetVolume(vol);
			Menu_Draw();
		}
		HW_HOST(HW_HOST_VOLUMES)=GetVolume();

		if(visible)
			HW_HOST(HW_HOST_CTRL)=bootstatus|HW_HOST_CTRLF_KEYBOARD;	// capture keyboard
		else
			HW_HOST(HW_HOST_CTRL)=bootstatus;	// release keyboard
	}

	return(0);
}

