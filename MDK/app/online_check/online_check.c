#include "online_check.h"
#include "comm/basecpld.h"



//state: 1:online    0:offline

uint8_t cpu_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_CPU);
	//bit1 
	state = state & 2;
	return state;
}

uint8_t mxm_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_MXM);
	//bit5:MXM  online
	state = state & 0x10;
	return state;
}

uint8_t alt1_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_ALT1);
	//bit3:ALT1 online check
	state = state & 4;
	return state;
}

uint8_t alt2_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_ALT2);
	///bit4:ALT2 online check
	state = state & 8;
	return state;
}

uint8_t msata1_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_MSATA1);
	//bit8:MSATA1 online check
	state = state & 0x80;
	return state;
}

uint8_t msata2_online_check(void)
{
	uint8_t state = 0;
	state = bc_read(BASE_MSATA2);
	///bit4:MSATA2 online check
	state = state & 8;
	return state;
}

