#ifndef ONLINE_CHECK
#define ONLINE_CHECK

#include "hk_common.h"

#define BASE_CPU	(0x04)	//bit2:CPU online check
#define BASE_ALT1	(0x04)	//bit3:ALT1 online check
#define BASE_ALT2	(0x04)	//bit4:ALT2 online check
#define BASE_MXM	(0x04)	//bit5:MXM  online check
#define BASE_MSATA1	(0x0B)	//bit8:MSATA1 online check
#define BASE_MSATA2	(0x0B)	//bit4:MSATA2 online check


uint8_t cpu_online_check(void);
uint8_t mxm_online_check(void);
uint8_t alt1_online_check(void);
uint8_t alt2_online_check(void);
uint8_t msata1_online_check(void);
uint8_t msata2_online_check(void);

#endif

