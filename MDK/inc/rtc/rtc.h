
#ifndef __HK_RTC_H__
#define __HK_RTC_H__

extern  rtc_parameter_struct   rtc_initpara;

void rtc_setup(void);
void rtc_show_time(void);
uint32_t rtc_get_work_time(void);
#endif /* __HK_RTC_H__ */

