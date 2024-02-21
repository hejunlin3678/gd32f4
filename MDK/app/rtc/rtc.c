
#include "gd32f4xx.h"
#include <stdio.h>
#include "string.h"
#define RTC_CLOCK_SOURCE_IRC32K
#define BKP_VALUE 0x32F1

static rtc_parameter_struct rtc_initpara;
static rtc_parameter_struct rtc_initpara_now;

__IO uint32_t prescaler_a = 0, prescaler_s = 0;

/*!
    \brief      RTC configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void rtc_pre_config(void)
{
#if defined(RTC_CLOCK_SOURCE_IRC32K)
    rcu_osci_on(RCU_IRC32K);
    rcu_osci_stab_wait(RCU_IRC32K);
    rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);

    prescaler_s = 0x13F;
    prescaler_a = 0x63;
#elif defined(RTC_CLOCK_SOURCE_LXTAL)
    rcu_osci_on(RCU_LXTAL);
    rcu_osci_stab_wait(RCU_LXTAL);
    rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);

    prescaler_s = 0xFF;
    prescaler_a = 0x7F;
#else
#error RTC clock source should be defined.
#endif /* RTC_CLOCK_SOURCE_IRC32K */

    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}

/*!
    \brief      use hyperterminal to setup RTC time and alarm
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_setup(void)
{
    /* enable PMU clock */
    rcu_periph_clock_enable(RCU_PMU);
    /* enable the access of the RTC registers */
    pmu_backup_write_enable();

    rtc_pre_config();

    /* check if RTC has aready been configured */
    if (BKP_VALUE != RTC_BKP0)
    {
        /* setup RTC time value */
        rtc_initpara.factor_asyn = prescaler_a;
        rtc_initpara.factor_syn = prescaler_s;
        rtc_initpara.year = 0x16;
        rtc_initpara.day_of_week = RTC_SATURDAY;
        rtc_initpara.month = RTC_APR;
        rtc_initpara.date = 0x30;
        rtc_initpara.display_format = RTC_24HOUR;
        rtc_initpara.am_pm = RTC_AM;

        /* current time input */
        rtc_initpara.hour = 0x00;
        rtc_initpara.minute = 0x00;
        rtc_initpara.second = 0x00;

        /* RTC current time configuration */
        if (ERROR == rtc_init(&rtc_initpara))
        {
            printf("\n\r** RTC time configuration failed! **\n\r");
        }
        else
        {
            printf("\n\r** RTC time configuration success! **\n\r");
            RTC_BKP0 = BKP_VALUE;
        }
    }
    else
    {
        /* detect the reset source */
        if (RESET != rcu_flag_get(RCU_FLAG_PORRST))
        {
            printf("power on reset occurred....\n\r");
        }
        else if (RESET != rcu_flag_get(RCU_FLAG_EPRST))
        {
            printf("external reset occurred....\n\r");
        }
    }
    rcu_all_reset_flag_clear();

    rtc_current_time_get(&rtc_initpara); // zy+
}

/*!
    \brief      display the current time
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_show_time(void)
{
    rtc_current_time_get(&rtc_initpara_now);
    printf("Current time: %0.2x:%0.2x:%0.2x \n\r",
           rtc_initpara_now.hour, rtc_initpara_now.minute, rtc_initpara_now.second);
}

/*!
    \brief      display the now time
    \param[in]  none
    \param[out] none
    \retval     目前锟较电工锟斤拷时锟戒，锟皆凤拷锟斤拷锟斤拷为锟斤拷位
*/
uint32_t rtc_get_work_time(void) //don't  thinking about nonleap year or  leap year
{

#define TEMP_BUFFER_SIZE 2

    rtc_current_time_get(&rtc_initpara_now);

    char temp1[TEMP_BUFFER_SIZE] = {0};

    sprintf(temp1, "%x", rtc_initpara.year);
    uint8_t old_year = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara.month);
    uint8_t old_month = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara.date);
    uint8_t old_date = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara.hour);
    uint8_t old_hour = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara.minute);
    uint8_t old_minute = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara.second);
    uint8_t old_second = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.year);
    uint8_t now_year = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.month);
    uint8_t now_month = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.date);
    uint8_t now_date = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.hour);
    uint8_t now_hour = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.minute);
    uint8_t now_minute = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    sprintf(temp1, "%x", rtc_initpara_now.second);
    uint8_t now_second = atoi(temp1);
    memset(temp1, 0, TEMP_BUFFER_SIZE);

    uint32_t old_total = old_year * 525600 + old_month * 43200 + old_date * 1440 + old_hour * 60 + old_minute;

    uint32_t now_total = now_year * 525600 + now_month * 43200 + now_date * 1440 + now_hour * 60 + now_minute;

    uint32_t total = now_total - old_total;

    // printf("[%d]now time: %0.2d:%0.2d:%0.2d-- %0.2d:%0.2d:%0.2d \n\r",
    //        total, now_year, now_month, now_date, now_hour, now_minute, now_second);
    return (total);
}
