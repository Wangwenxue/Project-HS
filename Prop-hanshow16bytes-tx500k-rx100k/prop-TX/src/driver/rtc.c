/**
 ****************************************************************************************
 *
 * @file rtc.c
 *
 * @brief real time clock driver for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  RTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rtc.h"
#if CONFIG_ENABLE_DRIVER_RTC==TRUE

#ifdef USE_STD_C_LIB_TIME
#include <time.h>
#endif



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///Structure defining RTC environment parameters
struct rtc_capture_env_tag
{
    void   (*callback)(void);
};


///RTC environment variable
struct rtc_env_tag rtc_env = {0};
#if RTC_CAP_CALLBACK_EN==TRUE
///RTC Capture environment variable
struct rtc_capture_env_tag rtc_capture_env = {0};
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Decade convert to BCD
 * @param[in]    decade           0 ~ 99
 * @return       bcd              BCD code
 *
 ****************************************************************************************
 */
uint8_t dec2bcd(uint8_t decade)
{
    uint8_t bcd;

    bcd = ((decade/10) << 4) + (decade%10);

    return bcd;
}

/**
 ****************************************************************************************
 * @brief BCD convert to decade
 * @param[in]    bcd           BCD code
 * @return       decade        0 ~ 99
 *
 ****************************************************************************************
 */
uint8_t bcd2dec(uint8_t bcd)
{
    uint8_t decade;

    decade = ((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F);

    return decade;
}

#if CONFIG_RTC_DEFAULT_IRQHANDLER==TRUE
/**
 ****************************************************************************************
 * @brief Real time clock interrupt handler
 ****************************************************************************************
 */
void RTC_IRQHandler(void)
{
    uint32_t reg;

    reg = rtc_rtc_GetSR(QN_RTC);
    if (reg & RTC_MASK_SEC_IF) {
        rtc_time_get();
    }

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_SR_SYNC_BUSY);
    // clear int flag
    rtc_rtc_ClrSR(QN_RTC, RTC_MASK_SEC_IF);

}
#endif /* CONFIG_RTC_DEFAULT_IRQHANDLER==TRUE */


#if CONFIG_RTC_CAP_DEFAULT_IRQHANDLER==TRUE
/**
 ****************************************************************************************
 * @brief Real time clock capture interrupt handler
 ****************************************************************************************
 */
void RTC_CAP_IRQHandler(void)
{
    uint32_t reg;

    reg = rtc_rtc_GetSR(QN_RTC);
    if (reg & RTC_MASK_CAP_IF) {
#if RTC_CAP_CALLBACK_EN==TRUE
        if(rtc_capture_env.callback) {
            rtc_capture_env.callback();
        }
#endif
    }

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_SR_SYNC_BUSY);
    // clear int flag
    rtc_rtc_ClrSR(QN_RTC, RTC_MASK_CAP_IF);
}
#endif /* CONFIG_RTC_CAP_DEFAULT_IRQHANDLER==TRUE */


/**
 ****************************************************************************************
 * @brief Real time clock initialization
 * @description
 *  Initial RTC environment variable, it consists of clear callback function pointer
 ****************************************************************************************
 */
void rtc_init(void)
{
    rtc_env.time.hour = 0;
    rtc_env.time.minute = 0;
    rtc_env.time.second = 0;

    rtc_env.date.year = 0;
    rtc_env.date.month = 0;
    rtc_env.date.day = 0;

#if RTC_CALLBACK_EN==TRUE
    rtc_env.callback = NULL;
#endif
#if RTC_CAP_CALLBACK_EN==TRUE
    rtc_capture_env.callback = NULL;
#endif

    rtc_clock_on();
}

/**
 ****************************************************************************************
 * @brief RTC calibration
 * @param[in]    dir            direction of calibration
 * @param[in]    ppm            part per million
 * @description
 *  This function is used to calibrate RTC, and should be called before setting time.
 ****************************************************************************************
 */
void rtc_calibration(uint8_t dir, uint16_t ppm)
{
    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CALIB_SYNC_BUSY);
    rtc_rtc_SetCalVal(QN_RTC, (dir << 16) | ppm);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    // Enable the real time clock calibration
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CAL_EN, MASK_ENABLE);
}

/**
 ****************************************************************************************
 * @brief RTC correction
 * @param[in]    sleep_count    add sleep count to RTC counter
 * @description
 *  This function is used to correct RTC time after CPU wakeup
 ****************************************************************************************
 */
void rtc_correction(uint32_t sleep_count)
{
    uint32_t reg;

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CORR_SYNC_BUSY);
    reg = ((sleep_count / 32000) << RTC_POS_SEC_CORR) + (sleep_count % 32000);
    rtc_rtc_SetCORR(QN_RTC, reg);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    // Enable the real time clock correction
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CORR_EN, MASK_ENABLE);
}

#if CONFIG_ENABLE_DRIVER_RTC_CAP==TRUE
/**
 ****************************************************************************************
 * @brief enable RTC capture
 * @param[in]    edge           RTC captrue edge selection: posedge or negedge
 * @param[in]    callback       callback function
 * @description
 *  This function is used to initialize and enable RTC capture mode.
 ****************************************************************************************
 */
void rtc_capture_enable(enum RTC_CAP_EDGE edge, void (*callback)(void))
{
    uint32_t reg;

    reg = (edge == RTC_CAP_EDGE_POS ? 0 : RTC_MASK_CAP_EDGE_SEL)
        | RTC_MASK_CAP_EN;   
    
#if CONFIG_RTC_CAP_ENABLE_INTERRUPT==TRUE
    reg |= RTC_MASK_CAP_IE;
    /* Enable the real time capture Interrupt */
    NVIC_EnableIRQ(RTC_CAP_IRQn);
#endif

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_SR_SYNC_BUSY);
    // clear int flag
    rtc_rtc_ClrSR(QN_RTC, RTC_MASK_CAP_IF);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CAP_EDGE_SEL|RTC_MASK_CAP_IE|RTC_MASK_CAP_EN, reg);
#if RTC_CAP_CALLBACK_EN==TRUE
    rtc_capture_env.callback = callback;
#endif
}

/**
 ****************************************************************************************
 * @brief Disable RTC capture
 * @description
 *  This function is used to disable RTC captrue function
 ****************************************************************************************
 */
void rtc_capture_disable(void)
{
    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CAP_EN, MASK_DISABLE);

#if RTC_CAP_CALLBACK_EN==TRUE
    rtc_capture_env.callback = NULL;
#endif
}
#endif

#if USE_STD_C_LIB_TIME==TRUE
/**
 ****************************************************************************************
 * @brief Update RTC time
 * @param[in]    year           base on 2000  eg. if the year is 2012, the param year = 12
 * @param[in]    month          1 ~ 12
 * @param[in]    day            1 ~ 31
 * @param[in]    hour           0 ~ 23
 * @param[in]    minute         0 ~ 59
 * @param[in]    second         0 ~ 59
 * @param[in]    callback       callback function
 * @description
 * The function is used to set RTC date, time and install callback function
 ****************************************************************************************
 */
void rtc_time_set(uint8_t year, uint8_t month, uint8_t day,
                      uint8_t hour, uint8_t minute, uint8_t second, void (*callback)(void))
{

    time_t time_sec;
    struct tm ltime = {0};

#if RTC_CALLBACK_EN==TRUE
    rtc_env.callback = callback;
#endif

    ltime.tm_sec = second;
    ltime.tm_min = minute;
    ltime.tm_hour = hour;
    ltime.tm_mday = day;
    ltime.tm_mon = month - 1;
    ltime.tm_year = year + 100;

    time_sec = mktime(&ltime);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_SEC_SYNC_BUSY);
    rtc_rtc_SetSecVal(QN_RTC, time_sec);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    // Enable the real time clock configuration
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CFG, RTC_MASK_CFG);


#if CONFIG_RTC_ENABLE_INTERRUPT==TRUE
    /* Enable the real time clock Interrupt */
    NVIC_EnableIRQ(RTC_IRQn);

    rtc_int_enable();
#endif
}

/**
 ****************************************************************************************
 * @brief Get current RTC time
 * @param[in]  time      total seconds start form 1970.01.01, 00:00:00
 * @description
 * This function is used to parse RTC counter value to date and time
 ****************************************************************************************
 */
static void rtc_time_parse(uint32_t time)
{
    struct tm *ltime;

    ltime = localtime(&time); // function inside memery manage, and should use gmtime() function but there have a problem

    rtc_env.date.year   = ltime->tm_year - 100;
    rtc_env.date.month  = ltime->tm_mon + 1;
    rtc_env.date.day    = ltime->tm_mday;
    rtc_env.time.hour   = ltime->tm_hour;
    rtc_env.time.minute = ltime->tm_min;
    rtc_env.time.second = ltime->tm_sec;
    rtc_env.date.week   = ltime->tm_wday;
}
#else

/**
 ****************************************************************************************
 * @brief Update RTC time
 * @param[in]    year           1 ~ 99
 * @param[in]    month          1 ~ 12
 * @param[in]    day            1 ~ 31
 * @param[in]    hour           0 ~ 23
 * @param[in]    minute         0 ~ 59
 * @param[in]    second         0 ~ 59
 * @param[in]    callback       callback function
 * @description
 * The function is used to set date, time to RTC, and to install callback function
 ****************************************************************************************
 */
void rtc_time_set(uint8_t year, uint8_t month, uint8_t day,
                      uint8_t hour, uint8_t minute, uint8_t second, void (*callback)(void))
{
    uint32_t days;
    uint32_t seconds;
    uint32_t i = 1;


    rtc_env.date.year = year;
    rtc_env.date.month = month;
    rtc_env.date.day = day;
    rtc_env.time.hour = hour;
    rtc_env.time.minute = minute;
    rtc_env.time.second = second;
#if RTC_CALLBACK_EN==TRUE
    rtc_env.callback = callback;
#endif

    // calculate days
    days = (year-1) * DAYINYEAR + (year-1) / 4;
    while (i < month) {
        switch (i) {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
            days += DAYINBIGMONTH; // big month has 31 days
            break;

        case 2:
            if (rtc_env.date.year % 4 == 0) { // leap year, february has 29 days
                days += 29;
            }
            else {
                days += 28;
            }
            break;

        case 4:
        case 6:
        case 9:
        case 11:
            days += DAYINLITTLEMONTH; // little month havs 30days
            break;
        default:
            break;
        }
        i++;
    }
    days += (day - 1);
    // calculate seconds
    seconds = days * SECONDINDAY + (hour * 3600) + (minute * 60) + second ;


    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_SEC_SYNC_BUSY);
    rtc_rtc_SetSecVal(QN_RTC, seconds);

    // wait until last time configuration synchronize
    while (rtc_rtc_GetSR(QN_RTC) & RTC_MASK_CR_SYNC_BUSY);
    // Enable the real time clock configuration
    rtc_rtc_SetCRWithMask(QN_RTC, RTC_MASK_CFG, MASK_ENABLE);

#if CONFIG_RTC_ENABLE_INTERRUPT==TRUE
    /* Enable the real time clock Interrupt */
    NVIC_EnableIRQ(RTC_IRQn);

    rtc_int_enable();
#endif
}

/**
 ****************************************************************************************
 * @brief Get current RTC time
 * @param[in]  time      total seconds start form 01.01.01,00:00:00 (01year01month01day,00:00:00)
 *
 *       month 1 2 3 4 5 6 7 8 9 10 11 12
 *   leap year 31 29 31 30 31 30 31 31 30 31 30 31 = 366
 * normal year 31 28 31 30 31 30 31 31 30 31 30 31 = 365
 * @description
 * This function is used to parse RTC counter value to date and time, only support 01 ~ 99 year
 ****************************************************************************************
 */
static void rtc_time_parse(uint32_t time)
{
    uint32_t days;
    uint32_t seconds;
    uint32_t loop;

    rtc_env.date.year = 0x01;
    rtc_env.date.month = 0x01;
    rtc_env.date.day = 0x01;
    rtc_env.time.hour = 0x00;
    rtc_env.time.minute = 0x00;
    rtc_env.time.second = 0x00;

    days = time / SECONDINDAY;
    // 1. fix year
    while (days >= DAYINYEAR) {

        if (rtc_env.date.year % 4 == 0) { // leap year
            if (days > DAYINYEAR) {
                days -= DAYINLEAPYEAR;
                rtc_env.date.year += 1;
            }
            else { // leap year but only have 365 days
                break;
            }
        }
        else {
            days -= DAYINYEAR;
            rtc_env.date.year += 1;
        }
    }
    // 2. fix month
    loop = TRUE;
    while ((loop == TRUE)) {
        switch (rtc_env.date.month) {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
            if (days >= DAYINBIGMONTH) { // big month has 31 days
                days -= DAYINBIGMONTH;
                rtc_env.date.month += 1;
            }
            else {
                loop = FALSE;
            }
            break;

        case 2:
            if (rtc_env.date.year % 4 == 0) { // leap year, february has 29 days
                if (days >= 29) {
                    days -= 29;
                    rtc_env.date.month += 1;
                }
                else {
                    loop = FALSE;
                }
            }
            else {
                if (days >= 28) {
                    days -= 28;
                    rtc_env.date.month += 1;
                }
                else {
                    loop = FALSE;
                }
            }
            break;

        case 4:
        case 6:
        case 9:
        case 11:
            if (days >= DAYINLITTLEMONTH) { // little month havs 30days
                days -= DAYINLITTLEMONTH;
                rtc_env.date.month += 1;
            }
            else {
                loop = FALSE;
            }
            break;
        default:
            break;
        }
    }
    // 3. fix day
    rtc_env.date.day += days;
    // 4. fix hour
    seconds = time % SECONDINDAY;
    rtc_env.time.hour += (seconds / SECONDINHOUR);
    // 5. fix mintue
    rtc_env.time.minute += (seconds / 60 - rtc_env.time.hour * 60);
    // 6. fix second
    rtc_env.time.second += (seconds % 60);

}
#endif

#if 0
/**
 ****************************************************************************************
 * @brief Get current RTC time
 * @return    uint32_t    second
 *
 ****************************************************************************************
 */
uint32_t rtc_time_get(void)
{
    return rtc_rtc_GetSecVal(QN_RTC);
}
#else
/**
 ****************************************************************************************
 * @brief Get current RTC time
 * @description
 *  This function is used to get current RTC time.
 ****************************************************************************************
 */
void rtc_time_get(void)
{
    uint32_t reg;

    reg = rtc_rtc_GetSecVal(QN_RTC);
    rtc_time_parse(reg);

#if RTC_CALLBACK_EN==TRUE
    if (rtc_env.callback != NULL) {
        rtc_env.callback();
    }
#endif
}
#endif

#endif /* CONFIG_ENABLE_DRIVER_RTC==TRUE */
/// @} RTC
