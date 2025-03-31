/**
* @file util_time.c
*****************************************************************************
* PROJECT NAME: Sonatus Automator
* ORIGINATOR: Sonatus
*
* @brief utilities related to time
*
* @authors Brian Le
*
* @date Aug. 13 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 08/13/2024|BL |Initial
*
*/

/*** Include Files ***/
#include "util_time.h"

/*** Module Definitions ***/
#define UT_ZERO_INIT_U       (0U)
#define FLAG_FALSE           (0U)
#define FLAG_TRUE            (1U)
#define YEAR_OFFSET          (1900U)       /* Base year for tm structure */
#define DAY_OFFSET           (1U)          /* Offset to adjust day count */
#define MONTH_OFFSET         (1U)          /* Offset to adjust month count */
#define SEC_TO_MSEC          (1000U)       /* Seconds to milliseconds conversion */
#define NSEC_TO_MSEC         (1000000U)    /* Nanoseconds to milliseconds conversion */


/*** Internal Types ***/

/*** Local Function Prototypes ***/

/*** External Variables ***/

/*** Internal Variables ***/

//************************************************************************
// Function UT_vGetDateTime
//************************************************************************
/**
* @brief Unit to get date and time in a custom structured format.
* 
* @param [in,out] pstDateRecord Date and time value structure
* 
* @return none
*/
void UT_vGetDateTime(DateRecord_t* const pstDateRecord)
{
    time_t stRawTime = UT_ZERO_INIT_U;
    struct tm *stGmTime = NULL;

    if(VALID_PTR(pstDateRecord)) {
        stRawTime = time(&stRawTime);
        stGmTime = gmtime(&stRawTime);

        if(VALID_PTR(stGmTime)) {
            pstDateRecord->u8Valid  = (uint8_t)FLAG_TRUE;
            pstDateRecord->u16Year  = (uint16_t)(YEAR_OFFSET + (uint16_t)(stGmTime->tm_year));
            pstDateRecord->u8Month  = (uint8_t)(stGmTime->tm_mon + MONTH_OFFSET);
            pstDateRecord->u8Day    = (uint8_t)(stGmTime->tm_mday + DAY_OFFSET);
            pstDateRecord->u8Hour   = (uint8_t)(stGmTime->tm_hour);
            pstDateRecord->u8Minute = (uint8_t)(stGmTime->tm_min);
            pstDateRecord->u8Second = (uint8_t)(stGmTime->tm_sec);
        }
        else {
            pstDateRecord->u8Valid = (uint8_t)FLAG_FALSE;
        }
    }
}

uint32_t UT_u32GetCurrentTime_ms(void)
{
    struct timespec ts;
    int32_t result;
    uint32_t u32ReturnValue = UT_ZERO_INIT_U;
    
    result = clock_gettime(CLOCK_MONOTONIC, &ts);
    if (result == 0) {
        // Fixed type conversions
        uint32_t u32Seconds = (uint32_t)ts.tv_sec;
        uint32_t u32Nanoseconds = (uint32_t)ts.tv_nsec;
        uint32_t u32MsFromSec = u32Seconds * SEC_TO_MSEC;  /* SEC_TO_MSEC as literal */
        uint32_t u32MsFromNs = u32Nanoseconds / NSEC_TO_MSEC;  /* NSEC_TO_MSEC as literal */
        u32ReturnValue = u32MsFromSec + u32MsFromNs;
    }
    
    return u32ReturnValue;
}
