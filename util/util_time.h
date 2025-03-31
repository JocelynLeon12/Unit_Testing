/**
* @file util_time.h
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
* 08/13/2024|BL |SUD baseline 0.4
*
*/

#ifndef UTIL_TIME_H
#define UTIL_TIME_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/


/*** Type Definitions ***/
typedef char char_t;

/**
 * @brief Structure for recording date and time
 */
typedef struct
{
    uint16_t u16Year;    /**< Year recorded */
    uint8_t u8Month;     /**< Month recorded */
    uint8_t u8Day;       /**< Day Recorded */
    uint8_t u8Hour;      /**< Hour recorded */
    uint8_t u8Minute;    /**< Minute recorded */
    uint8_t u8Second;    /**< Seconds recorded */
    uint8_t u8Valid;     /**< Flag for if the time is valid */
} DateRecord_t;

/*** Functions Provided to other modules ***/
extern void UT_vGetDateTime(DateRecord_t* const pstDateRecord);
extern uint32_t UT_u32GetCurrentTime_ms(void);

/*** Variables Provided to other modules ***/

#endif /* UTIL_TIME_H */
