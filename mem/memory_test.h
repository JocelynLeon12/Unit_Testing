//*****************************************************************************
/**
* @file memory_test.h
*****************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief module to implement memory tests
*
* @authors Brian Le
*
* @date July 01 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 07/01/2024|BL |Initial
* 08/05/2024|BL |Baseline
* 08/06/2024|BL |corrections to baseline
* 08/08/2024|BL |correct names of param and func
*
*/
//*****************************************************************************
#ifndef MEMORY_TEST_H
#define MEMORY_TEST_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/
///General value for memory test has failed
#define MEM_TEST_GEN_FAIL           (0U)
///	General value for memory test has passed
#define MEM_TEST_GEN_PASSED         (1U)

/*** Type Definitions ***/

/*** Functions Provided to other modules ***/
uint8_t MEM_u8RamPatternTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize);
uint8_t MEM_u8RamMarchTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize);
uint8_t MEM_u8CrcTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize);


/*** Variables Provided to other modules ***/


/*** Functions to test module ***/



#endif /* MEMORY_TEST_H */