//*****************************************************************************
/**
* @file start_up_test.h
*****************************************************************************
* PROJECT NAME: Automator Safety Interlock
* OWNER: LHP Engineering Solutions
*
* @brief Module to implement start up test
*
* @authors Brian Le
*
* @date June 19 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 06/19/2024|BL |Initial version
*
*/
//*****************************************************************************

#ifndef START_UP_TEST_H
#define START_UP_TEST_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/
#define TEST_RUN_INCOMPLETE        ((uint8_t)0U)                      /**< Test or test run is not complete */
#define TEST_RUN_COMPLETE          ((uint8_t)1U)                      /**< Test or test run is complete */

/* Constants for vehicle speed */
#define VEHICLE_SPEED_ZERO         ((float32_t)0.0F)              /**< Zero speed reference value */

/*** Type Definitions ***/
typedef enum
{
    enSUT_A = 0,
    enSUT_B,
    enSUT_C,
    enTotalSUT
}enSUTList;

typedef enum
{
    enAL_A = 0,
    enAL_B,
    enTotalActionListTests
}enActionListTest;

typedef enum
{
    enPCL_A = 0,
    enPCL_B,
    enTotalPrecondListTests
}enPrecondListTest;

typedef enum
{
    enMemTest_A = 0,
    enMemTest_B,
    enMemTest_C,
    enTotalMemoryTests
}enMemoryTest;

/* Result of test or subtest */
typedef enum
{
    TestResult_NotReached = 0U,                        /**< Test has not been reached (default) */
    TestResult_Skipped,                                /**< Test has been skipped */
    TestResult_Failed,                                 /**< Test has failed */
    TestResult_Passed                                  /**< Test has passed */
}TestResult_t;

/* Index constants for action/precondition list tests */
#define ListTestIndex_NoPre     ((uint8_t)0U)                  /**< Index for list test with no preconditions */
#define ListTestIndex_Pre       ((uint8_t)1U)                  /**< Index for list test with preconditions */
#define ListTestIndex_Tot       ((uint8_t)2U)                  /**< Total list tests */

/// @brief Type for holding action and precondition list test results 
typedef struct
{
    TestResult_t enSubTestResult[ListTestIndex_Tot];  /* Result of Tests */
                                                            /**< [ListTestIndex_NoPre] - Action/Precondition list test with 'no precondition' */
                                                            /**< [ListTestIndex_Pre] - Action/Precondition list test with 'precondition' */
    uint8_t u8Completion;                             /**< Whether test has been completed */
                                                            /**< TEST_RUN_INCOMPLETE  - test not complete */
                                                            /**< TEST_RUN_COMPLETE      - test is complete */
    TestResult_t enGroupResult;                       /**< Result of Group of Tests */
}AraTestResults_t;              

/* Index constants for memory test results */
#define MemTestIndex_Pattern    ((uint8_t)0U)                 /**< Index of pattern test in MemTestResult_t */
#define MemTestIndex_March      ((uint8_t)1U)                 /**< Index of march test in MemTestResult_t */
#define MemTestIndex_CRC        ((uint8_t)2U)                 /**< Index of crc test in MemTestResult_t */
#define MemTestIndex_tot        ((uint8_t)3U)                 /**< Total memory test indexes */

/* Type for holding memory test results */
typedef struct
{
    TestResult_t enSubTestResult[MemTestIndex_tot];  /* Result of memory subtests */
                                                            /**< [MemTestIndex_Pattern] - Pattern */
                                                            /**< [MemTestIndex_March] - March */
                                                            /**< [MemTestIndex_CRC] - CRC */
    uint8_t u8Completion;                            /**< Whether test has been completed */
                                                            /**< TEST_RUN_INCOMPLETE  - test not complete */
                                                            /**< TEST_RUN_COMPLETE      - test is complete */
    TestResult_t enGroupResult;                      /**< Result of Group of Tests */
}MemTestResult_t;

/* Index constants for SUT test results */
#define SutTestIndex_ActList    ((uint8_t)0U)                /**< Index of action list test results */
#define SutTestIndex_PreList    ((uint8_t)1U)                /**< Index of precondition list test results */
#define SutTestIndex_Mem        ((uint8_t)2U)                /**< Index of memory test results */
#define SutTestIndex_tot        ((uint8_t)3U)                /**< Total sut test indexes */

/* Type for holding total Start Up Test Results test results */
typedef struct
{
    TestResult_t enRunResult[SutTestIndex_tot];     /**< Result of Tests */
                                                            /**< [SutTestIndex_ActList] - Action List */
                                                            /**< [SutTestIndex_PreList] - Precondition List */
                                                            /**< [SutTestIndex_Mem] - Memory Test */
    uint8_t u8SkippedTests;                        /**< number of skipped runs */
    uint8_t u8Completion;                          /**< Whether test has been completed */
                                                            /**< TEST_RUN_INCOMPLETE  - test not complete */
                                                            /**< TEST_RUN_COMPLETE      - test is complete */
    TestResult_t enFinalResult;                    /**< Result of of start up test as a whole */
}SutTestResults_t;

/*** Functions Provided to other modules ***/
void SUT_vMainTask(void);

/*** Variables Provided to other modules ***/


#endif
