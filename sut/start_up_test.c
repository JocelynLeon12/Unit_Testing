//*****************************************************************************
/**
* @file start_up_test.c
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


/*** Include Files ***/
#include "memory_test.h"
#include "itcom.h"
#include "util_time.h"
#include "fault_manager.h"
#include "action_request_approver.h"
#include "state_machine.h"
#include "storage_handler.h"

#include "start_up_test.h"

/*** Module Definitions ***/
///RAM size in 32 bit words
#define RAM_SIZE_32BIT                    ((uint8_t)5U)
///Invalid action request ID
#define ACTION_INVALID_ID                 ((uint16_t)0xFFFFU)
/// Start-up test execution time limit
#define SUT_EXEC_TIME_LIMIT               ((uint32_t)10U)
/// Initial time value
#define INITIAL_TIME_VALUE                ((uint32_t)0U)
/// Initial message ID
#define MESSAGE_ID_ZERO                   ((uint16_t)0U)
/// Initial sequence number
#define SEQUENCE_NUMBER_ZERO              ((uint16_t)0U)
/// Initial value for zero
#define STARTUP_ZERO_INIT_U               ((uint8_t)0U)

/*** Internal Types ***/
/// Type to mark that the conditions are appropriate for self test
typedef enum
{
    testCondition_NotCorrect = 0,	///<Not proper conditions for start up test
 	testCondition_Correct = 1,      ///<Proper conditions for start up test are occurring
}testCondition_t;



/*** Local Function Prototypes ***/
static testCondition_t sut_enStartUpTestConditions(void);
//action list test
static void sut_vActionListTestSetup(AraTestResults_t* stActListResult);
static void sut_vActionListTestRun(AraTestResults_t* stActListResult);
static void sut_vAcitonListTestComplete(SutTestResults_t* stTestResult, AraTestResults_t* stActListResult);
//Precondition list test
static void sut_vPrecondTestSetup(AraTestResults_t* stPreListResult);
static void sut_vPrecondTestRun(AraTestResults_t* stPreListResult);
static void sut_vPrecondTestComplete(SutTestResults_t* stTestResult, AraTestResults_t* stPreListResult);
//memory test
static void sut_vMemTestSetup(MemTestResult_t* stMemResult);
static void sut_vMemTestRun(MemTestResult_t* stMemResult);
static void sut_vMemTestComplete(SutTestResults_t* stTestResult, MemTestResult_t* stMemResult);



/*** External Variables ***/

/*** Internal Variables ***/


//*****************************************************************************
// FUNCTION NAME : SUT_vMainTask
//*****************************************************************************
/**
*
* @brief Main task of Start Up test that will run through the steps of all 
*
* @param none
*
* 
* @return none
*/
//*****************************************************************************
void SUT_vMainTask(void)
{
    // Variables with proper type initialization
    SutTestResults_t stTestResult = {
        {TestResult_NotReached, TestResult_NotReached, TestResult_NotReached},
        0U,
        TEST_RUN_COMPLETE,
        TestResult_NotReached
    };
    
    AraTestResults_t stActionListResult = {
        {TestResult_NotReached, TestResult_NotReached},
        TEST_RUN_COMPLETE,
        TestResult_NotReached
    };
    
    AraTestResults_t stPrecondListResult = {
        {TestResult_NotReached, TestResult_NotReached},
        TEST_RUN_COMPLETE,
        TestResult_NotReached
    };
    
    MemTestResult_t stMemoryResult = {
        {TestResult_NotReached, TestResult_NotReached, TestResult_NotReached},
        TEST_RUN_COMPLETE,
        TestResult_NotReached
    };

    testCondition_t u8StartUpCond = testCondition_NotCorrect;
    DateRecord_t stDateRecord = {0U};
    int16_t s16NotificationResult;
    uint32_t u32StartTime = INITIAL_TIME_VALUE;
    uint32_t u32ElapsedTime = INITIAL_TIME_VALUE;

    // Check SUT conditions
    u8StartUpCond = sut_enStartUpTestConditions();
    log_message(global_log_file, LOG_DEBUG, "Start-up Tests condition: %d", (int)u8StartUpCond);

    if(u8StartUpCond == (testCondition_t)testCondition_Correct)
    {
        // Action List Test
        u32StartTime = UT_u32GetCurrentTime_ms();
        sut_vActionListTestSetup(&stActionListResult);
        sut_vActionListTestRun(&stActionListResult);
        u32ElapsedTime = UT_u32GetCurrentTime_ms() - u32StartTime;

        if (u32ElapsedTime <= SUT_EXEC_TIME_LIMIT) {
            sut_vAcitonListTestComplete(&stTestResult, &stActionListResult);
            u8StartUpCond = sut_enStartUpTestConditions();
            if(stActionListResult.enGroupResult == (TestResult_t)TestResult_Failed) {
                int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_ACTION_LIST_ERROR);
                if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue) {
                    log_message(global_log_file, LOG_ERROR, "Failed to set action list error event: %d", s16ErrorEventResult);
                }
            }
            log_message(global_log_file, LOG_DEBUG, "ACTION LIST TEST EXECUTED, test conditions: %d", (int)u8StartUpCond);
        } else {
            stTestResult.u8SkippedTests++;
            stTestResult.enRunResult[SutTestIndex_ActList] = TestResult_Skipped;
            log_message(global_log_file, LOG_DEBUG, "ACTION LIST TEST TIMED OUT AND SKIPPED");
        }
    }
    else
    {
        // Skipped action list test
        stTestResult.u8SkippedTests++;
        stTestResult.enRunResult[SutTestIndex_ActList] = TestResult_Skipped;
        log_message(global_log_file, LOG_DEBUG, "ACTION LIST TEST SKIPPED");
    }

    if(u8StartUpCond == (testCondition_t)testCondition_Correct)
    {
        // Precondition Test
        u32StartTime = UT_u32GetCurrentTime_ms();
        sut_vPrecondTestSetup(&stPrecondListResult);
        sut_vPrecondTestRun(&stPrecondListResult);
        u32ElapsedTime = UT_u32GetCurrentTime_ms() - u32StartTime;

        if (u32ElapsedTime <= SUT_EXEC_TIME_LIMIT) {
            sut_vPrecondTestComplete(&stTestResult, &stPrecondListResult);
            u8StartUpCond = sut_enStartUpTestConditions();
            if(stPrecondListResult.enGroupResult == (TestResult_t)TestResult_Failed) {
                int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_PRECOND_LIST_ERROR);
                if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue) {
                    log_message(global_log_file, LOG_ERROR, "Failed to set precondition list error event: %d", s16ErrorEventResult);
                }
            }
            log_message(global_log_file, LOG_DEBUG, "PRECONDITION LIST TEST EXECUTED, test conditions: %d", (int)u8StartUpCond);
        } else {
            stTestResult.u8SkippedTests++;
            stTestResult.enRunResult[SutTestIndex_PreList] = TestResult_Skipped;
            log_message(global_log_file, LOG_DEBUG, "PRECONDITION LIST TEST TIMED OUT AND SKIPPED");
        }
    }
    else
    {
        // Skipped precondition list test
        stTestResult.u8SkippedTests++;
        stTestResult.enRunResult[SutTestIndex_PreList] = TestResult_Skipped;
        log_message(global_log_file, LOG_DEBUG, "PRECONDITION LIST TEST SKIPPED");
    }

    if(u8StartUpCond == (testCondition_t)testCondition_Correct)
    {
        // Memory Test
        u32StartTime = UT_u32GetCurrentTime_ms();
        sut_vMemTestSetup(&stMemoryResult);
        sut_vMemTestRun(&stMemoryResult);
        u32ElapsedTime = UT_u32GetCurrentTime_ms() - u32StartTime;

        if (u32ElapsedTime <= SUT_EXEC_TIME_LIMIT) {
            sut_vMemTestComplete(&stTestResult, &stMemoryResult);
            u8StartUpCond = sut_enStartUpTestConditions();
            if(stMemoryResult.enGroupResult == (TestResult_t)TestResult_Failed) {
                int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_STARTUP_MEM_ERROR);
                if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue) {
                    log_message(global_log_file, LOG_ERROR, "Failed to set memory error event: %d", s16ErrorEventResult);
                }
            }
            log_message(global_log_file, LOG_DEBUG, "MEMORY LIST TEST EXECUTED, test conditions: %d", (int)u8StartUpCond);
        } else {
            stTestResult.u8SkippedTests++;
            stTestResult.enRunResult[SutTestIndex_Mem] = TestResult_Skipped;
            log_message(global_log_file, LOG_DEBUG, "MEMORY LIST TEST TIMED OUT AND SKIPPED");
        }
    }
    else
    {
        // Skipped memory test
        stTestResult.u8SkippedTests++;
        stTestResult.enRunResult[SutTestIndex_Mem] = TestResult_Skipped;
        log_message(global_log_file, LOG_DEBUG, "MEMORY LIST TEST SKIPPED");
    }

    // Final results processing
    if(u8StartUpCond == (testCondition_t)testCondition_NotCorrect)
    {
        int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_SUT_TERM);
        if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue) {
            log_message(global_log_file, LOG_ERROR, "Failed to set SUT termination error event: %d", s16ErrorEventResult);
        }
        
        s16NotificationResult = ITCOM_s8LogNotificationMessage(
            MESSAGE_ID_ZERO,
            SEQUENCE_NUMBER_ZERO,
            (uint8_t)enUnfinishedSUT,
            (uint8_t)enStartUpTestNotification
        );
        if (s16NotificationResult < 0) {
            log_message(global_log_file, LOG_ERROR, "Failed to log unfinished SUT notification: %d", s16NotificationResult);
        }
        stTestResult.u8Completion = TEST_RUN_INCOMPLETE;
    }
    else if((stTestResult.enRunResult[SutTestIndex_ActList] == TestResult_Passed)
         && (stTestResult.enRunResult[SutTestIndex_PreList] == TestResult_Passed)
         && (stTestResult.enRunResult[SutTestIndex_Mem] == TestResult_Passed))
    {
        stTestResult.enFinalResult = TestResult_Passed;
        stTestResult.u8Completion = TEST_RUN_COMPLETE;
        s16NotificationResult = ITCOM_s8LogNotificationMessage(
            MESSAGE_ID_ZERO,
            SEQUENCE_NUMBER_ZERO,
            (uint8_t)enSuccesfulSUT,
            (uint8_t)enStartUpTestNotification
        );
        if (s16NotificationResult < 0) {
            log_message(global_log_file, LOG_ERROR, "Failed to log successful SUT notification: %d", s16NotificationResult);
        }
        log_message(global_log_file, LOG_DEBUG, "SUT COMPLETED, STATUS SUCCESS");
    }
    else
    {
        stTestResult.enFinalResult = TestResult_Failed;
        stTestResult.u8Completion = TEST_RUN_COMPLETE;
        s16NotificationResult = ITCOM_s8LogNotificationMessage(
            MESSAGE_ID_ZERO,
            SEQUENCE_NUMBER_ZERO,
            (uint8_t)enFailedSUT,
            (uint8_t)enStartUpTestNotification
        );
        if (s16NotificationResult < 0) {
            log_message(global_log_file, LOG_ERROR, "Failed to log failed SUT notification: %d", s16NotificationResult);
        }
        log_message(global_log_file, LOG_DEBUG, "SUT COMPLETED, STATUS FAILED");
    }

    UT_vGetDateTime(&stDateRecord);
    ITCOM_vRecordSutCompTime(stDateRecord);
    ITCOM_vWriteSUTRes(stTestResult);
}

//action list test
//*****************************************************************************
// FUNCTION NAME : sut_enStartUpTestConditions
//*****************************************************************************
/**
*
* @brief Check if conditions for self test are correct.
*
* conditions are:
* - vehicle is parked
* - speed is 0
* - state is self test
* - ASI initialization done
*
* @param none
* 
* @return Value representing whether condition are correct for start up test
* - testCondition_NotCorrect-Incorrect
* - testCondition_Correct-Correct
*/
//*****************************************************************************
static testCondition_t sut_enStartUpTestConditions(void)
{
    testCondition_t stResult = testCondition_NotCorrect; // assume not true
    uint8_t u8ParkStatus = (uint8_t)VEHICLE_PARK;
    float32_t f32Speed = VEHICLE_SPEED_ZERO;
    states_t stState = STATE_INITIAL;
    uint8_t u8InitDone = STARTUP_ZERO_INIT_U;
    uint8_t u8ParkInfoStatus;
    uint16_t u16SpeedInfoStatus;
    uint8_t bValidData = (uint8_t)TRUE;  // Flag to track if all data retrieved is valid

    // Get park status and check info status
    u8ParkInfoStatus = (uint8_t)ITCOM_u8GetParkStatus(&u8ParkStatus);
    if (u8ParkInfoStatus != (uint8_t)INFO_UPDATED) {
        log_message(global_log_file, LOG_ERROR, 
                   "Park status OUTDATED, info status: %u", 
                   (unsigned int)u8ParkInfoStatus);
        bValidData = (uint8_t)FALSE;
    }

    // Get vehicle speed and check info status
    u16SpeedInfoStatus = (uint16_t)ITCOM_u8GetVehicleSpeed(&f32Speed);
    if (u16SpeedInfoStatus != (uint16_t)INFO_UPDATED) {
        log_message(global_log_file, LOG_ERROR, 
                   "Vehicle speed OUTDATED, info status: %u", 
                   (unsigned int)u16SpeedInfoStatus);
        bValidData = (uint8_t)FALSE;
    }

    // Get ASI current state
    stState = (states_t)ITCOM_u8GetASIState();
    
    // Check if event is active
    u8InitDone = ITCOM_u8GetInitFlagStatus();

    // Log all status information with proper type casting for format specifiers
    log_message(global_log_file, LOG_DEBUG, 
                "PARK STATUS: %u (info: %u), VEHICLE SPEED: %f (info: %u), ASI STATE: %d, INIT DONE: %u", 
                (uint8_t)u8ParkStatus, 
                (uint8_t)u8ParkInfoStatus, 
                (float32_t)f32Speed,
                (uint8_t)u16SpeedInfoStatus, 
                (states_t)stState, 
                (uint8_t)u8InitDone);

    // Only proceed with condition check if all data is valid
    if (bValidData == (uint8_t)TRUE) 
    {
        // Set as correct if all conditions are met
        if ((u8ParkStatus == (uint8_t)enParkStatus) && 
            ((float32_t)f32Speed >= (float32_t)(-VEHICLE_SPEED_ERROR_MARGIN)) && 
            ((float32_t)f32Speed <= (float32_t)VEHICLE_SPEED_ERROR_MARGIN) && 
            (stState == (states_t)STATE_STARTUP_TEST) && 
            (u8InitDone == (uint8_t)ACTIVE_FLAG))
        {
            stResult = testCondition_Correct;
            log_message(global_log_file, LOG_DEBUG, 
                       "All start-up test conditions met");
        }
        else 
        {
            log_message(global_log_file, LOG_DEBUG, 
                       "One or more start-up test conditions not met");
        }
    }
    else 
    {
        log_message(global_log_file, LOG_ERROR, 
                   "Cannot verify start-up conditions due to invalid data");
        stResult = testCondition_NotCorrect;
    }

    return stResult;
}
//action list test
//*****************************************************************************
// FUNCTION NAME : sut_vActionListTestSetup
//*****************************************************************************
/**
*
* @brief Set up parameters for Action List Test in Action Request Approver
*
* @param [in, out] stActListResult   Results of start up test action list test run with sub test results
*
* 
* 
* @return none
*/
//*****************************************************************************
static void sut_vActionListTestSetup(AraTestResults_t* stActListResult)
{
    /* Input parameter validation */
    if (stActListResult != NULL)
    {
        /* Reset sub test results - using proper type assignment */
        stActListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_NotReached;
        stActListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_NotReached;
        
        /* Reset group result - using proper enum assignment */
        stActListResult->enGroupResult = TestResult_NotReached;
        
        /* Reset completion status - using proper constant */
        stActListResult->u8Completion = TEST_RUN_INCOMPLETE;
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Action List Test Setup called with NULL pointer");
    }
    
    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vActionListTestRun
//*****************************************************************************
/**
*
* @brief Run Action List subtest
*
* @param [in, out] stActListResult   	Results of start up test action list test run with sub test results
*
* 
* 
* @return none
*
* @note assumes result is for not reached before test
*/
//*****************************************************************************
static void sut_vActionListTestRun(AraTestResults_t* stActListResult)
{
    /* Input parameter validation */
    if (stActListResult != NULL)
    {
        uint16_t u16TestResult = STARTUP_ZERO_INIT_U;
        
        /* Define test action requests with proper type initialization */
        action_request_t stNotOnActionListPre = {
            ACTION_INVALID_ID,           /* Using defined constant */
            (uint16_t)PreID_Park,       /* Explicit casting for precondition ID */
            {0U, 0U}                    /* Proper initialization of array elements */
        };
        
        action_request_t stNotOnActionListNoPre = {
            ACTION_INVALID_ID,           /* Using defined constant */
            (uint16_t)PreID_None,       /* Explicit casting for precondition ID */
            {0U, 0U}                    /* Proper initialization of array elements */
        };

        /* Test action request without preconditions */
        u16TestResult = (uint16_t)ARA_u8ActionListCheck(&stNotOnActionListNoPre);
        
        /* Set results with proper enum assignments */
        if (u16TestResult == (uint16_t)TEST_ON_AL) /* Is on action list thus failed */
        {
            stActListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_Failed;
            
            /* Log test failure */
            log_message(global_log_file, LOG_DEBUG, 
                       "Action List Test (No Pre) Failed: Request found on list when not expected");
        }
        else /* Is not on action list thus passed */
        {
            stActListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_Passed;
            
            /* Log test success */
            log_message(global_log_file, LOG_DEBUG, 
                       "Action List Test (No Pre) Passed: Request correctly not found on list");
        }

        /* Test action request with preconditions */
        u16TestResult = (uint16_t)ARA_u8ActionListCheck(&stNotOnActionListPre);
        
        /* Set results with proper enum assignments */
        if (u16TestResult == (uint16_t)TEST_ON_AL) /* Is on action list thus failed */
        {
            stActListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_Failed;
            
            /* Log test failure */
            log_message(global_log_file, LOG_DEBUG, 
                       "Action List Test (With Pre) Failed: Request found on list when not expected");
        }
        else /* Is not on action list thus passed */
        {
            stActListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_Passed;
            
            /* Log test success */
            log_message(global_log_file, LOG_DEBUG, 
                       "Action List Test (With Pre) Passed: Request correctly not found on list");
        }
		stActListResult->u8Completion = TEST_RUN_COMPLETE;
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Action List Test Run called with NULL pointer");
    }

    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vAcitonListTestComplete
//*****************************************************************************
/**
*
* @brief Actions to perform once complete with action list test. 
*
* Mark Results of test,: pass or fail, or skipped
* @param [in,out] stTestResult   Results of start up test as whole with final run results
*                               runResult[SutTestIndex_ActList]- Action List Test
* @param [in,out] stActListResult  Results of start up test action list test run with sub test results
*
* 
* @return none
*/
//*****************************************************************************
static void sut_vAcitonListTestComplete(SutTestResults_t* stTestResult, AraTestResults_t* stActListResult)
{
    /* Input parameter validation */
    if ((stTestResult != NULL) && (stActListResult != NULL))
    {
        /* Check if test result passed by comparing individual results */
        if ((stActListResult->enSubTestResult[ListTestIndex_NoPre] == TestResult_Passed) && 
            (stActListResult->enSubTestResult[ListTestIndex_Pre] == TestResult_Passed))
        {
            /* Set results as passed - using proper enum assignments */
            stActListResult->enGroupResult = TestResult_Passed;
            stTestResult->enRunResult[SutTestIndex_ActList] = TestResult_Passed;
            
            /* Log success */
            log_message(global_log_file, LOG_DEBUG, 
                       "Action List Test completed successfully");
        }
        else /* If any test did not pass */
        {
            /* Set results as failed - using proper enum assignments */
            stActListResult->enGroupResult = TestResult_Failed;
            stTestResult->enRunResult[SutTestIndex_ActList] = TestResult_Failed;
            
            /* Set error event with proper type casting */
            int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_ACTION_LIST_ERROR);
            
            /* Check error event result */
            if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, 
                           "Failed to set action list error event in completion: %d", 
                           (int)s16ErrorEventResult);
            }
            else
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Action List Test error event set successfully");
            }
            
            /* Log which subtests failed */
            if (stActListResult->enSubTestResult[ListTestIndex_NoPre] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Action List Test (No Pre) failed");
            }
            if (stActListResult->enSubTestResult[ListTestIndex_Pre] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Action List Test (With Pre) failed");
            }
        }

        /* Send result with completion status */
        stActListResult->u8Completion = TEST_RUN_COMPLETE;
        ITCOM_vSetActionListTestResult(*stActListResult);
        
        log_message(global_log_file, LOG_DEBUG, 
                   "Action List Test results sent, completion status: %u", 
                   (unsigned int)stActListResult->u8Completion);
    }
    else
    {
        /* Log error if either pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Action List Test Complete called with NULL pointer(s)");
    }

    return;
}

//Precondition list test
//*****************************************************************************
// FUNCTION NAME : sut_vPrecondTestSetup
//*****************************************************************************
/**
*
* @brief Set up parameters for Precondition List Test in Action Request Approver
*
* @param [in,out] stPreListResult   Results of start up test precondition list test run with sub test results
*
* 
* 
* @return none
*/
//*****************************************************************************
static void sut_vPrecondTestSetup(AraTestResults_t* stPreListResult)
{
    /* Input parameter validation */
    if (stPreListResult != NULL)
    {
        /* Reset sub test results - using proper enum assignments */
        stPreListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_NotReached;
        stPreListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_NotReached;
        
        /* Reset precondition list test results */
        stPreListResult->enGroupResult = TestResult_NotReached;
        
        /* Reset completion status using defined constant */
        stPreListResult->u8Completion = TEST_RUN_INCOMPLETE;
        
        /* Log setup completion */
        log_message(global_log_file, LOG_DEBUG, 
                   "Precondition List Test setup completed");
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Precondition List Test Setup called with NULL pointer");
    }
    
    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vPrecondTestRun
//*****************************************************************************
/**
*
* @brief Run tests for action request precondition list check
*
* @param [in,out] stPreListResult   Results of start up test precondition list test run with sub test results
*
* 
* @return none
*/
//*****************************************************************************
static void sut_vPrecondTestRun(AraTestResults_t* stPreListResult)
{
    /* Input parameter validation */
    if (stPreListResult != NULL)
    {
        uint16_t u16TestResult = STARTUP_ZERO_INIT_U;
        
        /* Initialize test action requests with proper types */
        const action_request_t stNotOnPreListPre = {
            0x0003U,                    /* Action ID for Seat Position Driver */
            (uint16_t)(PreID_Total + 1U), /* Precondition ID out of range */
            {0U, 0U}                    /* Proper initialization of array elements */
        };
        
        const action_request_t stNotOnPreListNoPre = {
            0x0004U,                    /* Action ID for Seat Position Passenger */
            (uint16_t)(PreID_Total + 1U), /* Precondition ID out of range */
            {0U, 0U}                    /* Proper initialization of array elements */
        };

        /* Test action request without preconditions */
        u16TestResult = (uint16_t)ARA_u8PrecondListCheck(stNotOnPreListNoPre);
        
        /* Set results with proper enum assignments */
        if (u16TestResult == (uint16_t)TEST_ON_PL) /* Is on precondition list thus failed */
        {
            stPreListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_Failed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Precondition List Test (No Pre) Failed: Request found on list when not expected");
        }
        else /* Is not on precondition list thus passed */
        {
            stPreListResult->enSubTestResult[ListTestIndex_NoPre] = TestResult_Passed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Precondition List Test (No Pre) Passed: Request correctly not found on list");
        }

        /* Test action request with preconditions */
        u16TestResult = (uint16_t)ARA_u8PrecondListCheck(stNotOnPreListPre);
        
        /* Set results with proper enum assignments */
        if (u16TestResult == (uint16_t)TEST_ON_PL) /* Is on precondition list thus failed */
        {
            stPreListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_Failed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Precondition List Test (With Pre) Failed: Request found on list when not expected");
        }
        else /* Is not on precondition list thus passed */
        {
            stPreListResult->enSubTestResult[ListTestIndex_Pre] = TestResult_Passed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Precondition List Test (With Pre) Passed: Request correctly not found on list");
        }

        /* Send result with completion status */
        stPreListResult->u8Completion = TEST_RUN_COMPLETE;
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Precondition List Test Run called with NULL pointer");
    }

    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vPrecondTestComplete
//*****************************************************************************
/**
*
* @brief Complete precondition test including marking results and recovering 
* from action request
*
* @param [in,out] stTestResult    Results of start up test as whole with final run results,
*                                   enRunResult[SutTestIndex_PreList]- Precondition List Test results
* @param [in,out] stPreListResult     Results of start up test precondition list test run with sub test results
*
* 
* @return none
*/
//*****************************************************************************
static void sut_vPrecondTestComplete(SutTestResults_t* stTestResult, AraTestResults_t* stPreListResult)
{
    /* Input parameter validation */
    if ((stTestResult != NULL) && (stPreListResult != NULL))
    {
        /* Check if test results passed by comparing individual results */
        if ((stPreListResult->enSubTestResult[ListTestIndex_NoPre] == TestResult_Passed) && 
            (stPreListResult->enSubTestResult[ListTestIndex_Pre] == TestResult_Passed))
        {
            /* Set results as passed - using proper enum assignments */
            stPreListResult->enGroupResult = TestResult_Passed;
            stTestResult->enRunResult[SutTestIndex_PreList] = TestResult_Passed;
            
            log_message(global_log_file, LOG_DEBUG, 
                       "Precondition List Test completed successfully");
        }
        else /* If any test did not pass */
        {
            /* Set results as failed - using proper enum assignments */
            stPreListResult->enGroupResult = TestResult_Failed;
            stTestResult->enRunResult[SutTestIndex_PreList] = TestResult_Failed;
            
            /* Set error event with proper type casting */
            int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_PRECOND_LIST_ERROR);
            
            /* Check error event result */
            if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, 
                           "Failed to set precondition list error event in completion: %d", 
                           (int)s16ErrorEventResult);
            }
            else
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Precondition List Test error event set successfully");
            }
            
            /* Log which subtests failed */
            if (stPreListResult->enSubTestResult[ListTestIndex_NoPre] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Precondition List Test (No Pre) failed");
            }
            if (stPreListResult->enSubTestResult[ListTestIndex_Pre] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Precondition List Test (With Pre) failed");
            }
        }

        /* Send result with completion status */
        stPreListResult->u8Completion = TEST_RUN_COMPLETE;
        ITCOM_vSetPrecondListTestResult(*stPreListResult);
        
        log_message(global_log_file, LOG_DEBUG, 
                   "Precondition List Test results sent, completion status: %u", 
                   (unsigned int)stPreListResult->u8Completion);
    }
    else
    {
        /* Log error if either pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Precondition List Test Complete called with NULL pointer(s)");
    }
    
    return;
}

//memory test
//*****************************************************************************
// FUNCTION NAME : sut_vMemTestSetup
//*****************************************************************************
/**
*
* @brief Set up for memory test run
*
* @param [in,out] stMemResult   Results of start up memory test run with sub test results
*
* 
* @return none
*/
//*****************************************************************************
static void sut_vMemTestSetup(MemTestResult_t* stMemResult)
{
    /* Input parameter validation */
    if (stMemResult != NULL)
    {
        /* Reset sub test results - using proper enum assignments */
        stMemResult->enSubTestResult[MemTestIndex_Pattern] = TestResult_NotReached;
        stMemResult->enSubTestResult[MemTestIndex_March] = TestResult_NotReached;
        stMemResult->enSubTestResult[MemTestIndex_CRC] = TestResult_NotReached;
        
        /* Reset test run result */
        stMemResult->enGroupResult = TestResult_NotReached;
        
        /* Reset completion status using defined constant */
        stMemResult->u8Completion = TEST_RUN_INCOMPLETE;
        
        /* Log setup completion */
        log_message(global_log_file, LOG_DEBUG, 
                   "Memory Test setup completed");
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Memory Test Setup called with NULL pointer");
    }
    
    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vMemTestRun
//*****************************************************************************
/**
*
* @brief Run multiple memory tests including the following on a block of RAM
*       -Generic pattern test
*       -March test
*       -CRC test
*
* @param [out] stMemResult Results of start up memory test Run
*                         - [MemTestIndex_Pattern]  Result of Pattern test
*                         - [MemTestIndex_March] - Result of March test
*                         - [MemTestIndex_CRC] - Result of CRC test
*
* 
* 
* @return none
*/
//*****************************************************************************
static void sut_vMemTestRun(MemTestResult_t* stMemResult)
{
    /* Input parameter validation */
    if (stMemResult != NULL)
    {
        uint8_t u8TestResult = MEM_TEST_GEN_FAIL;
        /* Initialize RAM test buffer with proper alignment and type */
        static uint32_t u32TempRam[RAM_SIZE_32BIT] = {0U, 0U, 0U, 0U, 0U};
        uint32_t * const u32RAMAddress = &u32TempRam[0];
        const uint32_t u32RamBlockSize = RAM_SIZE_32BIT;

        /* Pattern test */
        u8TestResult = MEM_u8RamPatternTest(u32RAMAddress, u32RamBlockSize);
        if (u8TestResult == (uint8_t)MEM_TEST_GEN_PASSED)
        {
            stMemResult->enSubTestResult[MemTestIndex_Pattern] = TestResult_Passed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory Pattern Test passed");
        }
        else
        {
            stMemResult->enSubTestResult[MemTestIndex_Pattern] = TestResult_Failed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory Pattern Test failed");
        }

        /* RAM march test */
        u8TestResult = MEM_u8RamMarchTest(u32RAMAddress, u32RamBlockSize);
        if (u8TestResult == (uint8_t)MEM_TEST_GEN_PASSED)
        {
            stMemResult->enSubTestResult[MemTestIndex_March] = TestResult_Passed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory March Test passed");
        }
        else
        {
            stMemResult->enSubTestResult[MemTestIndex_March] = TestResult_Failed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory March Test failed");
        }

        /* CRC test */
        u8TestResult = MEM_u8CrcTest(u32RAMAddress, u32RamBlockSize);
        if (u8TestResult == (uint8_t)MEM_TEST_GEN_PASSED)
        {
            stMemResult->enSubTestResult[MemTestIndex_CRC] = TestResult_Passed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory CRC Test passed");
        }
        else
        {
            stMemResult->enSubTestResult[MemTestIndex_CRC] = TestResult_Failed;
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory CRC Test failed");
        }

        /* Send result with completion status */
        stMemResult->u8Completion = TEST_RUN_COMPLETE;
    }
    else
    {
        /* Log error if pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Memory Test Run called with NULL pointer");
    }

    return;
}

//*****************************************************************************
// FUNCTION NAME : sut_vMemTestComplete
//*****************************************************************************
/**
*
* @brief Complete memory test results, determine pass or fail and whether to set fault
*
* @param [in,out] stTestResult	 Results of start up test as whole with final run results
*                               - enRunResult[SutTestIndex_Mem]- Memory Test Result
* @param [in,out] stMemResult     Results of start up test run with sub test results
*
* 
* @return none
*/
//*****************************************************************************
static void sut_vMemTestComplete(SutTestResults_t* stTestResult, MemTestResult_t* stMemResult)
{
    /* Input parameter validation */
    if ((stTestResult != NULL) && (stMemResult != NULL))
    {
        /* Check if all tests passed by comparing individual results */
        if ((stMemResult->enSubTestResult[MemTestIndex_Pattern] == TestResult_Passed) && 
            (stMemResult->enSubTestResult[MemTestIndex_March] == TestResult_Passed) && 
            (stMemResult->enSubTestResult[MemTestIndex_CRC] == TestResult_Passed))
        {
            /* Set results as passed - using proper enum assignments */
            stMemResult->enGroupResult = TestResult_Passed;
            stTestResult->enRunResult[SutTestIndex_Mem] = TestResult_Passed;
            
            log_message(global_log_file, LOG_DEBUG, 
                       "Memory Test completed successfully");
        }
        else /* If any test did not pass */
        {
            /* Set results as failed - using proper enum assignments */
            stMemResult->enGroupResult = TestResult_Failed;
            stTestResult->enRunResult[SutTestIndex_Mem] = TestResult_Failed;
            
            /* Set error event with proper type casting */
            int16_t s16ErrorEventResult = ITCOM_s16SetErrorEvent((uint16_t)EVENT_ID_FAULT_STARTUP_MEM_ERROR);
            
            /* Check error event result */
            if (s16ErrorEventResult != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, 
                           "Failed to set memory test error event in completion: %d", 
                           (int)s16ErrorEventResult);
            }
            else
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Memory Test error event set successfully");
            }
            
            /* Log which subtests failed */
            if (stMemResult->enSubTestResult[MemTestIndex_Pattern] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Memory Pattern Test failed");
            }
            if (stMemResult->enSubTestResult[MemTestIndex_March] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Memory March Test failed");
            }
            if (stMemResult->enSubTestResult[MemTestIndex_CRC] != TestResult_Passed)
            {
                log_message(global_log_file, LOG_DEBUG, 
                           "Memory CRC Test failed");
            }
        }

        /* Send result with completion status */
        stMemResult->u8Completion = TEST_RUN_COMPLETE;
        ITCOM_vSetMemoryTestResult(*stMemResult);
        
        log_message(global_log_file, LOG_DEBUG, 
                   "Memory Test results sent, completion status: %u", 
                   (unsigned int)stMemResult->u8Completion);
    }
    else
    {
        /* Log error if either pointer is NULL */
        log_message(global_log_file, LOG_ERROR, 
                   "Memory Test Complete called with NULL pointer(s)");
    }
    
    return;
}



