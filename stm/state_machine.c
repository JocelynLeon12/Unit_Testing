/**
* @file state_machine.c
*****************************************************************************
* PROJECT NAME: Sonatus Automator
* ORIGINATOR: Sonatus
*
* @brief module to implement state machine
*
* @authors Brian Le
*
* @date June 05 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 06/05/2024|BL |Initial
* 08/09/2024|BL |Change interface and var naming
* 08/13/2024|BL |SUD baseline 0.4
* 10/04/2024|TP |Update for consistent safe state handling across child process restarts
*
*/
//*****************************************************************************


/*** Include Files ***/
#include "itcom.h"
#include "fault_manager.h"
#include "action_request_approver.h"
#include "start_up_test.h"
#include "storage_handler.h"
#include "thread_management.h"
#include "start_up_test.h"

#include "state_machine.h"


/*** Module Definitions ***/



/*** Internal Types ***/
///Values of flag for if a fault has occurred
typedef enum
{
    noFail = 0,         ///<No critical fault has been detected
    Failure = 1         ///<A critical fault has been detected
} fail_flag_t;


/*** Local Function Prototypes ***/
//state machine management
static void stm_vCollectStatuses(fail_flag_t* enFailFlag);
static void stm_vTransitions(fail_flag_t enFailFlag);
//safe state
static void stm_vSsEntryActions(void);

/*** External Variables ***/

/*** Internal Variables ***/
///Current state of state machine
static states_t m_enState = STATE_INITIAL;


//*****************************************************************************
// FUNCTION NAME : STM_vInit
//*****************************************************************************
/**
*
* @brief Initialize state machine
* 
* 
* @param none
*
* @global{out; m_enState)
* 
* @return none
*/
//*****************************************************************************
void STM_vInit(void)
{
    uint8_t u8InitFlagStatus = ITCOM_u8GetInitFlagStatus();
    m_enState = ITCOM_u8GetASIState();
    if (m_enState == STATE_SAFE_STATE) {
        log_message(global_log_file, LOG_INFO, "STM Initializing with Safe State");
    } else {
        m_enState = STATE_INITIAL;
        ITCOM_vSetASIState(m_enState);
        log_message(global_log_file, LOG_INFO, "STM Initializing with Initial State");
    }
    u8InitFlagStatus = (u8InitFlagStatus == ACTIVE_FLAG ? u8InitFlagStatus : INACTIVE_FLAG);
    ITCOM_vSetInitFlagStatus(u8InitFlagStatus);
}

//*****************************************************************************
// FUNCTION NAME : STM_vMainTask
//*****************************************************************************
/**
*
* @brief main task for state machine to perform periodically
*
* performs data collection,
*
* @global{in,out; m_enState}
* 
* @pre Assumes that STM_vInit was called before for initialization
*
* @note need to run initialization first
*
* @return none
*/
//*****************************************************************************
void STM_vMainTask(void)
{
    fail_flag_t enFailFlag;

	//collect relevant data
    stm_vCollectStatuses(&enFailFlag);
    //transition
    stm_vTransitions(enFailFlag);
}



//*****************************************************************************
// FUNCTION NAME : stm_vCollectStatuses
//*****************************************************************************
/**
*
* @brief Unit to gather statuses for running state machine
*
* @param [in,out] enFailFlag        flag to indicate if critical fault has occured
* @param [in,out] enStartUpTestFin  flag for if start up test finised
* @param [in,out] enInitDone
* @global{in; m_enState; current state of state machine}
* 
* @return none
*/
//*****************************************************************************
static void stm_vCollectStatuses(fail_flag_t* enFailFlag) 
{
    //variables
    uint8_t u8CriticalFlagValue = ITCOM_u8GetCriticalFaultStatus();
    uint8_t u8InitFlagValue = ITCOM_u8GetInitFlagStatus();

    if(u8CriticalFlagValue == (uint8_t)ACTIVE_FLAG || u8InitFlagValue == (uint8_t)INACTIVE_FLAG)
    {
        *enFailFlag = Failure;
    }
    else
    {
        *enFailFlag = noFail;
    }
}

//*****************************************************************************
// FUNCTION NAME : stm_vTransitions
//*****************************************************************************
/**
*
* @brief Perform state transitions when conditions are correct. 
* Also perform entry and exit actions for states
*
* @param none
*
* @param [in] enFailFlag flag to indicate if critical fault has occured
* @global{in,out; m_enState; current state pf state machine}
* 
* @return none
*/
//*****************************************************************************
static void stm_vTransitions(fail_flag_t enFailFlag)
{
    StateMonitor_t stStateMonitorData = {STATE_INITIAL, FALSE};
    //transition based on state and conditions
    if((enFailFlag == Failure) && (!get_thread_exit())) {
        m_enState = (states_t)STATE_SAFE_STATE;
        ITCOM_vSetASIState(m_enState);
        stm_vSsEntryActions();
        log_message(global_log_file, LOG_INFO, "ASI TRANSITIONED TO: SAFE STATE");
    }
    else {
        m_enState = ITCOM_u8GetASIState();
        if((m_enState == STATE_INITIAL) && (!get_thread_exit()))
        {
            m_enState = STATE_STARTUP_TEST;
            ITCOM_vSetASIState(m_enState);
            log_message(global_log_file, LOG_INFO, "ASI TRANSITIONED TO: START-UP TEST");
        }
        else if ((m_enState == STATE_STARTUP_TEST) && (!get_thread_exit()))
        {
            stVehicleStatusInfo_t stVehicleStatus = {0};
            stVehicleStatus.u8InfoStatus[0] = ITCOM_u8GetParkStatus(&stVehicleStatus.u8ParkStatus);
            stVehicleStatus.u8InfoStatus[1] = ITCOM_u8GetVehicleSpeed(&stVehicleStatus.fVehicleSpeed);

            if(stVehicleStatus.u8InfoStatus[0] == INFO_UPDATED && stVehicleStatus.u8InfoStatus[1] == INFO_UPDATED) {
                SUT_vMainTask();
                m_enState = STATE_NORM_OP;
                ITCOM_vSetASIState(m_enState);
                log_message(global_log_file, LOG_INFO, "ASI TRANSITIONED TO: NORMAL OPERATION");
            }
        }
        else if (m_enState == STATE_NORM_OP)
        {
            // do nothing
        }
        else if ((m_enState == STATE_SAFE_STATE) && (!get_thread_exit()))
        {
            stm_vSsEntryActions();
            log_message(global_log_file, LOG_INFO, "ASI IN: SAFE STATE");
        }
        else {
            // do nothing
        }
    }
    ITCOM_vGetStateMonitorTestData(&stStateMonitorData);
    stStateMonitorData.stCurrentState = m_enState;
    ITCOM_vSetStateMonitorTestData(stStateMonitorData);
}



//*****************************************************************************
// FUNCTION NAME : stm_vSsEntryActions
//*****************************************************************************
/**
*
* @brief Safe state actions to be performed when entering safe state
*
* Drops current action requests and and send message to VAM
* 
* @param none
*
* @return none
*/
//*****************************************************************************
static void stm_vSsEntryActions(void)
{
    int8_t s8EequeueStatus = ITCOM_s8LogSSMessage();
    if (s8EequeueStatus == ENQUEUE_OPERATION_FAILURE) {
        log_message(global_log_file, LOG_ERROR, "STM_vSsEntryActions: Failed to send Safe State message");
    }
}
