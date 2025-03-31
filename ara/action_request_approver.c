/*****************************************************************************
 * @file action_request_approver.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Action Request Processing and Safety Validation Implementation
 *
 * @details
 * This file implements the Action Request Approver (ARA) module for the Sonatus
 * Automator project. It provides comprehensive validation and processing of
 * vehicle action requests with strict safety checks.
 *
 * Key Features:
 * - Action request validation and safety checks
 * - Vehicle state monitoring (PRNDL status)
 * - Precondition definitions for safety-critical operations
 * - Range limit validation for various action parameters
 * - Structured action request processing
 * - Vehicle status monitoring with error margin handling
 *
 * The implementation ensures that vehicle actions are only executed when all
 * safety conditions are met, with particular emphasis on vehicle state (park
 * status) and parameter range validation.
 *
 * @authors Brian Le (BL), Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date August 14 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/14/2024 | BL     | Initial Implementation
 * 10/03/2024 | TP     | Refactored for Action Request Timeout
 * 10/24/2024 | AT     | Clean up actions
 * 11/22/2024 | TP     | Cleaning up the code
 */

/*** Include Files ***/
#include "storage_handler.h"
#include "fault_manager.h"
#include "itcom.h"
#include "icm.h"

#include "action_request_approver.h"

/*** Module Definitions ***/
///Total number of action requests
#define TOTAL_AR    (12U)   

#define MSG_LENGTH_ONE_BYTE          (0x01U)
#define MSG_LENGTH_TWO_BYTES         (0x02U)
#define MSG_LENGTH_FOUR_BYTES        (0x04U)
#define MSG_LENGTH_EIGHT_BYTES       (0x08U)

#define PROCESS_REQUEST_CONTINUE     (1U)
#define PROCESS_REQUEST_DISCARD      (0U)

#define ARA_ZERO_INIT_U              (0U)

#define INVALID_ACTION_ID            (0xFFFFU)
#define INITIAL_SEQUENCE_NUM         (0U)

#define MSG_DATA_INDEX_0             (0U)
#define MSG_DATA_INDEX_1             (1U)
#define MSG_DATA_INDEX_2             (2U)
#define MSG_DATA_INDEX_3             (3U)

/* Shift amount for eight bits */
#define ARA_SHIFT_EIGHT              (8U)
/* Shift amount for sixteen bits */
#define ARA_SHIFT_SIXTEEN            (16U)
/* Shift amount for twenty-four bits */
#define ARA_SHIFT_TWENTY_FOUR        (24U)


#define PREDEFINED_ACTION_LIST { \
    /*Action ID,    Precondition,       low au32RangeLimits,    upper au32RangeLimits*/ \
    {0x0000,        PreID_None,         {0x00,                  0x04}},         /* enHVACFanSpeed */ \
    {0x0001,        PreID_None,         {0x32,                  0x64}},         /* enHVACCabinTemperature */ \
    {0x0002,        PreID_None,         {0x00,                  0x04}},         /* enWindshieldWiperSpeed */ \
    {0x0003,        PreID_Park,         {0x00,                  0x64}},         /* enSeatPositionDriver */ \
    {0x0004,        PreID_None,         {0x00,                  0x64}},         /* enSeatPositionPassenger */ \
    {0x0005,        PreID_None,         {0x00,                  0x04}},         /* enSeatHeaterDriver */ \
    {0x0006,        PreID_None,         {0x00,                  0x04}},         /* enSeatHeaterPassenger */ \
    {0x0007,        PreID_Park,         {0x00,                  0x01}},         /* enDoorLockState */ \
    {0x0008,        PreID_None,         {0x00,                  0x03}},         /* enTurnSignalState */ \
    {0x0009,        PreID_None,         {0x00,                  0xFFFFFU}},      /* enAmbientLighting */ \
    {0x000A,        PreID_Park,         {0x00,                  0xFF}},         /* enTorqueVecMotorCalib */ \
    {0x07D0,        PreID_None,         {0x00,                  0x04}},         /* enRainSensor */ \
}

/*** Internal Types ***/

/*** Local Function Prototypes ***/
static uint8_t ara_u8RangeCheckEvaluation(stProcessMsgData stMsgData);

/*** External Variables ***/

/*** Internal Variables ***/
static const action_request_t m_stActionList[TOTAL_AR] = PREDEFINED_ACTION_LIST;
static uint8_t m_u8VehicleStatus = VEHICLE_NOT_PARK;

/*** Functions Provided to other modules ***/

/**
 * @brief Monitors and processes vehicle action requests with safety validation
 *
 * @details
 * This function implements the core action request processing logic with comprehensive
 * safety checks and validation. It performs the following operations in sequence:
 * 
 * 1. Checks if the system is in normal operation state
 * 2. Dequeues action requests from the integrity queue
 * 3. Validates each request against:
 *    - Predefined action list
 *    - Parameter range limits
 *    - Required safety preconditions
 * 4. Processes or discards requests based on validation results
 *
 * Processing Flow:
 * ---------------
 * 1. System State Check:
 *    - Only processes requests when system is in normal operation
 *    - Monitors ASI (Automator Safety Interlock) state
 *
 * 2. Request Validation:
 *    - Validates action ID against predefined action list
 *    - Performs range checking on action parameters
 *    - Verifies preconditions (e.g., vehicle park status)
 *
 * 3. Error Handling:
 *    - Logs events for validation failures
 *    - Sends notifications for invalid requests
 *    - Handles timeout conditions
 *
 * 4. Request Processing:
 *    - Queues valid requests for execution
 *    - Handles precondition-dependent actions
 *    - Manages request timeouts
 *
 * Error Events Generated:
 * ----------------------
 * - EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR: Invalid action ID
 * - EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR: Parameter out of range
 * - EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR: Precondition failure
 * - EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT: Processing timeout
 *
 * @note This function is typically called periodically from the main processing loop
 *       to handle incoming action requests.
 *
 * @warning Ensure that vehicle status information is up-to-date before processing
 *          safety-critical actions.
 *
 * @pre The ASI system must be initialized and operational
 * @pre The integrity queue must be properly initialized
 *
 * @post Valid requests are queued for processing
 * @post Invalid requests are logged and discarded
 * 
 * @return void
 *
 */
void ARA_vActionRequestMonitor(void)
{
    uint8_t u8ActionListCheck = TEST_NOT_ON_AL;
    uint8_t u8ActionRangeCheck = RANGE_CHECK_FAILED;
    uint8_t u8ASI_State = STATE_INITIAL;
    int8_t s8DequeueStatus = -1;
    stProcessMsgData stTempMsgData = {.stMsgPairData.u16MsgId = INVALID_ACTION_ID, .stMsgPairData.u16SequenceNum = INITIAL_SEQUENCE_NUM};
    action_request_t stActionReqData = {0U};
    int8_t s8Return = QUEUE_ACTION_FAILURE_DEFAULT;
    uint8_t u8ProcessRequest = PROCESS_REQUEST_CONTINUE;

    u8ASI_State = ITCOM_u8GetASIState();

    if ((uint8_t)u8ASI_State == (uint8_t)STATE_NORM_OP)
    {
        s8DequeueStatus = ITCOM_s8DequeueActionReq(&stTempMsgData, DATA_INTEGRITY_QUEUE);

        if (s8DequeueStatus >= 0)
        {
            stActionReqData.u16ActionId = stTempMsgData.stMsgPairData.u16MsgId;
            u8ActionRangeCheck = ara_u8RangeCheckEvaluation(stTempMsgData);
            u8ActionListCheck = ARA_u8ActionListCheck(&stActionReqData);

            if ((uint8_t)u8ActionListCheck == (uint8_t)TEST_NOT_ON_AL)
            {
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set action list error event: %d", s16ErrorStatus);
                }

                int8_t s8NotifyStatus = (int8_t)ITCOM_s8LogNotificationMessage(
                    stTempMsgData.stMsgPairData.u16MsgId,
                    stTempMsgData.stMsgPairData.u16SequenceNum,
                    (uint8_t)enInvalidActionReq,
                    (uint8_t)enActionNotification);
                if ((int8_t)s8NotifyStatus < 0)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to log invalid action request notification: %d", s8NotifyStatus);
                }
                u8ProcessRequest = (uint8_t)PROCESS_REQUEST_DISCARD;
            }

            if (((uint8_t)u8ProcessRequest != (uint8_t)PROCESS_REQUEST_DISCARD) && ((uint8_t)u8ActionRangeCheck == (uint8_t)RANGE_CHECK_FAILED))
            {
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set action range check error event: %d", s16ErrorStatus);
                }

                int8_t s8NotifyStatus = (int8_t)ITCOM_s8LogNotificationMessage(
                    stTempMsgData.stMsgPairData.u16MsgId,
                    stTempMsgData.stMsgPairData.u16SequenceNum,
                    (uint8_t)enInvalidActionReq,
                    (uint8_t)enActionNotification);
                if ((int8_t)s8NotifyStatus < 0)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to log invalid action list notification: %d", s8NotifyStatus);
                }
                u8ProcessRequest = (uint8_t)PROCESS_REQUEST_DISCARD;
            }

            if ((uint8_t)u8ProcessRequest != (uint8_t)PROCESS_REQUEST_DISCARD)
            {
                if ((precondition_id_t)stActionReqData.enPrecondId == (precondition_id_t)PreID_None)
                {
                    s8Return = ITCOM_s8QueueActionReq(&stTempMsgData);
                    if (s8Return == QUEUE_ACTION_TIMEOUT)
                    {
                        int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT);
                        if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                        {
                            log_message(global_log_file, LOG_ERROR, "Failed to set action timeout error event: %d", s16ErrorStatus);
                        }
                        log_message(global_log_file, LOG_WARNING,
                                    "Action request processing timeout for ID: 0x%04X",
                                    stTempMsgData.stMsgPairData.u16MsgId);
                    }
                }
                else if (((precondition_id_t)stActionReqData.enPrecondId == (precondition_id_t)PreID_Park) && ((uint8_t)m_u8VehicleStatus == (uint8_t)VEHICLE_PARK))
                {
                    s8Return = ITCOM_s8QueueActionReq(&stTempMsgData);
                    if (s8Return == QUEUE_ACTION_TIMEOUT)
                    {
                        int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT);
                        if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                        {
                            log_message(global_log_file, LOG_ERROR, "Failed to set action timeout error event: %d", s16ErrorStatus);
                        }
                        log_message(global_log_file, LOG_WARNING,
                                    "Action request processing timeout for ID: 0x%04X",
                                    stTempMsgData.stMsgPairData.u16MsgId);
                    }
                }
                else
                {
                    int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR);
                    if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                    {
                        log_message(global_log_file, LOG_ERROR, "Failed to set precondition list error event: %d", s16ErrorStatus);
                    }

                    int8_t s8NotifyStatus = (int8_t)ITCOM_s8LogNotificationMessage(
                        stTempMsgData.stMsgPairData.u16MsgId,
                        stTempMsgData.stMsgPairData.u16SequenceNum,
                        (uint8_t)enPreconditionFail,
                        (uint8_t)enActionNotification);
                    if ((int8_t)s8NotifyStatus < 0)
                    {
                        log_message(global_log_file, LOG_ERROR, "Failed to log precondition fail notification: %d", s8NotifyStatus);
                    }
                }
            }
        }
    }
}

/**
 * @brief Monitors and validates vehicle status for safety-critical operations
 *
 * @details
 * This function monitors critical vehicle status parameters to ensure safe operation
 * of the action request system. It specifically tracks and validates:
 * - Vehicle park status (PRNDL position)
 * - Vehicle speed
 * - Information freshness/validity
 *
 * Processing Flow:
 * ---------------
 * 1. Status Information Collection:
 *    - Retrieves current park status via ITCOM_u8GetParkStatus()
 *    - Retrieves current vehicle speed via ITCOM_u8GetVehicleSpeed()
 *    - Validates information freshness for both parameters
 *
 * 2. Information Validation:
 *    - Checks if either park status or speed information is outdated
 *    - Logs appropriate error events for outdated information
 *    - Proceeds with status evaluation only if information is current
 *
 * 3. Vehicle State Evaluation:
 *    - Verifies park status (PRNDL position)
 *    - Validates vehicle speed within error margin when in park
 *    - Sets vehicle status flag based on combined conditions
 *
 * Status Conditions:
 * -----------------
 * Vehicle is considered "in park" when:
 * - PRNDL indicates Park position (enParkStatus)
 * - Vehicle speed is within ±VEHICLE_SPEED_ERROR_MARGIN (±0.20)
 *
 * Error Events Generated:
 * ----------------------
 * - EVENT_ID_INFO_VEHICLE_STATUS_ERROR: Status information outdated
 * - EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH: Speed/Park status mismatch
 *
 * Global State Affected:
 * ---------------------
 * - m_u8VehicleStatus: Updated to reflect current vehicle state
 *    - VEHICLE_PARK: Vehicle safely parked
 *    - VEHICLE_NOT_PARK: Vehicle not in safe park condition
 *
 * @note This function should be called periodically to ensure current 
 *       vehicle status information is available for action request processing
 *
 * @warning Outdated status information or mismatched conditions will trigger
 *          error events and may prevent safety-critical actions
 *
 * @pre ITCOM communication system must be initialized and operational
 * @pre Vehicle status signals must be properly configured
 *
 * @post m_u8VehicleStatus is updated to reflect current vehicle state
 * @post Appropriate error events are logged for any detected issues
 *
 * @return void
 *
 */
void ARA_vVehicleStatusMonitor(void)
{
    stVehicleStatusInfo_t stVehicleStatus = {0U};
    PRNDL_SignalValues_t enCurrentGearStatus;

    stVehicleStatus.u8InfoStatus[0] = ITCOM_u8GetParkStatus(&stVehicleStatus.u8ParkStatus);
    stVehicleStatus.u8InfoStatus[1] = ITCOM_u8GetVehicleSpeed(&stVehicleStatus.fVehicleSpeed);

    if (((uint8_t)stVehicleStatus.u8InfoStatus[0] == (uint8_t)INFO_OUTDATED) || ((uint8_t)stVehicleStatus.u8InfoStatus[1] == (uint8_t)INFO_OUTDATED))
    {
        log_message(global_log_file, LOG_DEBUG, "Vehicle status information OUTDATED during evaluation");
        int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_VEHICLE_STATUS_ERROR);
        if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set vehicle status error event: %d", s16ErrorStatus);
        }
    }
    else
    {
        /* Converting raw status to enum type for better type safety */
        enCurrentGearStatus = (PRNDL_SignalValues_t)stVehicleStatus.u8ParkStatus;

        if ((PRNDL_SignalValues_t)enCurrentGearStatus == (PRNDL_SignalValues_t)enParkStatus)
        {
            if (((float32_t)stVehicleStatus.fVehicleSpeed >= (float32_t)(-VEHICLE_SPEED_ERROR_MARGIN)) && ((float32_t)stVehicleStatus.fVehicleSpeed <= (float32_t)VEHICLE_SPEED_ERROR_MARGIN))
            {
                m_u8VehicleStatus = (uint8_t)VEHICLE_PARK;
            }
            else
            {
                m_u8VehicleStatus = (uint8_t)VEHICLE_NOT_PARK;
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set vehicle status error event: %d", s16ErrorStatus);
                }
            }
        }
        else
        {
            m_u8VehicleStatus = (uint8_t)VEHICLE_NOT_PARK;
        }
    }
}

/**
 * @brief Validates action requests against predefined action list and retrieves preconditions
 *
 * @details
 * This function performs validation of incoming action requests by checking them against
 * a predefined list of allowed actions (m_stActionList). For valid actions, it also 
 * retrieves and assigns the associated precondition requirements.
 *
 * @param[in,out] stActionRequest Pointer to action request structure
 *                               - Input: Contains action ID to validate
 *                               - Output: Updated with precondition ID if match found
 *
 * @note This function is called during the action request processing phase
 *       to validate requests before precondition checking
 *
 * @warning
 * - Caller must ensure stActionRequest pointer is valid
 * - Function assumes m_stActionList is properly initialized
 * - Only the first matching action's preconditions are used
 *
 * @pre 
 * - m_stActionList must be initialized with valid action definitions
 * - stActionRequest must point to a valid action_request_t structure
 * - TOTAL_AR must accurately reflect the size of m_stActionList
 *
 * @post
 * - stActionRequest->enPrecondId is updated if action is found
 * - Return value indicates whether action was found in list
 *
 * @return uint8_t Status of action list check
 * @retval TEST_ON_AL (1U) Action found in predefined list
 * @retval TEST_NOT_ON_AL (0U) Action not found in list or invalid input
 *
 */
uint8_t ARA_u8ActionListCheck(action_request_t *stActionRequest)
{
    uint8_t u8Result = (uint8_t)TEST_NOT_ON_AL;
    uint8_t u8Index = 0;

    if (VALID_PTR(stActionRequest))
    {
        for (u8Index = 0; u8Index < (uint8_t)TOTAL_AR; u8Index++)
        {
            if ((uint16_t)m_stActionList[u8Index].u16ActionId == (uint16_t)stActionRequest->u16ActionId)
            {
                u8Result = (uint8_t)TEST_ON_AL;
                stActionRequest->enPrecondId = m_stActionList[u8Index].enPrecondId;
                break;
            }
            else
            {
                u8Result = (uint8_t)TEST_NOT_ON_AL;
            }
        }
    }

    return u8Result;
}

/**
 * @brief Validates action request preconditions against defined precondition list
 *
 * @details
 * This function validates whether an action request's precondition ID falls within
 * the valid range of defined preconditions in the precondition_id_t enumeration.
 * It ensures that only recognized precondition types are processed by the system.
 *
 * @param[in] stActionRequest Action request structure containing precondition to validate
 *                           Uses enPrecondId member for validation
 *
 * @note This function is typically called after ARA_u8ActionListCheck() has
 *       populated the precondition ID
 *
 * @warning
 * - Function assumes precondition_id_t enumeration remains stable
 * - Any changes to precondition_id_t must be reflected in validation logic
 * - Invalid preconditions will prevent action request processing
 *
 * @pre
 * - precondition_id_t enumeration must be properly defined
 * - stActionRequest must contain a populated enPrecondId field
 *
 * @post
 * - Return value indicates precondition validity status
 * - No modification of input parameters
 *
 * @return uint8_t Status of precondition validation
 * @retval TEST_ON_PL (1U) Precondition is valid and recognized
 * @retval TEST_NOT_ON_PL (0U) Precondition is invalid or unrecognized
 *
 */
uint8_t ARA_u8PrecondListCheck(action_request_t stActionRequest)
{
    uint8_t u8Result = (uint8_t)TEST_NOT_ON_PL; // default not on precondition list

    if (((precondition_id_t)stActionRequest.enPrecondId > (precondition_id_t)PreID_Invalid) &&
        ((precondition_id_t)stActionRequest.enPrecondId < (precondition_id_t)PreID_Total))
    {
        u8Result = (uint8_t)TEST_ON_PL;
    }

    return u8Result;
}

/*** Private Functions ***/

/**
 * @brief Validates action request parameters against predefined range limits
 *
 * @details
 * This function performs range validation for action request parameters based on
 * message length and predefined limits in m_stActionList. It supports multiple
 * data lengths and formats, ensuring parameters fall within safe operational bounds.
 *
 * @param[in] stMsgData Process message data structure containing:
 *                      - u16MsgId: Message identifier
 *                      - u16Length: Message data length
 *                      - au8MsgData: Array of message data bytes
 *
 * @note This function is called during action request processing to ensure
 *       parameters are within safe operational limits
 *
 * @warning
 * - Function assumes m_stActionList range limits are properly configured
 * - Message index must correspond to action list index after base adjustment
 * - Unmatched message lengths result in range check failure
 *
 * @pre
 * - m_stActionList must be initialized with valid range limits
 * - stMsgData must contain valid message length and data
 * - Message ID must be registered in the system
 *
 * @post
 * - Return value indicates whether all parameters are within range
 * - No modification of input parameters
 *
 * @return uint8_t Range check evaluation result
 * @retval RANGE_CHECK_PASSED (1U) All parameters within defined ranges
 * @retval RANGE_CHECK_FAILED (0U) Parameters out of range or validation error
 *
 */
static uint8_t ara_u8RangeCheckEvaluation(stProcessMsgData stMsgData)
{
    int16_t s16MsgIndx = 0;
    uint8_t u8MsgPayload = ARA_ZERO_INIT_U;
    uint16_t u16MsgPayload = ARA_ZERO_INIT_U;
    uint32_t u32MsgPayload = ARA_ZERO_INIT_U;
    uint8_t u8RangeCheckResult = RANGE_CHECK_FAILED;

    s16MsgIndx = ITCOM_s16GetMessageEnumById(stMsgData.stMsgPairData.u16MsgId);
    if ((int16_t)s16MsgIndx != (int16_t)MESSAGE_NOT_FOUND)
    {
        uint8_t i;
        s16MsgIndx = s16MsgIndx - enHVACFanSpeed;
        switch ((uint16_t)stMsgData.u16Length)
        {

        case MSG_LENGTH_ONE_BYTE:
            u8MsgPayload = stMsgData.au8MsgData[0];
            if ((uint8_t)u8MsgPayload >= (uint8_t)m_stActionList[s16MsgIndx].au32RangeLimits[0] &&
                (uint8_t)u8MsgPayload <= (uint8_t)m_stActionList[s16MsgIndx].au32RangeLimits[1])
            {
                u8RangeCheckResult = RANGE_CHECK_PASSED;
            }
            break;

        case MSG_LENGTH_TWO_BYTES:
            u16MsgPayload = (uint16_t)((stMsgData.au8MsgData[1] << ARA_SHIFT_EIGHT) | stMsgData.au8MsgData[0]);
            if ((uint16_t)u16MsgPayload >= (uint16_t)m_stActionList[s16MsgIndx].au32RangeLimits[0] &&
                (uint16_t)u16MsgPayload <= (uint16_t)m_stActionList[s16MsgIndx].au32RangeLimits[1])
            {
                u8RangeCheckResult = RANGE_CHECK_PASSED;
            }
            break;

        case MSG_LENGTH_FOUR_BYTES:
            u32MsgPayload = (uint32_t)((stMsgData.au8MsgData[MSG_DATA_INDEX_3] << ARA_SHIFT_TWENTY_FOUR) |
                                       (stMsgData.au8MsgData[MSG_DATA_INDEX_2] << ARA_SHIFT_SIXTEEN) |
                                       (stMsgData.au8MsgData[MSG_DATA_INDEX_1] << ARA_SHIFT_EIGHT) |
                                       stMsgData.au8MsgData[MSG_DATA_INDEX_0]);
            if ((uint32_t)u32MsgPayload >= (uint32_t)m_stActionList[s16MsgIndx].au32RangeLimits[0] &&
                (uint32_t)u32MsgPayload <= (uint32_t)m_stActionList[s16MsgIndx].au32RangeLimits[1])
            {
                u8RangeCheckResult = RANGE_CHECK_PASSED;
            }
            break;

        case MSG_LENGTH_EIGHT_BYTES:
            for (i = MSG_DATA_INDEX_0; i < (uint8_t)stMsgData.u16Length; i++)
            {
                if ((uint8_t)stMsgData.au8MsgData[i] >= (uint8_t)m_stActionList[s16MsgIndx].au32RangeLimits[0] &&
                    (uint8_t)stMsgData.au8MsgData[i] <= (uint8_t)m_stActionList[s16MsgIndx].au32RangeLimits[1])
                {
                    u8RangeCheckResult = RANGE_CHECK_PASSED;
                }
                else
                {
                    break;
                }
            }
            break;

        default:
            break;
        }
    }

    return u8RangeCheckResult;
}
