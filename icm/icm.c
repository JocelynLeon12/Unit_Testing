/*****************************************************************************
 * @file icm.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Interface Communication Manager Implementation
 *
 * @details
 * This file implements the core communication management system for the Sonatus
 * Automator project. It provides comprehensive message handling, validation, and
 * transmission capabilities for TCP-based communication between Vehicle Automation
 * Manager (VAM) and Communication Middleware (CM).
 *
 * Key Features:
 * - Message integrity validation with CRC checks
 * - Rolling counter management for sequence tracking
 * - Rate-limited message transmission control
 * - Cycle count tracking for timeout detection
 * - Automatic message acknowledgment handling
 * - Support for multiple communication channels (VAM/CM)
 * - Structured message logging with timestamps
 * - Thread-safe message queue management
 * - Status notification system for VAM
 * - Calibration data handling and verification
 * - Vehicle status data management
 * - Error event generation and handling
 *
 * System Components:
 * - Message Reception Handler
 * - Message Transmission Controller
 * - Message Integrity Validator
 * - Cycle Count Manager
 * - Rate Limiter
 * - Status Notification System
 * - Vehicle Data Handler
 *
 * @authors Alejandro Tollola (AT), Tusar Palauri (TP)
 * @date August 13 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/13/2024 | AT     | Initial Implementation
 * 10/03/2024 | TP     | Refactored for Action Request Timeout
 * 10/03/2024 | AT     | Thread crashing handling update
 * 11/15/2024 | TP     | MISRA & LHP compliance fixes
 * 11/22/2024 | TP     | Cleaning up the code
 */

/*** Include Files ***/
#include "itcom.h"
#include "crc.h"
#include "data_queue.h"
#include "instance_manager.h"
#include "storage_handler.h"
#include "state_machine.h"
#include "fault_manager.h"

#include "icm.h"

/*** Module Definitions ***/
#define RATE_LIMIT_MSG                      (10)
#define RATE_LIMIT_TIME_PERIOD              (100)
#define SCALE_FACTOR                        (100)
#define ASI_STATUS_MESSAGE_PERIOD           (20)
#define MESSAGE_COUNT_INIT                  (0U)
#define VEHICLE_SPEED_LOW_LIMIT             (0x000)
#define VEHICLE_SPEED_HIGH_LIMIT            (0x190)
#define FLOAT_COMPARISON_EPSILON            (0.001f)

#define MSG_STATIC_INTEGRITY_CONFIG_TABLE { \
/*	 TimeoutLimit							CycleCount_Flag			ActionReqTimer_Flag		TypeLength_Flag		CRC_Flag			RC_Flag				RSN_Flag			CyclicMsg_Flag		SeqNumAssigner		TimeoutEventID							MsgName*/	\
    /* enMessageListVAM Group */ \
    {ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},  			/* enHVACFanSpeed */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enHVACCabinTemperature */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enWindshieldWiperSpeed */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enSeatPositionDriver */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enSeatPositionPassenger */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enSeatHeaterDriver */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enSeatHeaterPassenger */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enDoorLockState */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enTurnSignalState */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enAmbientLighting */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,		    ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enTorqueVecMotorCalib */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			ACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enRainSensor */ \
	{TIMEOUT_NA,			                INACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		enTotalEventIds},			        /* enAckVAM */ \
	\
	/* enMessageListCM Group */ \
	{MSG_TIMEOUT_MAX_VALUE,					ACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_FAULT_MSG_TIMEOUT},			/* enPRNDL */ \
	{MSG_TIMEOUT_MAX_VALUE,					ACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_FAULT_MSG_TIMEOUT},			/* enVehicleSpeed */ \
	{CAL_READBACK_RESPONSE_TIME_LIMIT,		INACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_FAULT_CAL_READBACK_TIMEOUT},   /* enCalibReadback */ \
	{TIMEOUT_NA,			                INACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		enTotalEventIds},			            /* enAckCM */ \
	{TIMEOUT_NA,							INACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		enTotalEventIds},			            /* enNonCriticalFail */ \
	{TIMEOUT_NA,							INACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		enTotalEventIds},			            /* enCriticalFail */ \
	\
	/* enMessageListASI Group */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_VAM,		EVENT_ID_INFO_ACK_LOSS},			/* enActionNotification */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_ASI,		EVENT_ID_INFO_ACK_LOSS},			/* enStatusNotificationASI */ \
	{ACK_MESG_RESPONSE_TIME_LIMIT,			ACTIVE_FLAG,			INACTIVE_FLAG,			ACTIVE_FLAG,		INACTIVE_FLAG,		ACTIVE_FLAG,		INACTIVE_FLAG,		INACTIVE_FLAG,		SEQ_NUM_ASI,		EVENT_ID_INFO_ACK_LOSS}			    /* enStartUpTestNotification */ \
}

/*** Internal Types ***/

typedef struct
{
    uint8_t u8TimeoutLimit;
    uint8_t u8CycleCountFlag;
    uint8_t u8ActionReqTimerFlag;
    uint8_t u8TypeLengthFlag;
    uint8_t u8CRCFlag;
    uint8_t u8RCFlag;
    uint8_t u8RSNFlag;
    uint8_t u8CyclicMsgFlag;
    uint8_t u8SeqNumberAssigner;
    uint8_t u8TimeoutEventId;
} MsgIntConfig_t;

/*** Local Function Prototypes ***/
static void icm_vSaveVehicleStatusData(int16_t s16Indx, uint8_t *pu8Data, uint8_t u8Status);
static int8_t icm_s8CRCEval(TLVMessage_t stReceivedMsg, uint8_t u8Indx);
static void icm_vRollingCountEval(TLVMessage_t stReceivedMsg, MsgIntConfig_t stMsgConfig, int16_t s16Indx);
static void icm_vCycleCountReset(TLVMessage_t stReceivedMsg, MsgIntConfig_t stMsgConfig, int16_t s16Indx, uint8_t u8ConnectionIndex);
static void icm_vSaveMsgData(TLVMessage_t *pstReceivedMsg, int16_t s16Indx, int16_t s16TypeIndx);
static void icm_vPopulateMsgHeader(TLVMessage_t *pstTempTxMsg, stProcessMsgData stMsgData, MessageDictionary_t stDictionaryData);
static void icm_vPopulateMsgPayload(TLVMessage_t *pstTempTxMsg, stProcessMsgData stMsgData, MessageDictionary_t stDictionaryData, MsgIntConfig_t stTempMsgConfig);
static int8_t icm_s16CheckRateLimit(RateLimiter_t *pstRateLimiter);
static float32_t icm_f32FixedPointToFloat(uint16_t u16Fixed, int16_t s16ScaleFactor);
static void icm_vProcessReceivedMessage(TLVMessage_t *pstReceivedTCPMsg, uint8_t u8ConnectionIndex);
static void icm_vProcessValidMessage(TLVMessage_t *pstReceivedTCPMsg, int16_t s16Indx, int16_t s16TypeIndx, MsgIntConfig_t *pstTempMsgConfig, uint8_t u8ConnectionIndex);
static void icm_vLogReceivedMessage(TLVMessage_t *pstReceivedTCPMsg, enTCPConnectionsASI enConnection);
static enTCPConnectionsASI icm_enPrepareTransmitMessage(stProcessMsgData *pstMsgData, TLVMessage_t *pstTxMsg);
static void icm_vTrackSentMessage(stProcessMsgData *pstMsgData);
static void icm_vUpdateTransmissionCounters(TLVMessage_t *pstTxMsg, enTCPConnectionsASI enConnection);
static void icm_vLogTransmittedMessage(const TLVMessage_t *pstTxMsg, enTCPConnectionsASI enConnection);

/*** External Variables ***/

/*** Internal Variables ***/
static MsgIntConfig_t icm_stIntConfigTable[] = MSG_STATIC_INTEGRITY_CONFIG_TABLE;

/*** Functions Provided to other modules ***/

/**
 * @brief Initializes the Interface Communication Manager (ICM) module
 *
 * @details
 * This function performs the complete initialization of the ICM module including:
 *
 * 1. Rate Limiter Configuration:
 *    - Sets up message rate limiting parameters
 *    - Configures allowed messages per time window (10 messages/100ms)
 *    - Initializes message counter and timing tracking
 *
 * 2. Message Tracking Setup:
 *    - Initializes PRNDL message tracking
 *    - Sets up vehicle speed monitoring
 *    - Configures message sequence numbering
 *    - Establishes cycle count tracking for critical parameters
 *
 * 3. Dictionary Configuration:
 *    - Sets up message type dictionaries
 *    - Configures message ID mappings
 *    - Initializes message enumeration tables
 *
 * @note
 * The initialization sequence is critical for proper system operation:
 * 1. Rate limiter must be initialized first for message flow control
 * 2. PRNDL tracking setup ensures vehicle state monitoring
 * 3. Vehicle speed tracking enables motion status monitoring
 * 4. Initialization flag status affects system behavior
 *
 * @param[in] None
 *
 * @return None
 *
 * @pre
 * - System power-up complete
 * - Memory subsystems initialized
 * - Log system ready
 *
 * @post
 * - Rate limiter configured and active
 * - Message tracking systems initialized
 * - Cycle count tracking enabled
 * - System ready for message processing
 *
 *
 * @warning
 * - Must be called before any other ICM functions
 * - Multiple calls to this function may reset critical tracking data
 *
 */
void ICM_vInit(void)
{
    log_message(global_log_file, LOG_INFO, "ICM_vInit: Initializing ICM...");

    RateLimiter_t stRateLimiter = RATE_LIMITER_INIT;
    stMsgIntegrityData stMsgTracker = MSG_INTEGRITY_DATA_INIT;
    MessageDictionary_t stDictionaryData = MESSAGE_DICTIONARY_INIT;
    MessageTypeDictionary_t stDictionaryTypeData = MESSAGE_TYPE_DICTIONARY_INIT;
    uint8_t u8InitFlagStatus = ITCOM_u8GetInitFlagStatus();

    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Initializing rate limiter");
    /* Allow 10 messages every 100 ms */
    stRateLimiter.u16AllowedMessages = RATE_LIMIT_MSG;
    stRateLimiter.u16TimeWindowMs = RATE_LIMIT_TIME_PERIOD;
    stRateLimiter.u16MessageCount = MESSAGE_COUNT_INIT;
    stRateLimiter.stStartTime = clock();

    ITCOM_vSetMsgRateLimiter(&stRateLimiter);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Rate limiter set - Allowed messages: %u, Time window: %u ms", RATE_LIMIT_MSG, RATE_LIMIT_TIME_PERIOD);

    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Setting up PRNDL message tracker");
    ITCOM_vGetMsgDictionaryEntryAtIndex(&stDictionaryData, enPRNDL);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: PRNDL Dictionary Data - ID: 0x%04X, Type: %u, Enum: %u", stDictionaryData.u16MessageId, stDictionaryData.u16MessageType, stDictionaryData.u8MessageEnum);

    ITCOM_vGetMsgTypeDictionaryEntryAtIndex(&stDictionaryTypeData, stDictionaryData.u16MessageType);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: PRNDL Dictionary Type Data - Type ID: 0x%04X", stDictionaryTypeData.u16MessageTypeID);

    stMsgTracker.stMsgPairData.u16MsgId = stDictionaryData.u16MessageId;
    stMsgTracker.stMsgPairData.u16SequenceNum = ICM_SEQUENCE_NUM_INIT;
    stMsgTracker.u8EnumAssigned = enPRNDL;
    stMsgTracker.u8ClearCondition = ICM_CLEAR_CONDITION_INIT;
    stMsgTracker.u16Type = stDictionaryTypeData.u16MessageTypeID;

    ITCOM_vSetMsgCycleCount(&stMsgTracker, ADD_ELEMENT);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: PRNDL message tracker set and added to cycle count");

    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Setting up Vehicle Speed message tracker");
    ITCOM_vGetMsgDictionaryEntryAtIndex(&stDictionaryData, enVehicleSpeed);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Vehicle Speed Dictionary Data - ID: 0x%04X, Type: %u, Enum: %u", stDictionaryData.u16MessageId, stDictionaryData.u16MessageType, stDictionaryData.u8MessageEnum);

    ITCOM_vGetMsgTypeDictionaryEntryAtIndex(&stDictionaryTypeData, stDictionaryData.u16MessageType);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Vehicle Speed Dictionary Type Data - Type ID: 0x%04X", stDictionaryTypeData.u16MessageTypeID);

    stMsgTracker.stMsgPairData.u16MsgId = stDictionaryData.u16MessageId;
    stMsgTracker.stMsgPairData.u16SequenceNum = ICM_SEQUENCE_NUM_INIT;
    stMsgTracker.u8EnumAssigned = enVehicleSpeed;
    stMsgTracker.u8ClearCondition = ICM_CLEAR_CONDITION_INIT;
    stMsgTracker.u16Type = stDictionaryTypeData.u16MessageTypeID;

    ITCOM_vSetMsgCycleCount(&stMsgTracker, ADD_ELEMENT);
    log_message(global_log_file, LOG_DEBUG, "ICM_vInit: Vehicle Speed message tracker set and added to cycle count");

    u8InitFlagStatus = (u8InitFlagStatus == ACTIVE_FLAG ? u8InitFlagStatus : INACTIVE_FLAG);
    ITCOM_vSetInitFlagStatus(u8InitFlagStatus);
    log_message(global_log_file, LOG_INFO, "ICM_vInit: ICM initialization completed");
}

/**
 * @brief Updates and manages message cycle counts and timeouts
 *
 * @details
 * This function manages the cycle count tracking system and timeout monitoring for
 * all tracked messages. It performs the following operations:
 *
 * 1. Global Cycle Counter Management:
 *    - Increments the global cycle counter
 *    - Handles counter rollover at UINT16_MAX
 *    - Updates system-wide timing reference
 *
 * 2. ASI Status Monitoring:
 *    - Generates periodic ASI status messages (every 500ms/20 cycles)
 *    - Logs status notifications with current system state
 *    - Tracks notification message delivery status
 *
 * 3. Action Message Tracking:
 *    - Monitors timeout for action requests
 *    - Processes all elements in action message buffer
 *    - Updates response cycle counts for pending messages
 *
 * 4. Timeout Handling:
 *    - Generates timeout events for expired messages
 *    - Handles specific timeout conditions for:
 *      * Calibration messages
 *      * PRNDL status
 *      * Vehicle speed updates
 *    - Removes expired messages from tracking
 *
 * @note
 * Critical timing parameters:
 * - ASI status messages: Generated every 20 cycles
 * - Action request timeout: Configured per message type
 * - Vehicle status timeout: Specific to message category
 *
 * @param[in] None
 *
 * @return None
 *
 * @pre
 * - ICM_vInit() must be called
 * - Message tracking buffers initialized
 * - ASI state must be valid
 *
 * @post
 * - Global cycle counter updated
 * - Timeout events generated if needed
 * - Expired messages removed from tracking
 * - Status notifications logged
 *
 * @warning
 * - Must be called periodically to maintain system timing
 * - Modifies global tracking data structures
 * - May trigger state changes on timeout conditions
 *
 */
void ICM_vCycleCountUpdater(void)
{
    int16_t s16i = ICM_INIT_VAL_S32;
    stMsgIntegrityData stTempIntData = MSG_INTEGRITY_DATA_INIT;
    stProcessMsgData stCalibCopyData = MSG_PROCESS_DATA_INIT;
    uint16_t u16InstanceElements = ICM_INIT_VAL_U16;
    uint16_t icm_u16GnrlCycleCount = ICM_INIT_VAL_U16;
    uint8_t u8ASI_State = ITCOM_u8GetASIState();
    MessageTypeDictionary_t stDictionaryTypeData = MESSAGE_TYPE_DICTIONARY_INIT;

    icm_u16GnrlCycleCount = ITCOM_u16GetCycleCountData();
    icm_u16GnrlCycleCount = (icm_u16GnrlCycleCount + 1) % UINT16_MAX_VALUE;
    ITCOM_vSetCycleCountData(icm_u16GnrlCycleCount);

    /* Log cyclic ASI status message every 500 ms */
    if ((icm_u16GnrlCycleCount % (uint16_t)ASI_STATUS_MESSAGE_PERIOD) == 0)
    {
        int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(ICM_INIT_VAL_U16, ICM_INIT_VAL_U16, (uint8_t)u8ASI_State, (uint8_t)enStatusNotificationASI);
        if (s8NotificationStatus < 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to log notification message for ASI status: %d", s8NotificationStatus);
        }
    }

    ITCOM_vGetMsgTypeDictionaryEntryAtIndex(&stDictionaryTypeData, enActionRequest);
    u16InstanceElements = ITCOM_u16GetTrackBufferSize(enActionMsgBuffer);
    log_message(global_log_file, LOG_DEBUG, "Number of tracked elements in enActionMsgBuffer: %d", u16InstanceElements);
    for (s16i = u16InstanceElements - 1; s16i >= 0; s16i--) /* Not all messages, only iterate for the saved instances */
    {
        ITCOM_vGetCycleSeqElementAtIndex(s16i, &stTempIntData, enActionMsgBuffer);
        stTempIntData.u8ResponseCycleCount++;
        ITCOM_vSetMsgCycleCount(&stTempIntData, UPDATE_ELEMENT);

        if (stTempIntData.u8ResponseCycleCount >= (uint8_t)icm_stIntConfigTable[stTempIntData.u8EnumAssigned].u8TimeoutLimit)
        {
            int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(icm_stIntConfigTable[stTempIntData.u8EnumAssigned].u8TimeoutEventId);
            if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set TimeoutEventID for message: %d", s16ErrorStatus);
            }

            if (stTempIntData.u16Type == (uint16_t)stDictionaryTypeData.u16MessageTypeID)
            {
                int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(stTempIntData.stMsgPairData.u16MsgId, stTempIntData.stMsgPairData.u16SequenceNum, (uint8_t)enTimeoutLimit, (uint8_t)enActionNotification);
                if (s8NotificationStatus < 0)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to log notification message for Action Request timeout: %d", s8NotificationStatus);
                }
            }

            /* Remove instances from calibration copy and readback buffers */
            if ((stTempIntData.u8EnumAssigned >= enTorqueVecMotorCalib && stTempIntData.u8EnumAssigned <= enTorqueVecMotorCalib) && stTempIntData.u8ClearCondition == enCalibReadback)
            {
                stCalibCopyData.u16Type = stTempIntData.u16Type;
                stCalibCopyData.stMsgPairData.u16MsgId = stTempIntData.stMsgPairData.u16MsgId;
                stCalibCopyData.stMsgPairData.u16SequenceNum = stTempIntData.stMsgPairData.u16SequenceNum;
                ITCOM_vSetCalibDataCopy(&stCalibCopyData, REMOVE_ELEMENT);
                ITCOM_vSetCalibReadbackData(&stCalibCopyData, REMOVE_ELEMENT);
            }

            if (stTempIntData.u8EnumAssigned == enPRNDL)
            {
                uint8_t u8VehicleStatus = ICM_INIT_VAL_U8;
                stTempIntData.u8ResponseCycleCount = ICM_RESPONSE_COUNT_INIT;
                (void)ITCOM_u8GetParkStatus(&u8VehicleStatus);
                ITCOM_vSetParkStatus(u8VehicleStatus, INFO_OUTDATED);
                ITCOM_vSetMsgCycleCount(&stTempIntData, UPDATE_ELEMENT);
            }
            else if (stTempIntData.u8EnumAssigned == enVehicleSpeed)
            {
                float32_t f32VehicleSpeed = ICM_INIT_VAL_F32;
                stTempIntData.u8ResponseCycleCount = ICM_RESPONSE_COUNT_INIT;
                (void)ITCOM_u8GetVehicleSpeed(&f32VehicleSpeed);
                ITCOM_vSetVehicleSpeed(f32VehicleSpeed, INFO_OUTDATED);
                ITCOM_vSetMsgCycleCount(&stTempIntData, UPDATE_ELEMENT);
            }
            else
            {
                ITCOM_vSetMsgCycleCount(&stTempIntData, REMOVE_ELEMENT);
                log_message(global_log_file, LOG_DEBUG, "MESSAGE TRACKING REACHED LIMIT, MSG: 0x%04X, SEQ NUM: 0x%04X", stTempIntData.stMsgPairData.u16MsgId, stTempIntData.stMsgPairData.u16SequenceNum);
            }
        }
    }
}

/**
 * @brief Receives and processes incoming TCP messages from multiple connections
 *
 * @details
 * This function handles the reception and processing of messages from both VAM
 * (Vehicle Automation Module) and CM (Control Module) TCP connections. It performs
 * comprehensive message validation and processing:
 *
 * 1. Connection State Management:
 *    - Checks connection availability for each server
 *    - Validates connection configurations
 *    - Handles connection state transitions
 *    - Manages safe state restrictions on message reception
 *
 * 2. Message Reception Process:
 *    - Non-blocking message reception (MSG_DONTWAIT)
 *    - Connection-specific message handling
 *    - Buffer overflow protection
 *    - Error condition handling
 *
 * 3. Message Validation:
 *    - Type-length validation
 *    - CRC verification
 *    - Message ID validation
 *    - Rolling counter verification
 *
 * 4. Action Request Handling:
 *    - Timestamp recording for action requests
 *    - Request validation and processing
 *    - Response tracking initialization
 *
 * @note
 * Connection Priority:
 * - In safe state, only CM messages are processed
 * - VAM messages are processed only in normal operation
 * - Connection state affects message reception
 *
 * @param[in] None
 *
 * @return None
 *
 * @pre
 * - ICM_vInit() must be called
 * - TCP connections must be initialized
 * - System state must be valid
 *
 * @post
 * - Received messages processed
 * - Connection states updated
 * - Action requests tracked
 * - Error events generated if needed
 *
 * @warning
 * - Uses non-blocking socket operations
 * - May modify connection states
 * - Critical for system communication integrity
 * - Must handle connection losses gracefully
 *
 * Connection States Handled:
 * - CONNECTION_STATE_CONNECTED: Normal operation
 * - CONNECTION_STATE_DISCONNECTED: Connection lost
 * - CONNECTION_STATE_ERROR: Error condition
 *
 * System States:
 * - STATE_SAFE_STATE: Limited to CM messages
 * - STATE_NORM_OP: All messages processed
 * - STATE_STARTUP_TEST: System initialization
 */
void ICM_vReceiveMessage(void)
{
    enTCPConnectionsASI enConnection = (enTCPConnectionsASI)ICM_INIT_VAL_S32;
    TLVMessage_t stReceivedTCPMsg = MSG_TLV_DATA_INIT;
    uint8_t u8ASI_State = ITCOM_u8GetASIState();
    uint8_t u8AvailableConnections = ICM_INIT_VAL_U8;
    uint8_t u8ValidConfigurations = ICM_INIT_VAL_U8;

    for (enConnection = (enTCPConnectionsASI)0; enConnection < (enTCPConnectionsASI)enTotalTCPConnections; enConnection++)
    {
        /* Skip connections other than VAM if in safe state */
        if ((u8ASI_State == (uint8_t)STATE_SAFE_STATE) && (enConnection != (enTCPConnectionsASI)enCMConnectionTCP))
        {
            log_message(global_log_file, LOG_INFO, "System in safe state. Skipping all message receptions besides CM.");
            continue;
        }

        TCPConnectionState_t enConnectionState = ITCOM_enGetTCPConnectionState(enConnection);
        if (enConnectionState != (TCPConnectionState_t)CONNECTION_STATE_CONNECTED)
        {
            log_message(global_log_file, LOG_DEBUG, "Connection %s is not available for message receiving",
                        enConnection == (enTCPConnectionsASI)enVAMConnectionTCP ? "VAM" : "CM");
            continue;
        }
        u8AvailableConnections++;

        const TCPConnectionConfig_t *config = SD_GetTCPConnectionConfig(enConnection);
        if (!config)
        {
            log_message(global_log_file, LOG_ERROR, "Invalid connection configuration for %s",
                        enConnection == (enTCPConnectionsASI)enVAMConnectionTCP ? "VAM" : "CM");
            continue;
        }
        u8ValidConfigurations++;

        ssize_t recv_result = recv(config->s16Socket, &stReceivedTCPMsg, sizeof(stReceivedTCPMsg), MSG_DONTWAIT);

        if (recv_result > 0)
        {
            MessageTypeDictionary_t stActionReqDict = MESSAGE_TYPE_DICTIONARY_INIT;
            ITCOM_vGetMsgTypeDictionaryEntryAtIndex(&stActionReqDict, enActionRequest);
            if (stReceivedTCPMsg.u16Type == (uint16_t)stActionReqDict.u16MessageTypeID)
            {
                ITCOM_vSetActionRequestStartTime(stReceivedTCPMsg.u16ID, stReceivedTCPMsg.u16SequenceNumber);
            }

            icm_vProcessReceivedMessage(&stReceivedTCPMsg, enConnection);
            ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_CONNECTED);
        }
        else if (recv_result == 0)
        {
            log_message(global_log_file, LOG_WARNING, "Connection closed by %s server",
                        enConnection == (enTCPConnectionsASI)enVAMConnectionTCP ? "VAM" : "CM");
            SD_vCloseTCPConnection(enConnection);
            ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_DISCONNECTED);
        }
        else
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                log_message(global_log_file, LOG_WARNING, "No data available from %s server",
                            enConnection == (enTCPConnectionsASI)enVAMConnectionTCP ? "VAM" : "CM");
            }
            else
            {
                error_string_t error_str = strerror(errno);
                log_message(global_log_file, LOG_ERROR, "Receive failed from %s server: %s",
                            enConnection == (enTCPConnectionsASI)enVAMConnectionTCP ? "VAM" : "CM", error_str);
                SD_vCloseTCPConnection(enConnection);
                ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_ERROR);
            }
        }
    }

    if (u8AvailableConnections == 0)
    {
        log_message(global_log_file, LOG_WARNING, "No connections available for message receiving. Check network status.");
    }
    else if (u8ValidConfigurations == 0)
    {
        log_message(global_log_file, LOG_ERROR, "All connection configurations are invalid. System may need to be reinitialized.");
    }
    else
    {
        /* Intentionally empty else block */
    }
}

/**
 * @brief Transmits messages based on system state and rate limiting constraints
 *
 * @details
 * This function manages the transmission of messages across TCP connections with
 * comprehensive state handling, rate limiting, and error management. The function
 * operates differently based on system state and handles various transmission scenarios:
 *
 * 1. Message Queue Selection:
 *    - Normal/Startup State: Dequeues from approved actions queue
 *    - Safe State: Dequeues from safe state queue
 *    - Handles queue-specific processing requirements
 *
 * 2. Transmission Process:
 *    - Message preparation and validation
 *    - Rate limit checking
 *    - Connection state verification
 *    - Actual transmission with error handling
 *    - Transmission counter updates
 *
 * 3. Rate Limiting:
 *    - Enforces message transmission rate limits
 *    - Handles rate limit violations with notifications
 *    - Updates rate limiting counters and windows
 *
 * 4. Error Handling:
 *    - Connection state errors
 *    - Transmission failures
 *    - Rate limit violations
 *    - Invalid configurations
 *
 * 5. Notification Management:
 *    - Generates action notifications for VAM
 *    - Handles transmission status notifications
 *    - Logs transmission events and status
 *
 * @note
 * Critical Operations:
 * - Message tracking must be maintained for acknowledgments
 * - Rate limiting is essential for system stability
 * - Connection state must be validated before transmission
 *
 * @param[in] None
 *
 * @return None
 *
 * @pre
 * - ICM_vInit() must be called
 * - TCP connections must be initialized
 * - Message queues must be properly maintained
 * - System state must be valid
 *
 * @post
 * - Message transmitted if conditions met
 * - Rate limiter updated
 * - Transmission counters updated
 * - Notifications generated as needed
 * - Message tracking updated
 *
 * @warning
 * - Function modifies global transmission counters
 * - May trigger state changes on transmission failure
 * - Critical for system communication integrity
 * - Rate limiting affects all transmissions
 *
 */
void ICM_vTransmitMessage(void)
{
    log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Entry into ICM_vTransmitMessage");
    uint8_t u8ASI_State = ITCOM_u8GetASIState();
    int8_t s8DequeueState = ICM_INIT_VAL_U8;
    stProcessMsgData stMsgData = MSG_PROCESS_DATA_INIT;
    TLVMessage_t stTxMsg = MSG_TLV_DATA_INIT;
    RateLimiter_t stRateLimiter = RATE_LIMITER_INIT;

    /* Dequeue appropriate message based on system state */
    if (u8ASI_State == (uint8_t)STATE_NORM_OP || u8ASI_State == (uint8_t)STATE_STARTUP_TEST)
    {
        log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Dequeuing approved actions message");
        s8DequeueState = ITCOM_s8DequeueActionReq(&stMsgData, APPROVED_ACTIONS_QUEUE);
    }
    else if (u8ASI_State == (uint8_t)STATE_SAFE_STATE)
    {
        log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Dequeuing safe state message");
        s8DequeueState = ITCOM_s8DequeueActionReq(&stMsgData, SAFE_STATE_QUEUE);
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "ICM_vTransmitMessage: Unknown system state. Skipping message transmission.");
        return;
    }

    if (s8DequeueState < 0)
    {
        log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: No messages to transmit. s8DequeueState = %d", s8DequeueState);
        return;
    }

    /* Prepare the message for transmission */
    enTCPConnectionsASI enConnection = icm_enPrepareTransmitMessage(&stMsgData, &stTxMsg);
    log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Message prepared for transmission");
    /* Check connection state and configuration */
    TCPConnectionState_t enConnectionState = ITCOM_enGetTCPConnectionState(enConnection);
    if (enConnectionState != CONNECTION_STATE_CONNECTED)
    {
        log_message(global_log_file, LOG_WARNING, "ICM_vTransmitMessage: Connection %s is not available for message transmission",
                    enConnection == enVAMConnectionTCP ? "VAM" : "CM");
        return;
    }

    const TCPConnectionConfig_t *config = SD_GetTCPConnectionConfig(enConnection);
    if (!config)
    {
        log_message(global_log_file, LOG_ERROR, "ICM_vTransmitMessage: Invalid connection configuration for %s",
                    enConnection == enVAMConnectionTCP ? "VAM" : "CM");
        return;
    }

    /* Check rate limiter */
    ITCOM_vGetMsgRateLimiter(&stRateLimiter);
    if (icm_s16CheckRateLimit(&stRateLimiter) != E_OK)
    {
        ITCOM_vSetMsgRateLimiter(&stRateLimiter);
        /* Action Notification message for VAM */
        if (enConnection == enCMConnectionTCP)
        {
            int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(stTxMsg.u16ID, stTxMsg.u16SequenceNumber, (uint8_t)enRateLimiterDrop, (uint8_t)enActionNotification);
            if (s8NotificationStatus < 0)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to log notification message for VAM: %d", s8NotificationStatus);
            }
        }
        log_message(global_log_file, LOG_WARNING, "ICM_vTransmitMessage: Rate limit exceeded, Message not sent");
        return;
    }

    /* Attempt to send the message */
    if (send(config->s16Socket, &stTxMsg, sizeof(stTxMsg), 0) >= 0)
    {
        log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Message sent successfully");
        icm_vLogTransmittedMessage(&stTxMsg, enConnection);
        stMsgData.stMsgPairData.u16SequenceNum = stTxMsg.u16SequenceNumber;
        icm_vTrackSentMessage(&stMsgData);
        icm_vUpdateTransmissionCounters(&stTxMsg, enConnection);
        ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_CONNECTED);

        /* Action Notification message for VAM */
        if (enConnection == enCMConnectionTCP)
        {
            int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(stMsgData.stMsgPairData.u16MsgId, stMsgData.stMsgPairData.u16SequenceNum, (uint8_t)enApprovedRequest, (uint8_t)enActionNotification);
            if (s8NotificationStatus < 0)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to log notification message for VAM: %d", s8NotificationStatus);
            }
            log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Action Notification message sent");
        }
    }
    else
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "ICM_vTransmitMessage: Failed to send message: %s", error_str);
        SD_vCloseTCPConnection(enConnection);
        ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_ERROR);

        /* Action Notification message for VAM */
        if (enConnection == enCMConnectionTCP)
        {
            int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(stMsgData.stMsgPairData.u16MsgId,
                                                                         stMsgData.stMsgPairData.u16SequenceNum,
                                                                         (uint8_t)enTransmissionFailed,
                                                                         (uint8_t)enActionNotification);
            if (s8NotificationStatus < QUEUE_ACTION_SUCCESS)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to log notification message for VAM: %d", s8NotificationStatus);
            }
        }
    }

    log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Exit from ICM_vTransmitMessage");
}

/*** Local Function Implementations ***/

/**
 * @brief Saves and validates vehicle status data for PRNDL and vehicle speed
 *
 * @details
 * This internal function processes and stores vehicle status information, handling
 * two primary types of data:
 * 1. PRNDL (Park/Reverse/Neutral/Drive/Low) status
 * 2. Vehicle speed data
 *
 * The function performs the following operations:
 * - Data validation
 * - Range checking
 * - Status updates
 * - Error event generation for invalid data
 *
 * Processing Steps:
 * 1. For PRNDL Status:
 *    - Validates gear status range
 *    - Updates system park status if valid
 *    - Generates error event if invalid
 *
 * 2. For Vehicle Speed:
 *    - Converts raw speed data to floating point
 *    - Validates speed range (0x000 to 0x190)
 *    - Updates speed status if valid
 *    - Generates error event if invalid
 *
 * @param[in] s16Indx     Message index identifying data type (enPRNDL or enVehicleSpeed)
 * @param[in] pu8Data     Pointer to raw data buffer containing status information
 * @param[in] u8Status    Status flag indicating data state (INFO_UPDATED/INFO_OUTDATED)
 *
 * @return None
 *
 */
static void icm_vSaveVehicleStatusData(int16_t s16Indx, uint8_t *pu8Data, uint8_t u8Status)
{
    float32_t f32TempSpeedData = ICM_INIT_VAL_F32;
    uint16_t u16TempRawSpeed = ICM_INIT_VAL_U16;
    uint8_t u8VehicleGearStatus = pu8Data[0];
    if (s16Indx == enPRNDL)
    {
        if (u8VehicleGearStatus < (uint8_t)enTotalVehicleStatus)
        {
            ITCOM_vSetParkStatus(pu8Data[0], u8Status);
            log_message(global_log_file, LOG_DEBUG, "Vehicle PRNDL Updated");
        }
        else
        {
            int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR);
            if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set vehicle status invalid data error event: %d", s16ErrorStatus);
            }
        }
    }
    else if (s16Indx == enVehicleSpeed)
    {
        u16TempRawSpeed = (uint16_t)((pu8Data[1] << ICM_BYTE_SHIFT_8) | pu8Data[0]);
        f32TempSpeedData = icm_f32FixedPointToFloat(u16TempRawSpeed, SCALE_FACTOR);
        if ((f32TempSpeedData >= ((float32_t)VEHICLE_SPEED_LOW_LIMIT - FLOAT_COMPARISON_EPSILON)) &&
            (f32TempSpeedData <= ((float32_t)VEHICLE_SPEED_HIGH_LIMIT + FLOAT_COMPARISON_EPSILON)))
        {
            ITCOM_vSetVehicleSpeed(f32TempSpeedData, u8Status);
            log_message(global_log_file, LOG_DEBUG, "Vehicle Speed Updated");
        }
        else
        {
            int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR);
            if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set vehicle status invalid data error event: %d", s16ErrorStatus);
            }
        }
    }
    else
    {
        /* Intentionally empty else block */
    }
}

/**
 * @brief Evaluates CRC integrity of received messages
 *
 * @details
 * This internal function validates the CRC of received messages to ensure data integrity.
 * The function performs the following operations:
 * - Calculates CRC for received message
 * - Compares calculated CRC with received CRC
 * - Manages CRC error counting
 * - Triggers error events on repeated failures
 *
 * @param[in] stReceivedMsg    TLV message structure containing received message
 * @param[in] u8Indx          Message index for error tracking
 *
 * @return int8_t  E_OK on CRC match, E_NOT_OK on CRC mismatch
 */
static int8_t icm_s8CRCEval(TLVMessage_t stReceivedMsg, uint8_t u8Indx)
{
    int8_t s8CrcEvalResult = E_NOT_OK;
    uint8_t u8SizeCrc = ICM_INIT_VAL_U8;
    uint16_t u16CalcCrc = ICM_INIT_VAL_U16;

    u8SizeCrc = sizeof(stReceivedMsg.u16SequenceNumber) + sizeof(stReceivedMsg.u16ID) + sizeof(stReceivedMsg.au8Value);
    u16CalcCrc = CRC_u16CalculateCrc((uint8_t *)&stReceivedMsg.u16SequenceNumber, u8SizeCrc);

    if (u16CalcCrc == stReceivedMsg.u16CRC)
    {
        s8CrcEvalResult = E_OK;
        ITCOM_vSetCrcErrorCount(u8Indx, ICM_INIT_VAL_U8);
    }
    else
    {
        uint8_t u8CrcErrorCount = ITCOM_u8GetCrcErrorCount(u8Indx);
        u8CrcErrorCount++;
        if (u8CrcErrorCount >= CRC_ERROR_MAX_VALUE)
        {
            u8CrcErrorCount = 0U;
            int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_MSG_CRC_CHECK);
            if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set CRC error event: %d", s16ErrorStatus);
            }
        }
        ITCOM_vSetCrcErrorCount(u8Indx, u8CrcErrorCount);
        s8CrcEvalResult = E_NOT_OK;
        log_message(global_log_file, LOG_DEBUG, "CRC eval failed, calculated CRC: 0x%04X", u16CalcCrc);
    }

    return s8CrcEvalResult;
}

/**
 * @brief Evaluates rolling counter for message sequence validation
 *
 * @details
 * This internal function validates message sequencing using rolling counters.
 * The function performs:
 * - Rolling counter delta calculation
 * - Sequence validation
 * - Error tracking for sequence violations
 *
 * @param[in] stReceivedMsg    TLV message containing rolling counter
 * @param[in] stMsgConfig     Message configuration structure
 * @param[in] s16Indx         Message index for tracking
 *
 * @return None
 */
static void icm_vRollingCountEval(TLVMessage_t stReceivedMsg, MsgIntConfig_t stMsgConfig, int16_t s16Indx)
{
    if (stMsgConfig.u8RCFlag == ACTIVE_FLAG)
    {
        uint16_t u16PrevRollingCount = ICM_INIT_VAL_U16;
        int32_t s32Delta = ICM_INIT_VAL_S32;
        u16PrevRollingCount = ITCOM_u16GetRCData(s16Indx, ROLLING_COUNT_RX);
        s32Delta = (int32_t)(stReceivedMsg.u16RollingCounter - u16PrevRollingCount);

        if (s32Delta >= 1 && s32Delta <= 3)
        {
            ITCOM_vSetRollingCountError((uint8_t)s16Indx, 0);
        }
        else
        {
            uint8_t u8RollingCountError = ITCOM_u8GetRollingCountError((uint8_t)s16Indx);
            u8RollingCountError++;
            ITCOM_vSetRollingCountError((uint8_t)s16Indx, u8RollingCountError);
            if (u8RollingCountError >= ROLLINC_COUNTER_ERROR_LIMIT)
            {
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_ROLL_COUNT);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set CRC Check Error event: %d", s16ErrorStatus);
                }
            }
        }
    }
}

/**
 * @brief Resets cycle count tracking for received messages
 *
 * @details
 * This internal function manages cycle count tracking for different message types.
 * The function handles:
 * - Cycle count reset for new messages
 * - Special handling for different message types
 * - Tracking data cleanup
 *
 * @param[in] stReceivedMsg    TLV message structure
 * @param[in] stMsgConfig     Message configuration data
 * @param[in] s16Indx         Message index
 * @param[in] u8ConnectionIndex Connection identifier
 *
 * @return None
 */
static void icm_vCycleCountReset(TLVMessage_t stReceivedMsg, MsgIntConfig_t stMsgConfig, int16_t s16Indx, uint8_t u8ConnectionIndex)
{
    stMsgIntegrityData stMsgTracker = MSG_INTEGRITY_DATA_INIT;
    int16_t s16MsgEnum = ITCOM_s16GetMessageEnumById(stReceivedMsg.u16ID);

    stMsgTracker.stMsgPairData.u16MsgId = stReceivedMsg.u16ID;
    stMsgTracker.u16Type = stReceivedMsg.u16Type;
    stMsgTracker.u8EnumAssigned = (uint8_t)s16MsgEnum;

    if (u8ConnectionIndex == enVAMConnectionTCP && s16Indx == enAckVAM)
    {
        stMsgTracker.u8ClearCondition = enAckVAM;
    }
    else if (u8ConnectionIndex == enCMConnectionTCP && s16Indx == enCalibReadback)
    {
        stMsgTracker.u8ClearCondition = enCalibReadback;
    }
    else if (u8ConnectionIndex == enCMConnectionTCP && s16Indx == enAckCM)
    {
        stMsgTracker.u8ClearCondition = enAckCM;
    }
    else
    {
        stMsgTracker.u8ClearCondition = ICM_INIT_VAL_U8;
    }

    if (stMsgConfig.u8CyclicMsgFlag == ACTIVE_FLAG)
    {
        stMsgTracker.stMsgPairData.u16SequenceNum = ICM_SEQUENCE_NUM_INIT;
        stMsgTracker.u8ResponseCycleCount = ICM_RESPONSE_COUNT_INIT;
        stMsgTracker.u8ClearCondition = ICM_CLEAR_CONDITION_INIT;
        ITCOM_vSetMsgCycleCount(&stMsgTracker, UPDATE_ELEMENT);
    }
    else
    {
        stMsgTracker.stMsgPairData.u16SequenceNum = stReceivedMsg.u16SequenceNumber;
        ITCOM_vSetMsgCycleCount(&stMsgTracker, REMOVE_ELEMENT);
    }
}

/**
 * @brief Saves received message data based on message type
 *
 * @details
 * This internal function processes and stores received message data.
 * The function handles:
 * - Message type-specific processing
 * - Data storage and validation
 * - Response tracking
 * - Acknowledgment handling
 *
 * @param[in] pstReceivedMsg  Pointer to received TLV message
 * @param[in] s16Indx         Message index for processing
 * @param[in] s16TypeIndx     Message type index
 *
 * @return None
 */
static void icm_vSaveMsgData(TLVMessage_t *pstReceivedMsg, int16_t s16Indx, int16_t s16TypeIndx)
{
    int8_t s8SaveMsgStatus = ICM_INIT_VAL_S32;
    stProcessMsgData stMsgDataTracker = {ICM_INIT_VAL_U16};
    generic_ptr_t memory_operation_result = NULL;

    /* Validate input parameters */
    if (pstReceivedMsg == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "icm_vSaveMsgData: Received NULL message pointer");
        return;
    }

    /* Initialize message tracker with received data */
    stMsgDataTracker.stMsgPairData.u16MsgId = pstReceivedMsg->u16ID;
    stMsgDataTracker.stMsgPairData.u16SequenceNum = pstReceivedMsg->u16SequenceNumber;
    stMsgDataTracker.u16Type = pstReceivedMsg->u16Type;
    stMsgDataTracker.u16Length = pstReceivedMsg->u16Length;

    /* Perform memory copy with error handling */
    memory_operation_result = memcpy(stMsgDataTracker.au8MsgData, pstReceivedMsg->au8Value, sizeof(stMsgDataTracker.au8MsgData));
    if (memory_operation_result != stMsgDataTracker.au8MsgData)
    {
        log_message(global_log_file, LOG_ERROR, "Memory copy failed in message data population for MsgID: 0x%04X", pstReceivedMsg->u16ID);
        return;
    }

    /* Record rolling counter */
    ITCOM_vRecordRC((uint8_t)s16Indx, pstReceivedMsg->u16RollingCounter, ROLLING_COUNT_RX);

    /* Process message based on type */
    uint8_t u8RollingCountError = ITCOM_u8GetRollingCountError((uint8_t)s16Indx);
    if (u8RollingCountError < ROLLINC_COUNTER_ERROR_LIMIT)
    {
        switch (s16TypeIndx)
        {
        case enActionRequest:
            s8SaveMsgStatus = ITCOM_s8SaveMsgData(&stMsgDataTracker, s16Indx);
            if (s8SaveMsgStatus < 0)
            {
                log_message(global_log_file, LOG_DEBUG, "Action Request NOT Saved");
            }
            else
            {
                log_message(global_log_file, LOG_DEBUG, "Action Request Saved");
            }
            break;

        case enStatusMessageCM:
            icm_vSaveVehicleStatusData(s16Indx, pstReceivedMsg->au8Value, INFO_UPDATED);
            break;

        case enAckMessage:
            if (pstReceivedMsg->au8Value[0] == (uint8_t)ACK_UNSUCCESFUL)
            {
                (void)ITCOM_s16SetErrorEvent(EVENT_ID_INFO_ACK_UNSUCCESS);
            }
            log_message(global_log_file, LOG_INFO, "ACK message response to %d received.", s16Indx);
            break;

        case enCalibReadbackMessage:
            ITCOM_vSetCalibReadbackData(&stMsgDataTracker, ADD_ELEMENT);
            log_message(global_log_file, LOG_INFO, "Calibration Readback message response to %d received.", s16Indx);
            break;

        default:
            break;
        }
    }
}

/**
 * @brief Populates the header section of a TLV message
 *
 * @details
 * This internal function fills the header fields of a TLV message structure.
 * The function handles:
 * - Type and length assignment
 * - CRC calculation
 * - Rolling counter management
 * - Timestamp generation
 *
 * @param[out] pstTempTxMsg      Pointer to TLV message to populate
 * @param[in]  stMsgData         Message data structure
 * @param[in]  stDictionaryData  Message dictionary data
 *
 * @return None
 */
static void icm_vPopulateMsgHeader(TLVMessage_t *pstTempTxMsg, stProcessMsgData stMsgData, MessageDictionary_t stDictionaryData)
{

    uint8_t u8SizeCrc = ICM_INIT_VAL_U8;
    pstTempTxMsg->u16Type = stMsgData.u16Type;
    pstTempTxMsg->u16Length = stMsgData.u16Length;

    /*Calculate CRC*/
    u8SizeCrc = sizeof(stMsgData.stMsgPairData) + sizeof(stMsgData.au8MsgData);
    pstTempTxMsg->u16CRC = CRC_u16CalculateCrc((uint8_t *)&stMsgData.stMsgPairData, u8SizeCrc);

    /*Get rolling counter data*/
    pstTempTxMsg->u16RollingCounter = ITCOM_u16GetRCData(stDictionaryData.u8MessageEnum, ROLLING_COUNT_TX);

    /*Populate timestamp*/
    time_t stTimeNow = time(NULL);
    pstTempTxMsg->u32TimeStamp = (uint32_t)stTimeNow; // Store as seconds since the Epoch
}

/**
 * @brief Populates the payload section of a TLV message
 *
 * @details
 * This internal function fills the payload section of a TLV message.
 * The function handles:
 * - Sequence number assignment
 * - Message ID validation
 * - Data copying and validation
 * - Buffer overflow protection
 *
 * @param[out] pstTempTxMsg      Pointer to TLV message
 * @param[in]  stMsgData         Message data structure
 * @param[in]  stDictionaryData  Dictionary data
 * @param[in]  stTempMsgConfig   Message configuration
 *
 * @return None
 */
static void icm_vPopulateMsgPayload(TLVMessage_t *pstTempTxMsg, stProcessMsgData stMsgData, MessageDictionary_t stDictionaryData, MsgIntConfig_t stTempMsgConfig)
{
    /* Input parameter validation */
    if (pstTempTxMsg == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: NULL TLV message pointer");
        return;
    }

    /* Validate message data size */
    if (stMsgData.u16Length > sizeof(pstTempTxMsg->au8Value))
    {
        log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: Message data size exceeds buffer capacity");
        return;
    }

    /* Get and validate Sequence Number */
    if (stTempMsgConfig.u8SeqNumberAssigner == SEQ_NUM_ASI)
    {
        uint16_t u16SeqNum = ITCOM_u16GetSeqNumASIRecord(stDictionaryData.u8MessageEnum);
        if (u16SeqNum == UINT16_MAX)
        {
            log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: Failed to get ASI sequence number for enum %d",
                        stDictionaryData.u8MessageEnum);
            return;
        }
        pstTempTxMsg->u16SequenceNumber = u16SeqNum;
    }
    else if (stTempMsgConfig.u8SeqNumberAssigner == SEQ_NUM_VAM)
    {
        pstTempTxMsg->u16SequenceNumber = stMsgData.stMsgPairData.u16SequenceNum;
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "icm_vPopulateMsgPayload: Invalid sequence number assigner: %d",
                    stTempMsgConfig.u8SeqNumberAssigner);
        return;
    }

    /* Validate Message ID */
    if (stMsgData.stMsgPairData.u16MsgId == 0)
    {
        log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: Invalid message ID");
        return;
    }
    pstTempTxMsg->u16ID = stMsgData.stMsgPairData.u16MsgId;

    /* Validate message length before copying */
    if (stMsgData.u16Length == 0)
    {
        log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: Zero message data length");
        return;
    }

    /* Copy message payload with validation */
    generic_ptr_t memory_operation_result = memcpy(pstTempTxMsg->au8Value,
                                                   stMsgData.au8MsgData,
                                                   stMsgData.u16Length);

    if (memory_operation_result != pstTempTxMsg->au8Value)
    {
        log_message(global_log_file, LOG_ERROR, "icm_vPopulateMsgPayload: Memory copy failed for message ID: 0x%04X",
                    stMsgData.stMsgPairData.u16MsgId);
        /* Clear the destination buffer in case of partial copy */
        memory_operation_result = memset(pstTempTxMsg->au8Value, 0, sizeof(pstTempTxMsg->au8Value));
        if (memory_operation_result != pstTempTxMsg->au8Value)
        {
            log_message(global_log_file, LOG_ERROR, "Memory initialization failed");
        }
        return;
    }

    /* Clear any remaining bytes in the destination buffer if message is shorter than buffer */
    if (stMsgData.u16Length < sizeof(pstTempTxMsg->au8Value))
    {
        memory_operation_result = memset(pstTempTxMsg->au8Value + stMsgData.u16Length,
                                         0,
                                         sizeof(pstTempTxMsg->au8Value) - stMsgData.u16Length);
        if (memory_operation_result != (pstTempTxMsg->au8Value + stMsgData.u16Length))
        {
            log_message(global_log_file, LOG_ERROR, "Buffer clearing failed");
            return;
        }
    }

    log_message(global_log_file, LOG_DEBUG, "icm_vPopulateMsgPayload: Successfully populated message payload - ID: 0x%04X, SeqNum: %d",
                pstTempTxMsg->u16ID, pstTempTxMsg->u16SequenceNumber);
}

/**
 * @brief Checks if message transmission is within rate limits
 *
 * @details
 * This internal function validates message transmission rate.
 * The function handles:
 * - Message count tracking
 * - Time window management
 * - Rate limit enforcement
 * - Counter reset logic
 *
 * @param[in,out] pstRateLimiter Pointer to rate limiter structure
 *
 * @return int8_t RATE_LIMIT_EXCEEDED or E_OK
 */
static int8_t icm_s16CheckRateLimit(RateLimiter_t *pstRateLimiter)
{
    int8_t s8Result = RATE_LIMIT_EXCEEDED;
    clock_t stCurrentTime = clock();
    float64_t f64ElapsedTimeMs = (float64_t)(stCurrentTime - pstRateLimiter->stStartTime) / CLOCKS_PER_SEC * ICM_TIME_FACTOR_MS;

    if (f64ElapsedTimeMs >= pstRateLimiter->u16TimeWindowMs)
    {
        pstRateLimiter->u16MessageCount = ICM_MSG_COUNT_INIT;
        pstRateLimiter->stStartTime = stCurrentTime;
    }

    if (pstRateLimiter->u16MessageCount < pstRateLimiter->u16AllowedMessages)
    {
        pstRateLimiter->u16MessageCount++;
        s8Result = E_OK;
    }

    return s8Result;
}

/**
 * @brief Converts fixed-point value to floating-point
 *
 * @details
 * This internal function performs fixed to float conversion.
 * The function handles:
 * - Fixed-point interpretation
 * - Scale factor application
 * - Floating-point conversion
 *
 * @param[in] u16Fixed        Fixed-point value
 * @param[in] s16ScaleFactor  Scale factor for conversion
 *
 * @return float32_t Converted floating-point value
 */
static float32_t icm_f32FixedPointToFloat(uint16_t u16Fixed, int16_t s16ScaleFactor)
{
    return (float32_t)u16Fixed / s16ScaleFactor;
}

/**
 * @brief Processes received TCP messages with validation
 *
 * @details
 * This internal function handles received message processing.
 * The function handles:
 * - Message validation
 * - CRC verification
 * - Type checking
 * - Error notification
 *
 * @param[in] pstReceivedTCPMsg   Pointer to received message
 * @param[in] u8ConnectionIndex   Connection identifier
 *
 * @return None
 */
static void icm_vProcessReceivedMessage(TLVMessage_t *pstReceivedTCPMsg, uint8_t u8ConnectionIndex)
{
    int8_t s8Eval = E_OK;
    int16_t s16Indx = ITCOM_s16GetMessageEnumFromTypeAndId(pstReceivedTCPMsg->u16Type, pstReceivedTCPMsg->u16ID, (enTCPConnectionsASI)u8ConnectionIndex);
    int16_t s16TypeIndx = ITCOM_s16GetMessageTypeEnum(pstReceivedTCPMsg->u16Type);

    icm_vLogReceivedMessage(pstReceivedTCPMsg, (enTCPConnectionsASI)u8ConnectionIndex);

    s8Eval = (ITCOM_s8ValidateMessageTypeLength(pstReceivedTCPMsg->u16Type, pstReceivedTCPMsg->u16Length) == E_OK) ? s8Eval : E_NOT_OK;
    s8Eval = (icm_s8CRCEval(*pstReceivedTCPMsg, s16Indx) == E_OK) ? s8Eval : E_NOT_OK;

    if (s8Eval == E_OK)
    {
        if (s16Indx != MESSAGE_NOT_FOUND && s16Indx >= 0)
        {
            MsgIntConfig_t stTempMsgConfig = icm_stIntConfigTable[s16Indx];
            ITCOM_vSetCrcErrorCount((uint8_t)s16Indx, 0);

            switch (s16Indx)
            {
            case enCriticalFail:
            {
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_ECU_CRITICAL_FAIL);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set ECU Critical Fail error event: %d", s16ErrorStatus);
                }
                break;
            }
            case enNonCriticalFail:
            {
                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL);
                if (s16ErrorStatus != (int16_t)enSuccess_EventAddedToQueue)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to set ECU Non-Critical Fail error event: %d", s16ErrorStatus);
                }
                break;
            }
            default:
                icm_vProcessValidMessage(pstReceivedTCPMsg, s16Indx, s16TypeIndx, &stTempMsgConfig, u8ConnectionIndex);
                break;
            }
        }
        else
        {
            /* Action Notification message for VAM */
            if (u8ConnectionIndex == enVAMConnectionTCP)
            {
                int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(pstReceivedTCPMsg->u16ID,
                                                                             pstReceivedTCPMsg->u16SequenceNumber,
                                                                             (uint8_t)enInvalidActionReq,
                                                                             (uint8_t)enActionNotification);
                if (s8NotificationStatus < QUEUE_ACTION_SUCCESS)
                {
                    log_message(global_log_file, LOG_ERROR, "Failed to log notification message");
                }
            }
            log_message(global_log_file, LOG_DEBUG, "Message index not found for Type: %d, ID: %d", pstReceivedTCPMsg->u16Type, pstReceivedTCPMsg->u16ID);
        }
    }
    else
    {
        /* Action Notification message for VAM */
        if (u8ConnectionIndex == enVAMConnectionTCP)
        {
            int8_t s8NotificationStatus = ITCOM_s8LogNotificationMessage(pstReceivedTCPMsg->u16ID,
                                                                         pstReceivedTCPMsg->u16SequenceNumber,
                                                                         (uint8_t)enInvalidActionReq,
                                                                         (uint8_t)enActionNotification);
            if (s8NotificationStatus < QUEUE_ACTION_SUCCESS)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to log notification message for VAM: %d", s8NotificationStatus);
            }
        }
        log_message(global_log_file, LOG_WARNING, "Message validation failed for Type: %d, ID: %d", pstReceivedTCPMsg->u16Type, pstReceivedTCPMsg->u16ID);
    }
}

/**
 * @brief Processes validated messages for storage
 *
 * @details
 * This internal function processes validated messages.
 * The function handles:
 * - Rolling counter evaluation
 * - Cycle count management
 * - Message data storage
 *
 * @param[in] pstReceivedTCPMsg   Pointer to received message
 * @param[in] s16Indx             Message index
 * @param[in] s16TypeIndx         Message type index
 * @param[in] pstTempMsgConfig    Message configuration
 * @param[in] u8ConnectionIndex   Connection identifier
 *
 * @return None
 */
static void icm_vProcessValidMessage(TLVMessage_t *pstReceivedTCPMsg, int16_t s16Indx, int16_t s16TypeIndx, MsgIntConfig_t *pstTempMsgConfig, uint8_t u8ConnectionIndex)
{
    icm_vRollingCountEval(*pstReceivedTCPMsg, *pstTempMsgConfig, s16Indx);
    icm_vCycleCountReset(*pstReceivedTCPMsg, *pstTempMsgConfig, s16Indx, u8ConnectionIndex);
    icm_vSaveMsgData(pstReceivedTCPMsg, s16Indx, s16TypeIndx);
}

/**
 * @brief Logs received message details
 *
 * @details
 * This internal function logs message information.
 * The function handles:
 * - Message field logging
 * - Connection type identification
 * - Debug information formatting
 *
 * @param[in] pstReceivedTCPMsg  Pointer to received message
 * @param[in] enConnection       Connection type
 *
 * @return None
 */
static void icm_vLogReceivedMessage(TLVMessage_t *pstReceivedTCPMsg, enTCPConnectionsASI enConnection)
{
    log_message(global_log_file, LOG_DEBUG,
                "TLVMessage_t RECEIVED: %s\n"
                "{\n"
                "    u16Type            : 0x%04X,\n"
                "    u16Length          : 0x%04X,\n"
                "    u16CRC             : 0x%04X,\n"
                "    u16RollingCounter  : 0x%04X,\n"
                "    u32TimeStamp       : 0x%08X,\n"
                "    u16SequenceNumber  : 0x%04X,\n"
                "    u16ID              : 0x%04X,\n"
                "    au8Value           : [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n"
                "}\n",
                (enConnection == enVAMConnectionTCP ? "VAM" : "CM"),
                pstReceivedTCPMsg->u16Type, pstReceivedTCPMsg->u16Length, pstReceivedTCPMsg->u16CRC,
                pstReceivedTCPMsg->u16RollingCounter, pstReceivedTCPMsg->u32TimeStamp, pstReceivedTCPMsg->u16SequenceNumber,
                pstReceivedTCPMsg->u16ID, pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_0], pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_1],
                pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_2], pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_3], pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_4],
                pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_5], pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_6], pstReceivedTCPMsg->au8Value[ICM_MSG_BYTE_7]);
}

/**
 * @brief Prepares message for transmission
 *
 * @details
 * This internal function prepares messages for sending.
 * The function handles:
 * - Message type identification
 * - Dictionary lookup
 * - Header and payload population
 *
 * @param[in]  pstMsgData  Pointer to message data
 * @param[out] pstTxMsg    Pointer to transmission message
 *
 * @return enTCPConnectionsASI Connection identifier
 */
static enTCPConnectionsASI icm_enPrepareTransmitMessage(stProcessMsgData *pstMsgData, TLVMessage_t *pstTxMsg)
{
    MessageDictionary_t stDictionaryData = MESSAGE_DICTIONARY_INIT;
    MsgIntConfig_t stTempMsgConfig = MSG_INT_CONFIG_INIT;
    enTCPConnectionsASI enConnectionindx = enTotalTCPConnections;
    int16_t s16Indx = 0;

    log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Preparing to Transmit Message - MessageType: 0x%04X, MessageID: 0x%04X",
                pstMsgData->u16Type, pstMsgData->stMsgPairData.u16MsgId);

    s16Indx = ITCOM_s16GetMessageEnumFromTypeAndId(pstMsgData->u16Type, pstMsgData->stMsgPairData.u16MsgId, enVAMConnectionTCP);
    if (s16Indx != MESSAGE_NOT_FOUND)
    {
        ITCOM_vGetMsgDictionaryEntryAtIndex(&stDictionaryData, s16Indx);
        stTempMsgConfig = icm_stIntConfigTable[s16Indx];
        enConnectionindx = (stDictionaryData.u16MessageType == enNotificationMessage) ? enVAMConnectionTCP : enCMConnectionTCP;

        icm_vPopulateMsgHeader(pstTxMsg, *pstMsgData, stDictionaryData);
        icm_vPopulateMsgPayload(pstTxMsg, *pstMsgData, stDictionaryData, stTempMsgConfig);
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "ICM_vTransmitMessage: Message not found in dictionary - Type: 0x%04X, ID: 0x%04X",
                    pstMsgData->u16Type, pstMsgData->stMsgPairData.u16MsgId);
    }

    log_message(global_log_file, LOG_DEBUG, "ICM_vTransmitMessage: Exiting icm_enPrepareTransmitMessage, returning connection index: %d", enConnectionindx);
    return enConnectionindx;
}

/**
 * @brief Tracks sent message information
 *
 * @details
 * This internal function maintains sent message tracking.
 * The function handles:
 * - Cycle count tracking
 * - Message type recording
 * - Calibration data management
 *
 * @param[in] pstMsgData  Pointer to message data
 *
 * @return None
 */
static void icm_vTrackSentMessage(stProcessMsgData *pstMsgData)
{
    MessageDictionary_t stDictionaryData = MESSAGE_DICTIONARY_INIT;
    MsgIntConfig_t stTempMsgConfig = MSG_INT_CONFIG_INIT;
    stMsgIntegrityData stTempIntData = MSG_INTEGRITY_DATA_INIT;
    enTCPConnectionsASI enConnectionindx = enTotalTCPConnections;
    int16_t s16Indx = 0;

    s16Indx = ITCOM_s16GetMessageEnumFromTypeAndId(pstMsgData->u16Type, pstMsgData->stMsgPairData.u16MsgId, enVAMConnectionTCP);
    if (s16Indx != MESSAGE_NOT_FOUND)
    {
        ITCOM_vGetMsgDictionaryEntryAtIndex(&stDictionaryData, s16Indx);
        stTempMsgConfig = icm_stIntConfigTable[s16Indx];
        enConnectionindx = (stDictionaryData.u16MessageType == enNotificationMessage) ? enVAMConnectionTCP : enCMConnectionTCP;

        /* Handle cycle count tracking if applicable */
        if (stTempMsgConfig.u8CycleCountFlag == ACTIVE_FLAG)
        {
            stTempIntData.stMsgPairData.u16MsgId = pstMsgData->stMsgPairData.u16MsgId;
            stTempIntData.stMsgPairData.u16SequenceNum = pstMsgData->stMsgPairData.u16SequenceNum;
            stTempIntData.u16Type = pstMsgData->u16Type;
            stTempIntData.u8ResponseCycleCount = ICM_RESPONSE_COUNT_INIT;
            stTempIntData.u8EnumAssigned = (uint8_t)s16Indx;

            /* Tracking conditions for the ACK messages response */
            if (enConnectionindx == enVAMConnectionTCP)
            {
                stTempIntData.u8ClearCondition = (uint8_t)enAckVAM;
            }
            else
            {
                stTempIntData.u8ClearCondition = (uint8_t)enAckCM;
            }

            if ((stDictionaryData.u16MessageType != enNotificationMessage) || (pstMsgData->au8MsgData[0] != (uint8_t)enTimeoutLimit))
            {
                ITCOM_vSetMsgCycleCount(&stTempIntData, ADD_ELEMENT);
            }

            /* Add aditional tracking for the calibration readback response */
            if (s16Indx >= enTorqueVecMotorCalib && s16Indx <= enTorqueVecMotorCalib)
            {
                stTempIntData.u8ClearCondition = (uint8_t)enCalibReadback;
                stTempIntData.u8EnumAssigned = (uint8_t)enCalibReadback;
                ITCOM_vSetMsgCycleCount(&stTempIntData, ADD_ELEMENT);
                ITCOM_vSetCalibDataCopy(pstMsgData, ADD_ELEMENT);
            }
        }
    }
}

/**
 * @brief Logs transmitted message details
 *
 * @details
 * This internal function logs transmission details.
 * The function handles:
 * - Message content logging
 * - Connection identification
 * - Debug information formatting
 *
 * @param[in] pstTxMsg      Pointer to transmitted message
 * @param[in] enConnection  Connection type
 *
 * @return None
 */
static void icm_vLogTransmittedMessage(const TLVMessage_t *pstTxMsg, enTCPConnectionsASI enConnection)
{
    log_message(global_log_file, LOG_DEBUG,
                "TLVMessage_t SENT: %s\n"
                "{\n"
                "    u16Type            : 0x%04X,\n"
                "    u16Length          : 0x%04X,\n"
                "    u16CRC             : 0x%04X,\n"
                "    u16RollingCounter  : 0x%04X,\n"
                "    u32TimeStamp       : 0x%08X,\n"
                "    u16SequenceNumber  : 0x%04X,\n"
                "    u16ID              : 0x%04X,\n"
                "    au8Value           : [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n"
                "}\n",
                (enConnection == enVAMConnectionTCP ? "VAM" : "CM"),
                pstTxMsg->u16Type, pstTxMsg->u16Length, pstTxMsg->u16CRC,
                pstTxMsg->u16RollingCounter, pstTxMsg->u32TimeStamp, pstTxMsg->u16SequenceNumber,
                pstTxMsg->u16ID, pstTxMsg->au8Value[ICM_MSG_BYTE_0], pstTxMsg->au8Value[ICM_MSG_BYTE_1],
                pstTxMsg->au8Value[ICM_MSG_BYTE_2], pstTxMsg->au8Value[ICM_MSG_BYTE_3], pstTxMsg->au8Value[ICM_MSG_BYTE_4],
                pstTxMsg->au8Value[ICM_MSG_BYTE_5], pstTxMsg->au8Value[ICM_MSG_BYTE_6], pstTxMsg->au8Value[ICM_MSG_BYTE_7]);
}

/**
 * @brief Updates transmission counter values
 *
 * @details
 * This internal function manages transmission counters.
 * The function handles:
 * - Rolling counter updates
 * - Sequence number management
 * - Counter overflow handling
 *
 * @param[in,out] pstTxMsg      Pointer to transmission message
 * @param[in]     enConnection  Connection type
 *
 * @return None
 */
static void icm_vUpdateTransmissionCounters(TLVMessage_t *pstTxMsg, enTCPConnectionsASI enConnection)
{
    int16_t s16Indx = ITCOM_s16GetMessageEnumFromTypeAndId(pstTxMsg->u16Type, pstTxMsg->u16ID, enConnection);
    MsgIntConfig_t stTempMsgConfig = icm_stIntConfigTable[s16Indx];

    pstTxMsg->u16RollingCounter = (pstTxMsg->u16RollingCounter + 1) % UINT16_MAX_VALUE;
    ITCOM_vRecordRC(s16Indx, pstTxMsg->u16RollingCounter, ROLLING_COUNT_TX);
    if (stTempMsgConfig.u8SeqNumberAssigner == SEQ_NUM_ASI)
    {
        pstTxMsg->u16SequenceNumber = (pstTxMsg->u16SequenceNumber + 1) % UINT16_MAX_VALUE;
        ITCOM_vSetSeqNumASIRecord(pstTxMsg->u16SequenceNumber, s16Indx);
    }
}
