/*****************************************************************************
 * @file icm.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 * 
 * @brief Interface Communication Manager Implementation
 *
 * @details
 * This header file defines the interface for the communication management system 
 * in the Sonatus Automator project. It provides declarations for message handling, 
 * data structures, and communication protocols between Vehicle Automation Module 
 * (VAM) and Control Module (CM).
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

#ifndef ICM_H
#define ICM_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Constants and Macros ***/

/* Status Flags */
#define INACTIVE_FLAG                        (0U)
#define ACTIVE_FLAG                          (1U)
#define INFO_UPDATED                         (0U)
#define INFO_OUTDATED                        (1U)

/* Sequence Numbers */
#define SEQ_NUM_ASI                          (0)
#define SEQ_NUM_VAM                          (1)

/* Rolling Counter Types */
#define ROLLING_COUNT_RX                     (0)
#define ROLLING_COUNT_TX                     (1)

/* ACK DATA PAYLOAD */
#define ACK_UNSUCCESFUL                      (1)

/* Buffer and Array Sizes */
#define MSG_QUEUE_BUFFER_SIZE                (20)
#define NUM_TRACKED_ELEMENTS                 (40)
#define MSG_PAYLOAD_SIZE                     (8U)
#define TLV_VALUE_SIZE                       (8U)

/* Timeout and Limit Values */
#define TIMEOUT_NA                           (0)
#define MSG_TIMEOUT_MAX_VALUE                (25)
#define ACK_MESG_RESPONSE_TIME_LIMIT         (35)
#define CAL_READBACK_RESPONSE_TIME_LIMIT     (50)
#define ROLLINC_COUNTER_ERROR_LIMIT          (3)
#define NO_MSG_ID_ASSIGN                     (0xFFFFU)

/* Generic Initialization Values */
#define ICM_INIT_VAL_U8                      (0U)
#define ICM_INIT_VAL_U16                     (0U)
#define ICM_INIT_VAL_F32                     (0.0F)
#define ICM_INIT_VAL_S32                     (0)
#define ICM_TIME_FACTOR_MS                   (1000.0)

/* Message Byte Indices */
#define ICM_MSG_BYTE_0                       (0U)
#define ICM_MSG_BYTE_1                       (1U)
#define ICM_MSG_BYTE_2                       (2U)
#define ICM_MSG_BYTE_3                       (3U)
#define ICM_MSG_BYTE_4                       (4U)
#define ICM_MSG_BYTE_5                       (5U)
#define ICM_MSG_BYTE_6                       (6U)
#define ICM_MSG_BYTE_7                       (7U)
#define ICM_BYTE_SHIFT_8                     (8U)

/* Message Initialization Values */
#define ICM_ALLOWED_MSGS_INIT                (0U)
#define ICM_TIME_WINDOW_INIT                 (0U)
#define ICM_MSG_COUNT_INIT                   (0U)
#define ICM_START_TIME_INIT                  (0U)
#define ICM_MSG_ID_INIT                      (0U)
#define ICM_MSG_TYPE_INIT                    (0U)
#define ICM_MSG_ENUM_INIT                    (0U)
#define ICM_SEQUENCE_NUM_INIT                (0U)
#define ICM_RESPONSE_COUNT_INIT              (0U)
#define ICM_CLEAR_CONDITION_INIT             (0U)

/* Structure Initialization Macros */
/**
 * @brief Rate limiter initialization macro
 * Initializes the rate limiting parameters for message handling
 */
#define RATE_LIMITER_INIT {                                                                  \
    ICM_ALLOWED_MSGS_INIT,                    /* u16AllowedMessages */                       \
    ICM_TIME_WINDOW_INIT,                     /* u16TimeWindowMs */                          \
    ICM_MSG_COUNT_INIT,                       /* u16MessageCount */                          \
    ICM_START_TIME_INIT                       /* stStartTime */                              \
}

/**
 * @brief Message integrity data initialization macro
 * Initializes the message integrity tracking structure
 */
#define MSG_INTEGRITY_DATA_INIT {                                                            \
    {ICM_MSG_ID_INIT,                         /* stMsgPairData.u16MsgId */                   \
     ICM_SEQUENCE_NUM_INIT},                  /* stMsgPairData.u16SequenceNum */             \
    ICM_RESPONSE_COUNT_INIT,                  /* u8ResponseCycleCount */                     \
    ICM_MSG_ENUM_INIT,                        /* u8EnumAssigned */                           \
    ICM_CLEAR_CONDITION_INIT,                 /* u8ClearCondition */                         \
    ICM_MSG_TYPE_INIT                         /* u16Type */                                  \
}

/**
 * @brief Message dictionary initialization macro
 * Initializes the message dictionary entry structure
 */
#define MESSAGE_DICTIONARY_INIT {                                                            \
    ICM_MSG_ID_INIT,                          /* u16MessageId */                             \
    ICM_MSG_TYPE_INIT,                        /* u16MessageType */                           \
    ICM_MSG_ENUM_INIT                         /* u8MessageEnum */                            \
}

/**
 * @brief Message type dictionary initialization macro
 * Initializes the message type dictionary entry
 */
#define MESSAGE_TYPE_DICTIONARY_INIT {                                                       \
    ICM_MSG_TYPE_INIT                         /* u16MessageTypeID */                         \
}

/**
 * @brief Message process data initialization macro
 * Initializes the message processing structure
 */
#define MSG_PROCESS_DATA_INIT {                                                              \
    ICM_MSG_TYPE_INIT                         /* u16Type */                                  \
}

/**
 * @brief TLV message initialization macro
 * Initializes the TLV (Type-Length-Value) message structure
 */
#define MSG_TLV_DATA_INIT {                                                                  \
    ICM_INIT_VAL_U16                          /* u16Type */                                  \
}

/**
 * @brief Message integrity configuration initialization macro
 * Initializes the message integrity configuration
 */
#define MSG_INT_CONFIG_INIT {                                                                \
    ICM_INIT_VAL_U8                           /* u8TimeoutLimit */                           \
}

/*** Type Definitions ***/

/* Enumerations */
typedef enum
{
    /* VAM Messages (0-12) */
    enHVACFanSpeed = 0,
    enHVACCabinTemperature,
    enWindshieldWiperSpeed,
    enSeatPositionDriver,
    enSeatPositionPassenger,
    enSeatHeaterDriver,
    enSeatHeaterPassenger,
    enDoorLockState,
    enTurnSignalState,
    enAmbientLighting,
    enTorqueVecMotorCalib,
    enRainSensor,
    enAckVAM,
    enTotalMessagesVAM
} enMessageListVAM;

typedef enum
{
    /* CM Messages (13-18) */
    enPRNDL = enTotalMessagesVAM,
    enVehicleSpeed,
    enCalibReadback,
    enAckCM,
    enNonCriticalFail,
    enCriticalFail,
    enTotalMessagesCM
} enMessageListCM;

typedef enum
{
    /* ASI Messages (19-21) */
    enActionNotification = enTotalMessagesCM,
    enStartUpTestNotification,
    enStatusNotificationASI,
    enTotalMessagesASI
} enMessageListASI;

typedef enum
{
    enActionRequest = 0,
    enStatusMessageCM,
    enAckMessage,
    enNotificationMessage,
    enCalibReadbackMessage,
    enTotalASIMessageClassification
} enMessageTypeListASI;

typedef enum
{
    enApprovedRequest = 0,
    enPreconditionFail,
    enInvalidActionReq,
    enSUTNotPerformed,
    enVehicleStatusFail,
    enRateLimiterDrop,
    enTimeoutLimit,
    enTransmissionFailed,
    enTotalNotificationActions
} enNotificationActions;

typedef enum
{
    enSuccesfulSUT = 0,
    enFailedSUT,
    enUnfinishedSUT,
    enTotalSUTNotifications
} enNotificationSUT;

typedef enum
{
    enActionMsgBuffer = 0,
    enCalibDataCopyBuffer,
    enCalibReadbackData,
    enTotalTrackBuffers
} enTrackingBuffers;

/* Structures */
typedef struct
{
    uint16_t u16SequenceNum;
    uint16_t u16MsgId;
} stIdSequencePair;

typedef struct
{
    uint16_t u16RollingCountRX;
    uint16_t u16RollingCountTX;
} stRollingCountData_t;

typedef struct
{
    uint16_t u16SeqNumberSender;
    uint16_t u16SeqNumberASI;
} stSequenceNumberData_t;

typedef struct
{
    stIdSequencePair stMsgPairData;
    uint8_t u8ResponseCycleCount;
    uint8_t u8EnumAssigned;
    uint8_t u8ClearCondition;
    uint16_t u16Type;
} stMsgIntegrityData;

typedef struct
{
    uint16_t u16Type;
    uint16_t u16Length;
    stIdSequencePair stMsgPairData;
    uint8_t au8MsgData[MSG_PAYLOAD_SIZE];
} stProcessMsgData;

typedef struct
{
    uint16_t u16Type;
    uint16_t u16Length;
    uint16_t u16CRC;
    uint16_t u16RollingCounter;
    uint32_t u32TimeStamp;
    uint16_t u16SequenceNumber;
    uint16_t u16ID;
    uint8_t au8Value[TLV_VALUE_SIZE];
} TLVMessage_t;

typedef struct
{
    uint16_t u16AllowedMessages;
    uint16_t u16TimeWindowMs;
    uint16_t u16MessageCount;
    clock_t stStartTime;
} RateLimiter_t;

/*** Functions Provided to other modules ***/
extern void ICM_vInit(void);
extern void ICM_vCycleCountUpdater(void);
extern void ICM_vReceiveMessage(void);
extern void ICM_vTransmitMessage(void);

/*** Variables Provided to other modules ***/

#endif /* ICM_H */
