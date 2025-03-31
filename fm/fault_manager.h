/*****************************************************************************
 * @file fault_manager.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 * 
 * @brief Fault and Error Event Management System Implementation
 *
 * @details
 * This header file defines the public interface and data structures for the 
 * Sonatus Automator's Fault Management System. It provides the necessary types,
 * enumerations, and function declarations for system-wide fault handling and
 * error management.
 *
 * Key Features:
 * - Multi-stage error event processing with timeout protection
 * - Thread-safe event queue management
 * - Configurable event severity levels (Critical, Normal, Minor)
 * - Automatic log file rotation and management
 * - System state snapshot capture during fault events
 * - Persistent storage of event processing state
 * - Structured event logging with timestamps
 * - Support for special event handling and skipped event logging
 *
 * @authors Brian Le (BL), Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date September 05 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/19/2024 | BL     | Initial
 * 09/05/2024 | TP     | Complete FM Implementation
 * 10/24/2024 | AT     | Cleaning up the code, removal of DEBUG_LOG and pointer checks added
 * 11/17/2024 | TP     | MISRA & LHP compliance fixes, Functionality Check PASSED
 * 11/22/2024 | TP     | Cleaning up the code
 */

#ifndef FM_FAULT_MANAGER_H
#define FM_FAULT_MANAGER_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/

#define FM_TIMESTAMP_STRING_LENGTH          ((uint8_t)20U)    /* Size of system time string buffer */

/*** Type Definitions ***/

/**
 * @enum EVENT_ID_t
 * @brief Enumeration of all possible event IDs in the system.
 *
 * This enum defines unique identifiers for various events that can occur in the system,
 * including faults, information events, and specific error conditions.
 */
typedef enum
{
    EVENT_ID_FAULT_MSG_CRC_CHECK = 0,                ///< Event is reported when received message does not pass CRC check
    EVENT_ID_FAULT_ROLL_COUNT,                       ///< Event is reported when received message does not pass rolling counter check
    EVENT_ID_FAULT_MSG_TYPE_LENGTH,                  ///< Event is reported when received message does not pass type-length check
    EVENT_ID_FAULT_MSG_TIMEOUT,                      ///< Event is reported when received message timeout
    EVENT_ID_INFO_ACK_LOSS,                          ///< Event is reported when a message Acknowledgement is not received
    EVENT_ID_INFO_ACK_UNSUCCESS,                     ///< Event is reported when a message Acknowledgement is received but the data sent was corrupted.
    EVENT_ID_FAULT_PRECOND_LIST_ERROR,               ///< Event is reported when the Start-Up test determines that...
    EVENT_ID_FAULT_ACTION_LIST_ERROR,                ///< Event is reported when the Start-Up test determines that...
    EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH,           ///< Event is reported when the Action Request Approver determines that the vehicle status information had a mismatch and it's not possible to determine vehicle status.
    EVENT_ID_INFO_VEHICLE_STATUS_ERROR,              ///< Event is reported when the Action Request Approver determines that the vehicle status information was outdated during evaluation.
    EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR, ///< Event is reported when the Action Request Approver determines that the vehicle status information does not have valid data within it's payload.
    EVENT_ID_FAULT_CAL_READBACK_ERROR,               ///< Event is reported if calibration readback if mismatch is determined
    EVENT_ID_FAULT_CAL_READBACK_TIMEOUT,             ///< Event is reported if calibration readback timeout
    EVENT_ID_FAULT_STARTUP_MEM_ERROR,                ///< Event is reported if start-up test does not pass memory check
    EVENT_ID_INFO_LOSS_COMM,                         ///< Event is reported if TCP connection is lost
    EVENT_ID_INFO_MSG_LOSS,                          ///< Event is reported if the sequence number comparison fails
    EVENT_ID_FAULT_SUT_TERM,                         ///< Event is reported when Start-up test is disrupted
    EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR,      ///< Event is reported when the Action Request Approver determines that the received action request from VAM does not have valid data within it's payload.
    EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR,      ///< Event is reported when the Action Request Approver determines that the received action request from VAM does not exist in the predefined action list
    EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR,     ///< Event is reported when the Action Request Approver determines that the received action request from VAM does not pass the preconditions evaluation.
    EVENT_ID_INIT_COMPLETE,                          ///< Event is reported when an ASI initialization process is completed
    EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT,    ///< Event is reported when the time limit is reached for processing an action request.
    EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL,            ///< Event is reported when the ASI receives the non-critical FailMessage.
    EVENT_ID_FAULT_ECU_CRITICAL_FAIL,                ///< Event is reported when the ASI receives the critical FailMessage.
    EVENT_ID_FAULT_OVERRUN,                          ///< Event is reported when an ASI tasks overruns their allocated timing.
    EVENT_ID_FAULT_SM_TRANSITION_ERROR,              ///< Event is reported when the State Monitor Test detects an invalid state transition.
    enTotalEventIds
} EVENT_ID_t;

/**
 * @enum SeverityType
 * @brief Enumeration of event severity levels.
 *
 * Defines the possible severity levels for events, ranging from minor to critical.
 */
typedef enum
{
    SEVERITY_MINOR = 0U,
    SEVERITY_NORMAL = 1U,
    SEVERITY_CRITICAL = 2U,
    enTotalSeverityTypes = 3U
} SeverityType;

/**
 * @struct SystemSnapshot_t
 * @brief Structure to hold snapshot data of the system state.
 *
 * Contains key system parameters captured at the time of an event occurrence.
 */
typedef struct
{
    float32_t VehicleSpeed;
    uint32_t GearShiftPosition;
    uint32_t ASI_State;
    fm_char_t SystemTime[FM_TIMESTAMP_STRING_LENGTH];
} SystemSnapshot_t;

/**
 * @struct ErrorEvent
 * @brief Structure to represent an error event in the system.
 *
 * Contains all relevant information about an error event, including its ID,
 * occurrence count, severity, associated notification function, and snapshot data.
 */
typedef struct
{
    EVENT_ID_t Error_Event_ID;
    uint32_t Error_Event_Counter;
    SeverityType Severity;
    void (*NotificationFunction)(void);
    SystemSnapshot_t SystemSnapshotData;
} ErrorEvent;

/**
 * @struct EventProcessingState
 * @brief Structure to maintain the state of event processing.
 *
 * Keeps track of the current event being processed, its processing stage,
 * start and end times, and total processing duration.
 */
typedef struct
{
    ErrorEvent *current_event;
    uint32_t processing_stage;
    struct timespec start_time;
    struct timespec end_time;
    float64_t total_processing_time;
} EventProcessingState;

/*** Functions Provided to other modules ***/

extern void FM_vCloseEventLogger(void);
extern void FM_vLogRemainingEvents(FILE *event_log_file);
extern void FM_vLoadEventDataFromStorage(void);
extern void FM_vMainFunction(void);
extern void FM_vLogSpecialEvent(FILE *event_log_file, event_type_t event_type, EVENT_ID_t current_event_id);
extern int8_t FM_s8SaveEventDataToStorage(void);
extern uint32_t FM_u32FindLeastSevereEvent(uint8_t *queue, uint32_t size);
extern uint8_t FM_u8GetEventSeverity(uint8_t u8Indx);

/*** Variables Provided to other modules ***/

#endif /* FM_FAULT_MANAGER_H */
