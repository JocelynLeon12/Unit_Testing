/*****************************************************************************
 * @file fault_manager.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 * 
 * @brief Fault and Error Event Management System Implementation
 *
 * @details
 * This file implements the core fault management system for the Sonatus Automator 
 * Safety Interlock project. It provides comprehensive error event handling, logging,
 * and notification capabilities for system-wide fault management.
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
 * @authors Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date September 05 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 09/05/2024 | TP     | Complete FM Implementation
 * 10/24/2024 | AT     | Cleaning up the code, removal of DEBUG_LOG and pointer checks added
 * 11/17/2024 | TP     | MISRA & LHP compliance fixes, Functionality Check PASSED
 * 11/22/2024 | TP     | Cleaning up the code
 */

/*** Include Files ***/
#include "itcom.h"
#include "storage_handler.h"
#include "thread_management.h"
#include "data_queue.h"

#include "fault_manager.h"

/*** Module Definitions ***/

/**
 * @def MAX_LOG_SIZE
 * @brief Maximum size of a single log file in bytes.
 *
 * Defines the maximum size that a single log file can grow to before it is rotated.
 * Set to 10 MB (10 * 1024 * 1024 bytes) to prevent excessively large log files.
 *
 */
#define MAX_LOG_SIZE (10ULL * 1024ULL * 1024ULL)

/**
 * @def MAX_LOG_FILES
 * @brief Maximum number of log files to maintain in rotation.
 *
 * Specifies the number of log files to keep when rotating logs.
 * When this limit is reached, the oldest log file is deleted to make room for a new one.
 *
 */
#define MAX_LOG_FILES 5

/**
 * @def EVENT_PROCESSING_TIMEOUT
 * @brief Timeout duration for processing a single error event, in seconds.
 *
 * Defines the maximum time allowed for processing a single error event.
 * If event processing exceeds this duration, it will be interrupted to prevent system hangs.
 *
 */
#define EVENT_PROCESSING_TIMEOUT 5

/*** Internal Types ***/

/*** Conversion Factors ***/
#define SEC_TO_MS                        (1000.0f)                /* Seconds to milliseconds */
#define NSEC_TO_MS                       (1000000.0f)             /* Nanoseconds to milliseconds */

/*** Buffer Sizes ***/
#define FM_TIMESTAMP_BUFFER_SIZE         ((uint8_t)20U)           /* Timestamp string buffer */
#define FM_FILENAME_BUFFER_SIZE          ((uint16_t)256U)         /* File path/name buffer */
#define FM_QUEUE_STRING_BUFFER_SIZE      ((uint8_t)128U)          /* Queue debug string buffer */
#define FM_LOG_BUFFER_SIZE               ((uint16_t)256U)         /* Basic log message buffer */
#define FM_FORMATTED_MSG_LOG_BUFFER_SIZE ((uint16_t)512U)         /* Full formatted log buffer */
#define FM_SKIPPED_EVENT_COMPARE_SIZE    ((uint8_t)12U)           /* Compare buffer for skipped events */

/*** Initial Values ***/
#define FM_ZERO_EVENT_ID                 ((uint8_t)0U)            /* Initial event ID */
#define FM_INITIAL_EVENT_COUNTER         ((uint32_t)0U)           /* Initial error event counter */
#define FM_ZERO_QUEUE_INDEX              ((uint8_t)0U)            /* Initial queue index starts at 0 */
#define FM_INITIAL_QUEUE_INDEX           ((int8_t)-1)             /* Initial queue index starts at -1 */
#define FM_ZERO_TIMESTAMP_RESULT         ((uint8_t)0U)            /* Initial timestamp result */
#define FM_INITIAL_EVENT_PROCESSING_TIME ((float64_t)0.0f)        /* Initial processing time */

/*** Processing Stage Constants ***/
#define FM_INITIAL_PROCESSING_STAGE      ((uint8_t)0U)            /* First processing stage */
#define FM_PROCESSING_STAGES_COMPLETE    ((uint8_t)4U)            /* Total number of stages */



/*** Local Function Prototypes ***/
static ErrorEvent *fm_GetErrorEvent(void);
static error_string_t fm_GetEventIDString(EVENT_ID_t event_id);
static void fm_vSnapshotDataCollection(ErrorEvent *event);
static void fm_vEventLogger(const ErrorEvent *event, string_t special_event_type);
static void fm_vProcessErrorEvent(EventProcessingState *state);
static void fm_vProcessErrorEventWithTimeout(EventProcessingState *state);
static void fm_vIncrementErrorCounter(ErrorEvent *event);
static void fm_vCallNotificationFunction(ErrorEvent *event);
static void fm_vLogErrorEvent(ErrorEvent *event);
static float64_t fm_f64GetElapsedTimeMs(struct timespec *start, struct timespec *end);
static void fm_vRotateLogFile(void);
static void fm_vPrintEventQueue(void);
static void fm_vResetErrorEventCounters(void);
static void fm_vCaptureSnapshotData(SystemSnapshot_t *snapshot);

/*** External Variables ***/

/*** Internal Variables ***/

/**
 * @var log_file
 * @brief Static file pointer for the event log file.
 *
 * This variable holds the FILE pointer to the current event log file.
 * It is initialized to NULL and is opened when the first log entry is written.
 * The file is used to record various events and error conditions in the system.
 *
 * @note This variable is static to maintain file handle persistence across function calls.
 *
 */
static FILE *log_file = NULL;

/**
 * @var current_log_size
 * @brief Tracks the current size of the event log file in bytes.
 *
 * This variable keeps track of the current size of the event log file.
 * It is used to determine when the log file should be rotated to prevent
 * it from growing too large. The size is updated each time a new log entry is written.
 *
 * @note This variable is static to maintain the log size information across function calls.
 *
 */
static uint64_t current_log_size = 0;

/**
 * @var Error_Events
 * @brief Static array containing predefined error event configurations.
 *
 * This array defines a set of 25 error events, each represented by an ErrorEvent structure.
 * It serves as a lookup table for various error conditions that can occur in the system.
 * Each entry in the array contains:
 *   - Event ID: A unique identifier for the error event.
 *   - Error Event Counter: Initially set to 0, used to track occurrences of each event.
 *   - Severity: Indicates the criticality of the error (SEVERITY_CRITICAL, SEVERITY_NORMAL, or SEVERITY_MINOR).
 *   - Notification Function: Points to the function to be called when the event occurs.
 *
 * Error events are categorized by severity:
 *   - SEVERITY_CRITICAL: Requires immediate attention and typically triggers SM_Notification.
 *   - SEVERITY_NORMAL: Less urgent, often handled by External_System_Notification.
 *   - SEVERITY_MINOR: Least severe, some may not require notification (NULL function pointer).
 *
 * @note This array is static and should not be modified during runtime.
 *       It's used as a reference for error handling and logging throughout the system.
 *
 */
static ErrorEvent Error_Events[enTotalEventIds] = {
    /*Enum Value              EVENT_ID_INDEX                                            Error_Event_ID                                Error_Event_Counter               Severity                 NotificationFunction              SystemSnapshotData  */
/*  [00]  */        [EVENT_ID_FAULT_MSG_CRC_CHECK]                        = {EVENT_ID_FAULT_MSG_CRC_CHECK,                       FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [01]  */        [EVENT_ID_FAULT_ROLL_COUNT]                           = {EVENT_ID_FAULT_ROLL_COUNT,                          FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [02]  */        [EVENT_ID_FAULT_MSG_TYPE_LENGTH]                      = {EVENT_ID_FAULT_MSG_TYPE_LENGTH,                     FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [03]  */        [EVENT_ID_FAULT_MSG_TIMEOUT]                          = {EVENT_ID_FAULT_MSG_TIMEOUT,                         FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [04]  */        [EVENT_ID_INFO_ACK_LOSS]                              = {EVENT_ID_INFO_ACK_LOSS,                             FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [05]  */        [EVENT_ID_INFO_ACK_UNSUCCESS]                         = {EVENT_ID_INFO_ACK_UNSUCCESS,                        FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [06]  */        [EVENT_ID_FAULT_PRECOND_LIST_ERROR]                   = {EVENT_ID_FAULT_PRECOND_LIST_ERROR,                  FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [07]  */        [EVENT_ID_FAULT_ACTION_LIST_ERROR]                    = {EVENT_ID_FAULT_ACTION_LIST_ERROR,                   FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [08]  */        [EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH]               = {EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH,              FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [09]  */        [EVENT_ID_INFO_VEHICLE_STATUS_ERROR]                  = {EVENT_ID_INFO_VEHICLE_STATUS_ERROR,                 FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [10]  */        [EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR]     = {EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR,    FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [11]  */        [EVENT_ID_FAULT_CAL_READBACK_ERROR]                   = {EVENT_ID_FAULT_CAL_READBACK_ERROR,                  FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [12]  */        [EVENT_ID_FAULT_CAL_READBACK_TIMEOUT]                 = {EVENT_ID_FAULT_CAL_READBACK_TIMEOUT,                FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [13]  */        [EVENT_ID_FAULT_STARTUP_MEM_ERROR]                    = {EVENT_ID_FAULT_STARTUP_MEM_ERROR,                   FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [14]  */        [EVENT_ID_INFO_LOSS_COMM]                             = {EVENT_ID_INFO_LOSS_COMM,                            FM_INITIAL_EVENT_COUNTER,         SEVERITY_MINOR,          NULL,                                     {0}          },
/*  [15]  */        [EVENT_ID_INFO_MSG_LOSS]                              = {EVENT_ID_INFO_MSG_LOSS,                             FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [16]  */        [EVENT_ID_FAULT_SUT_TERM]                             = {EVENT_ID_FAULT_SUT_TERM,                            FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [17]  */        [EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR]          = {EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR,         FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [18]  */        [EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR]          = {EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR,         FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [19]  */        [EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR]         = {EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR,        FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [20]  */        [EVENT_ID_INIT_COMPLETE]                              = {EVENT_ID_INIT_COMPLETE,                             FM_INITIAL_EVENT_COUNTER,         SEVERITY_MINOR,          NULL,                                     {0}          },
/*  [21]  */        [EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT]        = {EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT,       FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [22]  */        [EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL]                = {EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL,               FM_INITIAL_EVENT_COUNTER,         SEVERITY_NORMAL,         &ITCOM_vExtSysNotification,               {0}          },
/*  [23]  */        [EVENT_ID_FAULT_ECU_CRITICAL_FAIL]                    = {EVENT_ID_FAULT_ECU_CRITICAL_FAIL,                   FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [24]  */        [EVENT_ID_FAULT_OVERRUN]                              = {EVENT_ID_FAULT_OVERRUN,                             FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          },
/*  [25]  */        [EVENT_ID_FAULT_SM_TRANSITION_ERROR]                  = {EVENT_ID_FAULT_SM_TRANSITION_ERROR,                 FM_INITIAL_EVENT_COUNTER,         SEVERITY_CRITICAL,       &ITCOM_vNotification_SM,                  {0}          }
};


/*** External Functions ***/

/**
 * @brief Main function for the Fault Management (FM) thread, executed every 25ms.
 *
 * This function implements the core logic of the Fault Management thread. It runs
 * in a continuous loop, processing error events from a queue. The function is designed
 * to be called every 25 milliseconds, synchronized by a semaphore.
 *
 * Key features:
 * - Uses a state machine approach for event processing
 * - Handles interruptions and resumes processing of interrupted events
 * - Implements timeout mechanism for event processing
 * - Logs start, resume, and completion of event processing
 * - Manages shared data for inter-thread communication
 * - Provides graceful termination handling
 *
 * @param
 *
 * @note This function is designed to run as a separate thread and should be
 *       terminated by setting the global thread_exit_flag.
 *
 * @warning This function assumes the existence of global variables and functions:
 *          - thread_exit_flag
 *          - shared_data
 *          - global_log_file
 *          - Various logging and event processing functions
 *
 */
void FM_vMainFunction(void)
{
    EventProcessingState state = {NULL, FM_INITIAL_PROCESSING_STAGE, {0}, {0}, FM_INITIAL_EVENT_PROCESSING_TIME};
    ErrorEvent stCurrentEvent;

    if (state.processing_stage == FM_INITIAL_PROCESSING_STAGE)
    {
        if (ITCOM_s16GetProcessingFlag())
        {
            ITCOM_vGetErrorEvent(&stCurrentEvent);
            state.current_event = &stCurrentEvent;
            log_message(global_log_file, LOG_INFO, "THRD_FM Resuming processing of Error Event ID: %d", state.current_event->Error_Event_ID);
            FM_vLogSpecialEvent(global_log_file, "RESUME PROCESSING EVENT", state.current_event->Error_Event_ID);
        }
        else
        {
            state.current_event = fm_GetErrorEvent();
            if (state.current_event != NULL)
            {
                log_message(global_log_file, LOG_INFO, "THRD_FM Processing Error Event ID: %d", state.current_event->Error_Event_ID);
                fm_vSnapshotDataCollection(state.current_event);
                ITCOM_vUpdateCurrentEvent(state.current_event);
                ITCOM_vSetErrorProcessingFlag(1);
                FM_vLogSpecialEvent(global_log_file, "START PROCESSING EVENT", state.current_event->Error_Event_ID);
            }
        }
    }

    if (state.current_event != NULL)
    {
        fm_vProcessErrorEventWithTimeout(&state);
    }

    if (state.processing_stage >= FM_PROCESSING_STAGES_COMPLETE)
    {
        FM_vLogSpecialEvent(global_log_file, "FINISH PROCESSING EVENT", state.current_event->Error_Event_ID);
        ITCOM_vSetErrorProcessingFlag(0);
        state.processing_stage = FM_PROCESSING_STAGES_COMPLETE;
        state.current_event = NULL;
        state.total_processing_time = FM_INITIAL_EVENT_PROCESSING_TIME;
        fm_vPrintEventQueue();
    }
}

/**
 * @brief Logs any remaining events in the queue before shutdown.
 *
 * This function is called during the shutdown process to ensure that any events
 * still present in the event queue are logged before the program terminates.
 * It prevents the loss of important error or status information that hasn't been
 * processed yet.
 *
 * @param event_log_file Pointer to the FILE where the event will be logged.
 *
 * The function performs the following operations:
 * 1. Locks the shared data mutex to ensure thread-safe access to the event queue.
 * 2. Checks if there are any events in the queue.
 * 3. If events exist, it logs a message indicating that remaining events will be logged.
 * 4. Iterates through all events in the queue, logging each one as a "SKIPPED EVENT".
 * 5. Clears the event queue after logging all events.
 * 6. Unlocks the shared data mutex.
 *
 * @note This function should be called before final program termination, typically
 *       in the shutdown or cleanup phase of the application.
 *
 * @warning This function assumes the existence of the following global variables:
 *          - shared_data: A pointer to the shared memory structure.
 *          - global_log_file: A file pointer for logging.
 *
 */
void FM_vLogRemainingEvents(FILE *event_log_file)
{
    if (event_log_file == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL file pointer passed to FM_vLogRemainingEvents");
        return;
    }
    uint8_t u8EventId = FM_ZERO_EVENT_ID;
    int8_t s16QueueIndx = FM_INITIAL_QUEUE_INDEX;

    s16QueueIndx = ITCOM_s16GetEventQueueIndx();

    if (s16QueueIndx > 0)
    {
        log_message(event_log_file, LOG_INFO, "Logging remaining events before shutdown:");

        int8_t i;
        for (i = 0; i < (int8_t)s16QueueIndx; i++)
        {
            ITCOM_vGetEventQueueId(&u8EventId, i);
            ErrorEvent *event = &Error_Events[u8EventId];
            fm_vEventLogger(event, "SKIPPED EVENT");
        }

        ITCOM_vSetEventQueueIndx(0);
    }
}

/**
 * @brief Logs a special event with a timestamp and event details.
 *
 * This function is used to log special events that occur during the program's
 * execution. It provides a standardized format for logging these events,
 * including a timestamp, event type, and the current event ID being processed.
 *
 * @param event_log_file Pointer to the FILE where the event will be logged.
 * @param event_type A string describing the type of special event (e.g., "SKIPPED EVENT", "RESUME PROCESSING EVENT").
 * @param current_event_id The ID of the event currently being processed when the special event occurred.
 *
 * The function performs the following operations:
 * 1. Generates a timestamp for the current time.
 * 2. Formats and writes a log entry with the following information:
 *    - Timestamp
 *    - Special event type
 *    - Current event ID
 *    - String representation of the current event ID
 * 3. Flushes the log file to ensure the entry is immediately written to disk.
 *
 * @note This function is typically called when significant events occur that
 *       are not part of the normal error event processing flow.
 *
 * @warning Ensure that the event_log_file is properly opened and writable before
 *          calling this function to avoid undefined behavior.
 *
 */
void FM_vLogSpecialEvent(FILE *event_log_file, event_type_t event_type, EVENT_ID_t current_event_id)
{
    if (event_log_file == NULL || event_type == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL pointer passed to FM_vLogSpecialEvent");
        return;
    }
    fm_char_t timestamp[FM_TIMESTAMP_BUFFER_SIZE];
    time_t now = time(NULL);

    size_t time_result = strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));
    if ((size_t)time_result == (size_t)FM_ZERO_TIMESTAMP_RESULT)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format timestamp");
        return;
    }

    /* Create a buffer for the complete log message */
    fm_char_t log_buffer[FM_LOG_BUFFER_SIZE];
    int32_t written_chars = snprintf(log_buffer, sizeof(log_buffer),
                                     "[%s] SPECIAL EVENT: %-20s Current Event ID: %d (%s)\n",
                                     timestamp, event_type, current_event_id, fm_GetEventIDString(current_event_id));

    if ((written_chars < 0) || ((size_t)written_chars >= sizeof(log_buffer)))
    {
        log_message(global_log_file, LOG_ERROR, "Buffer overflow while formatting log message");
        return;
    }

    /* Write the formatted buffer to file using write() */
    ssize_t bytes_written = write(fileno(event_log_file), log_buffer, (size_t)written_chars);
    if (bytes_written < 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to write to log file");
        return;
    }

    int32_t flush_result = fflush(event_log_file);
    if (flush_result != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to flush log file");
    }
}

/**
 * @brief Captures snapshot data for an error event.
 *
 * This function captures the current state of various vehicle parameters
 * and stores them in the provided SnapShotData structure. It is typically
 * used to record the system state when an error event occurs.
 *
 * @param snapshot Pointer to a SnapShotData structure where the captured data will be stored.
 *
 * The function captures the following data:
 * 1. Vehicle Speed: Retrieved using ITCOM_f32GetVehicleSpeed()
 * 2. Gear Shift Position: Retrieved using ITCOM_u8GetGearShiftPositionStatus()
 * 3. ASI State: Retrieved using ITCOM_u8GetASIState()
 *
 * @note This function assumes that the ITCOM interface functions are available
 *       and properly initialized.
 *       At present we are capturing only Vehicle Speed, ASI State,
 *       but we can add other parameters as well based on the requirements.
 *
 * @warning This function directly accesses hardware or system interfaces.
 *          Ensure that all necessary initializations have been performed
 *          before calling this function.
 *
 */
static void fm_vCaptureSnapshotData(SystemSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL snapshot pointer passed to fm_vCaptureSnapshotData");
        return;
    }
    uint8_t u8VehicleSpeedInfoStatus = ITCOM_u8GetVehicleSpeed(&snapshot->VehicleSpeed);
    if (u8VehicleSpeedInfoStatus != (uint8_t)INFO_UPDATED)
    {
        log_message(global_log_file, LOG_ERROR, "Vehicle speed information OUTDATED during snapshot capture");
    }
    uint8_t u8ParkStatusInfoStatus = ITCOM_u8GetParkStatus((uint8_t *)&snapshot->GearShiftPosition);
    if (u8ParkStatusInfoStatus != (uint8_t)INFO_UPDATED)
    {
        log_message(global_log_file, LOG_ERROR, "Park status information OUTDATED during snapshot capture");
    }
    snapshot->ASI_State = ITCOM_u8GetASIState();
}

/**
 * @brief Closes the event logger file.
 *
 * This function is responsible for safely closing the event logger file
 * that has been used for logging events throughout the program's execution.
 * It should be called during the program's shutdown process to ensure all
 * logged data is properly flushed and the file is closed.
 *
 * The function performs the following operations:
 * 1. Checks if the log file pointer (log_file) is not NULL.
 * 2. If the file is open, it closes the file using fclose().
 * 3. Sets the log_file pointer to NULL to prevent further access attempts.
 *
 * @note This function should be called once during program termination,
 *       typically in the cleanup phase of the main process or when
 *       shutting down the logging subsystem.
 *
 * @warning After calling this function, any attempts to log events will
 *          fail unless a new log file is opened. Ensure all necessary
 *          logging is complete before calling this function.
 *
 */
void FM_vCloseEventLogger(void)
{
    if (log_file != NULL)
    {
        int32_t close_result = fclose(log_file);
        if (close_result != -1)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to close file: %s", error_str);
        }
        log_file = NULL;
    }
}

/**
 * @brief Saves current event data to persistent storage.
 *
 * This function is responsible for persisting the current state of event processing
 * to a file on disk. It saves information about whether an event is currently being
 * processed and, if so, the details of that event.
 *
 * @return int Returns 0 on successful save, -1 on failure.
 *
 * The function performs the following operations:
 * 1. Constructs the filename for the event data storage file.
 * 2. Opens the file for writing, creating it if it doesn't exist.
 * 3. Writes the current processing flag to the file.
 * 4. If an event is being processed, writes the current event data to the file.
 * 5. Closes the file.
 * 6. Logs the result of the save operation.
 *
 * @note This function should be called at strategic points in the program execution,
 *       such as during graceful shutdown and at regular intervals, to ensure event
 *       processing can be resumed in case of unexpected termination.
 *
 * @warning This function assumes the existence of global variables:
 *          - shared_data: Pointer to shared memory containing event processing state.
 *          - global_log_file: FILE pointer for logging messages.
 *
 */
int8_t FM_s8SaveEventDataToStorage(void)
{
    fm_char_t filename[FM_FILENAME_BUFFER_SIZE];
    int32_t result = snprintf(filename, sizeof(filename), "%s/event_data.bin", STORAGE_DIR_PATH);
    if ((result < 0) || (result >= (int32_t)sizeof(filename)))
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format filename");
        return -1;
    }

    int32_t s32FileDesc = open(filename, (int32_t)(O_WRONLY | O_CREAT | O_TRUNC), (mode_t)(S_IRUSR | S_IWUSR));
    if (s32FileDesc == -1)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to open event data file for writing: %s", error_str);
        return -1;
    }

    ssize_t bytes_written;
    size_t total_bytes = 0;
    int16_t s16Processing = ITCOM_s16GetProcessingFlag();

    bytes_written = write(s32FileDesc, &s16Processing, sizeof(int16_t));
    if (bytes_written == -1)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to write processing flag: %s", error_str);
        if (close(s32FileDesc) == -1)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
        }
        return -1;
    }
    total_bytes += bytes_written;

    if (s16Processing)
    {
        ErrorEvent stErrorEvent;
        ITCOM_vGetErrorEvent(&stErrorEvent);
        bytes_written = write(s32FileDesc, &stErrorEvent, sizeof(ErrorEvent));
        if (bytes_written == -1)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to write current event: %s", error_str);
            if (close(s32FileDesc) == -1)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
            }
            return -1;
        }
        total_bytes += bytes_written;
    }

    if (close(s32FileDesc) == -1)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
        return -1;
    }

    log_message(global_log_file, LOG_INFO, "Successfully saved %zu bytes of event data to storage", total_bytes);
    return 0;
}

/**
 * @brief Loads event data from persistent storage during startup.
 *
 * This function retrieves previously saved event processing state from a file
 * and restores it to the system. It's typically called during system initialization
 * to recover from unexpected shutdowns or to continue processing from a known state.
 *
 * The function performs the following operations:
 * 1. Constructs the filename for the event data storage file.
 * 2. Attempts to open the file for reading.
 * 3. If the file exists and is readable:
 *    a. Reads the processing flag.
 *    b. If an event was being processed, reads the event data.
 * 4. Closes the file.
 * 5. Clears the current Event_Queue and resets error counters.
 * 6. If an interrupted event was found, adds it back to the queue for processing.
 * 7. Logs the results of the loading operation.
 *
 * @note This function should be called once during system startup, before
 *       normal event processing begins.
 *
 * @warning This function modifies global state, including:
 *          - shared_data: Updates the event queue and processing state.
 *          - Error_Events: Resets error counters.
 *
 */
void FM_vLoadEventDataFromStorage(void)
{
    fm_char_t filename[FM_FILENAME_BUFFER_SIZE];
    int32_t result = snprintf(filename, sizeof(filename), "%s/event_data.bin", STORAGE_DIR_PATH);
    if ((result < 0) || (result >= (int32_t)sizeof(filename)))
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format filename");
        return;
    }

    int32_t s32FileDesc = open(filename, (int32_t)O_RDONLY);
    if (s32FileDesc == -1)
    {
        if (errno != ENOENT)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to open event data file for reading: %s", error_str);
        }
        return;
    }

    ssize_t bytes_read;
    size_t total_bytes = 0;
    int16_t s16Processing = ITCOM_s16GetProcessingFlag();

    bytes_read = read(s32FileDesc, &s16Processing, sizeof(int16_t));
    if (bytes_read == -1)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to read processing flag: %s", error_str);
        if (close(s32FileDesc) == -1)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
        }
        return;
    }
    total_bytes += bytes_read;

    ErrorEvent current_event;
    if (s16Processing)
    {
        bytes_read = read(s32FileDesc, &current_event, sizeof(ErrorEvent));
        if (bytes_read == -1)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to read current event: %s", error_str);
            if (close(s32FileDesc) == -1)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
            }
            return;
        }
        total_bytes += bytes_read;
    }

    if (close(s32FileDesc) == -1)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to close event data file: %s", error_str);
    }

    log_message(global_log_file, LOG_INFO, "Successfully loaded %zu bytes of event data from storage", total_bytes);

    ITCOM_vSetEventQueueIndx(FM_ZERO_QUEUE_INDEX);
    uint8_t i;
    for (i = FM_ZERO_QUEUE_INDEX; i < DATA_QUEUE_MAX_SIZE; i++)
    {
        ITCOM_vSetEventQueueId(FM_ZERO_EVENT_ID, i);
    }

    fm_vResetErrorEventCounters();

    if (s16Processing)
    {
        int16_t s16Result = ITCOM_s16SetErrorEvent(current_event.Error_Event_ID);
        if (s16Result != 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set error event");
        }
        log_message(global_log_file, LOG_INFO, "Resumed processing of interrupted event: %d", current_event.Error_Event_ID);
    }

    log_message(global_log_file, LOG_INFO, "Event queue cleared and error counters reset on startup");
    log_message(global_log_file, LOG_INFO, "Remaining events from previous run have been cleared");
}

/**
 * @brief Finds the index of the least severe event in the queue.
 *
 * This function scans through the event queue to determine which event
 * has the lowest severity. It's typically used when the event queue is full
 * and a decision needs to be made about which event to potentially replace.
 *
 * @param queue Pointer to the array of unsigned chars representing event IDs.
 * @param size The number of elements in the queue to examine.
 *
 * @return int The index of the least severe event in the queue.
 *
 * The function performs the following operations:
 * 1. Initializes the least severe index to 0 and gets its severity.
 * 2. Iterates through the rest of the queue:
 *    a. Compares the severity of each event to the current least severe.
 *    b. Updates the least severe index if a less severe event is found.
 * 3. Returns the index of the least severe event.
 *
 * @note The function assumes that the Error_Events array is globally accessible
 *       and contains the severity information for each event ID.
 *
 * @warning This function does not modify the queue; it only returns an index.
 *          The caller is responsible for any queue modifications.
 *
 */
uint32_t FM_u32FindLeastSevereEvent(uint8_t *queue, uint32_t size)
{
    uint32_t least_severe_index = 0;
    SeverityType least_severity = Error_Events[queue[0]].Severity;

    uint32_t i;
    for (i = 1; i < (uint32_t)size; i++)
    {
        if (Error_Events[queue[i]].Severity < least_severity)
        {
            least_severe_index = i;
            least_severity = Error_Events[queue[i]].Severity;
        }
    }

    return least_severe_index;
}

/**
 * @brief Prints the current state of the event queue to the log.
 *
 * This function creates a string representation of the current event queue
 * and logs it. It's useful for debugging and monitoring the state of the
 * event processing system.
 *
 * The function performs the following operations:
 * 1. Initializes a string buffer to store the queue representation.
 * 2. Iterates through the event queue in shared memory:
 *    a. Appends each event ID to the string buffer.
 *    b. Separates event IDs with spaces.
 * 3. Logs the resulting string using the log_message function.
 *
 * @note This function does not take any parameters and does not return a value.
 *       It operates on the global shared memory structure.
 *
 * @warning This function assumes the existence of:
 *          - shared_data: A global pointer to the shared memory structure.
 *          - global_log_file: A global file pointer for logging.
 *          - log_message: A function for writing to the log file.
 *
 * Example log output:
 * "Current Event Queue: 1 3 5 2 4", where 1 3 5 2 4 are Event IDs
 */
static void fm_vPrintEventQueue(void)
{
    fm_char_t queue_str[FM_QUEUE_STRING_BUFFER_SIZE] = {0};
    uint32_t str_position = 0;
    uint8_t u8EventId = FM_ZERO_EVENT_ID;
    int8_t s16QueueIndx = FM_INITIAL_QUEUE_INDEX;

    s16QueueIndx = ITCOM_s16GetEventQueueIndx();

    int8_t i;
    for (i = 0; i < (int8_t)s16QueueIndx; i++)
    {
        ITCOM_vGetEventQueueId(&u8EventId, i);
        int32_t written = snprintf(queue_str + str_position, sizeof(queue_str) - str_position, "%d ", u8EventId);

        if (written < 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to format queue string");
            return;
        }

        if (written >= (int32_t)(sizeof(queue_str) - str_position))
        {
            log_message(global_log_file, LOG_WARNING, "Queue string truncated");
            break; /* Exit loop if buffer is full */
        }

        str_position += (uint32_t)written;
    }

    log_message(global_log_file, LOG_INFO, "Current Event Queue: %s", queue_str);
}

/**
 * @brief Resets all error event counters to zero.
 *
 * This function initializes all error event counters to zero, effectively
 * clearing the history of error occurrences. It also resets the processing
 * state of the Fault Management (FM) thread.
 *
 * The function performs the following operations:
 * 1. Iterates through all error events in the Error_Events array:
 *    a. Sets each event's Error_Event_Counter to 0.
 * 2. Resets the processing flag in the FM thread's shared data to 0.
 * 3. Clears the current event data in the FM thread's shared data.
 * 4. Logs a message indicating that all counters have been reset.
 *
 * @note This function should be called during system initialization or when
 *       a complete reset of error history is required, such as after a
 *       system recovery or major state transition.
 *
 * @warning This function modifies global state:
 *          - Error_Events: Resets all event counters.
 *          - shared_data: Modifies the FM thread's processing state.
 *
 */
static void fm_vResetErrorEventCounters(void)
{
    ErrorEvent stErrorEvent;
    void *pvResult;
    int32_t i;
    for (i = 0; i < enTotalEventIds; i++)
    {
        Error_Events[i].Error_Event_Counter = 0;
    }
    ITCOM_vSetErrorProcessingFlag(0);
    pvResult = memset(&stErrorEvent, 0, sizeof(ErrorEvent));
    if (pvResult == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to reset error event");
        return;
    }
    ITCOM_vUpdateCurrentEvent(&stErrorEvent);
    log_message(global_log_file, LOG_INFO, "All error event counters have been reset to 0 and processing state cleared");
}

/**
 * @brief ...
 *
 * This function attempts to...
 *
 * @return
 *
 * The function performs the following operations:
 * Returns the severity of the error event selected by the index input parameter.
 *
 * @note
 *
 * @warning
 *
 */
uint8_t FM_u8GetEventSeverity(uint8_t u8Indx)
{
    uint8_t u8Severity = (uint8_t)enTotalSeverityTypes;
    if (u8Indx < enTotalEventIds)
    {
        u8Severity = Error_Events[u8Indx].Severity;
    }

    return u8Severity;
}

/*** Local Function Implementations ***/

/**
 * @brief Calculates the elapsed time between two timespec structures in milliseconds.
 *
 * This function computes the time difference between a start and end timespec,
 * returning the result in milliseconds. It's useful for measuring durations of
 * operations or time intervals in the program.
 *
 * @param start Pointer to the timespec structure representing the start time.
 * @param end Pointer to the timespec structure representing the end time.
 * @return double The elapsed time in milliseconds.
 *
 * The function performs the following calculation:
 * 1. Computes the difference in seconds and converts to milliseconds.
 * 2. Adds the difference in nanoseconds, converted to milliseconds.
 *
 * The formula used is:
 * elapsed_ms = (end->tv_sec - start->tv_sec) * 1000.0 +
 *              (end->tv_nsec - start->tv_nsec) / 1000000.0
 *
 * @note This function assumes that the end time is later than the start time.
 *       If end is earlier than start, the result may be negative.
 *
 * @warning The function does not check for NULL pointers or validate the
 *          input timespec structures. Ensure valid pointers are passed.
 *
 */
static float64_t fm_f64GetElapsedTimeMs(struct timespec *start, struct timespec *end)
{
    if (start == NULL || end == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL pointer passed to fm_f64GetElapsedTimeMs");
        return 0.0;
    }
    float64_t sec_diff = (float64_t)(end->tv_sec - start->tv_sec);
    float64_t nsec_diff = (float64_t)(end->tv_nsec - start->tv_nsec);
    float64_t ms_per_sec = (float64_t)SEC_TO_MS;
    float64_t ns_per_ms = (float64_t)NSEC_TO_MS;
    return (sec_diff * ms_per_sec) + (nsec_diff / ns_per_ms);
}

/**
 * @brief Collects snapshot data for an error event.
 *
 * This function captures the current state of the system and stores it in the
 * SnapshotData field of the provided ErrorEvent structure. It's used to record
 * relevant system information at the time an error event occurs.
 *
 * @param event Pointer to the ErrorEvent structure where snapshot data will be stored.
 *
 * The function performs the following operations:
 * 1. Locks the shared data mutex to ensure thread-safe access.
 * 2. Calls fm_vCaptureSnapshotData() to populate the SnapshotData structure.
 * 3. Unlocks the shared data mutex.
 * 4. Logs debug information about the captured raw values.
 * 5. Generates a timestamp for the snapshot.
 * 6. Logs debug information about the stored snapshot values.
 *
 * @note This function uses global shared data and logging mechanisms.
 *
 * @warning Ensure that the event pointer is valid before calling this function.
 *          The function does not check for NULL pointers.
 *          Basically, be cautious when calling this function.
 *
 */
static void fm_vSnapshotDataCollection(ErrorEvent *event)
{
    if (event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL event pointer passed to fm_vSnapshotDataCollection");
        return;
    }
    fm_vCaptureSnapshotData(&event->SystemSnapshotData);
    log_message(global_log_file, LOG_DEBUG, "Snapshot_DataCollection: Raw values - VehicleSpeed: %f, GearShiftPosition: %u, ASI_State: %u",
                event->SystemSnapshotData.VehicleSpeed,
                event->SystemSnapshotData.GearShiftPosition,
                event->SystemSnapshotData.ASI_State);
    time_t now = time(NULL);
    size_t time_result = strftime((fm_char_t *)event->SystemSnapshotData.SystemTime,
                                  sizeof(event->SystemSnapshotData.SystemTime),
                                  "%Y-%m-%d %H:%M:%S",
                                  localtime(&now));
    if ((size_t)time_result == (size_t)FM_ZERO_TIMESTAMP_RESULT)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format time string");
    }
    log_message(global_log_file, LOG_DEBUG, "Snapshot_DataCollection: Stored values - VehicleSpeed: %f, GearShiftPosition: %u, ASI_State: %u",
                event->SystemSnapshotData.VehicleSpeed,
                event->SystemSnapshotData.GearShiftPosition,
                event->SystemSnapshotData.ASI_State);
}

/**
 * @brief Rotates the log file to manage file size and maintain log history.
 *
 * This function performs log file rotation to prevent a single log file from growing too large
 * and to maintain a history of log files. It performs the following operations:
 *
 * 1. Closes the current log file.
 * 2. Renames existing log files, shifting them to higher index numbers.
 * 3. Renames the current log file to have the .0 extension.
 *
 * The rotation process works as follows:
 * - The current log file (EVENT_LOG_PATH) becomes EVENT_LOG_PATH.0
 * - EVENT_LOG_PATH.0 becomes EVENT_LOG_PATH.1
 * - EVENT_LOG_PATH.1 becomes EVENT_LOG_PATH.2
 * - And so on, up to MAX_LOG_FILES - 1
 *
 * After rotation, a new log file will be created the next time logging occurs.
 *
 * @note This function assumes global variables:
 *       - log_file: FILE pointer to the current log file
 *       - EVENT_LOG_PATH: String constant for the log file path
 *       - MAX_LOG_FILES: Maximum number of log files to maintain
 *
 * @warning This function does not check for errors during file operations.
 *          It's recommended to add error checking in a production environment.
 *
 */
static void fm_vRotateLogFile(void)
{
    int32_t s32Result = fclose(log_file);
    if (s32Result != 0)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to close log file: %s", error_str);
    }
    log_file = NULL;

    uint8_t i;
    for (i = (uint8_t)(MAX_LOG_FILES - 1U); i > FM_ZERO_QUEUE_INDEX; i--)
    {
        fm_char_t old_name[FM_FILENAME_BUFFER_SIZE], new_name[FM_FILENAME_BUFFER_SIZE];
        int32_t written = snprintf(old_name, sizeof(old_name), "%s.%u", EVENT_LOG_PATH, (uint32_t)i - 1U);
        if ((written < (int32_t)0) || ((uint32_t)written >= sizeof(old_name)))
        {
            log_message(global_log_file, LOG_ERROR, "Failed to format old filename");
            continue;
        }

        written = snprintf(new_name, sizeof(new_name), "%s.%u", EVENT_LOG_PATH, i);
        if ((written < (int32_t)0) || ((uint32_t)written >= sizeof(new_name)))
        {
            log_message(global_log_file, LOG_ERROR, "Failed to format new filename");
            continue;
        }

        int32_t result = rename(old_name, new_name);
        if (result != 0)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to rename log file: %s", error_str);
        }
    }

    fm_char_t backup_name[FM_FILENAME_BUFFER_SIZE];
    int32_t written = snprintf(backup_name, sizeof(backup_name), "%s.0", EVENT_LOG_PATH);
    if (written < 0 || written >= (int32_t)sizeof(backup_name))
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format backup filename");
        return;
    }

    int32_t result = rename(EVENT_LOG_PATH, backup_name);
    if (result != 0)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to create backup log file: %s", error_str);
    }
}

/**
 * @brief Converts an EVENT_ID_t enumeration value to its corresponding string representation.
 *
 * This function takes an event ID of type EVENT_ID_t and returns a constant string
 * that represents the human-readable name of the event. It's useful for logging
 * and debugging purposes, allowing for easy interpretation of event IDs.
 *
 * @param event_id The EVENT_ID_t enumeration value to be converted to a string.
 * @return const char* A pointer to a constant string representing the event ID.
 *                     Returns "UNKNOWN_EVENT_ID" if the event_id doesn't match any known values.
 *
 * @note The function uses a switch statement to map each EVENT_ID_t value to its
 *       corresponding string. This allows for fast lookups and easy maintenance.
 *
 * @warning This function assumes that the EVENT_ID_t enumeration is up-to-date
 *          with all possible event IDs. If new event IDs are added to the
 *          enumeration, this function should be updated accordingly.
 *
 */
static error_string_t fm_GetEventIDString(EVENT_ID_t event_id)
{
    error_string_t result;

    switch (event_id)
    {
    case EVENT_ID_FAULT_MSG_CRC_CHECK:
        result = "EVENT_ID_FAULT_MSG_CRC_CHECK";
        break;
    case EVENT_ID_FAULT_ROLL_COUNT:
        result = "EVENT_ID_FAULT_ROLL_COUNT";
        break;
    case EVENT_ID_FAULT_MSG_TYPE_LENGTH:
        result = "EVENT_ID_FAULT_MSG_TYPE_LENGTH";
        break;
    case EVENT_ID_FAULT_MSG_TIMEOUT:
        result = "EVENT_ID_FAULT_MSG_TIMEOUT";
        break;
    case EVENT_ID_INFO_ACK_LOSS:
        result = "EVENT_ID_INFO_ACK_LOSS";
        break;
    case EVENT_ID_INFO_ACK_UNSUCCESS:
        result = "EVENT_ID_INFO_ACK_UNSUCCESS";
        break;
    case EVENT_ID_FAULT_PRECOND_LIST_ERROR:
        result = "EVENT_ID_FAULT_PRECOND_LIST_ERROR";
        break;
    case EVENT_ID_FAULT_ACTION_LIST_ERROR:
        result = "EVENT_ID_FAULT_ACTION_LIST_ERROR";
        break;
    case EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH:
        result = "EVENT_ID_INFO_VEHICLE_STATUS_MISMATCH";
        break;
    case EVENT_ID_INFO_VEHICLE_STATUS_ERROR:
        result = "EVENT_ID_INFO_VEHICLE_STATUS_ERROR";
        break;
    case EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR:
        result = "EVENT_ID_INFO_VEHICLE_STATUS_INVALID_INFO_ERROR";
        break;
    case EVENT_ID_FAULT_CAL_READBACK_ERROR:
        result = "EVENT_ID_FAULT_CAL_READBACK_ERROR";
        break;
    case EVENT_ID_FAULT_CAL_READBACK_TIMEOUT:
        result = "EVENT_ID_FAULT_CAL_READBACK_TIMEOUT";
        break;
    case EVENT_ID_FAULT_STARTUP_MEM_ERROR:
        result = "EVENT_ID_FAULT_STARTUP_MEM_ERROR";
        break;
    case EVENT_ID_INFO_LOSS_COMM:
        result = "EVENT_ID_INFO_LOSS_COMM";
        break;
    case EVENT_ID_INFO_MSG_LOSS:
        result = "EVENT_ID_INFO_MSG_LOSS";
        break;
    case EVENT_ID_FAULT_SUT_TERM:
        result = "EVENT_ID_FAULT_SUT_TERM";
        break;
    case EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR:
        result = "EVENT_ID_INFO_ACTION_REQ_RANGE_CHECK_ERROR";
        break;
    case EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR:
        result = "EVENT_ID_INFO_ACTION_REQ_ACTION_LIST_ERROR";
        break;
    case EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR:
        result = "EVENT_ID_INFO_ACTION_REQ_PRECOND_LIST_ERROR";
        break;
    case EVENT_ID_INIT_COMPLETE:
        result = "EVENT_ID_INIT_COMPLETE";
        break;
    case EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT:
        result = "EVENT_ID_INFO_ACTION_REQUEST_PROCESS_TIMEOUT";
        break;
    case EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL:
        result = "EVENT_ID_FAULT_ECU_NON_CRITICAL_FAIL";
        break;
    case EVENT_ID_FAULT_ECU_CRITICAL_FAIL:
        result = "EVENT_ID_FAULT_ECU_CRITICAL_FAIL";
        break;
    case EVENT_ID_FAULT_OVERRUN:
        result = "EVENT_ID_FAULT_OVERRUN";
        break;
    case EVENT_ID_FAULT_SM_TRANSITION_ERROR:
        result = "EVENT_ID_FAULT_SM_TRANSITION_ERROR";
        break;
    default:
        result = "UNKNOWN_EVENT_ID";
        break;
    }

    return result;
}

/**
 * @brief Processes an error event with a timeout mechanism.
 *
 * This function handles the processing of an error event, implementing a timeout
 * to prevent indefinite processing. It processes the event in stages and updates
 * the shared data after each stage.
 *
 * @param state Pointer to the EventProcessingState structure containing the current
 *              processing state and event information.
 *
 * The function performs the following operations:
 * 1. Initializes timing measurements for the overall process.
 * 2. Enters a loop to process the event in stages (up to 4 stages).
 * 3. Checks for a timeout (EVENT_PROCESSING_TIMEOUT) during processing.
 * 4. For each stage:
 *    - Measures the start time.
 *    - Calls fm_vProcessErrorEvent() to handle the current stage.
 *    - Updates shared data with the current event and processing status.
 *    - Measures the end time and calculates the stage processing time.
 *    - Logs the stage processing time.
 *    - Checks for interruptions (using a semaphore).
 * 5. After all stages or on timeout, calculates and logs the total processing time.
 *
 * @note This function uses global variables and functions:
 *       - shared_data: For updating shared memory.
 *       - global_log_file: For logging messages.
 *       - EVENT_PROCESSING_TIMEOUT: Constant for the maximum allowed processing time.
 *
 * @warning Ensure that the state pointer is valid before calling this function.
 *          The function does not check for NULL pointers.
 *
 */
static void fm_vProcessErrorEventWithTimeout(EventProcessingState *state)
{
    if (state == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL state pointer passed to fm_vProcessErrorEventWithTimeout");
        return;
    }
    int32_t start_time_result = clock_gettime(CLOCK_MONOTONIC, &state->start_time);
    if (start_time_result != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to get start time");
        return;
    }

    time_t start_time = time(NULL);

    while (state->processing_stage < FM_PROCESSING_STAGES_COMPLETE)
    {
        if (difftime(time(NULL), start_time) > EVENT_PROCESSING_TIMEOUT)
        {
            log_message(global_log_file, LOG_WARNING, "Error event processing timeout for Event ID: %d", state->current_event->Error_Event_ID);
            break;
        }

        struct timespec stage_start, stage_end;

        int32_t stage_start_result = clock_gettime(CLOCK_MONOTONIC, &stage_start);
        if (stage_start_result == (int32_t)-1)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to get stage start time");
            return;
        }

        fm_vProcessErrorEvent(state);

        ITCOM_vUpdateCurrentEvent(state->current_event);
        ITCOM_vSetErrorProcessingFlag(1);

        int32_t stage_end_result = clock_gettime(CLOCK_MONOTONIC, &stage_end);
        if (stage_end_result != 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to get stage end time");
            return;
        }

        float64_t stage_time = fm_f64GetElapsedTimeMs(&stage_start, &stage_end);

        log_message(global_log_file, LOG_INFO, "Event ID: %d, Stage %d processing time: %.2f ms",
                    state->current_event->Error_Event_ID, state->processing_stage, stage_time);

        if (ITCOM_vSemaphoreTryWait() == 0)
        {
            log_message(global_log_file, LOG_INFO, "THRD_FM interrupted, will resume at stage %d", state->processing_stage);
            return;
        }
    }

    int32_t end_time_result = clock_gettime(CLOCK_MONOTONIC, &state->end_time);
    if (end_time_result == (int32_t)-1)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to get end time");
        return;
    }

    state->total_processing_time = fm_f64GetElapsedTimeMs(&state->start_time, &state->end_time);

    if (state->processing_stage >= FM_PROCESSING_STAGES_COMPLETE)
    {
        log_message(global_log_file, LOG_INFO, "Error event processing completed for Event ID: %d, Total processing time: %.2f ms",
                    state->current_event->Error_Event_ID, state->total_processing_time);
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "Error event processing incomplete for Event ID: %d, Total processing time: %.2f ms",
                    state->current_event->Error_Event_ID, state->total_processing_time);
    }
}

/**
 * @brief Logs error events and special events to a file.
 *
 * This function is responsible for logging error events and special events to a designated log file.
 * It handles log file rotation, ensures thread-safe logging, and formats the log entries.
 *
 * @param event Pointer to the ErrorEvent structure containing the event details to be logged.
 * @param special_event_type Pointer to a string specifying a special event type. If NULL, a regular event is logged.
 *
 * The function performs the following operations:
 * 1. Opens the log file if it's not already open.
 * 2. Checks if log rotation is needed based on the current log file size.
 * 3. Generates a timestamp for the log entry.
 * 4. Determines the severity level string based on the event's severity.
 * 5. Formats and writes the log entry, which includes:
 *    - Timestamp
 *    - Special event type (if provided)
 *    - Event ID string
 *    - Severity level
 *    - Error event counter
 *    - Snapshot data (vehicle speed, gear shift position, ASI state)
 * 6. Flushes the file buffer to ensure the entry is written.
 * 7. Updates the current log file size.
 *
 * @note This function uses global variables:
 *       - log_file: FILE pointer to the current log file
 *       - current_log_size: Tracks the size of the current log file
 *       - MAX_LOG_SIZE: Maximum size of a log file before rotation
 *       - EVENT_LOG_PATH: Path to the log file
 *
 * @warning This function assumes that the event pointer is valid and not NULL.
 *          It's the caller's responsibility to ensure this.
 *
 */
static void fm_vEventLogger(const ErrorEvent *event, string_t special_event_type)
{
    int32_t s32Result = 0;
    if (event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL event pointer passed to fm_vEventLogger");
        return;
    }
    if (log_file == NULL)
    {
        log_file = fopen(EVENT_LOG_PATH, "a");
        if (log_file == NULL)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to open %s: %s", EVENT_LOG_PATH, error_str);
            return;
        }
        s32Result = fseek(log_file, 0, SEEK_END);
        if (s32Result != 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to seek to end of file");
            return;
        }
        current_log_size = ftell(log_file);
    }

    if ((uint64_t)current_log_size >= (uint64_t)MAX_LOG_SIZE)
    {
        fm_vRotateLogFile();
        log_file = fopen(EVENT_LOG_PATH, "a");
        if (log_file == NULL)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to open %s after rotation: %s", EVENT_LOG_PATH, error_str);
            return;
        }
        current_log_size = 0;
    }

    fm_char_t timestamp[FM_TIMESTAMP_STRING_LENGTH];
    time_t now = time(NULL);
    struct tm *timeinfo = localtime(&now);
    if (timeinfo == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to get local time");
        return;
    }

    size_t time_result = strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
    if ((size_t)time_result == (size_t)FM_ZERO_TIMESTAMP_RESULT)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to format timestamp");
        return;
    }

    string_t severity_str;
    switch (event->Severity)
    {
    case SEVERITY_CRITICAL:
        severity_str = "Severity_Critical";
        break;

    case SEVERITY_NORMAL:
        severity_str = "Severity_Normal";
        break;

    case SEVERITY_MINOR:
        severity_str = "Severity_Minor";
        break;

    default:
        severity_str = "Unknown";
        break;
    }

    /* Create a log buffer to store the formatted message */
    fm_char_t log_buffer[FM_FORMATTED_MSG_LOG_BUFFER_SIZE];
    int32_t written_chars = 0;
    int32_t total_chars = 0;
    size_t remaining_space = sizeof(log_buffer);

    /* Format the initial part of the message */
    if (special_event_type != NULL)
    {
        if (strncmp(special_event_type, "SKIPPED EVENT", (size_t)FM_SKIPPED_EVENT_COMPARE_SIZE) == 0)
        {
            written_chars = snprintf(log_buffer, remaining_space, "[%s]        SKIPPED EVENT:         ", timestamp);
        }
        else
        {
            written_chars = snprintf(log_buffer, remaining_space, "[%s] SPECIAL EVENT: %-20s ", timestamp, special_event_type);
        }
    }
    else
    {
        written_chars = snprintf(log_buffer, remaining_space, "[%s]    EVENT LOGGED:    ", timestamp);
    }

    if ((written_chars < (int32_t)0) || ((size_t)written_chars >= remaining_space))
    {
        log_message(global_log_file, LOG_ERROR, "Buffer overflow while formatting log message");
        return;
    }

    /* Update remaining space and track total characters */
    total_chars = written_chars;
    remaining_space -= (size_t)written_chars;

    /* Format the event details */
    written_chars = snprintf(log_buffer + (size_t)total_chars, remaining_space,
                             "%-50s Fault-Level = %-20s Error_Event_Counter = %-6d VehicleSpeed = %-10.2f GearShiftPosition = %-6u ASI_State = %u\n",
                             fm_GetEventIDString(event->Error_Event_ID),
                             severity_str,
                             event->Error_Event_Counter,
                             event->SystemSnapshotData.VehicleSpeed,
                             event->SystemSnapshotData.GearShiftPosition,
                             event->SystemSnapshotData.ASI_State);

    if ((written_chars < (int32_t)0) || ((size_t)written_chars >= remaining_space))
    {
        log_message(global_log_file, LOG_ERROR, "Buffer overflow while formatting event details");
        return;
    }

    total_chars += written_chars;

    /* Write the formatted buffer to file using write() with retry for partial writes */
    size_t bytes_to_write = (size_t)total_chars;
    size_t total_written = 0;
    while (total_written < bytes_to_write)
    {
        int32_t bytes_written = (int32_t)write(fileno(log_file),
                                               log_buffer + total_written,
                                               bytes_to_write - total_written);
        if (bytes_written < (int32_t)0)
        {
            error_string_t error_str = strerror(errno);
            log_message(global_log_file, LOG_ERROR, "Failed to write to log file: %s", error_str);
            return;
        }
        total_written += (size_t)bytes_written;
    }

    current_log_size += (uint64_t)total_written;
    s32Result = fflush(log_file);
    if (s32Result != 0)
    {
        error_string_t error_str = strerror(errno);
        log_message(global_log_file, LOG_ERROR, "Failed to flush log file: %s", error_str);
    }
}

/**
 * @brief Processes an error event through multiple stages.
 *
 * This function handles the multi-stage processing of an error event. It executes
 * different actions based on the current processing stage and can be interrupted
 * between stages.
 *
 * @param state Pointer to the EventProcessingState structure containing the current
 *              processing state and event information.
 *
 * The function performs the following operations:
 * 1. Checks if there is a current event to process.
 * 2. Executes a loop that processes the event through up to 4 stages:
 *    - Stage 0: Increments the error counter
 *    - Stage 1: Calls the notification function
 *    - Stage 2: Logs the error event
 *    - Stage 3: Removes the processed event from the queue
 * 3. Increments the processing stage after each step.
 * 4. Checks for interruptions after each stage using a semaphore.
 * 5. Logs the completion or interruption of event processing.
 *
 * @note This function uses global variables and functions:
 *       - shared_data: For accessing shared memory and synchronization primitives.
 *       - global_log_file: For logging messages.
 *       - fm_vIncrementErrorCounter(), fm_vCallNotificationFunction(),
 *         fm_vLogErrorEvent(), remove_processed_event(): Helper functions for each stage.
 *
 * @warning Ensure that the state pointer is valid before calling this function.
 *          The function checks for a NULL current event but not for a NULL state.
 *
 */
static void fm_vProcessErrorEvent(EventProcessingState *state)
{
    if (state->current_event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "fm_vProcessErrorEvent: No current event to process");
        return;
    }

    while (state->processing_stage < FM_PROCESSING_STAGES_COMPLETE)
    {
        switch (state->processing_stage)
        {
        case 0:
            fm_vIncrementErrorCounter(state->current_event);
            break;
        case 1:
            fm_vCallNotificationFunction(state->current_event);
            break;
        case 2:
            fm_vLogErrorEvent(state->current_event);
            break;
        case 3:
            ITCOM_vRemoveProcessedEvent();
            break;
        default:
            /* This case should never be reached due to the while loop condition,
               but is required for MISRA compliance */
            break;
        }
        state->processing_stage++;

        if (ITCOM_vSemaphoreTryWait() == 0)
        {
            log_message(global_log_file, LOG_INFO, "THRD_FM interrupted, will resume at stage %d", state->processing_stage);
            break;
        }
    }

    if (state->processing_stage >= FM_PROCESSING_STAGES_COMPLETE)
    {
        log_message(global_log_file, LOG_INFO, "Error event processing completed for Event ID: %d", state->current_event->Error_Event_ID);
    }
}

/**
 * @brief Increments the error counter for a given error event.
 *
 * This function increases the error counter associated with the specified
 * ErrorEvent by one. It's typically used as part of the error event
 * processing to keep track of how many times a particular error has occurred in the present ASI Operation-Cycle.
 *
 * @param event Pointer to the ErrorEvent structure whose counter should be incremented.
 *
 * @note This function assumes that the Error_Event_Counter is an integer type
 *       that can be safely incremented without overflow checking.
 *
 * @warning This function does not perform any null pointer checks. It's the
 *          caller's responsibility to ensure that a valid ErrorEvent pointer
 *          is provided.
 *
 */
static void fm_vIncrementErrorCounter(ErrorEvent *event)
{
    if (event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL event pointer passed to fm_vIncrementErrorCounter");
        return;
    }
    event->Error_Event_Counter++;
}

/**
 * @brief Calls the notification function associated with an error event.
 *
 * This function checks if a notification function is associated with the given
 * error event and calls it if present. It's typically used as part of the error
 * event processing to trigger any necessary notifications or actions in response to the error.
 * (Ex: Transitioning ASI to a Safe State)
 *
 * @param event Pointer to the ErrorEvent structure containing the notification
 *              function to be called.
 *
 * The function performs the following operations:
 * 1. Checks if the NotificationFunction pointer in the ErrorEvent is not NULL.
 * 2. If a function is present, it calls that function.
 *
 * @note The notification function, if present, is called without any parameters.
 *       If the notification requires event-specific information, it should be
 *       designed to access that information through other means.
 *
 * @warning This function does not perform any null pointer checks on the event
 *          parameter itself. It's the caller's responsibility to ensure that a
 *          valid ErrorEvent pointer is provided.
 *
 */
static void fm_vCallNotificationFunction(ErrorEvent *event)
{
    if (event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL event pointer passed to fm_vCallNotificationFunction");
        return;
    }
    if (event->NotificationFunction != NULL)
    {
        event->NotificationFunction();
    }
}

/**
 * @brief Logs an error event to the event log.
 *
 * This function is responsible for logging the details of an error event
 * to the system's event log. It utilizes the fm_vEventLogger function to
 * perform the actual logging operation.
 *
 * @param event Pointer to the ErrorEvent structure containing the details
 *              of the error event to be logged.
 *
 * The function performs the following operation:
 * 1. Calls fm_vEventLogger with the provided error event and NULL as the
 *    special event type, indicating a standard error event log entry.
 *
 * @note This function serves as a wrapper around fm_vEventLogger, simplifying
 *       the interface for logging standard error events.
 *
 * @warning This function assumes that the event pointer is valid and not NULL.
 *          It's the caller's responsibility to ensure this. No null check
 *          is performed within this function.
 *
 */
static void fm_vLogErrorEvent(ErrorEvent *event)
{
    if (event == NULL)
    {
        log_message(global_log_file, LOG_ERROR, "NULL event pointer passed to fm_vLogErrorEvent");
        return;
    }
    fm_vEventLogger(event, NULL);
}

/**
 * @brief Retrieves the next error event from the event queue.
 *
 * This function attempts to fetch the next error event from the shared event queue.
 * It provides thread-safe access to the queue by using mutex locks.
 *
 * @return ErrorEvent* Pointer to the next ErrorEvent in the queue, or NULL if the queue is empty.
 *
 * The function performs the following operations:
 * 1. Locks the mutex for the shared event queue.
 * 2. Checks if there are any events in the queue (Event_Queue_Index > 0).
 * 3. If an event exists:
 *    - Retrieves the event ID from the first position in the queue.
 *    - Uses this ID to get a pointer to the corresponding ErrorEvent from the Error_Events array.
 *    - Logs debug information about the retrieved event.
 * 4. If no events exist, logs a debug message indicating an empty queue.
 * 5. Unlocks the mutex.
 *
 * @note This function assumes the existence of:
 *       - A global shared data structure (shared_data) containing the event queue and associated mutex.
 *       - A global Error_Events array containing ErrorEvent structures.
 *       - A global logging function (log_message).
 *
 * @warning This function does not remove the event from the queue. The caller is
 *          responsible for calling remove_processed_event() after processing.
 *
 */
static ErrorEvent *fm_GetErrorEvent(void)
{
    ErrorEvent *event = NULL;
    uint8_t u8EventId = FM_ZERO_EVENT_ID;

    if (ITCOM_s16GetEventQueueIndx() > 0)
    {
        ITCOM_vGetEventQueueId(&u8EventId, FM_ZERO_QUEUE_INDEX);
        event = &Error_Events[u8EventId];
        log_message(global_log_file, LOG_DEBUG, "get_errorevent: Retrieved Event ID %d, Severity: %d", u8EventId, event->Severity);
    }
    else
    {
        log_message(global_log_file, LOG_DEBUG, "get_errorevent: No events in queue");
    }

    return event;
}
