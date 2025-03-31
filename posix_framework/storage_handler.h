/*****************************************************************************
 * @file storage_handler.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Storage handler module header providing interfaces for persistent storage operations,
 *        data management, and logging infrastructure.
 *
 * @details Core header module defining storage management interfaces including:
 * Storage Management:
 * - File system operations (read/write interfaces)
 * - Storage directory management functions
 * - Data persistence and recovery mechanisms
 * - Parent/Child process storage synchronization
 * Logging Infrastructure:
 * - Multi-level logging system definitions
 * - Thread-safe logging interfaces
 * - Process-specific log file handling
 * - Event logging support
 * File System Interfaces:
 * - Secure file operation definitions
 * - Permission management structures
 * - Path definitions and constants
 * - Storage flags and status codes
 *
 * @authors Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date August 13 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/13/2024 | AT     | Initial
 * 10/04/2024 | TP     | Added save_all_shared_data_to_storage function
 * 11/15/2024 | TP     | MISRA & LHP compliance fixes
 * 11/22/2024 | TP     | Cleanup v1.0
 *
 */

#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

/*** Include Files ***/
#include "gen_std_types.h"
#include "itcom.h"

/*** Definitions Provided to other modules ***/
// File paths for logs and storage
/**
 * @def PARENT_LOG_FILE_PATH
 * @brief File path for the parent process log file.
 *
 * Specifies the location and name of the parent process log file.
 * This file is used to record the status and actions performed in
 * the parent process execution.
 *
 */
#define PARENT_LOG_FILE_PATH "ASI_DATA/LOG/parent_process.log"
/**
 * @def CHILD_LOG_FILE_PATH
 * @brief File path for the child process log file.
 *
 * Specifies the location and name of the child process log file.
 * This file is used to record the status and actions performed in
 * the child process execution.
 *
 */
#define CHILD_LOG_FILE_PATH "ASI_DATA/LOG/child_process.log"
/**
 * @def EVENT_LOG_PATH
 * @brief File path for the event log file.
 *
 * Specifies the location and name of the main event log file.
 * This file is used to record various system events and errors.
 *
 */
#define EVENT_LOG_PATH "ASI_DATA/LOG/Event_Logger.log"

/**
 * @def STORAGE_DIR_PATH
 * @brief Directory path for persistent storage.
 *
 * Defines the directory where persistent data, including log files and other
 * application-specific data, is stored. This path is used for creating and
 * accessing various storage files used by the application.
 *
 */
#define STORAGE_DIR_PATH "ASI_DATA/STORAGE"
#define PARENT_STORAGE_PATH "ASI_DATA/STORAGE/parent_storage.bin"
#define CHILD_STORAGE_PATH "ASI_DATA/STORAGE/child_storage.bin"

#define STORAGE_FILE_PARENT          (1)
#define STORAGE_FILE_CHILD           (2)

#define STORAGE_SUCCESS              (0)
#define STORAGE_ERROR                (-1)

/*** Type Definitions ***/

/* Type definitions for MISRA compliance */
typedef int           ret_status_t;         /* Return status type */
typedef int           file_desc_t;          /* File descriptor type */
typedef int           valid_status_t;       /* Validation status type */
typedef int           storage_flags_t;      /* Storage flags type */
typedef int           log_level_t;          /* Log level type */
typedef const char*   str_const_t;          /* Constant string type */
typedef char          str_t;                /* String type */

/*** Functions Provided to other modules ***/
extern void log_message(FILE *storage_log_file, const log_level_t level, const str_const_t format, ...);
extern ret_status_t create_storage_directory(void);
extern void write_shared_data_to_file(str_const_t filename, DataOnSharedMemory *data);
extern ret_status_t compare_and_load_storage(DataOnSharedMemory *const shared_data);
extern ret_status_t initialize_storage_files(const storage_flags_t storage_flags);
extern void save_all_shared_data_to_storage(DataOnSharedMemory *shared_data);

/*** Variables Provided to other modules ***/
extern FILE *global_log_file;

#endif /* STORAGE_HANDLER_H */
