/*****************************************************************************
 * @file main.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Main entry point for the ASI application providing process management and
 *        initialization services for safety-critical operations.
 *
 * @details
 * Core process management module implementing parent-child architecture for the ASI
 * system. The parent process handles initialization, monitoring and fault tolerance
 * while the child process executes the safety-critical application logic.
 *
 * @authors Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date August 29, 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/29/2024 | AT     | Initial Implementation
 * 10/02/2024 | TP     | Refactoring v1.0
 * 11/13/2024 | TP     | MISRA & LHP compliance fixes
 * 11/22/2024 | TP     | Cleanup v1.0
 */

/*** Include Files ***/
#include "gen_std_types.h"
#include "itcom.h"
#include "action_request_approver.h"
#include "fault_manager.h"
#include "icm.h"
#include "memory_test.h"
#include "process_management.h"
#include "storage_handler.h"
#include "thread_management.h"
#include "state_machine.h"
#include "start_up_test.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

/*** Module Definitions ***/

/*** Internal Variables ***/

/*** External Variables ***/

/**
 * @brief Main entry point for the ASI application that initializes and manages the parent-child process architecture
 *
 * @details
 * This function performs the following key operations:
 * - Opens and initializes logging for parent process
 * - Creates/verifies storage directory and files
 * - Initializes shared memory and signal handlers
 * - Creates child process and manages parent/child execution paths
 * - Handles graceful shutdown and cleanup
 *
 * The function implements fault tolerance through:
 * - Storage file validation and recovery
 * - Child process monitoring and restart capabilities
 * - Comprehensive error handling and logging
 * - Coordinated resource cleanup
 *
 * @return int Returns 0 on successful execution, 1 on critical errors
 *
 * @note The parent process maintains monitoring of the child process and can
 *       restart it if abnormal termination is detected
 */
int main(void)
{
    FILE *parent_log_file = NULL;
    pid_t child_process_id;
    enRestartReason restart_reason = enHardRestart;

    /* Open the parent process log file */
    parent_log_file = fopen(PARENT_LOG_FILE_PATH, "w");
    if (parent_log_file == NULL)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to open parent log file: %s", strerror(errno));
        return 1; /* Exit on failure to open log file */
    }
    global_log_file = parent_log_file;

    (void)log_message(parent_log_file, LOG_INFO, "Parent process started");

    /* Create storage directory if it doesn't exist */
    if (create_storage_directory() == -1)
    {
        (void)log_message(parent_log_file, LOG_ERROR, "Failed to create or verify storage directory");
        (void)fclose(parent_log_file);
        return 1; /* Exit on failure to create or verify directory */
    }

    /* Check for existence of storage files */
    bool parent_exists = (access(PARENT_STORAGE_PATH, F_OK) == 0) ? true : false;
    bool child_exists = (access(CHILD_STORAGE_PATH, F_OK) == 0) ? true : false;

    /* Handle cases based on the existence of storage files */
    if (!parent_exists && !child_exists)
    {
        (void)log_message(parent_log_file, LOG_INFO, "Both Parent and Child storage files do not exist.");
        if (initialize_storage_files(0) == -1)
        {
            (void)log_message(parent_log_file, LOG_ERROR, "Failed to initialize storage files");
            (void)fclose(parent_log_file);
            return 1; /* Exit on failure to initialize storage files */
        }
        (void)log_message(parent_log_file, LOG_INFO, "Both storage files created");
    }
    else if (!parent_exists)
    {
        (void)log_message(parent_log_file, LOG_INFO, "Parent storage file does not exist.");
        if (initialize_storage_files(STORAGE_FILE_PARENT) == -1)
        {
            (void)log_message(parent_log_file, LOG_ERROR, "Failed to initialize Parent storage file");
            (void)fclose(parent_log_file);
            return 1; /* Exit on failure to initialize parent storage file */
        }
        (void)log_message(parent_log_file, LOG_INFO, "Parent storage file created");
    }
    else if (!child_exists)
    {
        (void)log_message(parent_log_file, LOG_INFO, "Child storage file does not exist.");
        if (initialize_storage_files(STORAGE_FILE_CHILD) == -1)
        {
            (void)log_message(parent_log_file, LOG_ERROR, "Failed to initialize Child storage file");
            (void)fclose(parent_log_file);
            return 1; /* Exit on failure to initialize child storage file */
        }
        (void)log_message(parent_log_file, LOG_INFO, "Child storage file created");
    }
    else
    {
        (void)log_message(parent_log_file, LOG_INFO, "Parent and Child storage files already exist");
    }

    /* Initialize shared memory and signal handlers */
    ITCOM_vSharedMemoryInit(parent_log_file, restart_reason);
    PROCMANAGEMENT_vSignalHandlerInit(parent_log_file);

    /* Fork to create child process */
    child_process_id = PROCMANAGEMENT_stCreateChildProcess();
    if (child_process_id < 0)
    {
        (void)log_message(parent_log_file, LOG_ERROR, "fork failed: %s", strerror(errno));
        (void)fclose(parent_log_file);
        return 1; /* Exit on failure to create child process */
    }
    else if (child_process_id == 0)
    {
        /* Child process logic */
        (void)fclose(parent_log_file);
        FILE *child_log_file = fopen(CHILD_LOG_FILE_PATH, "w");
        if (child_log_file == NULL)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to open child log file: %s", strerror(errno));
            return 1; /* Exit on failure to open log file */
        }
        global_log_file = child_log_file;
        (void)log_message(child_log_file, LOG_INFO, "Child process started");

        /* Set up signal handlers for the child process */
        setup_child_signal_handlers();

        /* Execute child process logic */
        ITCOM_vChildProcessWrapper(child_log_file, restart_reason);

        (void)fclose(child_log_file);
        return 0; /* Exit successfully after child process logic */
    }
    else
    {
        /* Parent process logic */
        (void)log_message(parent_log_file, LOG_INFO, "Parent process continuing. Child PID: %ld", (long int)child_process_id);

        /* Execute parent process logic */
        ITCOM_vParentdProcessWrapper(parent_log_file);
        (void)log_message(parent_log_file, LOG_INFO, "Parent process cleaning up...");

        /* Save event-related data to storage before exiting */
        (void)FM_s8SaveEventDataToStorage();
        (void)log_message(parent_log_file, LOG_INFO, "Event data saved to storage");

        /* Close the event logger */
        (void)FM_vCloseEventLogger();
        (void)log_message(parent_log_file, LOG_INFO, "Event logger closed");

        (void)log_message(parent_log_file, LOG_INFO, "Parent process exited successfully");
        (void)fclose(parent_log_file);
    }

    /* Restore the main thread's signal mask before exiting */
    (void)restore_main_thread_sigmask();
    ITCOM_vCleanResources();

    return 0; /* Exit successfully */
}
