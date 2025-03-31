/*****************************************************************************
 * @file process_management.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Header file for process management module defining interfaces for
 *        process control, monitoring and fault recovery services.
 *
 * @details
 * Defines external interfaces for:
 * Process Control:
 * - Parent-child process creation and initialization
 * - Process lifecycle management functions
 * - Signal handler initialization and setup
 * - Parent/child process main loop functions
 * Process Monitoring:
 * - Child process status tracking
 * - Health monitoring interfaces
 * - Recovery mechanism controls
 * Signal Handling:
 * - Signal handler registration
 * - Custom handler interfaces
 * - Signal mask configuration
 * Error Management:
 * - Status codes and return types
 * - Error reporting mechanisms
 * - Event logging interfaces
 * Includes necessary type definitions, data structures and constants required
 * by other modules to interact with process management functionality.
 *
 * @authors Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date August 29, 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/13/2024 | AT     | Initial
 * 10/02/2024 | TP     | Improved Signal Handling, Process Management & Error Reporting
 * 10/04/2024 | TP     | All shared data is saved during graceful shutdown
 * 10/09/2024 | TP     | Multiple ASI_APP restart issues fixed
 * 11/15/2024 | TP     | MISRA & LHP compliance fixes
 * 11/22/2024 | TP     | Cleanup v1.0
 */

#ifndef PROCESS_MANAGEMENT_H
#define PROCESS_MANAGEMENT_H

/*** Include Files ***/
#include "gen_std_types.h"
#include "itcom.h"
#include "storage_handler.h"
#include "thread_management.h"

/*** Definitions Provided to other modules ***/

/*** Type Definitions ***/

/*** Functions Provided to other modules ***/
extern void PROCMANAGEMENT_vSignalHandlerInit(FILE *proc_log_file);
extern pid_t PROCMANAGEMENT_stCreateChildProcess(void);
extern void child_process(DataOnSharedMemory *shared_data, FILE *proc_log_file, enRestartReason start_reason);
extern void parent_process(DataOnSharedMemory *shared_data, FILE *proc_log_file);
extern void setup_child_signal_handlers(void);

/*** Variables Provided to other modules ***/

#endif /* PROCESS_MANAGEMENT_H */
