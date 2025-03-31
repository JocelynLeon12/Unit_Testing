/*****************************************************************************
* @file thread_management.c
*****************************************************************************
* Project Name: Sonatus Automator Safety Interlock(ASI)
*
* @brief Headers for the thread management module providing real-time thread control, 
         monitoring, synchronization and fault detection services.
*
* @details
* Core module implementing sophisticated thread management including:
* Thread Control:
* - Real-time thread creation and scheduling using POSIX timers
* - Thread prioritization and execution timing
* - Thread synchronization via mutexes and semaphores
* - Graceful thread termination
* Monitoring & Safety:
* - Thread health monitoring and fault detection
* - Execution timing measurement and overrun detection
* - Automatic thread restart on failures
* - Thread crash recovery
* Resource Management:
* - Timer setup and management
* - Mutex/semaphore lifecycle control
* - Signal mask configuration
* - Thread status tracking
* Fault Tolerance:
* - Thread timing statistics
* - Overrun detection and reporting
* - Abnormal termination handling
* - Recovery mechanisms
*
* @authors Tusar Palauri (TP), Alejandro Tollola (AT)
* @date August 29, 2024
*
* Version History:
* ---------------
* Date       | Author | Description
* -----------|--------|-------------
* 08/13/2024 | AT     | Initial
* 10/02/2024 | TP     | Improved Signal Handling, Thread Management & Error Reporting
* 10/04/2024 | TP     | All shared data is saved during graceful shutdown
* 10/09/2024 | TP     | Multiple ASI_APP restart issues fixed
* 11/15/2024 | TP     | MISRA & LHP compliance fixes
* 11/22/2024 | TP     | Cleanup v1.0
*/

#ifndef THREAD_MANAGEMENT_H
#define THREAD_MANAGEMENT_H

/*** Include Files ***/
#include "gen_std_types.h"
#include "itcom.h"

#include "storage_handler.h"

/*** Definitions Provided to other modules ***/
#define THREAD_OVERRUN_THRESHOLD_FACTOR      (1.2f)

/*** Type Definitions ***/

/* Enumerations */
enum thread_status_code {
    THREAD_STATUS_SUCCESS     = 0,           /* Operation successful */
    THREAD_STATUS_AGAIN       = EAGAIN,      /* Resource temporarily unavailable, try again */
    THREAD_STATUS_NOMEM       = ENOMEM,      /* Not enough memory */
    THREAD_STATUS_PERM        = EPERM,       /* Operation not permitted */
    THREAD_STATUS_INVAL       = EINVAL,      /* Invalid argument */
    THREAD_STATUS_BUSY        = EBUSY,       /* Device or resource busy */
    THREAD_STATUS_EXIST       = EEXIST,      /* Object already exists */
    THREAD_STATUS_DEADLK      = EDEADLK,     /* Resource deadlock would occur */
    THREAD_STATUS_FAULT       = EFAULT,      /* Bad address */
    THREAD_STATUS_CANCELED    = ECANCELED,   /* Operation canceled */
    THREAD_STATUS_INTR        = EINTR,       /* Interrupted system call */
    THREAD_STATUS_NOTSUP      = ENOTSUP,     /* Operation not supported */
    THREAD_STATUS_OVERFLOW    = EOVERFLOW,   /* Value too large */
    THREAD_STATUS_RANGE       = ERANGE       /* Result too large */
};


/* Structures */
typedef struct {
    thread_name_t name;
    thread_priority_t priority;
    thread_period_t periodicity;
    sem_t *thread_sem; 
} thread_info_t;

typedef struct {
    thread_status_code_t abnormal_terminations;
    time_t last_termination_time;
    sigjmp_buf context;
} thread_status_t;

typedef enum {
	enThread_CCU = 0,
	enThread_FM,
	enThread_STM,
	enThread_ICM_RX,
	enThread_ICM_TX,
	enThread_ARA,
	enThread_CRV,
	enThread_SD,
	enTotalThreads
}thread_label_t;

typedef struct {
    sig_num_t sig_number;
    sig_name_t signal_name;
} signal_info_t;

typedef struct {
    struct timespec start_time;
    struct timespec end_time;
    long last_execution_time_ms;
    uint32_t overrun_count;
    bool is_executing;
    sem_t *thread_sem;
} thread_timing_t;

/*** Functions Provided to other modules ***/
extern void set_thread_exit(sig_atomic_t value);
extern sig_atomic_t get_thread_exit(void);
extern void destroy_mutexes_and_sems(DataOnSharedMemory *shared_data);
extern sig_atomic_t get_abnormal_termination(void);
extern void set_thread_crashed(sig_atomic_t value);
extern void init_mutexes_and_sems(DataOnSharedMemory *shared_data);
extern void destroy_timers(void);
extern thread_status_code_t start_threads(DataOnSharedMemory *shared_data, FILE *thread_mgmt_log_file);
extern void monitor_threads(DataOnSharedMemory *shared_data);
extern void handle_thread_termination(DataOnSharedMemory *shared_data);
extern void initiate_graceful_shutdown(DataOnSharedMemory *shared_data);
extern thread_name_t get_current_thread_name(void);
extern void restore_main_thread_sigmask(void);
extern sig_name_t get_signal_name(sig_num_t sig_number);

/*** Variables Provided to other modules ***/


#endif /* THREAD_MANAGEMENT_H */
