/*****************************************************************************
 * @file thread_management.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Thread management module providing real-time thread control, monitoring,
 *        synchronization and fault detection services.
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

/*** Include Files ***/
#include "thread_management.h"
#include "process_management.h"

/*** Module Definitions ***/
#define THRD_CCU_PRIORITY              (90)
#define THRD_FM_PRIORITY               (80)
#define THRD_STM_PRIORITY              (80)
#define THRD_ICM_RX_PRIORITY           (70)
#define THRD_ICM_TX_PRIORITY           (70)
#define THRD_ARA_PRIORITY              (60)
#define THRD_CRV_PRIORITY              (50)
#define THRD_SD_PRIORITY               (40)

#define THRD_CCU_PERIOD_25MS           (25)
#define THRD_FM_PERIOD_25MS            (25)
#define THRD_STM_PERIOD_50MS           (50)
#define THRD_ICM_RX_PERIOD_50MS        (50)
#define THRD_ICM_TX_PERIOD_50MS        (50)
#define THRD_ARA_PERIOD_50MS           (50)
#define THRD_CRV_PERIOD_50MS           (50)
#define THRD_SD_PERIOD_200MS           (200)

#define SEC_TO_MS                      (1000U)               /* Seconds to milliseconds */
#define NSEC_TO_MS                     (1000000U)            /* Nanoseconds to milliseconds */


/*** Internal Types ***/

/*** Local Function Prototypes ***/
static generic_ptr_t thread_function(generic_ptr_t arg);
static generic_ptr_t thread_function_wrapper(generic_ptr_t arg);
static void THRD_CycleCountUpdater_20ms(generic_ptr_t arg);
static void THRD_StateMachine_50ms(generic_ptr_t arg);
static void THRD_InterfaceCommManager_Rx_50ms(generic_ptr_t arg);
static void THRD_ActionRequestApprover_50ms(generic_ptr_t arg);
static void THRD_InterfaceCommManager_Tx_50ms(generic_ptr_t arg);
static void THRD_FaultManager_25ms(generic_ptr_t arg);
static void THRD_SystemDiagnostic_200ms(generic_ptr_t arg);
static void THRD_CalibrationReadbackVerification_50ms(generic_ptr_t arg);
static void timer_handler(union sigval sv);
static timer_status_t setup_timer(timer_t *p_timer_id, sem_t *sem, timer_period_t period_ms);
static void report_abnormal_termination(thread_id_t thread_id, sig_num_t signal_number);
static void init_thread_timing(void);
static void start_thread_execution_timing(thread_label_t thread_id);
static void end_thread_execution_timing(thread_label_t thread_id);

/*** External Variables ***/

/*** Internal Variables ***/
static thread_info_t thread_info[enTotalThreads] = {
    [enThread_CCU]    = {"THRD_CCU",    THRD_CCU_PRIORITY,    THRD_CCU_PERIOD_25MS,    NULL},
    [enThread_FM]     = {"THRD_FM",     THRD_FM_PRIORITY,     THRD_FM_PERIOD_25MS,     NULL},
    [enThread_STM]    = {"THRD_STM",    THRD_STM_PRIORITY,    THRD_STM_PERIOD_50MS,    NULL},
    [enThread_ICM_RX] = {"THRD_ICM_RX", THRD_ICM_RX_PRIORITY, THRD_ICM_RX_PERIOD_50MS, NULL},
    [enThread_ICM_TX] = {"THRD_ICM_TX", THRD_ICM_TX_PRIORITY, THRD_ICM_TX_PERIOD_50MS, NULL},
    [enThread_ARA]    = {"THRD_ARA",    THRD_ARA_PRIORITY,    THRD_ARA_PERIOD_50MS,    NULL},
    [enThread_CRV]    = {"THRD_CRV",    THRD_CRV_PRIORITY,    THRD_CRV_PERIOD_50MS,    NULL},
    [enThread_SD]     = {"THRD_SD",     THRD_SD_PRIORITY,     THRD_SD_PERIOD_200MS,    NULL},
};

static pthread_t threads[enTotalThreads];
static thread_status_t thread_status_info[enTotalThreads];
static timer_t stTimerCCU, stTimerSTM, stTimerICM_RX, stTimerARA, stTimerICM_TX, stTimerFM, stTimerSD, stTimerCRV;
static sigset_t main_thread_sigmask;
static volatile sig_atomic_t thread_crashed = 0;
static volatile sig_atomic_t thread_exit_flag = 0;
static volatile sig_atomic_t thread_abnormal_termination = 0;
static thread_timing_t thread_timing[enTotalThreads];

/*** Functions Provided to other modules ***/

/*** Getter & Setter functions ***/

/**
 * @brief Sets the global thread exit flag
 *
 * This function controls the graceful shutdown of threads by setting a global atomic flag
 * that indicates whether threads should exit their main execution loops.
 *
 * @param value The value to set:
 *              - 1: Signals all threads to exit their execution loops
 *              - 0: Allows threads to continue execution
 *
 */
void set_thread_exit(sig_atomic_t value)
{
    thread_exit_flag = value;
}

/**
 * @brief Retrieves the current state of the global thread exit flag
 *
 * This function checks whether threads have been signaled to exit their execution loops
 * by returning the current value of the global thread exit flag.
 *
 * @return sig_atomic_t Current value of the thread exit flag:
 *         - 1: Threads should exit their execution loops
 *         - 0: Threads should continue normal execution
 *
 */
sig_atomic_t get_thread_exit(void)
{
    return thread_exit_flag;
}

/**
 * @brief Sets the abnormal termination flag for the thread management system
 *
 * This function indicates that a thread has terminated abnormally, requiring
 * intervention from the thread management system for potential recovery or
 * system shutdown.
 *
 * @param value The value to set:
 *              - 1: Indicates abnormal termination has occurred
 *              - 0: Resets the abnormal termination flag
 *
 */
static void set_abnormal_termination(sig_atomic_t value)
{
    thread_abnormal_termination = value;
}

/**
 * @brief Checks if any thread has terminated abnormally
 *
 * This function returns the current state of the abnormal termination flag,
 * indicating whether any thread in the system has experienced an abnormal
 * termination that requires handling.
 *
 * @return sig_atomic_t Current state of the abnormal termination flag:
 *         - 1: Abnormal termination has occurred
 *         - 0: No abnormal terminations detected
 *
 */
sig_atomic_t get_abnormal_termination(void)
{
    return thread_abnormal_termination;
}

/**
 * @brief Sets the thread crashed flag in the thread management system
 *
 * This function indicates that a thread has crashed due to an unhandled exception,
 * signal, or other critical error condition. It triggers the crash recovery
 * mechanism in the thread management system.
 *
 * @param value The value to set:
 *              - 1: Indicates a thread has crashed
 *              - 0: Resets the thread crashed flag
 *
 */
void set_thread_crashed(sig_atomic_t value)
{
    thread_crashed = value;
}

/**
 * @brief Checks if any thread has crashed
 *
 * This function returns the current state of the thread crashed flag, indicating
 * whether any thread in the system has experienced a crash condition requiring
 * immediate attention.
 *
 * @return sig_atomic_t Current state of the thread crashed flag:
 *         - 1: A thread crash has occurred
 *         - 0: No thread crashes detected
 *
 */
static sig_atomic_t get_thread_crashed(void)
{
    return thread_crashed;
}

/**
 * @brief Retrieves the thread ID of the most recently crashed thread
 *
 * This function returns the pthread_t identifier of the thread that most recently
 * crashed in the system. This information is used by the thread management system
 * to identify and potentially restart the crashed thread.
 *
 * @return pthread_t The thread ID of the crashed thread
 *         Returns 0 if no thread has crashed or if the crashed thread ID is not available
 *
 */
static pthread_t get_crashed_thread_id(void)
{
    static pthread_t thrd_mgmt_crashed_thread_id = 0;
    return thrd_mgmt_crashed_thread_id;
}

/**
 * @brief Initializes the signal handling infrastructure for the thread management system
 *
 * This function configures the initial signal handling setup for all threads in the system
 * by establishing a comprehensive signal mask for the main thread that will be inherited
 * by child threads. It's a critical initialization step that must be called before any
 * threads are created.
 *
 * The function performs the following operations:
 * 1. Creates a new signal mask that blocks all signals
 *    - Uses sigfillset() to create a full signal mask
 *    - Ensures consistent signal handling across threads
 *
 * 2. Saves the current signal mask of the main thread
 *    - Stores the original mask in main_thread_sigmask
 *    - Allows restoration of original signal handling if needed
 *
 * 3. Applies the new signal mask to the main thread
 *    - Uses pthread_sigmask() with SIG_SETMASK operation
 *    - Ensures all subsequently created threads inherit this mask
 *
 * Error Handling:
 * - Validates signal set creation (sigfillset)
 * - Validates signal mask application (pthread_sigmask)
 * - Logs detailed error messages on failure
 * - Terminates program with exit(1) on critical failures
 *
 * @note This function must be called from the main thread before creating any child threads
 * @note All subsequently created threads will inherit the signal mask set here
 *
 * @warning Failure to call this function before thread creation may result in undefined
 *          signal handling behavior across threads
 * @warning Program termination occurs if signal handling setup fails
 *
 * Signal Mask Effects:
 * - All signals are initially blocked
 * - Specific signals can be unblocked per thread as needed
 * - Ensures predictable signal handling in multi-threaded environment
 *
 */
static void init_thread_signal_handling(void)
{
    sigset_t block_mask;
    int sig_fill_result = sigfillset(&block_mask);

    if (sig_fill_result != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to fill signal set");
        exit(1);
    }

    /* Save the current signal mask (should be called in main thread) */
    if (pthread_sigmask(SIG_SETMASK, &block_mask, &main_thread_sigmask) != 0)
    {
        error_string_t error_str = strerror(errno);
        if (error_str != NULL)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set initial signal mask: %s", error_str);
        }
        else
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set initial signal mask with unknown error");
        }
        exit(1);
    }
}

/**
 * @brief Converts a signal number to its corresponding human-readable name
 *
 * This function performs a lookup operation to convert POSIX signal numbers into their
 * corresponding string names using a static lookup table. It supports all standard
 * signals used in the ASI system's signal handling infrastructure.
 *
 * Supported signals include:
 * - Process Control: SIGTERM, SIGINT
 * - Fatal Errors: SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT
 * - System Signals: SIGSYS, SIGQUIT, SIGXCPU, SIGXFSZ
 * - Resource Signals: SIGPIPE, SIGTRAP, SIGALRM
 * - Other Signals: SIGHUP, SIGPWR, SIGPOLL, SIGSTKFLT
 *
 * @param sig_number Signal number to convert [type: sig_num_t]
 *                   Must be a valid POSIX signal number
 *
 * @return const char* String representation of the signal name
 *         - Returns the corresponding signal name if found in lookup table
 *         - Returns "Unknown" if the signal number is not recognized
 *
 * @note This function is primarily used for logging and debugging purposes
 * @note The returned string points to static memory and should not be modified
 *
 */
sig_name_t get_signal_name(sig_num_t sig_number)
{
    static const signal_info_t SIGNAL_LOOKUP_TABLE[] = {
        {SIGTERM, "SIGTERM"},
        {SIGINT, "SIGINT"},
        {SIGSEGV, "SIGSEGV"},
        {SIGBUS, "SIGBUS"},
        {SIGFPE, "SIGFPE"},
        {SIGILL, "SIGILL"},
        {SIGABRT, "SIGABRT"},
        {SIGSYS, "SIGSYS"},
        {SIGQUIT, "SIGQUIT"},
        {SIGXCPU, "SIGXCPU"},
        {SIGXFSZ, "SIGXFSZ"},
        {SIGPIPE, "SIGPIPE"},
        {SIGTRAP, "SIGTRAP"},
        {SIGALRM, "SIGALRM"},
        {SIGHUP, "SIGHUP"},
        {SIGPWR, "SIGPWR"},
        {SIGPOLL, "SIGPOLL"},
        {SIGSTKFLT, "SIGSTKFLT"}};

    const size_t signal_table_size = sizeof(SIGNAL_LOOKUP_TABLE) / sizeof(SIGNAL_LOOKUP_TABLE[0]);
    size_t i;
    for (i = 0; i < signal_table_size; i++)
    {
        if (SIGNAL_LOOKUP_TABLE[i].sig_number == sig_number)
        {
            return SIGNAL_LOOKUP_TABLE[i].signal_name;
        }
    }
    return "Unknown";
}

/**
 * @brief Initializes all mutexes and semaphores in the shared memory structure
 *
 * This function performs comprehensive initialization of all synchronization primitives
 * used for inter-thread and inter-process communication in the ASI system. It sets up
 * both process-shared mutexes and semaphores for each module's data protection.
 *
 * @param shared_data Pointer to the shared memory structure containing all mutex and
 *                   semaphore objects for the ASI system [type: DataOnSharedMemory*]
 *
 * Initializes mutexes for the following modules:
 * - CCU (Cycle Count Updater)
 * - STM (State Machine)
 * - ICM_RX (Interface Communication Manager - Receive)
 * - ARA (Action Request Approver)
 * - ICM_TX (Interface Communication Manager - Transmit)
 * - FM (Fault Manager)
 * - SD (System Diagnostic)
 * - CRV (Calibration Readback Verification)
 * - Common Data
 *
 * Initializes semaphores for:
 * - All above modules with initial value 0
 * - Process-shared attribute enabled
 *
 * Error Handling:
 * - Validates all mutex and semaphore initialization operations
 * - Performs cascading cleanup on any initialization failure
 * - Logs detailed error messages including errno information
 * - Returns immediately if any critical initialization fails
 *
 * Mutex Attributes:
 * - PTHREAD_PROCESS_SHARED for inter-process synchronization
 * - Standard error checking enabled
 * - Default priority inheritance
 *
 * Semaphore Attributes:
 * - Process-shared (value 1 in sem_init)
 * - Initial value 0
 * - Used for thread scheduling synchronization
 *
 * Resource Cleanup:
 * - Destroys mutex attributes after initialization
 * - Destroys all initialized resources on failure
 * - Maintains system integrity during partial initialization
 *
 * @note Must be called after shared memory allocation but before thread creation
 * @note All initialized primitives must be destroyed using destroy_mutexes_and_sems()
 *
 * @warning Failing to initialize these synchronization primitives will result in
 *          undefined behavior in thread and process synchronization
 * @warning This function is not thread-safe and should only be called during
 *          system initialization
 *
 */
void init_mutexes_and_sems(DataOnSharedMemory *shared_data)
{
    pthread_mutexattr_t mutex_attr;
    int ret_val;

    ret_val = pthread_mutexattr_init(&mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize mutex attributes: %s", strerror(ret_val));
        return;
    }

    ret_val = pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to set mutex process shared attribute: %s", strerror(ret_val));
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_CCU.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize CCU mutex: %s", strerror(ret_val));
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_STM.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize STM mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_ICM_RX.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ICM_RX mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_ARA.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ARA mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_ICM_TX.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ICM_TX mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_FM.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize FM mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_SD.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize SD mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThreadsCommonData.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize common data mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = pthread_mutex_init(&shared_data->stThread_CRV.mutex, &mutex_attr);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize CRV mutex: %s", strerror(ret_val));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_CCU.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize CCU semaphore: %s", strerror(errno));
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_STM.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize STM semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_ICM_RX.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ICM_RX semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_ARA.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ARA semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_RX.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_ICM_TX.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize ICM_TX semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_RX.sem);
        (void)sem_destroy(&shared_data->stThread_ARA.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_FM.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize FM semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_RX.sem);
        (void)sem_destroy(&shared_data->stThread_ARA.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_TX.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_SD.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize SD semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_RX.sem);
        (void)sem_destroy(&shared_data->stThread_ARA.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_TX.sem);
        (void)sem_destroy(&shared_data->stThread_FM.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    ret_val = sem_init(&shared_data->stThread_CRV.sem, 1, 0);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize CRV semaphore: %s", strerror(errno));
        (void)sem_destroy(&shared_data->stThread_CCU.sem);
        (void)sem_destroy(&shared_data->stThread_STM.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_RX.sem);
        (void)sem_destroy(&shared_data->stThread_ARA.sem);
        (void)sem_destroy(&shared_data->stThread_ICM_TX.sem);
        (void)sem_destroy(&shared_data->stThread_FM.sem);
        (void)sem_destroy(&shared_data->stThread_SD.sem);
        (void)pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
        (void)pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
        (void)pthread_mutexattr_destroy(&mutex_attr);
        return;
    }

    (void)pthread_mutexattr_destroy(&mutex_attr);
}

/**
 * @brief Destroys all mutexes and semaphores in the shared memory structure
 *
 * This function performs systematic cleanup of all synchronization primitives
 * in the ASI system's shared memory, ensuring proper resource deallocation during
 * system shutdown or cleanup.
 *
 * @param shared_data Pointer to the shared memory structure containing all mutex and
 *                   semaphore objects to be destroyed [type: DataOnSharedMemory*]
 *
 * Destroys mutexes for the following modules in order:
 * - CCU (Cycle Count Updater)
 * - STM (State Machine)
 * - ICM_RX (Interface Communication Manager - Receive)
 * - ARA (Action Request Approver)
 * - ICM_TX (Interface Communication Manager - Transmit)
 * - FM (Fault Manager)
 * - SD (System Diagnostic)
 * - Common Data
 * - CRV (Calibration Readback Verification)
 *
 * Destroys semaphores for:
 * - All above modules in the same order as mutexes
 *
 * Error Handling:
 * - Attempts to destroy all primitives even if some destructions fail
 * - Logs each destruction failure separately with errno information
 * - Continues execution after individual failures to ensure maximum cleanup
 * - Does not exit on errors to allow system shutdown to proceed
 *
 * Cleanup Behavior:
 * - Continues attempting to destroy remaining primitives after any failure
 * - Ensures maximum possible resource cleanup
 * - Logs all failures for diagnostic purposes
 * - Handles each primitive independently to maximize cleanup success
 *
 * Resource Management:
 * - Uses pthread_mutex_destroy() for mutexes
 * - Uses sem_destroy() for semaphores
 * - Handles each destruction atomically
 * - Logs status of each destruction operation
 *
 * @note Should be called during system shutdown or cleanup phase
 * @note All threads should be terminated before calling this function
 * @note Complements init_mutexes_and_sems() initialization function
 *
 * @warning Attempting to use synchronization primitives after destruction will result
 *          in undefined behavior
 * @warning This function should only be called when no threads are actively using
 *          any of the synchronization primitives
 * @warning Failing to call this function during cleanup may result in resource leaks
 *
 */
void destroy_mutexes_and_sems(DataOnSharedMemory *shared_data)
{
    int ret_val;

    ret_val = pthread_mutex_destroy(&shared_data->stThread_CCU.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy CCU mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_STM.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy STM mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_ICM_RX.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ICM_RX mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_ARA.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ARA mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_ICM_TX.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ICM_TX mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_FM.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy FM mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_SD.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy SD mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThreadsCommonData.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy common data mutex: %s", strerror(ret_val));
    }

    ret_val = pthread_mutex_destroy(&shared_data->stThread_CRV.mutex);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy CRV mutex: %s", strerror(ret_val));
    }

    ret_val = sem_destroy(&shared_data->stThread_CCU.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy CCU semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_STM.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy STM semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_ICM_RX.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ICM_RX semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_ARA.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ARA semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_ICM_TX.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy ICM_TX semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_FM.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy FM semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_SD.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy SD semaphore: %s", strerror(errno));
    }

    ret_val = sem_destroy(&shared_data->stThread_CRV.sem);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to destroy CRV semaphore: %s", strerror(errno));
    }
}

/**
 * @brief Destroys all POSIX timers used for thread scheduling in the ASI system
 *
 * This function systematically deletes all POSIX timers that were created for periodic
 * thread execution scheduling. It ensures proper cleanup of timer resources during
 * system shutdown or cleanup phase.
 *
 * Destroys timers for the following threads in order:
 * - CCU (Cycle Count Updater) - 25ms period timer
 * - STM (State Machine) - 50ms period timer
 * - ICM_RX (Interface Communication Manager - Receive) - 50ms period timer
 * - ARA (Action Request Approver) - 50ms period timer
 * - ICM_TX (Interface Communication Manager - Transmit) - 50ms period timer
 * - FM (Fault Manager) - 25ms period timer
 * - SD (System Diagnostic) - 200ms period timer
 * - CRV (Calibration Readback Verification) - 50ms period timer
 *
 * Error Handling:
 * - Attempts to delete all timers even if some deletions fail
 * - Logs each deletion failure independently with errno information
 * - Continues execution after individual failures to ensure maximum cleanup
 * - Does not propagate errors to allow system shutdown to proceed
 *
 * Timer Cleanup Behavior:
 * - Uses timer_delete() for each POSIX timer
 * - Processes each timer independently
 * - Continues to next timer if current deletion fails
 * - Logs status of each timer deletion operation
 *
 * Resource Management:
 * - Disarms timers automatically as part of deletion
 * - Frees associated timer resources in the kernel
 * - Handles each deletion atomically
 * - Timer IDs become invalid after deletion
 *
 * @note Should be called during system shutdown sequence
 * @note All threads should be terminated before calling this function
 * @note Complements setup_timer() initialization function
 *
 * @warning Attempting to use timers after destruction will result in undefined behavior
 * @warning This function should only be called when no threads are actively waiting
 *          on timer signals
 * @warning Failing to call this function during cleanup may result in timer resource leaks
 *
 */
void destroy_timers(void)
{
    int ret_val;

    ret_val = timer_delete(stTimerCCU);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete CCU timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerSTM);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete STM timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerICM_RX);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete ICM_RX timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerARA);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete ARA timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerICM_TX);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete ICM_TX timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerFM);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete FM timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerSD);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete SD timer: %s", strerror(errno));
    }

    ret_val = timer_delete(stTimerCRV);
    if (ret_val != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to delete CRV timer: %s", strerror(errno));
    }
}

/**
 * @brief Initializes and starts all real-time threads in the ASI system
 *
 * This function performs comprehensive initialization and launching of all system threads,
 * configuring them with appropriate real-time attributes, priorities, and scheduling parameters.
 *
 * @param shared_data Pointer to shared memory structure containing synchronization primitives
 *                   and thread data [type: DataOnSharedMemory*]
 * @param thread_mgmt_log_file File pointer for thread management logging [type: FILE*]
 *
 * @return thread_status_code_t Status of thread initialization:
 *         - THREAD_STATUS_SUCCESS: All threads started successfully
 *         - THREAD_STATUS_NOMEM: Memory allocation failure
 *         - THREAD_STATUS_PERM: Permission denied for real-time scheduling
 *         - THREAD_STATUS_INVAL: Invalid thread attributes
 *         - THREAD_STATUS_AGAIN: Resource temporarily unavailable
 *         - THREAD_STATUS_FAULT: General failure
 *
 * Initialization Sequence:
 * 1. Thread Timing Setup:
 *    - Initializes execution timing monitoring
 *    - Sets up performance metrics tracking
 *
 * 2. Signal Handling:
 *    - Initializes thread signal handling infrastructure
 *    - Sets up signal masks for all threads
 *
 * 3. Thread Semaphore Assignment:
 *    - CCU: 25ms period, priority 90
 *    - FM: 25ms period, priority 80
 *    - STM: 50ms period, priority 80
 *    - ICM_RX: 50ms period, priority 70
 *    - ICM_TX: 50ms period, priority 70
 *    - ARA: 50ms period, priority 60
 *    - CRV: 50ms period, priority 50
 *    - SD: 200ms period, priority 40
 *
 * 4. POSIX Timer Setup:
 *    - Creates timers for each thread
 *    - Associates timers with thread semaphores
 *    - Configures timer periods
 *
 * 5. Thread Attribute Configuration:
 *    - Sets SCHED_FIFO real-time scheduling policy
 *    - Configures explicit scheduling inheritance
 *    - Sets thread priorities
 *
 * 6. Thread Creation:
 *    - Creates threads with configured attributes
 *    - Associates threads with their respective functions
 *    - Verifies successful thread creation
 *
 * Error Handling:
 * - Validates all initialization steps
 * - Provides detailed error logging
 * - Performs cleanup on initialization failures
 * - Maintains system integrity during partial initialization
 *
 * Thread Attributes:
 * - SCHED_FIFO scheduling policy for real-time behavior
 * - Process-shared synchronization primitives
 * - Explicit scheduling parameters
 * - Custom priority levels per thread
 *
 * @note Must be called after shared memory and synchronization primitive initialization
 * @note Requires root privileges for real-time scheduling
 *
 * @warning Failure to start any thread may compromise system functionality
 * @warning Real-time scheduling requires appropriate permissions
 * @warning Thread priorities must be carefully chosen to prevent starvation
 *
 */
thread_status_code_t start_threads(DataOnSharedMemory *shared_data, FILE *thread_mgmt_log_file)
{
    /* Initialize thread attributes and scheduling parameters */
    pthread_attr_t attr;
    struct sched_param param;

    /* Initialize thread timing monitoring */
    init_thread_timing();

    /* Initialize signal handling for threads */
    init_thread_signal_handling();

    /* Assign semaphores to thread_info structures */
    thread_info[enThread_CCU].thread_sem = &shared_data->stThread_CCU.sem;
    thread_info[enThread_FM].thread_sem = &shared_data->stThread_FM.sem;
    thread_info[enThread_STM].thread_sem = &shared_data->stThread_STM.sem;
    thread_info[enThread_ICM_RX].thread_sem = &shared_data->stThread_ICM_RX.sem;
    thread_info[enThread_ICM_TX].thread_sem = &shared_data->stThread_ICM_TX.sem;
    thread_info[enThread_ARA].thread_sem = &shared_data->stThread_ARA.sem;
    thread_info[enThread_CRV].thread_sem = &shared_data->stThread_CRV.sem;
    thread_info[enThread_SD].thread_sem = &shared_data->stThread_SD.sem;

    /* Set up timers for each thread with their respective periodicities */
    if (setup_timer(&stTimerCCU, thread_info[enThread_CCU].thread_sem, thread_info[enThread_CCU].periodicity) != 0 ||
        setup_timer(&stTimerSTM, thread_info[enThread_STM].thread_sem, thread_info[enThread_STM].periodicity) != 0 ||
        setup_timer(&stTimerICM_RX, thread_info[enThread_ICM_RX].thread_sem, thread_info[enThread_ICM_RX].periodicity) != 0 ||
        setup_timer(&stTimerARA, thread_info[enThread_ARA].thread_sem, thread_info[enThread_ARA].periodicity) != 0 ||
        setup_timer(&stTimerICM_TX, thread_info[enThread_ICM_TX].thread_sem, thread_info[enThread_ICM_TX].periodicity) != 0 ||
        setup_timer(&stTimerFM, thread_info[enThread_FM].thread_sem, thread_info[enThread_FM].periodicity) != 0 ||
        setup_timer(&stTimerSD, thread_info[enThread_SD].thread_sem, thread_info[enThread_SD].periodicity) != 0 ||
        setup_timer(&stTimerCRV, thread_info[enThread_CRV].thread_sem, thread_info[enThread_CRV].periodicity) != 0)
    {
        log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to set up timers");
        return THREAD_STATUS_NOTSUP;
    }

    /* Initialize thread attributes */
    int ret_val = pthread_attr_init(&attr);
    if (ret_val != 0)
    {
        log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to initialize thread attributes");
        return THREAD_STATUS_NOMEM;
    }

    /* Set the scheduling inheritance mode to explicit */
    ret_val = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret_val != 0)
    {
        log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to set scheduling inheritance mode");
        ret_val = pthread_attr_destroy(&attr);
        if (ret_val != 0)
        {
            log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to destroy thread attributes");
        }
        return THREAD_STATUS_PERM;
    }

    /* Set the scheduling policy to FIFO (real-time) */
    ret_val = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret_val != 0)
    {
        log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to set scheduling policy");
        ret_val = pthread_attr_destroy(&attr);
        if (ret_val != 0)
        {
            log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to destroy thread attributes");
        }
        return THREAD_STATUS_PERM;
    }

    /* Iterate through each thread to create it */
    thread_label_t thread_label;
    for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
    {
        /* Set the thread priority */
        param.sched_priority = thread_info[thread_label].priority;
        if (pthread_attr_setschedparam(&attr, &param) != 0)
        {
            error_string_t error_str = strerror(errno);
            if (error_str != NULL)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set thread priority for %s: %s",
                            thread_info[thread_label].name, error_str);
            }
            else
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set thread priority for %s with unknown error",
                            thread_info[thread_label].name);
            }
            ret_val = pthread_attr_destroy(&attr);
            if (ret_val != 0)
            {
                log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to destroy thread attributes");
            }
            return THREAD_STATUS_PERM;
        }

        /* Create the thread with the specified attributes and function */
        int ret = pthread_create(&threads[thread_label], &attr, &thread_function_wrapper,
                                 (generic_ptr_t)&thread_info[thread_label]);
        if (ret != 0)
        {
            /* Mapping pthread_create errors to our status codes */
            thread_status_code_t status;
            switch (ret)
            {
            case EAGAIN:
                status = THREAD_STATUS_AGAIN;
                break;
            case EINVAL:
                status = THREAD_STATUS_INVAL;
                break;
            case EPERM:
                status = THREAD_STATUS_PERM;
                break;
            default:
                status = THREAD_STATUS_FAULT;
                break;
            }

            (void)log_message(thread_mgmt_log_file, LOG_ERROR, "Error creating thread %s: %s",
                              thread_info[thread_label].name, strerror(ret));
            ret_val = pthread_attr_destroy(&attr);
            if (ret_val != 0)
            {
                log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to destroy thread attributes");
            }
            return status;
        }

        log_message(thread_mgmt_log_file, LOG_INFO, "Thread %s created successfully",
                    thread_info[thread_label].name);
    }

    /* Clean up thread attributes */
    ret_val = pthread_attr_destroy(&attr);
    if (ret_val != 0)
    {
        log_message(thread_mgmt_log_file, LOG_ERROR, "Failed to destroy thread attributes");
        return THREAD_STATUS_FAULT;
    }
    return THREAD_STATUS_SUCCESS;
}

/**
 * @brief Monitors the status of all threads and handles abnormal terminations.
 *
 * This function is responsible for checking the status of all threads in the system
 * and initiating appropriate actions in case of abnormal terminations. It serves as
 * a central monitoring point to ensure system stability and handle thread crashes.
 *
 * The function performs the following operations:
 *
 * 1. Crash Detection:
 *    - Checks the global thread crash flag using get_thread_crashed().
 *    - If a crash is detected, it calls handle_thread_termination() to manage
 *      the crashed thread.
 *
 * 2. Restart Threshold Check:
 *    - Iterates through all threads in the system.
 *    - For each thread, it checks if the number of abnormal terminations
 *      (thread_status[i].abnormal_terminations) has exceeded the maximum
 *      allowed restarts (THREAD_MAX_RESTART_THRESOLD).
 *    - If the threshold is exceeded for any thread:
 *      - Logs an error message indicating which thread exceeded the restart limit.
 *      - Initiates a graceful shutdown of the entire process by calling
 *        initiate_graceful_shutdown().
 *
 * @param shared_data Pointer to the DataOnSharedMemory structure containing shared
 *                    resources and thread information.
 *
 */
void monitor_threads(DataOnSharedMemory *shared_data)
{
    if (get_thread_crashed())
    {
        handle_thread_termination(shared_data);
    }

    thread_label_t thread_label;
    for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
    {
        if (thread_status_info[thread_label].abnormal_terminations >= THREAD_MAX_RESTART_THRESOLD)
        {
            log_message(global_log_file, LOG_ERROR,
                        "Thread %s exceeded max restarts. Initiating graceful shutdown.",
                        thread_info[thread_label].name);
            initiate_graceful_shutdown(shared_data);
            return;
        }
    }
}

/**
 * @brief Handles the abnormal termination of threads and attempts to restart them if necessary.
 *
 * This function is called when a thread abnormal termination is detected. It performs
 * a series of actions to manage the terminated thread and maintain system stability.
 *
 * The function performs the following steps:
 *
 * 1. Thread Identification:
 *    - Retrieves the ID of the crashed thread using get_crashed_thread_id().
 *    - Identifies the index of the crashed thread in the threads array.
 *
 * 2. Termination Tracking:
 *    - Updates the abnormal termination counter for the identified thread.
 *    - Records the current time as the last termination time for the thread.
 *
 * 3. Restart Evaluation:
 *    - Checks if the number of abnormal terminations exceeds the maximum allowed (THREAD_MAX_RESTART_THRESOLD).
 *    - If the threshold is exceeded within the monitoring interval (THREAD_CRASH_MONITORING_INTERVAL):
 *      - Logs an error message.
 *      - Initiates a graceful shutdown of the entire process.
 *    - If the threshold is not exceeded:
 *      - Attempts to restart the terminated thread.
 *
 * 4. Thread Cancellation and Restart:
 *    - Cancels the existing thread using pthread_cancel().
 *    - Creates a new thread with the same parameters using pthread_create().
 *    - Logs the restart attempt, whether successful or not.
 *
 * 5. Reset Crash Flag:
 *    - Resets the global thread crashed flag using set_thread_crashed().
 *
 * @param shared_data Pointer to the DataOnSharedMemory structure containing shared
 *                    resources and thread information.
 *
 */
void handle_thread_termination(DataOnSharedMemory *shared_data)
{
    pthread_t crashed_id = get_crashed_thread_id();
    thread_label_t thread_index = enTotalThreads;
    time_t current_time = time(NULL);

    thread_label_t thread_label;
    for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
    {
        if (pthread_equal(threads[thread_label], crashed_id))
        {
            thread_index = thread_label;
            break;
        }
    }

    if (thread_index == (thread_label_t)enTotalThreads)
    {
        log_message(global_log_file, LOG_ERROR, "Could not identify crashed thread");
        return;
    }

    if (current_time - thread_status_info[thread_index].last_termination_time <= THREAD_CRASH_MONITORING_INTERVAL)
    {
        thread_status_info[thread_index].abnormal_terminations++;

        if (thread_status_info[thread_index].abnormal_terminations > THREAD_MAX_RESTART_THRESOLD)
        {
            log_message(global_log_file, LOG_ERROR,
                        "Thread %s exceeded max restarts within monitoring interval. Initiating graceful shutdown.",
                        thread_info[thread_index].name);
            initiate_graceful_shutdown(shared_data);
            return;
        }
    }
    else
    {
        thread_status_info[thread_index].abnormal_terminations = 1;
    }

    thread_status_info[thread_index].last_termination_time = current_time;

    int cancel_ret = pthread_cancel(threads[thread_index]);
    if (cancel_ret != 0)
    {
        (void)log_message(global_log_file, LOG_WARNING, "Failed to cancel thread %s: %s",
                          thread_info[thread_index].name, strerror(cancel_ret));
    }

    int create_ret = pthread_create(&threads[thread_index], NULL, &thread_function_wrapper,
                                    (generic_ptr_t)&thread_info[thread_index]);
    if (create_ret != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to restart thread %s: %s",
                          thread_info[thread_index].name, strerror(create_ret));
    }
    else
    {
        (void)log_message(global_log_file, LOG_INFO, "Thread %s restarted",
                          thread_info[thread_index].name);
    }

    set_thread_crashed(0);
}

/**
 * @brief Initiates a graceful shutdown of all threads and performs cleanup operations.
 *
 * This function is responsible for safely terminating all running threads and performing
 * necessary cleanup operations. It's typically called when the program receives a
 * termination signal or when a critical error occurs that requires the program to shut down.
 *
 * The function performs the following sequence of operations:
 *
 * 1. Thread Termination:
 *    - Sets the global thread_exit_flag to signal all threads to terminate.
 *    - Waits for all threads to complete their execution using pthread_join().
 *    - Logs the successful termination of all threads.
 *
 * 2. Network Shutdown:
 *    - Calls SD_vTCPConnectionShutdown() to close any open network connections.
 *
 * 3. Data Persistence:
 *    - Calls FM_s8SaveEventDataToStorage() to ensure all critical data is saved.
 *
 * 4. Resource Cleanup:
 *    - Calls destroy_mutexes_and_sems() to clean up synchronization primitives.
 *    - Calls destroy_timers() to clean up any POSIX timers.
 *
 * 5. Final Logging:
 *    - Logs a message indicating that the graceful shutdown has completed.
 *
 * @param shared_data Pointer to the DataOnSharedMemory structure containing shared
 *                    resources that need to be cleaned up during shutdown.
 *
 */
void initiate_graceful_shutdown(DataOnSharedMemory *shared_data)
{
    set_thread_exit(1);

    thread_label_t thread_label;
    for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
    {
        int join_result = pthread_join(threads[thread_label], NULL);
        if (join_result != 0)
        {
            (void)log_message(global_log_file, LOG_WARNING, "Failed to join thread %s: %s", thread_info[thread_label].name, strerror(join_result));
        }
    }

    log_message(global_log_file, LOG_INFO, "All threads terminated gracefully");

    SD_vCloseTCPConnection(enVAMConnectionTCP);
    SD_vCloseTCPConnection(enCMConnectionTCP);

    save_all_shared_data_to_storage(shared_data);

    log_message(global_log_file, LOG_INFO, "Graceful shutdown completed");
}

/**
 * @brief Retrieves the name of the currently executing thread.
 *
 * This function determines the name of the currently executing thread by comparing
 * the thread ID of the calling thread with the IDs of known threads in the system.
 * It's useful for logging, debugging, and any operations that need to identify
 * the current thread by name rather than ID.
 *
 * @return const char*
 *         Returns a pointer to a string containing the name of the current thread.
 *         This will be one of the predefined thread names (e.g., "THRD_CCU", "THRD_STM", etc.)
 *         if the thread is recognized, or "Unknown" if the thread is not recognized.
 *
 */
thread_name_t get_current_thread_name(void)
{
    pthread_t current_thread = pthread_self();
    thread_label_t thread_label;
    for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
    {
        if (pthread_equal(current_thread, threads[thread_label]))
        {
            return thread_info[thread_label].name;
        }
    }
    return "Unknown";
}

/*** Local Function Implementations ***/

/**
 * @brief Main function for thread execution, dispatching to specific thread tasks.
 *
 * This function serves as the central dispatch point for all threads in the system.
 * It interprets the thread information provided and calls the appropriate thread-specific
 * function based on the thread name. The function supports the following threads:
 *
 * - THRD_CCU (Cycle Count Updater)
 * - THRD_STM (State Machine)
 * - THRD_ICM_RX (Interface Communication Manager - Receive)
 * - THRD_ARA (Action Request Approver)
 * - THRD_ICM_TX (Interface Communication Manager - Transmit)
 * - THRD_FM (Fault Manager)
 * - THRD_SD (System Diagnostic)
 * - THRD_CRV (Calibration Readback Verification)
 *
 * Each thread is associated with a specific function that handles its main logic:
 *
 * - THRD_CCU calls THRD_CycleCountUpdater_20ms
 * - THRD_STM calls THRD_StateMachine_50ms
 * - THRD_ICM_RX calls THRD_InterfaceCommManager_Rx_50ms
 * - THRD_ARA calls THRD_ActionRequestApprover_50ms
 * - THRD_ICM_TX calls THRD_InterfaceCommManager_Tx_50ms
 * - THRD_FM calls THRD_FaultManager_25ms
 * - THRD_SD calls THRD_SystemDiagnostic_200ms
 * - THRD_CRV calls THRD_CalibrationReadbackVerification_200ms
 *
 * The function uses string comparison to identify the thread type and then calls
 * the appropriate function. If an unknown thread type is encountered, an error
 * is logged.
 *
 * @param arg Pointer to a thread_info_t structure containing thread information.
 *            This structure is expected to have a 'name' field that identifies
 *            the thread type.
 *
 * @return NULL
 *         The function always returns NULL. Actual thread execution and potential
 *         return values are handled within the specific thread functions called.
 *
 */
static generic_ptr_t thread_function(generic_ptr_t arg)
{
    thread_info_t *info = (thread_info_t *)arg;
    const size_t MAX_THREAD_NAME_LEN = 32U;
    thread_label_t thread_id = enTotalThreads;           /* Initialize to invalid state */
    static uint32_t test_counters[enTotalThreads] = {0}; /* Cycle counters for each thread */

    /* First identify the thread ID - do this only once */
    if ((NULL != info) && (NULL != info->name))
    {
        thread_label_t i;
        for (i = 0; i < enTotalThreads; i++)
        {
            if ((NULL != thread_info[i].name) &&
                (0 == strncmp(info->name, thread_info[i].name, MAX_THREAD_NAME_LEN)))
            {
                thread_id = i;
                break;
            }
        }
    }

    if (thread_id == enTotalThreads || NULL == info || NULL == info->name)
    {
        log_message(global_log_file, LOG_ERROR, "Invalid thread information or unknown thread type");
        return NULL;
    }

    /* Initialize function pointer array for thread tasks */
    typedef void (*thread_func_t)(generic_ptr_t arg);
    static const thread_func_t thread_functions[enTotalThreads] = {
        [enThread_CCU] = &THRD_CycleCountUpdater_20ms,
        [enThread_FM] = &THRD_FaultManager_25ms,
        [enThread_STM] = &THRD_StateMachine_50ms,
        [enThread_ICM_RX] = &THRD_InterfaceCommManager_Rx_50ms,
        [enThread_ICM_TX] = &THRD_InterfaceCommManager_Tx_50ms,
        [enThread_ARA] = &THRD_ActionRequestApprover_50ms,
        [enThread_CRV] = &THRD_CalibrationReadbackVerification_50ms,
        [enThread_SD] = &THRD_SystemDiagnostic_200ms};

    log_message(global_log_file, LOG_INFO, "Thread %s initialized and starting main loop",
                thread_info[thread_id].name);

    /* Main thread loop */
    while (!get_thread_exit())
    {
        int sem_result;

        /* Wait for timer semaphore with timeout to allow checking exit flag */
        struct timespec timeout;
        if (clock_gettime(CLOCK_REALTIME, &timeout) != 0)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to get time for semaphore timeout");
            continue;
        }

        /* Set timeout to 100ms to allow periodic exit flag checking */
        timeout.tv_nsec += 100000000; /* 100ms in nanoseconds */
        if (timeout.tv_nsec >= 1000000000)
        {
            timeout.tv_sec += 1;
            timeout.tv_nsec -= 1000000000;
        }

        sem_result = sem_timedwait(info->thread_sem, &timeout);

        if (sem_result == 0)
        {
            /* Check exit flag again after semaphore wake-up */
            if (get_thread_exit())
            {
                break; /* Exit cleanly if shutdown was initiated while waiting */
            }

            /* Execute the thread function if it exists */
            if (thread_functions[thread_id])
            {
                start_thread_execution_timing(thread_id);
                thread_functions[thread_id](arg);
                end_thread_execution_timing(thread_id);
            }

            /* Only measure end timing if we haven't started shutdown */
            if (!get_thread_exit())
            {
                end_thread_execution_timing(thread_id);
                log_message(global_log_file, LOG_INFO,
                            "Thread %s completed execution cycle %u",
                            thread_info[thread_id].name, test_counters[thread_id]);
            }
        }
        else if (errno != ETIMEDOUT)
        {
            /* Log error only if it's not a timeout (timeout is expected) */
            (void)log_message(global_log_file, LOG_ERROR, "Semaphore wait failed for thread %s: %s",
                              thread_info[thread_id].name, strerror(errno));
        }
        else
        {
            /* Intentionally empty else block */
        }

        /* Add a small yield to prevent tight loop in case of errors */
        if (sem_result != 0)
        {
            struct timespec yield_time = {0, 1000000}; /* 1ms */
            (void)nanosleep(&yield_time, NULL);
        }
    }

    /* Clean shutdown */
    log_message(global_log_file, LOG_INFO, "Thread %s exiting cleanly",
                thread_info[thread_id].name);

    return NULL;
}

/**
 * @brief Wrapper function providing protected execution environment for thread operations with error handling and monitoring.
 *
 * This function serves as a robust protective wrapper around the main thread function, implementing
 * thread safety mechanisms, error detection, and reporting. It creates a controlled execution
 * environment for each thread in the system.
 *
 * Key responsibilities:
 * 1. Thread Setup:
 *    - Identifies thread by mapping thread_info_t name to internal thread ID
 *    - Configures thread-specific signal handling
 *    - Sets up cancellation states for proper cleanup
 *
 * 2. Signal Handling:
 *    - Creates thread-specific signal mask
 *    - Unblocks SIGTERM and SIGINT for shutdown handling
 *    - Preserves parent process signal mask
 *    - Configures custom signal handlers for thread safety
 *
 * 3. Thread Control:
 *    - Enables deferred cancellation (PTHREAD_CANCEL_DEFERRED)
 *    - Sets cancellation points for safe termination
 *    - Manages thread state transitions
 *
 * 4. Error Management:
 *    - Detects and reports thread crashes
 *    - Tracks abnormal terminations
 *    - Provides detailed error logging
 *    - Implements cleanup on failure
 *
 * 5. Resource Protection:
 *    - Validates all input parameters
 *    - Ensures proper resource initialization
 *    - Implements cleanup on errors
 *    - Maintains thread status information
 *
 * @param arg Pointer to thread_info_t structure containing:
 *           - name: Thread identifier string
 *           - priority: Thread scheduling priority
 *           - periodicity: Thread execution period
 *           - thread_sem: Pointer to thread synchronization semaphore
 *
 * @return void* Always returns NULL. Thread termination is handled through:
 *         - Normal completion of thread_function()
 *         - pthread cancellation mechanism
 *         - Error detection and handling
 *
 * @note Thread Safety:
 *       - Function is thread-safe by design
 *       - Implements thread-local storage for critical data
 *       - Uses atomic operations for status flags
 *
 * @warning
 *       - Must be called only through pthread_create()
 *       - Requires valid thread_info_t structure
 *       - Signal mask modifications affect thread signal handling
 *
 * Error Handling:
 * - Invalid thread info: Logs error and returns NULL
 * - Signal mask setup failure: Logs error and returns NULL
 * - Thread cancellation setup failure: Logs error and returns NULL
 * - Thread function failure: Reports via thread crash mechanism
 *
 */
static generic_ptr_t thread_function_wrapper(generic_ptr_t arg)
{
    thread_info_t *info = (thread_info_t *)arg;
    thread_label_t thread_id = enTotalThreads; /* Initialize to invalid state */
    const size_t MAX_THREAD_NAME_LEN = 32U;
    int ret_val;

    /* Set up the signal mask for this thread */
    sigset_t thread_mask;
    ret_val = sigemptyset(&thread_mask);
    if (ret_val != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to empty signal set for thread %s",
                    info->name);
        return NULL;
    }

    ret_val = sigaddset(&thread_mask, SIGTERM);
    if (ret_val != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to add SIGTERM to signal set for thread %s",
                    info->name);
        return NULL;
    }

    ret_val = sigaddset(&thread_mask, SIGINT);
    if (ret_val != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to add SIGINT to signal set for thread %s",
                    info->name);
        return NULL;
    }

    if (pthread_sigmask(SIG_UNBLOCK, &thread_mask, NULL) != 0)
    {
        error_string_t error_str = strerror(errno);
        if (error_str != NULL)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set signal mask for thread %s: %s",
                        info->name, error_str);
        }
        else
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set signal mask for thread %s with unknown error",
                        info->name);
        }
        return NULL;
    }

    if ((NULL != info) && (NULL != info->name))
    {
        thread_label_t thread_label;
        for (thread_label = 0; thread_label < (thread_label_t)enTotalThreads; thread_label++)
        {
            if ((NULL != thread_info[thread_label].name) &&
                (0 == strncmp(info->name, thread_info[thread_label].name, MAX_THREAD_NAME_LEN)))
            {
                thread_id = thread_label;
                break;
            }
        }
    }

    ret_val = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    if (ret_val != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to set cancel state for thread %s",
                    info->name);
        return NULL;
    }

    ret_val = pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    if (ret_val != 0)
    {
        log_message(global_log_file, LOG_ERROR, "Failed to set cancel type for thread %s",
                    info->name);
        return NULL;
    }

    if (thread_id == (thread_label_t)enTotalThreads)
    { /* Check for invalid state */
        log_message(global_log_file, LOG_ERROR, "Unknown thread type: %s", info->name);
        return NULL;
    }

    /* Execute the thread function */
    generic_ptr_t thread_result = thread_function(arg);
    if (thread_result != NULL)
    {
        log_message(global_log_file, LOG_WARNING, "Thread function returned non-NULL value");
    }

    /* Check for any errors that occurred during thread execution */
    if (get_thread_crashed())
    {
        report_abnormal_termination(thread_id, 0);
    }

    return NULL;
}

/**
 * @brief Restores the main thread's original signal mask to its initial state.
 *
 * This function restores the signal mask of the main thread using the previously saved
 * mask in main_thread_sigmask. It ensures proper signal handling state is recovered
 * after temporary modifications.
 *
 * @pre
 *   - main_thread_sigmask must contain the valid original signal mask
 *   - global_log_file must be initialized
 *
 * @note
 *   - Should only be called from the main thread
 *   - Does not modify signal handlers, only the signal mask
 *
 * Error Handling:
 * - Logs error message if pthread_sigmask() fails
 * - Continues execution even on failure for system stability
 *
 */
void restore_main_thread_sigmask(void)
{
    if (pthread_sigmask(SIG_SETMASK, &main_thread_sigmask, NULL) != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to restore main thread signal mask: %s", strerror(errno));
    }
}

/**
 * @brief Main function for Thread CCU (Cycle Count Updater), executing every 20ms.
 *
 * This function serves as the entry point for the Cycle Count Updater thread, which is
 * scheduled to run at 20ms intervals. Its primary responsibility is to update and manage
 * cycle counts within the system, which is likely crucial for timing and synchronization
 * purposes.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_CycleCountUpdater_20ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_CCU();
}

/**
 * @brief Main function for Thread STM (State Machine), executing every 50ms.
 *
 * This function serves as the entry point for the State Machine thread, which is
 * scheduled to run at 50ms intervals. Its primary responsibility is to manage and
 * update the system's state machine, playing a crucial role in controlling the overall
 * behavior and operational mode of the system.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_StateMachine_50ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_STM();
}

/**
 * @brief Main function for Thread ICM_RX (Interface Communication Manager Receive), executing every 50ms.
 *
 * This function serves as the entry point for the Interface Communication Manager Receive thread,
 * which is scheduled to run at 50ms intervals. Its primary responsibility is to manage the reception
 * of data from external interfaces or devices into the system.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_InterfaceCommManager_Rx_50ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_ICM_RX();
}

/**
 * @brief Main function for Thread ARA (Action Request Approver), executing every 50ms.
 *
 * This function serves as the entry point for the Action Request Approver thread,
 * which is scheduled to run at 50ms intervals. Its primary responsibility is to
 * evaluate and approve (or deny) action requests within the system, playing a crucial
 * role in system control and decision-making processes.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_ActionRequestApprover_50ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_ARA();
}

/**
 * @brief Main function for Thread ICM_TX (Interface Communication Manager Transmit), executing every 50ms.
 *
 * This function serves as the entry point for the Interface Communication Manager Transmit thread,
 * which is scheduled to run at 50ms intervals. Its primary responsibility is to manage the transmission
 * of data from the system to external interfaces or devices.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_InterfaceCommManager_Tx_50ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_ICM_TX();
}

/**
 * @brief Main function for Thread FM (Fault Manager), executing every 25ms.
 *
 * This function serves as the entry point for the Fault Manager thread, which is
 * scheduled to run at 25ms intervals. Its primary responsibility is to manage events and
 * handle different error conditions within the system, playing a crucial role in maintaining
 * system reliability and safety.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_FaultManager_25ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_FM();
}

/**
 * @brief Main function for Thread SD (System Diagnostic), executing every 200ms.
 *
 * This function serves as the entry point for the System Diagnostic thread, which is
 * scheduled to run at 200ms intervals. Its primary responsibility is to perform
 * TCP connections checks, which are crucial for monitoring and maintaining the communication
 * health and performance of the system.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_SystemDiagnostic_200ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_SD();
}

/**
 * @brief Main function for Thread CRV (Calibration Readback Verification), executing every 50ms.
 *
 * This function serves as the entry point for the Calibration Readback Verification thread,
 * which is scheduled to run at 50ms intervals. Its primary responsibility is to perform
 * calibration readback verification tasks.
 *
 * @param arg Pointer to thread arguments. In this implementation, it's not used and is
 *            cast to void to suppress compiler warnings about unused parameters.
 *
 */
static void THRD_CalibrationReadbackVerification_50ms(generic_ptr_t arg)
{
    (void)arg; /* Cast to void to suppress warning */
    ITCOM_vWrapperThread_CRV();
}

/**
 * @brief POSIX timer expiration callback handler for thread scheduling.
 *
 * This function is invoked automatically when a POSIX timer expires, serving as the
 * timer's signal handler. It posts to the associated thread's semaphore to trigger
 * periodic thread execution.
 *
 * @param sv Union containing the semaphore pointer in sival_ptr:
 *          - sv.sival_ptr: Points to sem_t structure for thread synchronization
 *
 * Key characteristics:
 * - Called from signal context
 * - Must be async-signal-safe
 * - Non-blocking design
 * - Quick execution
 *
 * Error Handling:
 * - Logs sem_post failures but continues execution
 * - Uses async-signal-safe error reporting
 *
 * @note
 * - Automatically registered through timer_create()
 * - Critical for real-time thread scheduling
 * - Errors are logged but cannot be returned
 *
 * @warning
 * - Must not call non-async-signal-safe functions
 * - Keep execution time minimal to prevent timer overruns
 *
 */
static void timer_handler(union sigval sv)
{
    sem_t *sem = (sem_t *)sv.sival_ptr;
    int post_result = sem_post(sem);
    if (post_result != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "sem_post failed in timer handler: %s", strerror(errno));
    }
}

/**
 * @brief Configures a POSIX timer for periodic thread scheduling.
 *
 * Creates and initializes a POSIX timer that triggers thread execution at specified intervals.
 * The timer uses the CLOCK_MONOTONIC time source for reliable periodic timing and signals
 * thread execution through semaphore posting.
 *
 * @param p_timer_id Pointer to timer_t object for storing created timer ID
 * @param sem Pointer to semaphore that will be posted when timer expires
 * @param period_ms Timer period in milliseconds defining thread execution frequency
 *
 * @return timer_status_t Status code indicating setup result:
 *         - THREAD_STATUS_SUCCESS: Timer setup successful
 *         - THREAD_STATUS_INVAL: Invalid parameters (NULL pointers or period <= 0)
 *         - THREAD_STATUS_NOTSUP: Timer creation failed
 *         - THREAD_STATUS_FAULT: Timer configuration failed
 *
 * Setup process:
 * 1. Validates input parameters
 * 2. Configures timer to use thread_handler callback
 * 3. Creates timer with CLOCK_MONOTONIC
 * 4. Sets timer period and starts it
 *
 * Error Handling:
 * - Parameter validation with detailed error returns
 * - Cleanup on partial initialization failure
 * - Logs detailed error information
 *
 * @note
 * - Period is converted to sec/nsec for timer configuration
 * - Timer deletion responsibility lies with caller
 *
 */
static timer_status_t setup_timer(timer_t *p_timer_id, sem_t *sem, timer_period_t period_ms)
{
    if (!p_timer_id || !sem || period_ms <= 0)
    {
        return THREAD_STATUS_INVAL;
    }

    struct sigevent sev;
    struct itimerspec its;

    /* Configure timer to use thread handler */
    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = &timer_handler;
    sev.sigev_value.sival_ptr = sem;
    sev.sigev_notify_attributes = NULL;

    if (timer_create(CLOCK_MONOTONIC, &sev, p_timer_id) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "timer_create failed: %s", strerror(errno));
        return THREAD_STATUS_NOTSUP;
    }

    /* Configure timer period */
    its.it_value.tv_sec = period_ms / SEC_TO_MS;
    its.it_value.tv_nsec = (period_ms % SEC_TO_MS) * NSEC_TO_MS;
    its.it_interval = its.it_value;

    log_message(global_log_file, LOG_DEBUG, "Setting up timer with period %d ms (%ld.%09ld)",
                period_ms, its.it_value.tv_sec, its.it_value.tv_nsec);

    if (timer_settime(*p_timer_id, 0, &its, NULL) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "timer_settime failed: %s", strerror(errno));
        (void)timer_delete(*p_timer_id);
        return THREAD_STATUS_FAULT;
    }

    return THREAD_STATUS_SUCCESS;
}

/**
 * @brief Records and reports abnormal thread termination events.
 *
 * Handles the logging and tracking of thread termination events, updates termination
 * statistics, and sets appropriate system flags for fault management.
 *
 * @param thread_id Thread identifier corresponding to the terminated thread's index
 *                  in thread_info and thread_status arrays
 * @param signal_number Signal that caused the termination (0 if cause unknown)
 *
 * Actions performed:
 * 1. Increments abnormal termination counter for the thread
 * 2. Updates last termination timestamp
 * 3. Sets global abnormal termination flag
 * 4. Logs termination event with thread name and signal info
 *
 * @note
 * - Called from signal handlers and error-catching contexts
 * - Designed to be quick and non-blocking
 * - Part of system's fault detection mechanism
 *
 * @warning
 * - Must be safe to call from signal context
 * - Should not allocate memory or use locks
 *
 */
static void report_abnormal_termination(thread_id_t thread_id, sig_num_t signal_number)
{
    thread_status_info[thread_id].abnormal_terminations++;
    thread_status_info[thread_id].last_termination_time = time(NULL);

    set_abnormal_termination(1);

    sig_name_t signal_name = get_signal_name(signal_number);

    log_message(global_log_file, LOG_ERROR, "Thread %s terminated abnormally: %s (%d)",
                thread_info[thread_id].name, signal_name, signal_number);
}

/**
 * @brief Initializes timing tracking structures for all threads.
 *
 * Sets up the timing monitoring infrastructure for all system threads by initializing
 * their respective timing statistics and measurement structures to a known state.
 *
 * Per-thread initialization:
 * - Execution state flag
 * - Last execution time
 * - Overrun counter
 * - Start/end timestamps
 *
 * @note
 * - Must be called before starting any threads
 * - Part of thread management initialization sequence
 * - Supports runtime timing analysis and overrun detection
 *
 */
static void init_thread_timing(void)
{
    thread_label_t i;
    log_message(global_log_file, LOG_INFO, "Initializing thread timing tracking...");

    /* Initialize timing data for each thread */
    for (i = 0; i < enTotalThreads; i++)
    {
        thread_timing[i].is_executing = false;
        thread_timing[i].last_execution_time_ms = 0;
        thread_timing[i].overrun_count = 0;
        thread_timing[i].start_time.tv_sec = 0;
        thread_timing[i].start_time.tv_nsec = 0;
        thread_timing[i].end_time.tv_sec = 0;
        thread_timing[i].end_time.tv_nsec = 0;
    }

    log_message(global_log_file, LOG_INFO, "Thread timing tracking initialized successfully");
}

/**
 * @brief Starts execution timing measurement for a specified thread.
 *
 * Captures the start timestamp for thread execution timing using CLOCK_MONOTONIC
 * and marks the thread as currently executing. Used for execution time monitoring
 * and overrun detection.
 *
 * @param thread_id Thread identifier for timing measurement (enThread_CCU,
 *                  enThread_FM, etc.)
 *
 * Actions performed:
 * 1. Logs timing start event
 * 2. Captures current timestamp using clock_gettime()
 * 3. Marks thread as executing
 * 4. Records start time in thread_timing structure
 *
 * Error handling:
 * - Logs error if clock_gettime() fails
 * - Returns without updating timing if error occurs
 *
 * @note
 * - Uses CLOCK_MONOTONIC for reliable timing measurement
 * - Should be paired with end_thread_execution_timing()
 * - Critical for thread overrun detection
 *
 */
static void start_thread_execution_timing(thread_label_t thread_id)
{
    log_message(global_log_file, LOG_INFO, "Starting timing for thread %s",
                thread_info[thread_id].name);

    struct timespec current_time;
    if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to get start time for thread %s: %s",
                          thread_info[thread_id].name, strerror(errno));
        return;
    }

    thread_timing[thread_id].start_time = current_time;
    thread_timing[thread_id].is_executing = true;

    log_message(global_log_file, LOG_INFO,
                "Thread %s timing started at %ld.%09ld",
                thread_info[thread_id].name,
                current_time.tv_sec,
                current_time.tv_nsec);
}

/**
 * @brief Completes execution timing measurement and performs overrun analysis for a thread.
 *
 * Captures thread execution end time, calculates execution duration, and checks for
 * timing violations against the thread's budget threshold. Handles timing statistics
 * and overrun detection.
 *
 * @param thread_id Thread identifier (enThread_CCU, enThread_FM, etc.)
 *
 * Actions performed:
 * 1. Validates thread ID and execution state
 * 2. Captures end timestamp using CLOCK_MONOTONIC
 * 3. Calculates execution time in milliseconds
 * 4. Stores timing results
 * 5. Checks for timing violations if not in shutdown
 * 6. Triggers EVENT_ID_FAULT_OVERRUN if threshold exceeded
 *
 * Overrun detection:
 * - Threshold = thread periodicity * THREAD_OVERRUN_THRESHOLD_FACTOR
 * - Logs detailed overrun information if detected
 * - Increments thread's overrun counter
 *
 * Error handling:
 * - Invalid thread ID
 * - Thread not marked as executing
 * - Clock reading failures
 *
 * @note
 * - Must be preceded by start_thread_execution_timing()
 * - Overrun detection skipped during shutdown
 * - Forces log flush on overrun detection
 *
 */
static void end_thread_execution_timing(thread_label_t thread_id)
{
    if (thread_id >= enTotalThreads)
    {
        log_message(global_log_file, LOG_ERROR, "Invalid thread ID in end timing: %d", thread_id);
        return;
    }

    if (!thread_timing[thread_id].is_executing)
    {
        log_message(global_log_file, LOG_ERROR,
                    "Thread %s timing ended while not executing",
                    thread_info[thread_id].name);
        return;
    }

    struct timespec current_time;
    if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to get end time for thread %s: %s",
                          thread_info[thread_id].name, strerror(errno));
        return;
    }

    /* Calculate execution time */
    long execution_time_ms = ((current_time.tv_sec - thread_timing[thread_id].start_time.tv_sec) * 1000) +
                             ((current_time.tv_nsec - thread_timing[thread_id].start_time.tv_nsec) / 1000000);

    /* Store the timing results before checking shutdown state */
    thread_timing[thread_id].last_execution_time_ms = execution_time_ms;
    thread_timing[thread_id].end_time = current_time;
    thread_timing[thread_id].is_executing = false;

    /* Log timing info */
    log_message(global_log_file, LOG_INFO,
                "Thread %s timing ENDED:"
                "\n    Start time: %ld.%09ld"
                "\n    End time:   %ld.%09ld"
                "\n    Duration:   %ld ms",
                thread_info[thread_id].name,
                thread_timing[thread_id].start_time.tv_sec,
                thread_timing[thread_id].start_time.tv_nsec,
                current_time.tv_sec,
                current_time.tv_nsec,
                execution_time_ms);

    /* Skip overrun detection during shutdown */
    if (get_thread_exit())
    {
        log_message(global_log_file, LOG_INFO,
                    "Thread %s final execution time during shutdown: %ld ms",
                    thread_info[thread_id].name, execution_time_ms);
        return;
    }

    /* Check for overrun during normal operation */
    float budget_threshold = thread_info[thread_id].periodicity * THREAD_OVERRUN_THRESHOLD_FACTOR;
    if (execution_time_ms > budget_threshold)
    {
        thread_timing[thread_id].overrun_count++;
        log_message(global_log_file, LOG_ERROR,
                    "\n!!! THREAD OVERRUN DETECTED !!!"
                    "\nThread: %s"
                    "\nExecution Time: %ld ms"
                    "\nBudget: %.1f ms"
                    "\nOverrun Amount: %.1f ms"
                    "\nTotal Overruns: %d",
                    thread_info[thread_id].name,
                    execution_time_ms,
                    budget_threshold,
                    (float)execution_time_ms - budget_threshold,
                    thread_timing[thread_id].overrun_count);

        /* Force log flush to ensure we see the overrun immediately */
        (void)fflush(global_log_file);

        (void)ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_OVERRUN);
    }
}
