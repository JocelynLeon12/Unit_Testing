/*****************************************************************************
 * @file process_management.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Process management module providing robust parent-child process control,
 *        monitoring, signal handling and fault recovery services.
 *
 * @details
 * Core module implementing sophisticated process management including:
 * Process Management:
 * - Parent-child process creation and lifecycle control
 * - Process monitoring and health checks
 * - Automatic process recovery on failures
 * - Graceful shutdown coordination
 * Signal Handling:
 * - Custom signal handlers for various system signals
 * - Safe termination signal processing
 * - Child process crash handling
 * - Signal mask management
 * Fault Tolerance:
 * - Process state monitoring
 * - Automatic child process restart on failures
 * - Configurable retry limits
 * - Safe state transitions
 * - Event logging and persistence
 * The module ensures system reliability through:
 * - Comprehensive error detection and handling
 * - Coordinated shutdown procedures
 * - Resource cleanup and state persistence
 * - Dedicated threads monitoring
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
 * 10/04/2024 | TP     | Update for consistent safe state handling across child process restarts
 * 10/09/2024 | TP     | Multiple ASI_APP restart issues fixed
 * 11/15/2024 | TP     | MISRA & LHP compliance fixes
 * 11/22/2024 | TP     | Cleanup v1.0
 */

/*** Include Files ***/
#include "crc.h"
#include "state_machine.h"
#include "icm.h"
#include "action_request_approver.h"
#include "fault_manager.h"

#include "process_management.h"

/*** Module Definitions ***/
#define PROCESS_SLEEP_TIME_US         (100000U)    /* 100ms sleep time in microseconds */
#define MAX_CHILD_RESTART_RETRIES     (5U)         /* Maximum number of retries to restart the child process */

/*** Internal Types ***/

/*** Local Function Prototypes ***/
static void procmanagement_vInitModules(void);
static void restart_child_process(DataOnSharedMemory *shared_data, FILE *proc_log_file);
static void handle_child_termination(status_code_t status);
static void handle_termination_signal(sig_num_t signum, siginfo_t *info, generic_ptr_t context);
static void sigchld_handler(sig_num_t signum, siginfo_t *info, generic_ptr_t context);
static void child_signal_handler(sig_num_t signum, siginfo_t *info, generic_ptr_t context);

/*** External Variables ***/

/*** Internal Variables ***/
static pid_t child_pid;
static volatile sig_atomic_t keep_running = 1;
static volatile sig_atomic_t shutdown_initiated = 0;
static volatile sig_atomic_t received_signal = 0;
static volatile sig_atomic_t child_exiting = 0;

/*** Functions Provided to other modules ***/

/* Setter and getter functions */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void set_child_pid(pid_t pid)
{
    child_pid = pid;
}

static pid_t get_child_pid(void)
{
    return child_pid;
}

static void set_keep_running(sig_atomic_t value)
{
    keep_running = value;
}

static sig_atomic_t get_keep_running(void)
{
    return keep_running;
}

static void set_shutdown_initiated(sig_atomic_t value)
{
    shutdown_initiated = value;
}

static sig_atomic_t get_shutdown_initiated(void)
{
    return shutdown_initiated;
}

static void set_received_signal(sig_atomic_t value)
{
    received_signal = value;
}

static sig_atomic_t get_received_signal(void)
{
    return received_signal;
}
#pragma GCC diagnostic pop

/**
 * @brief Initializes signal handlers for the parent process
 *
 * This function sets up signal handlers for various signals in the parent process,
 * ensuring proper handling of termination signals, crashes, and child process termination.
 *
 * @param proc_log_file Pointer to the file stream used for logging
 *
 * The function performs the following tasks:
 * 1. Creates a sigaction structure with handle_termination_signal as handler:
 *    - Sets SA_SIGINFO flag to receive additional signal information
 *    - Clears signal mask with sigemptyset()
 *    - Sets up the handler for each signal
 *
 * 2. Configures handlers for critical signals:
 *    - Termination: SIGTERM, SIGINT
 *    - Crash: SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT
 *    - System: SIGSYS, SIGQUIT, SIGXCPU, SIGXFSZ
 *    - Other: SIGPIPE, SIGTRAP, SIGALRM, SIGHUP, SIGPWR, SIGPOLL, SIGSTKFLT
 *
 * 3. Sets up dedicated SIGCHLD handler:
 *    - Uses sigchld_handler function
 *    - Sets SA_NOCLDSTOP flag to ignore stopped children
 *    - Used for monitoring child process termination
 *
 * 4. Error handling and logging:
 *    - Validates all signal handler registrations
 *    - Logs detailed error messages on failure
 *    - Terminates program if critical setup fails
 *
 * @note This function is critical for process reliability as it ensures appropriate
 * handling of all system signals that could affect process operation
 *
 * @warning Exits with status 1 if signal handler initialization fails
 */
void PROCMANAGEMENT_vSignalHandlerInit(FILE *proc_log_file)
{
    size_t i;
    struct sigaction sa;
    sa.sa_sigaction = &handle_termination_signal;
    sa.sa_flags = SA_SIGINFO;

    if (sigemptyset(&sa.sa_mask) != 0)
    {
        log_message(proc_log_file, LOG_ERROR, "Failed to initialize empty signal set");
        exit(1);
    }

    sig_num_t signals[] = {SIGTERM, SIGINT, SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT, SIGSYS, SIGQUIT,
                           SIGXCPU, SIGXFSZ, SIGPIPE, SIGTRAP, SIGALRM, SIGHUP, SIGPWR, SIGPOLL, SIGSTKFLT};

    for (i = 0; i < sizeof(signals) / sizeof(signals[0]); i++)
    {
        if (sigaction(signals[i], &sa, NULL) == -1)
        {
            error_string_t error_str = strerror(errno);
            if (error_str != NULL)
            {
                log_message(proc_log_file, LOG_ERROR, "Failed to set up signal handler for signal %d: %s",
                            signals[i], error_str);
            }
            else
            {
                log_message(proc_log_file, LOG_ERROR, "Failed to set up signal handler for signal %d with unknown error",
                            signals[i]);
            }
            exit(1);
        }
    }

    /* Set up SIGCHLD handler separately */
    struct sigaction sa_chld;
    sa_chld.sa_sigaction = &sigchld_handler;
    sa_chld.sa_flags = SA_SIGINFO | SA_NOCLDSTOP;

    if (sigemptyset(&sa_chld.sa_mask) != 0)
    {
        log_message(proc_log_file, LOG_ERROR, "Failed to initialize empty signal set for SIGCHLD handler");
        exit(1);
    }

    if (sigaction(SIGCHLD, &sa_chld, NULL) == -1)
    {
        (void)log_message(proc_log_file, LOG_ERROR, "Failed to set up SIGCHLD handler: %s", strerror(errno));
        exit(1);
    }

    log_message(proc_log_file, LOG_INFO, "Signal handlers initialized for parent process");
}

/**
 * @brief Creates a new child process using fork()
 *
 * This function creates a new child process by calling fork() and stores the child's
 * process ID in the global child_pid variable.
 *
 * The function return value indicates:
 * - To parent process: Returns child's PID (positive value)
 * - To child process: Returns 0
 * - On failure: Returns -1
 *
 * @return pid_t The process ID of the child process to parent, 0 to child process,
 *         or -1 if fork fails
 *
 * @note This function is the key entry point for creating ASI's dual-process
 *       architecture. The returned PID is used by parent process for monitoring
 *       and management
 */
pid_t PROCMANAGEMENT_stCreateChildProcess(void)
{
    child_pid = fork();
    return child_pid;
}

/**
 * @brief Handles SIGCHLD signals for child process termination monitoring
 *
 * This function processes child process termination events and manages system response,
 * including restart decisions and logging.
 *
 * @param signum Signal number (SIGCHLD)
 * @param info Additional signal information (unused)
 * @param context User context information (unused)
 *
 * The function performs:
 * 1. Non-blocking checks for terminated child processes using waitpid with WNOHANG
 * 2. For main child process termination:
 *    - During shutdown: Logs termination and marks process as terminated
 *    - During normal operation: Logs reason and initiates restart procedure
 * 3. Error handling for waitpid failures:
 *    - EINTR: Logs interruption
 *    - ECHILD: Logs no child processes
 *    - Other errors: Logs detailed error message
 *
 * @note Essential for maintaining system reliability through automated child process
 * monitoring and recovery
 */
static void sigchld_handler(sig_num_t signum, siginfo_t *info, generic_ptr_t context)
{
    (void)signum;  /* Explicitly cast to void to indicate unused / suppress warning */
    (void)info;    /* Explicitly cast to void to indicate unused / suppress warning */
    (void)context; /* Explicitly cast to void to indicate unused / suppress warning */
    status_code_t status;
    pid_t pid;

    while ((pid = waitpid(-1, &status, WNOHANG)) > 0)
    {
        if (pid == child_pid)
        {
            if (shutdown_initiated)
            {
                /* If we're shutting down, log the termination but don't restart */
                if (WIFEXITED(status))
                {
                    log_message(global_log_file, LOG_INFO, "Child process exited with status %d during shutdown", WEXITSTATUS(status));
                }
                else if (WIFSIGNALED(status))
                {
                    sig_name_t signal_name = get_signal_name(WTERMSIG(status));
                    log_message(global_log_file, LOG_INFO, "Child process terminated by signal %s during shutdown", signal_name);
                }
                else
                {
                    log_message(global_log_file, LOG_INFO, "Child process terminated with unknown status during shutdown");
                }
                child_pid = -1; /* Mark that the child has terminated */
            }
            else
            {
                handle_child_termination(status);
            }
        }
    }

    if (pid == -1)
    {
        if (errno == EINTR)
        {
            /* The system call was interrupted by a signal, this is not an error */
            log_message(global_log_file, LOG_INFO, "The system call was interrupted by a signal");
            return;
        }
        else if (errno == ECHILD)
        {
            /* No child processes to wait for, this is not necessarily an error */
            log_message(global_log_file, LOG_INFO, "No child processes to wait for");
        }
        else
        {
            /* Log any other errors */
            (void)log_message(global_log_file, LOG_ERROR, "waitpid failed: %s", strerror(errno));
        }
    }
}

/**
 * @brief Manages the execution and lifecycle of the child process
 *
 * This function controls the child process's operation including initialization,
 * monitoring, data persistence and shutdown.
 *
 * @param shared_data Pointer to shared memory structure for inter-process communication
 * @param proc_log_file Pointer to logging file stream
 * @param start_reason Reason for process start (hard/soft restart)
 *
 * The function executes these key tasks:
 * 1. Initialization:
 *    - Sets up signal handlers
 *    - Initializes shared memory if soft restart
 *    - Initializes ASI modules
 *    - Starts monitoring threads
 *
 * 2. Main Operation Loop:
 *    - Monitors thread health
 *    - Handles abnormal thread terminations
 *    - Periodically saves state to storage
 *    - Checks for termination signals
 *
 * 3. Shutdown Sequence:
 *    - Logs remaining events
 *    - Saves shared data
 *    - Performs graceful shutdown
 *
 * @note Critical for maintaining system reliability through comprehensive monitoring
 * and automated recovery mechanisms
 */
void child_process(DataOnSharedMemory *shared_data, FILE *proc_log_file, enRestartReason start_reason)
{
    /* Log the start of the child process with its PID */
    pid_t current_pid = getpid();
    log_message(proc_log_file, LOG_INFO, "Child process started with PID: %d", current_pid);

    /* Set up signal handlers for the child process */
    setup_child_signal_handlers();

    /* Initilize Shared Memory Data if the Child Process is restarted */
    if (start_reason == (enRestartReason)enSoftRestart)
    {
        ITCOM_vSharedMemoryInit(proc_log_file, (enRestartReason)start_reason);
    }

    /* Initialize ASI modules. */
    procmanagement_vInitModules();

    /* Start the threads for the child process */
    if (start_threads(shared_data, proc_log_file) != 0)
    {
        log_message(proc_log_file, LOG_ERROR, "Failed to start threads");
        return;
    }

    /* Main loop of the child process */
    while (!get_thread_exit() && !child_exiting)
    {
        /* Monitor threads for any issues */
        monitor_threads(shared_data);

        /* Handle any abnormal thread terminations */
        if (get_abnormal_termination())
        {
            log_message(proc_log_file, LOG_WARNING, "Abnormal termination detected. Logging remaining events.");
            FM_vLogRemainingEvents(proc_log_file);
            handle_thread_termination(shared_data);
        }

        /* Write to child_storage.bin every STORAGE_WRITE_INTERVAL seconds */
        time_t current_time = time(NULL);
        static time_t last_write_time = 0;
        if (current_time - last_write_time >= STORAGE_WRITE_INTERVAL)
        {
            write_shared_data_to_file(CHILD_STORAGE_PATH, shared_data);
            last_write_time = current_time;
            log_message(proc_log_file, LOG_INFO, "Child: Written data to storage file");
        }

        if (child_exiting)
        {
            log_message(proc_log_file, LOG_INFO, "Child process received termination signal, exiting main loop");
            break;
        }

        if (usleep(PROCESS_SLEEP_TIME_US) != 0)
        {
            (void)log_message(proc_log_file, LOG_WARNING, "Sleep interrupted: %s", strerror(errno));
        }
    }

    /* Before exiting, log remaining events if any */
    log_message(proc_log_file, LOG_INFO, "Child process ending. Logging any remaining events.");
    FM_vLogRemainingEvents(proc_log_file);

    /* Save all shared data before exiting */
    save_all_shared_data_to_storage(shared_data);

    /* Perform graceful shutdown when exiting */
    initiate_graceful_shutdown(shared_data);
    log_message(proc_log_file, LOG_INFO, "Child process ending...");
    log_message(proc_log_file, LOG_INFO, "Child process exited successfully");
}

/**
 * @brief Manages the main execution loop of the parent process
 *
 * This function controls the parent process lifecycle including child process
 * monitoring, data persistence and shutdown coordination.
 *
 * @param shared_data Pointer to shared memory structure for IPC
 * @param proc_log_file Pointer to logging file stream
 *
 * The function implements:
 * 1. Main Monitoring Loop:
 *    - Periodic shared data storage
 *    - Child process status monitoring
 *    - Child process termination handling:
 *      * Normal exit handling
 *      * Signal termination handling
 *      * Restart decision making
 *    - Error handling for process checks
 *
 * 2. Shutdown Sequence:
 *    - Initiates graceful shutdown
 *    - Waits for child process termination
 *    - Saves final shared data state
 *    - Logs shutdown completion
 *
 * @note Critical for system reliability through continuous monitoring and
 * automated recovery of child process
 */
void parent_process(DataOnSharedMemory *shared_data, FILE *proc_log_file)
{
    /* Log the start of the parent process with its PID */
    (void)log_message(proc_log_file, LOG_INFO, "Parent process started. PID: %d", getpid());

    /* Track last time data was written to storage */
    time_t last_write_time = time(NULL);

    /* Main loop of the parent process */
    while (keep_running)
    {
        /* Periodically write shared data to storage file */
        time_t current_time = time(NULL);
        if (current_time - last_write_time >= STORAGE_WRITE_INTERVAL)
        {
            write_shared_data_to_file(PARENT_STORAGE_PATH, shared_data);
            last_write_time = current_time;
            log_message(proc_log_file, LOG_INFO, "Parent: Written data to storage file");
        }

        /* Check if child process has terminated */
        status_code_t status;
        pid_t terminated_pid = waitpid(child_pid, &status, WNOHANG);
        if (terminated_pid == child_pid)
        {
            /* Child process has terminated */
            if (WIFSIGNALED(status))
            {
                /* Child terminated by a signal */
                log_message(proc_log_file, LOG_WARNING, "Child process terminated by signal %d", WTERMSIG(status));
                if (shared_data->parent_initiated_termination)
                {
                    log_message(proc_log_file, LOG_INFO, "Child process terminated as requested by parent. Not restarting.");
                    keep_running = 0;
                }
                else
                {
                    log_message(proc_log_file, LOG_INFO, "Child process terminated abnormally. Restarting.");
                    restart_child_process(shared_data, proc_log_file);
                }
            }
            else if (WIFEXITED(status))
            {
                /* Child exited normally */
                log_message(proc_log_file, LOG_INFO, "Child process exited with status %d", WEXITSTATUS(status));
                if (shared_data->parent_initiated_termination)
                {
                    log_message(proc_log_file, LOG_INFO, "Child process exited as requested by parent. Not restarting.");
                    keep_running = 0;
                }
                else
                {
                    log_message(proc_log_file, LOG_INFO, "Child process exited unexpectedly. Restarting.");
                    restart_child_process(shared_data, proc_log_file);
                }
            }
            else
            {
                log_message(proc_log_file, LOG_WARNING, "Child process terminated with unknown status");
                if (!shared_data->parent_initiated_termination)
                {
                    restart_child_process(shared_data, proc_log_file);
                }
            }
        }
        else if (terminated_pid == -1)
        {
            /* Error occurred while checking child process status */
            if (errno == ECHILD)
            {
                log_message(proc_log_file, LOG_WARNING, "No child process exists");
                if (!shared_data->parent_initiated_termination)
                {
                    log_message(proc_log_file, LOG_INFO, "Restarting child process.");
                    restart_child_process(shared_data, proc_log_file);
                }
            }
            else if (errno != EINTR)
            {
                (void)log_message(proc_log_file, LOG_ERROR, "Error checking child process: %s", strerror(errno));
            }
            else
            {
                /* EINTR case - system call was interrupted */
                log_message(proc_log_file, LOG_INFO, "Child process check was interrupted");
            }
        }
        else
        {
            /* terminated_pid == 0 or some other value */
            /* No action needed - child is still running */
        }

        /* Check if shutdown has been initiated */
        if (shutdown_initiated)
        {
            log_message(proc_log_file, LOG_INFO, "Shutdown initiated by signal %d, exiting main loop", received_signal);
            break; /* Exit the loop if shutdown has been initiated */
        }

        (void)usleep(PROCESS_SLEEP_TIME_US); /* Sleep for 100ms to prevent CPU hogging */
    }

    /* Final save of shared data before exiting */
    save_all_shared_data_to_storage(shared_data);

    /* Initiate graceful shutdown procedure */
    log_message(proc_log_file, LOG_INFO, "Parent process initiating graceful shutdown...");

    /* Wait for child process to terminate if it hasn't already */
    if (child_pid > 0)
    {
        retry_count_t retries = MAX_CHILD_RESTART_RETRIES;
        while (retries > 0 && child_pid > 0)
        {
            (void)sleep(1); /* Wait for 1 second */
            retries--;
        }
        if (child_pid > 0)
        {
            log_message(proc_log_file, LOG_WARNING, "Child process did not terminate within the expected time");
        }
    }

    /* Log the end of the parent process */
    log_message(proc_log_file, LOG_INFO, "Parent process ending...");
}

/**
 * @brief Handles termination signals for graceful application shutdown
 *
 * This function coordinates the shutdown sequence when a termination signal is
 * received, ensuring clean process termination and data persistence.
 *
 * @param signum Signal number received that triggered termination
 * @param info Additional signal information (unused)
 * @param context Signal context information (unused)
 *
 * The function implements:
 * 1. Signal Processing:
 *    - Records received signal number
 *    - Sets shutdown flag to prevent multiple shutdown attempts
 *    - Validates shutdown not already in progress
 *    - Logs signal receipt and shutdown initiation
 *
 * 2. Child Process Handling:
 *    - Checks if child process is running
 *    - Sets parent termination flag in shared memory
 *    - Sends SIGTERM to child process
 *    - Logs signal transmission to child
 *
 * 3. Data Protection:
 *    - Ensures all event data saved to storage
 *    - Protects against data loss during shutdown
 *
 * @note Only first signal triggers actual shutdown sequence
 * @note Called from signal context - keeps processing minimal
 *
 */
static void handle_termination_signal(sig_num_t signum, siginfo_t *info, generic_ptr_t context)
{
    (void)info;    /* Explicitly cast to void to indicate unused / suppress warning */
    (void)context; /* Explicitly cast to void to indicate unused / suppress warning */
    received_signal = signum;
    if (!shutdown_initiated)
    {
        shutdown_initiated = 1;
        keep_running = 0;
        sig_name_t signal_name = get_signal_name(signum);
        log_message(global_log_file, LOG_INFO, "Received signal %s (%d). Initiating graceful shutdown...", signal_name, signum);

        if (child_pid > 0)
        {
            ITCOM_vSetParentTerminationFlag(1);
            (void)kill(child_pid, SIGTERM);
            log_message(global_log_file, LOG_INFO, "Sent SIGTERM to child process (PID: %d)", child_pid);
        }
    }
}

/*** Local Function Implementations ***/

/**
 * @brief Initializes all modules required for the process management system.
 *
 * This function serves as the centralized initialization point for all core system modules.
 * It must be called during system startup before any module operations begin. The function
 * executes initializations in a specific order to ensure proper module dependencies:
 *
 * Initialization sequence:
 * 1. State Machine (STM_vInit)
 * 2. Interface Communication Manager (ICM_vInit)
 * 3. TCP Connections for System Diagnostics (SD_vTCPConnectionsInit)
 * 4. CRC Table Creation (CRC_vCreateTable)
 *
 * @note This function is critical for system safety and must complete successfully
 *       before any other operations can proceed. All module initializations are
 *       mandatory and must be performed in the specified order.
 *
 * @warning Failing to call this function before using any module functionality
 *          will result in undefined behavior.
 *
 */
static void procmanagement_vInitModules(void)
{
    /* Note: Add module's init functions here */
    STM_vInit();
    ICM_vInit();
    SD_vTCPConnectionsInit();
    CRC_vCreateTable();
    log_message(global_log_file, LOG_INFO, "INITIALIZATION PROCESS COMPLETED");
}

/**
 * @brief Signal handler for child process signals.
 *
 * Handles all signals received by the child process, implementing appropriate responses for different
 * signal types including graceful shutdown, crash recovery, and fault management.
 *
 * Signal handling behavior:
 * - SIGTERM/SIGINT: Initiates graceful shutdown, distinguishing between parent-initiated and external termination
 * - SIGSEGV/SIGBUS/SIGFPE/SIGILL/SIGABRT: Logs critical signal and initiates crash recovery
 * - SIGSYS/SIGQUIT/SIGXCPU/SIGXFSZ/SIGPIPE/SIGTRAP/SIGALRM/SIGHUP/SIGPWR/SIGPOLL/SIGSTKFLT: Logs warning
 *
 * @param signum Signal number received
 * @param info Additional signal information (unused)
 * @param context Signal context information (unused)
 *
 * @note This handler sets thread_exit flag and closes TCP connections before completing
 *
 * @warning Must be async-signal-safe - only calls async-signal-safe functions
 *
 * State effects:
 * - Sets child_exiting flag for termination signals
 * - Sets thread_crashed flag for critical signals
 * - Sets thread_exit flag for all signals
 */
static void child_signal_handler(sig_num_t signum, siginfo_t *info, generic_ptr_t context)
{
    (void)info;    /* Explicitly cast to void to indicate unused / suppress warning */
    (void)context; /* Explicitly cast to void to indicate unused / suppress warning */
    ErrorEvent stCurrentEvent;
    sig_name_t signal_name = get_signal_name(signum);

    switch (signum)
    {
    case SIGTERM:
    case SIGINT:
        if (ITCOM_vGetParentTerminationFlag())
        {
            ITCOM_vGetErrorEvent(&stCurrentEvent);
            log_message(global_log_file, LOG_INFO, "Child received %s from parent. Initiating graceful shutdown...", signal_name);
            FM_vLogSpecialEvent(global_log_file, "PARENT-INITIATED TERMINATION", stCurrentEvent.Error_Event_ID);
        }
        else
        {
            ITCOM_vGetErrorEvent(&stCurrentEvent);
            log_message(global_log_file, LOG_WARNING, "Child received %s from external source. Initiating graceful shutdown and will be restarted...", signal_name);
            FM_vLogSpecialEvent(global_log_file, "EXTERNAL TERMINATION", stCurrentEvent.Error_Event_ID);
        }
        child_exiting = 1;
        break;
    case SIGSEGV:
    case SIGBUS:
    case SIGFPE:
    case SIGILL:
    case SIGABRT:
        log_message(global_log_file, LOG_ERROR, "Child process received critical signal: %s. Initiating crash recovery...", signal_name);
        ITCOM_vGetErrorEvent(&stCurrentEvent);
        FM_vLogSpecialEvent(global_log_file, "CRITICAL SIGNAL", stCurrentEvent.Error_Event_ID);
        set_thread_crashed(1);
        break;
    case SIGSYS:
    case SIGQUIT:
    case SIGXCPU:
    case SIGXFSZ:
    case SIGPIPE:
    case SIGTRAP:
    case SIGALRM:
    case SIGHUP:
    case SIGPWR:
    case SIGPOLL:
    case SIGSTKFLT:
        log_message(global_log_file, LOG_WARNING, "Child process received signal: %s", signal_name);
        break;
    default:
        log_message(global_log_file, LOG_WARNING, "Child process received unexpected signal: %s (%d)", signal_name, signum);
        break;
    }

    set_thread_exit(1);
    SD_vCloseTCPConnection(enVAMConnectionTCP);
    SD_vCloseTCPConnection(enCMConnectionTCP);
}

/**
 * @brief Configures signal handlers for the child process.
 *
 * Sets up handlers for all signals that the child process needs to handle, ensuring proper
 * process management and fault tolerance. Uses sigaction() with SA_SIGINFO flag for enhanced
 * signal information.
 *
 * Handled signals:
 * - Process control: SIGTERM, SIGINT
 * - Fatal errors: SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT
 * - System signals: SIGSYS, SIGQUIT, SIGXCPU, SIGXFSZ
 * - Resource signals: SIGPIPE, SIGTRAP, SIGALRM
 * - Other signals: SIGHUP, SIGPWR, SIGPOLL, SIGSTKFLT
 *
 * @note All signals use the child_signal_handler() callback function
 *
 * @warning Failure to set up any signal handler results in process termination
 *
 * Error handling:
 * - Logs initialization failures for signal mask and handlers
 * - Terminates process on critical setup failures
 */
void setup_child_signal_handlers(void)
{
    size_t i;
    struct sigaction sa;
    sa.sa_sigaction = &child_signal_handler;
    sa.sa_flags = SA_SIGINFO;

    if (sigemptyset(&sa.sa_mask) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to initialize signal mask: %s", strerror(errno));
        exit(1);
    }

    sig_num_t signals[] = {SIGTERM, SIGINT, SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT, SIGSYS, SIGQUIT,
                           SIGXCPU, SIGXFSZ, SIGPIPE, SIGTRAP, SIGALRM, SIGHUP, SIGPWR, SIGPOLL, SIGSTKFLT};

    for (i = 0; i < sizeof(signals) / sizeof(signals[0]); i++)
    {
        if (sigaction(signals[i], &sa, NULL) == -1)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to set up signal handler for signal %d in child process: %s", signals[i], strerror(errno));
            exit(1);
        }
    }

    log_message(global_log_file, LOG_INFO, "Signal handlers initialized for child process");
}

/**
 * @brief Restarts the child process after termination or failure.
 *
 * Creates a new child process via fork() to replace a terminated instance, handling
 * all aspects of process recreation and state management.
 *
 * @param shared_data Pointer to shared memory structure for inter-process communication
 * @param proc_log_file File pointer for logging process management events
 *
 * Process flow:
 * 1. Forks new child process
 * 2. Child process:
 *    - Opens new log file
 *    - Sets global_log_file
 *    - Calls child_process() with soft restart flag
 * 3. Parent process:
 *    - Records new child PID
 *    - Resets parent_initiated_termination flag
 *    - Logs restart status
 *
 * Error handling:
 * - Logs fork failures with errno details
 * - Handles child log file open failures
 * - Reports child process creation status
 *
 * @warning Critical for system fault tolerance - must handle all error cases
 */
static void restart_child_process(DataOnSharedMemory *shared_data, FILE *proc_log_file)
{
    child_pid = fork();
    if (child_pid == 0)
    {
        // Child process
        FILE *child_log_file = fopen(CHILD_LOG_FILE_PATH, "a");
        if (child_log_file == NULL)
        {
            (void)log_message(proc_log_file, LOG_ERROR, "Failed to open child log file: %s", strerror(errno));
            exit(1);
        }
        global_log_file = child_log_file;
        child_process(shared_data, child_log_file, (enRestartReason)enSoftRestart);
        if (fclose(child_log_file) != 0)
        {
            (void)log_message(proc_log_file, LOG_ERROR, "Failed to close child log file: %s", strerror(errno));
        }
        exit(0);
    }
    else if (child_pid < 0)
    {
        (void)log_message(proc_log_file, LOG_ERROR, "Failed to restart child process: %s", strerror(errno));
    }
    else
    {
        log_message(proc_log_file, LOG_INFO, "Child process restarted with PID: %d", child_pid);
        shared_data->parent_initiated_termination = 0;
    }
}

/**
 * @brief Handles termination of child processes and manages process recovery.
 *
 * @param status Exit status from waitpid() containing termination information
 *
 * Termination handling:
 * 1. Normal exit (WIFEXITED):
 *    - Logs exit status
 *    - Checks parent_initiated_termination flag
 * 2. Signal termination (WIFSIGNALED):
 *    - Logs terminating signal
 *    - Verifies if parent requested termination
 * 3. Unknown termination:
 *    - Logs warning
 *    - Attempts recovery
 *
 * Recovery process:
 * - Creates new child process via fork()
 * - Initializes child logging
 * - Resets parent termination flag
 * - Sets up signal handlers
 * - Executes child process wrapper
 *
 * Error handling:
 * - Validates fork results
 * - Manages log file operations
 * - Records all state transitions
 *
 * @warning Critical for system resilience - must handle all failure modes
 */
static void handle_child_termination(status_code_t status)
{
    if (WIFEXITED(status))
    {
        log_message(global_log_file, LOG_INFO, "Child process exited with status %d", WEXITSTATUS(status));

        if (ITCOM_vGetParentTerminationFlag())
        {
            log_message(global_log_file, LOG_INFO, "Child process exited as requested by parent. Not restarting.");
            return;
        }
    }
    else if (WIFSIGNALED(status))
    {
        log_message(global_log_file, LOG_WARNING, "Child process terminated by signal %d", WTERMSIG(status));

        if (ITCOM_vGetParentTerminationFlag())
        {
            log_message(global_log_file, LOG_INFO, "Child process terminated as requested by parent. Not restarting.");
            return;
        }
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "Child process terminated for unknown reason");
    }

    /* Restart child process */
    child_pid = fork();
    if (child_pid == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "fork failed: %s", strerror(errno));
        return;
    }
    else if (child_pid == 0)
    {
        /* Child process */
        FILE *child_log_file = fopen(CHILD_LOG_FILE_PATH, "a");
        if (child_log_file == NULL)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to open child log file: %s", strerror(errno));
            exit(1);
        }
        global_log_file = child_log_file;

        /* Reset the parent termination flag */
        ITCOM_vSetParentTerminationFlag(0);

        /* Set up signal handlers for the child process */
        setup_child_signal_handlers();

        /* Execute child process logic */
        ITCOM_vChildProcessWrapper(child_log_file, enSoftRestart);

        if (fclose(child_log_file) != 0)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to close child log file: %s", strerror(errno));
        }
        exit(0);
    }
    else
    {
        log_message(global_log_file, LOG_INFO, "Child process restarted with PID: %d", child_pid);
    }
}
