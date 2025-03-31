/*****************************************************************************
 * @file storage_handler.c
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Storage handler implementation providing robust storage management,
 *        data persistence, and logging functionality.
 *
 * @details
File Operations:
 * - Thread-safe file reading and writing
 * - Atomic storage operations
 * - Permission management
 * - Resource cleanup
 *Data Management:
 * - Shared data persistence
 * - Storage file validation
 * - Data integrity checks
 * - State synchronization
 *Logging System:
 * - Timestamped log entries
 * - Multiple severity levels
 * - Process-specific logging
 * - Thread-safe logging operations
 *Error Handling:
 * - Comprehensive error detection
 * - Robust recovery mechanisms
 * - Resource leak prevention
 * - State consistency maintenance
 *The implementation ensures:
 * - POSIX-compliant file operations
 * - Proper synchronization mechanisms
 * - Secure file handling practices
 * - Reliable data persistence
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

/*** Include Files ***/
#include "storage_handler.h"

/*** Module Definitions ***/

/*** Internal Types ***/

/*** Local Function Prototypes ***/
static ret_status_t create_storage_file(str_const_t const filepath);
static valid_status_t is_file_valid(str_const_t const filepath);
static void read_shared_data_from_file(str_const_t const filename, DataOnSharedMemory *const data);

/*** External Variables ***/
FILE *global_log_file = NULL;

/*** Internal Variables ***/

/*** Functions Provided to other modules ***/

/**
 * @brief Writes a formatted log message with timestamp and log level to a specified file.
 *
 * This function provides thread-safe logging capabilities with the following features:
 *  - Automatic timestamp generation in YYYY-MM-DD HH:MM:SS format
 *  - Log level categorization (ERROR, WARNING, INFO, DEBUG)
 *  - Variable argument support for formatted messages
 *  - File locking for thread safety
 *  - Immediate flush to disk
 *  - Buffer overflow protection
 *
 * @param[in] storage_log_file Pointer to the FILE where log messages will be written.
 *                            If NULL, function returns without action.
 * @param[in] level           Log level indicator. Valid values are:
 *                           - LOG_ERROR   (0)
 *                           - LOG_WARNING (1)
 *                           - LOG_INFO    (2)
 *                           - LOG_DEBUG   (3)
 *                           Invalid values default to "UNKNOWN"
 * @param[in] format         Printf-style format string for the log message
 * @param[in] ...           Variable arguments matching the format string
 *
 * @note
 * - Thread Safety: This function is thread-safe through the use of flockfile/funlockfile
 * - Buffer Limits: Message buffer is limited to 1024 bytes
 * - Timestamp buffer is limited to 20 bytes
 * - The function performs immediate flush after writing
 *
 * @warning
 * - The function assumes the log file has been properly opened with write permissions
 * - No retry mechanism is implemented for failed write operations
 * - The function does not validate the format string against the variable arguments
 *
 * @return
 * The function does not return a value. Error conditions are handled as follows:
 * - NULL file pointer: Silent return
 * - Failed localtime_r(): Silent return
 * - Failed strftime(): Silent return
 * - Failed write operations: Message is lost
 */
void log_message(FILE *storage_log_file, const log_level_t level, const str_const_t format, ...)
{
    if (storage_log_file == NULL)
    {
        return;
    }

    va_list args;
    str_t buffer[1024];
    str_t timestamp[20];
    time_t now = time(NULL);
    struct tm time_struct;

    /* Use thread-safe localtime_r instead of localtime */
    if (localtime_r(&now, &time_struct) == NULL)
    {
        return;
    }

    if (strftime((char *)timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &time_struct) == 0)
    {
        return;
    }

    /* Array of level strings matching existing LOG_* defines */
    static const str_const_t level_strings[] = {"ERROR", "WARNING", "INFO", "DEBUG", "UNKNOWN"};

    /* Bounds checking for level parameter */
    const str_const_t level_str = (level >= 0 && level <= LOG_DEBUG) ? level_strings[level] : level_strings[4];

    /* Thread-safe file operations */
    flockfile(storage_log_file);

    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    (void)fprintf(storage_log_file, "[%s] [%s] %s\n", timestamp, level_str, buffer);
    (void)fflush(storage_log_file);

    funlockfile(storage_log_file);
}

/**
 * @brief Creates and configures the storage directory with appropriate permissions.
 *
 * This function ensures the existence of a storage directory (STORAGE_DIR_PATH) with 
 * proper access permissions. The function performs the following operations:
 *  - Checks for directory existence using stat()
 *  - Creates directory if it doesn't exist
 *  - Sets standardized UNIX permissions
 *  - Logs operation status and errors
 *
 * @return ret_status_t Return status indicating success or failure:
 *         - STORAGE_SUCCESS (0): Directory exists or was created successfully with proper permissions
 *         - STORAGE_ERROR (-1): Failed to create directory or set permissions
 *
 * @note Directory Permissions:
 * The function sets the following permissions (0755):
 *  - Owner (User):  Read, Write, Execute (rwx)
 *  - Group:         Read, Execute (r-x)
 *  - Others:        Read, Execute (r-x)
 *
 * @note Permissions are set using symbolic constants:
 *  - S_IRWXU (Owner permissions)
 *  - S_IRGRP | S_IXGRP (Group permissions)
 *  - S_IROTH | S_IXOTH (Others permissions)
 *
 * @note
 * This function is part of the storage initialization sequence and should be 
 * called before any storage file operations are attempted.
 *
 * @par Dependencies:
 *  - Requires write access to parent directory of STORAGE_DIR_PATH
 *  - Requires global_log_file to be initialized for error logging
 *  - Requires STORAGE_DIR_PATH macro to be defined
 *
 * @par Platform Compatibility:
 * The function uses platform-independent permission bits (S_IRWXU, etc.) 
 * instead of octal values for better portability across different UNIX-like systems.
 *
 * @par Thread Safety:
 * This function is not guaranteed to be thread-safe. It should be called during 
 * initialization before any concurrent operations begin.
 *
 * @warning
 *  - Function does not handle race conditions if multiple processes attempt
 *    to create the directory simultaneously
 *  - Function does not create parent directories if they don't exist
 *  - Function assumes global_log_file is properly initialized
 */
ret_status_t create_storage_directory(void)
{
    struct stat st = {0};

    /* Use platform-independent permission bits */
    mode_t const dir_permissions = S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH;

    if (stat(STORAGE_DIR_PATH, &st) == -1)
    {
        /* 493U (0755) maintained for backward compatibility but use symbolic form internally */
        if (mkdir(STORAGE_DIR_PATH, dir_permissions) == -1)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to create storage directory: %s", strerror(errno));
            return STORAGE_ERROR;
        }
    }

    /* Set directory permissions explicitly */
    if (chmod(STORAGE_DIR_PATH, dir_permissions) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to set directory permissions: %s", strerror(errno));
        return STORAGE_ERROR;
    }

    return STORAGE_SUCCESS;
}

/**
 * @brief Writes shared data structure to a persistent storage file with proper synchronization and permissions.
 *
 * This function performs atomic writing of shared data to a specified file with the following guarantees:
 *  - Atomic file writing using O_WRONLY | O_CREAT | O_TRUNC flags
 *  - Explicit file permissions control
 *  - Data synchronization to disk using fsync()
 *  - Complete error handling and logging
 *
 * @param[in] filename    Path to the target storage file. Must be non-NULL.
 * @param[in] data        Pointer to the shared data structure to be written.
 *                        Must be non-NULL and point to valid DataOnSharedMemory instance.
 *
 * @note File Permissions:
 * The function sets the following permissions (0644):
 *  - Owner: Read, Write (rw-)
 *  - Group: Read (r--)
 *  - Others: Read (r--)
 * 
 * Permissions are set using symbolic constants:
 *  - S_IRUSR | S_IWUSR (Owner permissions)
 *  - S_IRGRP (Group permissions)
 *  - S_IROTH (Others permissions)
 *
 * @par Error Handling:
 * The function handles and logs the following error conditions:
 *  - NULL parameters (silent return)
 *  - File open failures
 *  - Write failures (both partial and complete)
 *  - fsync failures
 *  - File close failures
 *  - Permission setting failures
 *
 * All errors are logged using log_message() with:
 *  - Error description
 *  - System error string (via strerror(errno))
 *  - Relevant context information
 *
 * @par Data Integrity:
 * The function ensures data integrity through:
 *  - Complete write verification (size checking)
 *  - Explicit fsync() call to ensure data is written to disk
 *  - Atomic file operation using O_TRUNC
 *
 * @par Implementation Details:
 * 1. Parameter validation
 * 2. File creation/opening with proper flags
 * 3. Single write operation for entire data structure
 * 4. Synchronization to disk
 * 5. File closure
 * 6. Permission setting
 *
 * @warning
 *  - Function overwrites existing file if it exists
 *  - No backup of existing file is maintained
 *  - Size of DataOnSharedMemory must match between writer and reader
 *  - The function assumes global_log_file is properly initialized
 *  - No file locking mechanism is implemented - concurrent access must be
 *    handled by the caller
 *
 * @dependencies
 *  - global_log_file must be initialized
 *  - Write permissions for target directory
 *  - Sufficient disk space
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent writes to the same file
 *  - No concurrent reads during write operation
 *  - Data structure is not modified during write
 *
 * @par Platform Compatibility:
 * Uses POSIX file operations and permission bits for compatibility across
 * UNIX-like systems. The following system calls are used:
 *  - open(), write(), fsync(), close(), chmod()
 *
 */
void write_shared_data_to_file(str_const_t filename, DataOnSharedMemory *data)
{
    if ((filename == NULL) || (data == NULL))
    {
        return;
    }

    /* Use platform-independent permission bits */
    mode_t const file_permissions = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH; /* 420U */

    file_desc_t const fd = open((const char *)filename, O_WRONLY | O_CREAT | O_TRUNC, file_permissions);

    if (fd == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to open file for writing: %s", strerror(errno));
        return;
    }

    ssize_t const write_result = write(fd, data, sizeof(DataOnSharedMemory));
    if (write_result != sizeof(DataOnSharedMemory))
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to write data to file: %s", (write_result == -1) ? strerror(errno) : "Incomplete write");
        (void)close(fd);
        return;
    }

    /* Ensure data is synced to disk */
    if (fsync(fd) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to sync file: %s", strerror(errno));
        (void)close(fd);
        return;
    }

    ret_status_t const close_result = close(fd);
    if (close_result == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to close file: %s", strerror(errno));
        return;
    }

    if (chmod(filename, file_permissions) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to set file permissions on %s: %s", filename, strerror(errno));
    }
}

/**
 * @brief Reads shared data structure from a storage file with error handling and data validation.
 *
 * This static function safely reads a DataOnSharedMemory structure from a specified file,
 * with comprehensive error handling and data validation. If reading fails, the output
 * structure is zeroed to ensure defined state.
 *
 * @param[in]  filename    Path to the storage file to read. Must be non-NULL.
 * @param[out] data        Pointer to DataOnSharedMemory structure where read data
 *                         will be stored. Must be non-NULL. Will be zeroed on read failure.
 *
 * @pre
 *  - filename must be a valid, null-terminated string
 *  - data must point to an allocated DataOnSharedMemory structure
 *  - global_log_file must be initialized for error logging
 *
 * @post
 *  - On successful read: data contains file contents
 *  - On failed read: data is zeroed
 *  - File is properly closed regardless of read success/failure
 *
 * @note
 * Function Behavior:
 *  1. Opens file in read-only mode
 *  2. Attempts to read exact size of DataOnSharedMemory
 *  3. Validates read size matches expected size
 *  4. Zeros data structure on any error
 *  5. Ensures file closure in all cases
 *
 * @par Error Handling:
 * The function handles and logs the following error conditions:
 *  - NULL parameters (silent return)
 *  - File open failures
 *  - Read failures (both partial and complete)
 *  - File close failures
 *  - Memory clear failures
 *
 * All errors are logged using log_message() with:
 *  - Specific error description
 *  - System error string (via strerror(errno))
 *  - Context information where relevant
 *
 * @par Data Safety:
 * The function ensures data safety through:
 *  - Read-only file access (O_RDONLY)
 *  - Complete structure read verification
 *  - Data structure zeroing on any error
 *  - Safe memory operations using memset()
 *
 * @par Implementation Details:
 * 1. Parameter validation
 * 2. File opening with O_RDONLY flag
 * 3. Single read operation for entire structure
 * 4. Size verification
 * 5. Error handling with structure zeroing
 * 6. File closure
 *
 * @warning
 *  - Function assumes DataOnSharedMemory size matches between writer and reader
 *  - No data validation beyond size checking is performed
 *  - No file locking mechanism is implemented - concurrent access must be
 *    handled by the caller
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent writes during read operation
 *  - No concurrent reads from same file
 *  - Output structure is not accessed during read
 *
 * @par Platform Compatibility:
 * Uses standard POSIX file operations:
 *  - open(), read(), close()
 * Memory operations:
 *  - memset() for structure clearing
 *
 */
static void read_shared_data_from_file(str_const_t const filename, DataOnSharedMemory *const data)
{
    if ((filename == NULL) || (data == NULL))
    {
        return;
    }

    file_desc_t const fd = open((const char *)filename, O_RDONLY);
    if (fd == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to open file for reading: %s", strerror(errno));
        return;
    }

    ssize_t const read_result = read(fd, data, sizeof(DataOnSharedMemory));
    if (read_result != sizeof(DataOnSharedMemory))
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to read data from file: %s", (read_result == -1) ? strerror(errno) : "Incomplete read");

        /* Clear the data structure to ensure it's initialized */
        void *const memset_result = memset(data, 0, sizeof(DataOnSharedMemory));
        if (memset_result != data)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Memory clear operation failed");
            (void)close(fd);
            return;
        }
    }

    ret_status_t const close_result = close(fd);
    if (close_result != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to close file: %s", strerror(errno));
    }
}

/**
 * @brief Compares and loads shared data from parent and child storage files with validation.
 *
 * This function implements a robust storage loading strategy by comparing parent and child
 * storage files, selecting the appropriate data source, and initializing the shared data
 * structure accordingly. The function follows a specific precedence and validation logic
 * to ensure data consistency.
 *
 * @param[out] shared_data Pointer to shared data structure where the loaded data
 *                        will be stored. Must be non-NULL.
 *
 * @return ret_status_t Status of the operation:
 *         - STORAGE_SUCCESS (0): Data successfully loaded from at least one valid source
 *         - STORAGE_ERROR (-1): No valid storage files found or critical error occurred
 *
 * @par Loading Strategy:
 * The function follows this precedence order:
 * 1. If both files exist and are valid:
 *    - If contents match: Load from child storage
 *    - If contents differ: Load from child storage (child preferred)
 * 2. If only child storage is valid: Load from child
 * 3. If only parent storage is valid: Load from parent
 * 4. If no valid storage: Return STORAGE_ERROR
 *
 * @par Validation Steps:
 * For each storage file:
 * 1. Check file existence
 * 2. Verify file size matches DataOnSharedMemory structure
 * 3. Validate read operations
 * 4. Compare data when both files are valid
 *
 * @note
 * Log messages are generated for:
 *  - Storage file validity status
 *  - File comparison results
 *  - Source selection decisions
 *  - Memory operation failures
 *
 * @par Error Handling:
 * The function handles:
 *  - NULL shared_data pointer
 *  - Invalid storage files
 *  - Memory copy failures
 *  - File read errors
 * All error conditions are logged via log_message()
 *
 * @par Implementation Details:
 * 1. Parameter validation
 * 2. Storage file validity checking
 * 3. Data structure initialization
 * 4. File content reading
 * 5. Data comparison when applicable
 * 6. Selected data copying to output structure
 *
 * @par Memory Management:
 * The function uses stack-allocated structures for temporary storage:
 *  - parent_data: Holds data read from parent storage
 *  - child_data: Holds data read from child storage
 * These are zero-initialized at declaration
 *
 * @warning
 *  - Function assumes parent and child storage paths are defined (PARENT_STORAGE_PATH, CHILD_STORAGE_PATH)
 *  - No file locking mechanism is implemented - concurrent access must be handled by caller
 *  - Memory copying uses memcpy() - structures must not contain pointers
 *  - The function assumes global_log_file is properly initialized
 *
 * @par Performance Considerations:
 * The function may perform:
 *  - Up to two file existence checks
 *  - Up to two complete file reads
 *  - One memory comparison (if both files valid)
 *  - One memory copy operation
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent modifications to storage files
 *  - No concurrent access to shared_data structure
 *
 * @par Dependencies:
 *  - is_file_valid() for file validation
 *  - read_shared_data_from_file() for data reading
 *  - log_message() for status logging
 *  - memcpy() for data copying
 *  - memcmp() for data comparison
 *
 * The function does not implement internal synchronization. The caller must ensure:
 *  - Atomic access to storage files
 *  - Atomic access to shared_data structure
 *  - Proper initialization of global resources
 */
ret_status_t compare_and_load_storage(DataOnSharedMemory *const shared_data)
{
    if (shared_data == NULL)
    {
        return STORAGE_ERROR;
    }

    /* Check if parent storage file exists and is valid */
    valid_status_t const parent_valid = is_file_valid(PARENT_STORAGE_PATH);

    /* Check if child storage file exists and is valid */
    valid_status_t const child_valid = is_file_valid(CHILD_STORAGE_PATH);

    /* Declare structures to hold data from storage files */
    DataOnSharedMemory parent_data = {0};
    DataOnSharedMemory child_data = {0};

    /* If parent storage file is valid, read its contents */
    if (parent_valid != 0)
    {
        read_shared_data_from_file(PARENT_STORAGE_PATH, &parent_data);
        (void)log_message(global_log_file, LOG_INFO, "Parent storage file is valid");
    }

    /* If child storage file is valid, read its contents */
    if (child_valid != 0)
    {
        read_shared_data_from_file(CHILD_STORAGE_PATH, &child_data);
        (void)log_message(global_log_file, LOG_INFO, "Child storage file is valid");
    }

    /* If both files are valid, compare their contents */
    if ((parent_valid != 0) && (child_valid != 0))
    {
        if (memcmp(&parent_data, &child_data, sizeof(DataOnSharedMemory)) == 0)
        {
            (void)log_message(global_log_file, LOG_INFO, "Parent & Child storage files are identical");
            if (memcpy(shared_data, &child_data, sizeof(DataOnSharedMemory)) != shared_data)
            {
                (void)log_message(global_log_file, LOG_ERROR, "Memory copy operation failed");
                return STORAGE_ERROR;
            }
        }
        else
        {
            if (memcpy(shared_data, &child_data, sizeof(DataOnSharedMemory)) != shared_data)
            {
                (void)log_message(global_log_file, LOG_ERROR, "Memory copy operation failed");
                return STORAGE_ERROR;
            }
            (void)log_message(global_log_file, LOG_INFO, "Parent & Child storage files content differ");
        }
    }
    else if (child_valid != 0)
    {
        if (memcpy(shared_data, &child_data, sizeof(DataOnSharedMemory)) != shared_data)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Memory copy operation failed");
            return STORAGE_ERROR;
        }
        (void)log_message(global_log_file, LOG_INFO, "Only Child storage file is valid, using it for SharedMemory");
    }
    else if (parent_valid != 0)
    {
        if (memcpy(shared_data, &parent_data, sizeof(DataOnSharedMemory)) != shared_data)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Memory copy operation failed");
            return STORAGE_ERROR;
        }
        (void)log_message(global_log_file, LOG_INFO, "Only Parent storage file is valid, using it for SharedMemory");
    }
    else
    {
        (void)log_message(global_log_file, LOG_INFO, "No storage file are valid, Returning a Failure");
        return STORAGE_ERROR;
    }

    return STORAGE_SUCCESS;
}

/**
 * @brief Initializes storage files with proper permissions and error handling.
 *
 * This function manages the creation and initialization of parent and/or child storage files
 * based on provided flags. It ensures proper file permissions, handles errors comprehensively,
 * and maintains a consistent storage state.
 *
 * @param[in] storage_flags Bit flags indicating which storage files to initialize:
 *                         - STORAGE_FILE_PARENT (1): Initialize parent storage
 *                         - STORAGE_FILE_CHILD  (2): Initialize child storage
 *                         - 0: Initialize both storage files (default behavior)
 *                         Any other flag values are considered invalid
 *
 * @return ret_status_t Status of the initialization:
 *         - STORAGE_SUCCESS (0): All requested files initialized successfully
 *         - STORAGE_ERROR (-1): Any of the following conditions:
 *           * Invalid flags provided
 *           * File creation failed
 *           * Permission setting failed
 *
 * @par Initialization Process:
 * For each requested file:
 * 1. Create/truncate file with initial permissions
 * 2. Set final permissions (0644):
 *    - Owner: Read, Write (rw-)
 *    - Group: Read (r--)
 *    - Others: Read (r--)
 *
 * @note Permission Details:
 * Permissions are set using symbolic constants:
 *  - S_IRUSR | S_IWUSR (Owner permissions)
 *  - S_IRGRP (Group permissions)
 *  - S_IROTH (Others permissions)
 *
 * @par Error Handling:
 * The function handles and logs:
 *  - Invalid flag values
 *  - File creation failures
 *  - Permission setting failures
 * All errors are logged via log_message() with:
 *  - Error description
 *  - System error string (strerror(errno))
 *  - Context information
 *
 * @par Implementation Details:
 * 1. Flag validation
 * 2. Local flag copy creation
 * 3. Default to both files if no flags specified
 * 4. Sequential file initialization:
 *    a. Parent file (if requested)
 *    b. Child file (if requested)
 * 5. Permission setting
 * 6. Status tracking
 *
 * @warning
 *  - Existing files will be truncated
 *  - No backup of existing files is maintained
 *  - Function assumes storage directory exists
 *  - The function assumes global_log_file is properly initialized
 *  - No file locking mechanism is implemented
 *
 * @par Performance Considerations:
 * For each file, the function performs:
 *  - One create_storage_file() call
 *  - One chmod() system call
 *  - Error logging when necessary
 *
 * @par Dependencies:
 *  - create_storage_file() for file creation
 *  - log_message() for error logging
 *  - chmod() for permission setting
 *  - PARENT_STORAGE_PATH and CHILD_STORAGE_PATH must be defined
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent file operations on target files
 *  - No concurrent access to storage directory
 *
 * @par Critical Requirements:
 *  - Write permission to storage directory
 *  - Valid storage paths defined
 *  - Sufficient disk space
 *  - Proper permission to set file modes
 *
 * @see
 *  - create_storage_file()
 *  - STORAGE_FILE_PARENT
 *  - STORAGE_FILE_CHILD
 *  - PARENT_STORAGE_PATH
 *  - CHILD_STORAGE_PATH
 *
 * @note Initialization Sequence:
 * This function should be called:
 *  - After storage directory creation
 *  - Before any storage operations
 *  - When storage files need to be reset
 *
 * @par Recovery Behavior:
 * If initialization of one file fails:
 *  - Error is logged
 *  - Function continues with next file
 *  - Returns STORAGE_ERROR after completing all operations
 */
ret_status_t initialize_storage_files(const storage_flags_t storage_flags)
{
    if ((storage_flags & ~(STORAGE_FILE_PARENT | STORAGE_FILE_CHILD)) != 0U)
    {
        /* Invalid flags provided */
        return STORAGE_ERROR;
    }

    /* Create a local copy of the storage flags */
    storage_flags_t const local_flags = (storage_flags == 0U) ? (STORAGE_FILE_PARENT | STORAGE_FILE_CHILD) : storage_flags;

    ret_status_t result = STORAGE_SUCCESS;
    mode_t const file_permissions = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH; /* 292U */

    /* Initialize parent storage file if requested */
    if ((local_flags & STORAGE_FILE_PARENT) != 0U)
    {
        ret_status_t const create_result = create_storage_file(PARENT_STORAGE_PATH);
        if (create_result == STORAGE_ERROR)
        {
            result = STORAGE_ERROR;
        }
        else if (chmod(PARENT_STORAGE_PATH, file_permissions) == -1)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to set file permissions on %s: %s", PARENT_STORAGE_PATH, strerror(errno));
            result = STORAGE_ERROR;
        }
        else
        {
            /* Successfully created and set permissions for parent file */
        }
    }

    /* Initialize child storage file if requested */
    if ((local_flags & STORAGE_FILE_CHILD) != 0U)
    {
        ret_status_t const create_result = create_storage_file(CHILD_STORAGE_PATH);
        if (create_result == STORAGE_ERROR)
        {
            result = STORAGE_ERROR;
        }
        else if (chmod(CHILD_STORAGE_PATH, file_permissions) == -1)
        {
            (void)log_message(global_log_file, LOG_ERROR, "Failed to set file permissions on %s: %s", CHILD_STORAGE_PATH, strerror(errno));
            result = STORAGE_ERROR;
        }
        else
        {
            /* Successfully created and set permissions for child file */
        }
    }

    return result;
}

/*** Local Functions ***/

/**
 * @brief Creates or truncates a storage file with proper permissions and synchronization.
 *
 * This static function handles the creation of storage files with appropriate permissions,
 * ensuring data consistency through explicit synchronization and comprehensive error handling.
 *
 * @param[in] filepath Path to the storage file to be created. Must be a non-NULL,
 *                    null-terminated string representing a valid file path.
 *
 * @return ret_status_t Status of the file creation:
 *         - STORAGE_SUCCESS (0): File created or truncated successfully
 *         - STORAGE_ERROR (-1): Any of these conditions:
 *           * NULL filepath
 *           * File creation failed
 *           * Sync to disk failed
 *           * File close failed
 *
 * @par File Creation Strategy:
 * The function opens/creates the file with:
 * - O_WRONLY: Write-only access
 * - O_CREAT: Create if doesn't exist
 * - O_TRUNC: Truncate if exists
 *
 * @note Permission Details:
 * Initial file permissions (0644):
 *  - Owner: Read, Write (rw-)
 *  - Group: Read (r--)
 *  - Others: Read (r--)
 *
 * Permissions are set using symbolic constants:
 *  - S_IRUSR | S_IWUSR (Owner permissions)
 *  - S_IRGRP (Group permissions)
 *  - S_IROTH (Others permissions)
 *
 * @par Data Integrity:
 * Function ensures data integrity through:
 * 1. Atomic file creation/truncation
 * 2. Explicit fsync() call
 * 3. Proper error checking
 * 4. Complete error handling
 *
 * @par Error Handling:
 * The function handles and logs:
 *  - NULL filepath parameter
 *  - File creation/opening failures
 *  - Synchronization failures
 *  - File closure failures
 *
 * All errors are logged via log_message() with:
 *  - Specific error description
 *  - System error string (via strerror(errno))
 *  - Relevant filepath information
 *
 * @par Implementation Details:
 * 1. Parameter validation
 * 2. File creation with proper flags and permissions
 * 3. Disk synchronization
 * 4. Resource cleanup
 * Each step includes error checking and logging
 *
 * @warning
 *  - Function will truncate existing files
 *  - No backup of existing files is maintained
 *  - Function assumes storage directory exists
 *  - The function assumes global_log_file is properly initialized
 *  - No file locking mechanism is implemented
 *
 * @par System Calls:
 * The function performs these system calls:
 * - open() with O_WRONLY | O_CREAT | O_TRUNC
 * - fsync() for disk synchronization
 * - close() for file handle
 *
 * @par Performance Considerations:
 * The function involves:
 *  - One file open operation
 *  - One fsync operation (can be slow)
 *  - One file close operation
 *  - Error logging when necessary
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent access to target file
 *  - No concurrent operations on the same filepath
 *
 * @par Platform Compatibility:
 * Uses POSIX-compliant system calls:
 *  - open(), fsync(), close()
 * Permission bits are platform-independent
 *
 * @par Critical Requirements:
 *  - Write permission to target directory
 *  - Valid filepath
 *  - Sufficient disk space
 *  - Proper permissions to create files
 *
 * @note
 * This is a static function, only available within the storage handler module.
 * It is primarily used by initialize_storage_files().
 *
 * @par Error Recovery:
 * On error conditions:
 * 1. Resources are properly cleaned up
 * 2. Error is logged
 * 3. STORAGE_ERROR is returned
 * 4. Caller must handle the error condition
 *
 */
static ret_status_t create_storage_file(str_const_t const filepath)
{
    if (filepath == NULL)
    {
        return STORAGE_ERROR;
    }

    /* Use platform-independent permission bits */
    mode_t const file_permissions = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;

    /* Open with explicit flags for better control and platform independence */
    file_desc_t const fd = open((const char *)filepath, O_WRONLY | O_CREAT | O_TRUNC, file_permissions);

    if (fd == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to create storage file %s: %s", filepath, strerror(errno));
        return STORAGE_ERROR;
    }

    /* Ensure the file is synced to disk */
    if (fsync(fd) == -1)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to sync file %s: %s", filepath, strerror(errno));
        (void)close(fd);
        return STORAGE_ERROR;
    }

    /* Ensure proper file closure */
    ret_status_t const close_result = close(fd);
    if (close_result != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to close file %s: %s", filepath, strerror(errno));
        return STORAGE_ERROR;
    }

    return STORAGE_SUCCESS;
}

/**
 * @brief Validates storage file existence and size.
 *
 * Checks if a file exists and has the exact size of DataOnSharedMemory structure.
 * Logs non-existence errors via log_message().
 *
 * @param[in] filepath Path to the file to validate. Must be non-NULL.
 *
 * @return valid_status_t Status of validation:
 *         - 1: File exists and has correct size
 *         - 0: File invalid (doesn't exist, wrong size, or NULL filepath)
 *
 * @note Thread-unsafe. Caller must handle synchronization.
 *
 */
static valid_status_t is_file_valid(str_const_t const filepath)
{
    if (filepath == NULL)
    {
        return 0;
    }

    struct stat st;
    if (stat((const char *)filepath, &st) == -1)
    {
        if (errno != ENOENT)
        { /* Only log if error is not "file doesn't exist" */
            (void)log_message(global_log_file, LOG_INFO, "File access error: %s", strerror(errno));
        }
        return 0;
    }

    return (valid_status_t)(st.st_size == sizeof(DataOnSharedMemory));
}

/**
 * @brief Persists all shared data to non-volatile storage with comprehensive error handling.
 *
 * This function orchestrates the complete persistence of shared data by coordinating
 * the storage of both shared memory data and event data. It implements a sequential
 * storage strategy with appropriate error logging and status tracking.
 *
 * @param[in] shared_data Pointer to the DataOnSharedMemory structure containing
 *                       the shared data to be persisted. Must be non-NULL and
 *                       point to a valid, initialized structure.
 *
 * @note Storage Operations:
 * The function performs storage in this sequence:
 * 1. Logs initiation of storage operation
 * 2. Writes shared memory data to parent storage file
 * 3. Saves event data through FM_s8SaveEventDataToStorage()
 * 4. Logs completion status
 *
 * @par Data Persistence Strategy:
 * Shared data is persisted in two components:
 * - Shared memory data -> PARENT_STORAGE_PATH
 * - Event data -> Through FM_s8SaveEventDataToStorage()
 *
 * @par Error Handling:
 * The function handles and logs:
 *  - NULL shared_data pointer (silent return)
 *  - Event data storage failures
 *  - Shared data writing failures (via write_shared_data_to_file)
 *
 * All operation stages are logged:
 *  - Operation initiation
 *  - Event data storage failures
 *  - Operation completion
 *
 * @warning
 *  - Function assumes global_log_file is properly initialized
 *  - No rollback mechanism if event data storage fails
 *  - Parent storage file will be overwritten
 *  - No automatic backup is created
 *  - The function does not verify data integrity after writing
 *
 * @par Implementation Details:
 * 1. Parameter validation (NULL check)
 * 2. Initial operation logging
 * 3. Shared memory data persistence
 * 4. Event data persistence
 * 5. Status logging
 *
 * @par Dependencies:
 * Function relies on:
 *  - write_shared_data_to_file() for shared memory persistence
 *  - FM_s8SaveEventDataToStorage() for event data persistence
 *  - log_message() for operation logging
 *  - Properly initialized global_log_file
 *
 * @par Performance Considerations:
 * The function performs:
 *  - Multiple logging operations
 *  - One complete shared data write
 *  - One event data save operation
 * Timing may vary based on:
 *  - Size of shared data
 *  - Volume of event data
 *  - Storage medium performance
 *
 * @par Thread Safety:
 * This function is not thread-safe. Caller must ensure:
 *  - No concurrent modifications to shared_data
 *  - No concurrent access to storage files
 *  - No concurrent event data modifications
 *
 * @par Critical Sections:
 * The function provides no internal synchronization. Caller must ensure:
 *  - Atomic access to shared_data structure
 *  - Proper synchronization with event data operations
 *  - Exclusive access to storage files
 *
 * @par Recovery Behavior:
 * On event data storage failure:
 *  - Error is logged
 *  - Function completes execution
 *  - No rollback of shared memory data
 *
 * @note Operational Context:
 * This function should be called:
 *  - During normal shutdown
 *  - At designated checkpointing intervals
 *  - Before system state changes
 *  - When data persistence is required
 *
 * @par Platform Requirements:
 *  - Write access to storage directory
 *  - Sufficient storage space
 *  - Properly configured file permissions
 *
 */
void save_all_shared_data_to_storage(DataOnSharedMemory *shared_data)
{
    if (shared_data == NULL)
    {
        return;
    }

    (void)log_message(global_log_file, LOG_INFO, "All shared data saving to persistent storage initiated...");

    /* Save shared data to file */
    write_shared_data_to_file(PARENT_STORAGE_PATH, shared_data);

    /* Save event data */
    int8_t const save_result = FM_s8SaveEventDataToStorage();
    if (save_result != 0)
    {
        (void)log_message(global_log_file, LOG_ERROR, "Failed to save event data");
    }

    (void)log_message(global_log_file, LOG_INFO, "All shared data saved to persistent storage");
}