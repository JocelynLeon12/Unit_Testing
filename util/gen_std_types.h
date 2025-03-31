/*********************************************************************************
 * @file gen_std_types.h
 *********************************************************************************
 * @brief Standard type definitions and common data types header file
 *        This module provides standardized type definitions, platform-independent 
 *        data types, and common constants used throughout the project. It ensures 
 *        type safety, portability, and MISRA C:2023 compliance.
 *
 * Key features:
 * - Fixed-width integer type definitions (8, 16, 32, 64-bit)
 * - Floating-point type definitions (32, 64-bit)
 * - String and status type definitions
 * - Common status codes and utility macros
 * - Platform-independent type definitions
 * - MISRA C:2023 compliant type definitions
 * 
 * Usage:
 * Include this header in modules that require standard type definitions
 * and common constants. All type definitions are guarded against
 * multiple definitions for safe inclusion.
 * 
 * @note This file should be included before any other project-specific headers
 *       to ensure consistent type definitions across the project.
 *
 * @authors Brian Le (BL), Tusar Palauri (TP), Alejandro Tollola (AT)
 * 
 * @date November 05, 2024
 *
 * Version History:
 * -------------------------------------------------------------------------------
 * Date       | Author | Description
 * -----------|--------|----------------------------------------------------------
 * 08/13/2024 | BL     | Initial Implementation
 * 11/05/2024 | TP     | Added comprehensive floating-point type support
 * 11/05/2024 | TP     | Added status codes and utility macros
 * 11/05/2024 | TP     | MISRA C:2023 compliance fixes
 * 
 */


#ifndef UTIL_GEN_STD_TYPES_H_
#define UTIL_GEN_STD_TYPES_H_


/*** Include Files ***/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <setjmp.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <sys/select.h>

/*** Type Definitions ***/
/* Integer type definitions if not provided by stdint.h */
#ifndef UINT8_T_DEFINED
#define UINT8_T_DEFINED
typedef unsigned char uint8_t;
#endif

#ifndef INT8_T_DEFINED
#define INT8_T_DEFINED
typedef signed char int8_t;
#endif

#ifndef UINT16_T_DEFINED
#define UINT16_T_DEFINED
typedef unsigned short uint16_t;
#endif

#ifndef INT16_T_DEFINED
#define INT16_T_DEFINED
typedef signed short int16_t;
#endif

#ifndef UINT32_T_DEFINED
#define UINT32_T_DEFINED
typedef unsigned int uint32_t;
#endif

#ifndef INT32_T_DEFINED
#define INT32_T_DEFINED
typedef signed int int32_t;
#endif

#ifndef UINT64_T_DEFINED
#define UINT64_T_DEFINED
typedef unsigned long uint64_t;
#endif

#ifndef INT64_T_DEFINED
#define INT64_T_DEFINED
typedef signed long int64_t;
#endif

/* Float type definitions */
#ifndef FLOAT32_T_DEFINED
#define FLOAT32_T_DEFINED
typedef float float32_t;
#endif

#ifndef FLOAT64_T_DEFINED
#define FLOAT64_T_DEFINED
typedef double float64_t;
#endif

/* String type definitions for specific uses */
typedef const char* error_string_t;
typedef const char* event_type_t;
typedef const char* thread_name_t;
typedef const char* file_path_t;
typedef const char* string_t;

/* Signal type definitions */
typedef const char* sig_name_t;
typedef sig_atomic_t sig_num_t;

/* Thread type definitions */
typedef int32_t thread_id_t;
typedef const char* thread_name_t;
typedef int32_t thread_priority_t;
typedef int32_t thread_period_t;
typedef int32_t thread_counter_t;

/* Thread status codes type definition */
typedef enum thread_status_code thread_status_code_t;

/* Timer type definitions */
typedef int32_t timer_id_t;
typedef int32_t timer_period_t;
typedef thread_status_code_t timer_status_t;

/* Status type definitions */
typedef int32_t status_t;
typedef int32_t mutex_status_t;

/* Alias for void pointer */
typedef void* generic_ptr_t;

/* Alias for const void pointer */
typedef const void* const_generic_ptr_t;

/* Character type definitions for fault manager */
typedef char fm_char_t;

/* Status code type definitions */
typedef int status_code_t;

/* Retry count type definitions */
typedef int32_t retry_count_t;


/*** Constant Definitions ***/
/* Alias for boolean type */
#define TRUE                            (1U)
#define FALSE                           (0U)

#define INACTIVE_FLAG					(0U)
#define ACTIVE_FLAG						(1U)

/* Status codes */
#define E_OK                            ((status_t)0)
#define E_NOT_OK                        ((status_t)-1)
#define MESSAGE_NOT_FOUND               ((status_t)-3)
#define MESSAGE_TYPE_NOT_FOUND          ((status_t)-4)
#define ASSOCIATED_LENGTH_NOT_FOUND     ((status_t)-5)
#define RATE_LIMIT_EXCEEDED             ((status_t)-6)

/* Utility macros */
#define VALID_PTR(ptr)                  ((NULL != (ptr)) ? TRUE : FALSE)

/* Size constants */
#define UINT16_MAX_VALUE                (65535U)

#endif /* UTIL_GEN_STD_TYPES_H_ */
