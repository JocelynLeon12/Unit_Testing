/*****************************************************************************
 * @file action_request_approver.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 *
 * @brief Action Request Processing and Safety Validation Implementation
 *
 * @details
 * This header file defines the interface for the Action Request Approver (ARA)
 * module in the Sonatus Automator project. It provides structures and definitions
 * for managing vehicle action requests with safety preconditions.
 *
 * Key Features:
 * - Action request validation and safety checks
 * - Vehicle state monitoring (PRNDL status)
 * - Precondition definitions for safety-critical operations
 * - Range limit validation for various action parameters
 * - Structured action request processing
 * - Vehicle status monitoring with error margin handling
 *
 * The implementation ensures that vehicle actions are only executed when all
 * safety conditions are met, with particular emphasis on vehicle state (park
 * status) and parameter range validation.
 *
 * @authors Brian Le (BL), Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date August 14 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 08/14/2024 | BL     | Initial Implementation
 * 10/03/2024 | TP     | Refactored for Action Request Timeout
 * 10/24/2024 | AT     | Clean up actions
 * 11/22/2024 | TP     | Cleaning up the code
 */

#ifndef ARA_ACTION_REQUEST_APPROVER_H
#define ARA_ACTION_REQUEST_APPROVER_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/

/* Identified as not on action list */
#define TEST_NOT_ON_AL (0U)
/* Identified as is on action list */
#define TEST_ON_AL (1U)
/* Identified as not on precondition list */
#define TEST_NOT_ON_PL (0U)
/* Identified as is on precondition list */
#define TEST_ON_PL (1U)
/* Gear selector is not 'P' for parked */
#define VEHICLE_NOT_PARK (0U)
/* Gear selector is 'P' for parked */
#define VEHICLE_PARK (1U)

#define RANGE_CHECK_FAILED (0U)
#define RANGE_CHECK_PASSED (1U)

#define VEHICLE_SPEED_ERROR_MARGIN (0.20F)

#define VEHICLE_STATUS_INFO_SIZE (2U)
#define ACTION_RANGE_LIMITS_SIZE (2U)

/*** Type Definitions ***/

/// @brief Values representing vehicle status from PRNDL signal
typedef enum
{
    enParkStatus = 0,
    enReverseStatus,
    enNeutralStatus,
    enDriveStatus,
    enLowStatus,
    enTotalVehicleStatus
} PRNDL_SignalValues_t;

/* Values representing precondition IDs */
typedef enum
{
    PreID_Invalid = 0, /* Invalid default precondition id */
    PreID_None = 1,    /* ID representing 'no preconditions' */
    PreID_Park = 2,    /* ID representing park and speed 0 precondition */
    PreID_Total = 3    /* Total IDs for preconditions */

} precondition_id_t;

/* Type structure to define the content of the vehicle status signals. */
typedef struct
{
    uint8_t u8InfoStatus[VEHICLE_STATUS_INFO_SIZE];
    uint8_t u8ParkStatus;
    float32_t fVehicleSpeed;
} stVehicleStatusInfo_t;

/* Type structure to define contents of an action request. */
typedef struct
{
    uint16_t u16ActionId;
    precondition_id_t enPrecondId;
    uint32_t au32RangeLimits[ACTION_RANGE_LIMITS_SIZE];
} action_request_t;

/*** Functions Provided to other modules ***/
extern void ARA_vActionRequestMonitor(void);
extern void ARA_vVehicleStatusMonitor(void);
extern uint8_t ARA_u8ActionListCheck(action_request_t *stActionRequest);
extern uint8_t ARA_u8PrecondListCheck(action_request_t stActionRequest);

/*** Variables Provided to other modules ***/

#endif /* ARA_ACTION_REQUEST_APPROVER_H */
