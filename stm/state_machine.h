//*****************************************************************************
/**
* @file state_machine.h
*****************************************************************************
* PROJECT NAME: Sonatus Automator
* ORIGINATOR: Sonatus
*
* @brief module to implement state machine
*
* @authors Brian Le
*
* @date June 05 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 06/05/2024|BL |Initial
* 08/13/2024|BL |SUD baseline 0.4
* 10/04/2024|TP |Update for consistent safe state handling across child process restarts
*
*/
//*****************************************************************************

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/*** Include Files ***/
#include "gen_std_types.h"


/*** Definitions Provided to other modules ***/


/*** Type Definitions ***/
/// state of state machine
typedef uint8_t states_t;

// Define state values as constants instead of enum
#define STATE_INITIAL      ((states_t)0U)
#define STATE_NORM_OP      ((states_t)1U)
#define STATE_STARTUP_TEST ((states_t)2U)
#define STATE_SAFE_STATE   ((states_t)3U)
#define STATE_INVALID      ((states_t)4U)

/*** Functions Provided to other modules ***/
void STM_vInit(void);
void STM_vMainTask(void);


/*** Variables Provided to other modules ***/


#endif /* STATE_MACHINE_H */

