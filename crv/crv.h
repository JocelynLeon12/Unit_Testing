/*****************************************************************************
 * @file crv.h
 *****************************************************************************
 * Project Name: Sonatus Automator Safety Interlock(ASI)
 * 
 * @brief Calibration Readback Verification (CRV) Module
 *
 * @details
 * This module implements the CRV functionality within the Sonatus Automator project.
 * It verifies the integrity of calibration data by comparing the readback data
 * against the original calibration data copy stored in the system.
 *
 * @authors Tusar Palauri (TP), Alejandro Tollola (AT)
 * @date November 02, 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 09/17/2024 | TP     | Initial Implementation
 * 09/23/2024 | AT     | Refactoring v1.0 [Functional Testing Issues Fixed]
 * 09/24/2024 | TP     | Refactoring v1.1
 * 09/25/2024 | TP     | Cleaning up the code
 * 10/24/2024 | AT     | Cleaning up the code, removal of DEBUG_LOG
 * 11/02/2024 | TP     | MISRA & LHP compliance fixes
 */

#ifndef CRV_H
#define CRV_H

/*** Include Files ***/
#include "gen_std_types.h"
#include "itcom.h"

/*** Definitions Provided to other modules ***/

/*** Type Definitions ***/

/*** Functions Provided to other modules ***/
extern void CRV_vMainFunction(void);

/*** Variables Provided to other modules ***/

#endif // CRV_H
