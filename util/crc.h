/*****************************************************************************
 * @file crc.c
 *****************************************************************************
 * @brief Cyclic Redundancy Check (CRC) Module
 *
 * @details
 * This module implements the CRC functionality within the Sonatus Automator project.
 * It provides CRC-16 CCITT calculation capabilities for data verification purposes.
 * The module uses a pre-calculated lookup table to optimize CRC calculations and
 * supports both single-byte and multi-byte data processing.
 *
 * @authors Brian Le (BL), Tusar Palauri (TP)
 * @date November 07, 2024
 *
 * Version History:
 * ---------------
 * Date       | Author | Description
 * -----------|--------|-------------
 * 06/10/2024 | BL     | Initial Implementation
 * 08/13/2024 | BL     | SUD baseline 0.4
 * 11/07/2024 | TP     | MISRA & LHP compliance fixes
 * 
 */

#ifndef CRC_H
#define CRC_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/
#define CRC_ERROR_MAX_VALUE     (3U)
#define CRC_TABLE_SIZE          (256U)
#define CRC_BITS_PER_BYTE      (8U)

/*** Functions Provided to other modules ***/
extern uint16_t CRC_u16CalculateCrc(const uint8_t* const pu8Data, const uint16_t u16Size);
extern void CRC_vCreateTable(void);

#endif /* CRC_H */
