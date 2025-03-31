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

/*** Include Files ***/
#include "itcom.h"

#include "crc.h"

/*** Module Definitions ***/
/* CRC polynomial (0x1021 polynomial of crc16 CCITT) */
#define CRC_POLYNOMIAL          (0x1021U)
/* Initial CRC value (crc16 CCITT) */
#define CRC_INITIAL_VALUE       (0xFFFFU)
/* Mask for most significant bit check */
#define CRC_MSB_MASK           (0x8000U)
/* Mask for byte value */
#define CRC_BYTE_MASK          (0xFFU)
/* Shift amount for single bit */
#define CRC_SHIFT_ONE          (1U)
/* Zero initialization value */
#define CRC_ZERO_INIT          (0U)

/*** Internal Types ***/

/*** Local Function Prototypes ***/

/*** External Variables ***/


/*** Internal Variables ***/
/* CRC lookup table - static since only used in this file */
static uint16_t ms_au16CrcTable[CRC_TABLE_SIZE] = { CRC_ZERO_INIT };


/**
 * @brief Calculates 16-bit CRC CCITT value for given data array.
 *
 * This function computes a CRC-16 CCITT checksum for data verification using
 * a pre-calculated lookup table approach for optimized performance. It processes
 * data byte by byte with MSB-first ordering.
 *
 * @param[in] pu8Data    Pointer to data array (MSB first)
 * @param[in] u16Size    Number of bytes to process
 *
 * @return uint16_t      Calculated CRC-16 CCITT value
 */
uint16_t CRC_u16CalculateCrc(const uint8_t* const pu8Data, const uint16_t u16Size)
{
    uint16_t u16CrcValue = CRC_INITIAL_VALUE;
    uint16_t u16Index = CRC_ZERO_INIT;
    uint16_t u16TableIndex;
    uint16_t u16TempValue;
    uint16_t u16ByteValue;
    const uint8_t* const pu8NullPtr = NULL;  /* Create typed NULL pointer for comparison */

    if ((pu8NullPtr != pu8Data) && (u16Size > (uint16_t)CRC_ZERO_INIT))
    {
        for (u16Index = CRC_ZERO_INIT; u16Index < u16Size; u16Index++)
        {
            /* Calculate table index */
            u16TempValue = u16CrcValue >> (uint16_t)CRC_BITS_PER_BYTE;
            u16ByteValue = (uint16_t)pu8Data[u16Index];
            u16TableIndex = (u16TempValue ^ u16ByteValue) & (uint16_t)CRC_BYTE_MASK;
            
            /* Get CRC value from table and update */
            u16TempValue = ms_au16CrcTable[u16TableIndex];
            u16CrcValue = (u16CrcValue << (uint16_t)CRC_BITS_PER_BYTE) ^ u16TempValue;
        }
    }

    return u16CrcValue;
}


/**
 * @brief Generates and initializes the CRC-16 CCITT lookup table.
 *
 * This function pre-calculates the CRC-16 CCITT lookup table values using the
 * standard polynomial (0x1021). The table is used to optimize CRC calculations
 * by avoiding repetitive bit-by-bit operations.
 *
 * @param void          No input parameters required
 * @return void         No return value
 */
void CRC_vCreateTable(void)
{
    uint16_t u16TableIndex = CRC_ZERO_INIT;
    uint16_t u16BitCount = CRC_ZERO_INIT;
    uint16_t u16CurrentValue = CRC_ZERO_INIT;
    uint8_t u8InitFlagStatus = ITCOM_u8GetInitFlagStatus();

    for (u16TableIndex = CRC_ZERO_INIT; u16TableIndex < (uint16_t)CRC_TABLE_SIZE; u16TableIndex++)
    {
        u16CurrentValue = u16TableIndex << (uint16_t)CRC_BITS_PER_BYTE;
        
        for (u16BitCount = CRC_ZERO_INIT; u16BitCount < (uint16_t)CRC_BITS_PER_BYTE; u16BitCount++)
        {
            uint16_t u16MaskedValue = u16CurrentValue & (uint16_t)CRC_MSB_MASK;
            if (u16MaskedValue == (uint16_t)CRC_MSB_MASK)
            {
                uint16_t u16ShiftedValue = u16CurrentValue << (uint16_t)CRC_SHIFT_ONE;
                u16CurrentValue = u16ShiftedValue ^ (uint16_t)CRC_POLYNOMIAL;
            }
            else
            {
                u16CurrentValue = u16CurrentValue << (uint16_t)CRC_SHIFT_ONE;
            }
        }
        ms_au16CrcTable[u16TableIndex] = u16CurrentValue;
    }
    u8InitFlagStatus = (u8InitFlagStatus == ACTIVE_FLAG ? u8InitFlagStatus : INACTIVE_FLAG);
    ITCOM_vSetInitFlagStatus(u8InitFlagStatus);
}