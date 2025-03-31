//*****************************************************************************
/**
* @file memory_test.c
*****************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief module to implement memory tests
*
* @authors Brian Le
*
* @date July 01 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 07/01/2024|BL |Initial
* 08/05/2024|BL |Baseline
* 08/06/2024|BL |corrections to baseline
* 08/08/2024|BL |correct names of param and func
* 08/13/2024|BL |SUD baseline 0.4
*
*/
//*****************************************************************************


/*** Include Files ***/
#include "memory_test.h"

#include "crc.h"


/*** Module Definitions ***/
//fail thresholds
///Threshold of max number of failures before a memory test is considered failed
#define MEM_TEST_FAIL_THRESH        (0U)
///32 bits of 0's
#define ALL_0S                      (0x00000000U)
///32 bits of 1's
#define ALL_1S                      (0xFFFFFFFFU)
//pattern test
///Pattern for 1st pattern test
#define PATTERN_PATTERN_1           (0xAAAAAAAAU)
///Pattern for 2nd pattern test
#define PATTERN_PATTERN_2           (0x55555555U)
///Pattern for 3rd pattern test
#define PATTERN_PATTERN_3           (0xAAAAAAAAU)
//march test
///Pattern for 1st march test
#define MARCH_PATTERN_1             (ALL_0S)
///Pattern for 2nd march test
#define MARCH_PATTERN_2             (ALL_1S)
///Pattern for 3rd march test
#define MARCH_PATTERN_3             (ALL_0S)
///Pattern for 4th march test
#define MARCH_PATTERN_4             (ALL_1S)
///known value for performing CRC test
#define CRC_KNOWN_VAL		        (0xFA567812U)
///Number of bits in 3 bytes
#define BITS_3BYTE		            (24U)
///Number of bits in 2 bytes
#define BITS_2BYTE		            (16U)
///Number of bits in 1 byte
#define BITS_1BYTE		            (8U)
///MASK for only 1 byte
#define BYTE_MASK		            (0xFFU)
///Index for most significant byte in 4-byte array
#define BYTE_INDEX_3                (3U)
///Index for second most significant byte in 4-byte array
#define BYTE_INDEX_2                (2U)
///Index for second least significant byte in 4-byte array
#define BYTE_INDEX_1                (1U)
///Index for least significant byte in 4-byte array
#define BYTE_INDEX_0                (0U)
///Size of 4-byte array
#define FOUR_BYTE_SIZE              (4U)



/*** Internal Types ***/

/*** Local Function Prototypes ***/
static void mem_vWriteToWord(volatile uint32_t* u32WordAddr, uint32_t u32Value);


/*** External Variables ***/

/*** Internal Variables ***/


//*****************************************************************************
// FUNCTION NAME : MEM_u8RamPatternTest
//*****************************************************************************
/**
*
* @brief Perform RAM pattern test
*
* Verifies the integrity of each memory cell in RAM by writing and reading 
* specific patterns, with alternating 1's and 0's patterns (AAAAAAAAH and 55555555H), 
* to detect static bit failures.
*
* @param [in, out]  u32RamStartAddr    Passes pointer to the first address of RAM block
* @param [in]       u32RamBlockSize    Passes size of ram block
* 
* @return Result of Ram PatternTest
*           -0 MEM_TEST_GEN_FAIL      - failed
*           -1 MEM_TEST_GEN_PASSED    - passed
*/
//*****************************************************************************
uint8_t MEM_u8RamPatternTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize)
{
    uint8_t u8TestResult = (uint8_t)MEM_TEST_GEN_PASSED;
    uint32_t u32FailureCount = 0;
    uint32_t u32Pattern1Fails = 0;
    uint32_t u32Pattern2Fails = 0;
    uint32_t u32Pattern3Fails = 0;
    uint32_t u32CellNum = 0;
    uint32_t u32TempStorage = 0;

    //loop through cells
    for (u32CellNum = 0; u32CellNum < u32RamBlockSize; u32CellNum++)
    {  
        //temp store original ram content
        u32TempStorage =  u32RamStartAddr[u32CellNum];
        //pattern test 1
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)PATTERN_PATTERN_1);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)PATTERN_PATTERN_1)
        {
            u32Pattern1Fails++;
        }
        //pattern test 2
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)PATTERN_PATTERN_2);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)PATTERN_PATTERN_2)
        {
            u32Pattern2Fails++;
        }

        //pattern test 3
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)PATTERN_PATTERN_3);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)PATTERN_PATTERN_3)
        {
            u32Pattern3Fails++;
        }

        //restore contents
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], u32TempStorage);

    }


    //sum failure count
    u32FailureCount = u32Pattern1Fails + u32Pattern2Fails + u32Pattern3Fails;

    //determine if failed
    if (u32FailureCount > (uint32_t)MEM_TEST_FAIL_THRESH)
    {
        u8TestResult = (uint8_t)MEM_TEST_GEN_FAIL;
    }
    return u8TestResult;
}

//*****************************************************************************
// FUNCTION NAME : MEM_u8RamMarchTest
//*****************************************************************************
/**
*
* @brief Perform RAM March test by writing 0's and 1's to data words
*
* @param [in, out]  u32RamStartAddr    Passes pointer to the first address of RAM block
* @param [in]       u32RamBlockSize    Passes size of ram block
*
* 
* @return Result of RAM march test
*           -0 MEM_TEST_GEN_FAIL      - failed
*           -1 MEM_TEST_GEN_PASSED    - passed
*/
//*****************************************************************************
uint8_t MEM_u8RamMarchTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize)
{
    uint8_t u8TestResult = (uint8_t)MEM_TEST_GEN_PASSED;//assume passed test
    uint32_t u32TempStorage = 0;
    uint32_t u32CellNum = 0;
    //fail counts
    uint32_t march1Fails = 0;
    uint32_t march2Fails = 0;
    uint32_t march3Fails = 0;
    uint32_t march4Fails = 0;
    uint32_t u32FailureCount = 0;



    //loop through cells
    for (u32CellNum = 0; u32CellNum < u32RamBlockSize; u32CellNum++)
    {  
        //temp store original ram content
        u32TempStorage = u32RamStartAddr[u32CellNum];
        //march test 1
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)MARCH_PATTERN_1);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)MARCH_PATTERN_1)
        {
            march1Fails++;
        }
        //march test 2
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)MARCH_PATTERN_2);  
        if (u32RamStartAddr[u32CellNum] != (uint32_t)MARCH_PATTERN_2)
        {
            march2Fails++;
        }

        //march test 3
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)MARCH_PATTERN_3);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)MARCH_PATTERN_3)
        {
            march3Fails++;
        }

        //march test 4
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], (uint32_t)MARCH_PATTERN_4);
        if (u32RamStartAddr[u32CellNum] != (uint32_t)MARCH_PATTERN_4)
        {
            march4Fails++;
        }

        //restore contents
        mem_vWriteToWord(&u32RamStartAddr[u32CellNum], u32TempStorage);

    }


    //sum failure count
    u32FailureCount = march1Fails + march2Fails + march3Fails + march4Fails;

    //determine if failure
    if (u32FailureCount > (uint32_t)MEM_TEST_FAIL_THRESH)
    {
        u8TestResult = (uint8_t)MEM_TEST_GEN_FAIL;
    }

    return u8TestResult;
}


//*****************************************************************************
// FUNCTION NAME : MEM_u8CrcTest
//*****************************************************************************
/**
*
* @brief Perform CRC test on memory.
*
* Calculates CRC for all words then rechecks CRC calculation.
*
*
* @param [in,out]   u32RamStartAddr    Passes pointer to the first address of RAM block
* @param [in]       u32RamBlockSize    Passes size of ram block
*
* 
* @return Result of RAM CRC test 
*           -0 MEM_TEST_GEN_FAIL      - failed
*           -1 MEM_TEST_GEN_PASSED    - passed
*/
//*****************************************************************************
uint8_t MEM_u8CrcTest(volatile uint32_t* u32RamStartAddr, uint32_t u32RamBlockSize)
{
    uint8_t u8TestResult = (uint8_t)MEM_TEST_GEN_PASSED;//assume passed
    uint32_t u32FailCount = 0;
    uint32_t u32CellNumber = 0;
    uint16_t u16Crc1 = 0;
    uint16_t u16Crc2 = 0;
    uint32_t u32TempStorage = 0;
    uint32_t u32ReReReadData = 0;
    uint8_t u8TempCrcData[FOUR_BYTE_SIZE] = {0};

    //change word form for crc calculation
    u8TempCrcData[BYTE_INDEX_0] = (uint8_t)((CRC_KNOWN_VAL >> BITS_3BYTE) & BYTE_MASK);
    u8TempCrcData[BYTE_INDEX_1] = (uint8_t)((CRC_KNOWN_VAL >> BITS_2BYTE) & BYTE_MASK);
    u8TempCrcData[BYTE_INDEX_2] = (uint8_t)((CRC_KNOWN_VAL >> BITS_1BYTE) & BYTE_MASK);
    u8TempCrcData[BYTE_INDEX_3] = (uint8_t)((CRC_KNOWN_VAL              ) & BYTE_MASK);
    //calculate crc
    u16Crc1 = CRC_u16CalculateCrc(&u8TempCrcData[BYTE_INDEX_0], FOUR_BYTE_SIZE);


    for(u32CellNumber = 0; u32CellNumber < u32RamBlockSize; u32CellNumber++)
    {
        //hold original data
        u32TempStorage = u32RamStartAddr[u32CellNumber];

        //store known value
        mem_vWriteToWord(&u32RamStartAddr[u32CellNumber], (uint32_t)CRC_KNOWN_VAL);

        //re read data
        u32ReReReadData = u32RamStartAddr[u32CellNumber];

        //change word form for crc calculation
        u8TempCrcData[BYTE_INDEX_0] = (uint8_t)((u32ReReReadData >> BITS_3BYTE) & BYTE_MASK);
        u8TempCrcData[BYTE_INDEX_1] = (uint8_t)((u32ReReReadData >> BITS_2BYTE) & BYTE_MASK);
        u8TempCrcData[BYTE_INDEX_2] = (uint8_t)((u32ReReReadData >> BITS_1BYTE) & BYTE_MASK);
        u8TempCrcData[BYTE_INDEX_3] = (uint8_t)((u32ReReReadData              ) & BYTE_MASK);
        //calculate 2nd crc
        u16Crc2 = CRC_u16CalculateCrc(&u8TempCrcData[BYTE_INDEX_0], FOUR_BYTE_SIZE);

        //crc does not match then a failed cell
        if (u16Crc1 != u16Crc2)
        {
            u32FailCount++;
        }

        //restore data
        mem_vWriteToWord(&u32RamStartAddr[u32CellNumber], u32TempStorage);
    }

    //check if failure
    if(u32FailCount > (uint32_t)MEM_TEST_FAIL_THRESH)
    {
        u8TestResult = (uint8_t)MEM_TEST_GEN_FAIL;
    }

    return u8TestResult;
}

//*****************************************************************************
// FUNCTION NAME : mem_vWriteToWord
//*****************************************************************************
/**
*
* @brief Writes a specified u32Value to a given 32 bits of memory .
*
* @param [in, out]  u32WordAddr    Passes pointer to the 32 bit word
* @param [in]       u32Value       Passes value to write to word
* 
* @return none
*/
//*****************************************************************************
static void mem_vWriteToWord(volatile uint32_t* u32WordAddr, uint32_t u32Value)
{
    //store value
    *u32WordAddr = u32Value;
}
