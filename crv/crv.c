/*****************************************************************************
 * @file crv.c
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

#include "icm.h"
#include "storage_handler.h"
#include "thread_management.h"
#include "crv.h"

/*** Module Definitions ***/
#define CRV_BUFFER_SIZE                 8U      /* Size of calibration data buffer */
#define CRV_NO_VALID_ITEM              -1       /* Indicator for invalid/not found item */
#define CRV_INIT_VALUE                  0U      /* Buffer initialization value */
#define CRV_TRUE_U16                    1U      /* Boolean TRUE representation */
#define CRV_FALSE_U16                   0U      /* Boolean FALSE representation */

/*** Internal Types ***/

/*** Local Function Prototypes ***/

/*** External Variables ***/

/*** Internal Variables ***/

/*** External Functions ***/

/**
 * @brief Performs the main Calibration Readback Verification process.
 *
 * This function compares calibration copy data against readback data to verify
 * data integrity. It processes data elements sequentially, logs results, and
 * manages the cleanup of verified elements from both buffers.
 *
 * @details
 * The function executes the following workflow:
 * 1. Initializes:
 *    - Local constants for comparisons (CRV_ZERO_U16, CRV_ONE_U16)
 *    - Local buffers (au8CalibCopyData, au8CalibReadbackData)
 *    - Process message data structure (stCalibCopyData)
 *
 * 2. Validates prerequisites:
 *    - Checks for valid log file handle
 *    - Verifies presence of elements in both copy and readback buffers
 *
 * 3. For each element in reverse order:
 *    - Extracts message ID and sequence number
 *    - Validates message ID
 *    - Performs buffer size validation
 *    - Copies data to temporary buffers
 *    - Retrieves corresponding readback data via ITCOM_s16GetCalibReadbackData
 *
 * 4. Handles comparison results:
 *    - For matches: Sets CRV_COMPARISON_MATCH_U8 and logs success
 *    - For mismatches:
 *      * Sets CRV_COMPARISON_MISMATCH_U8
 *      * Records error event (EVENT_ID_FAULT_CAL_READBACK_ERROR)
 *      * Logs error with message ID and sequence number
 *
 * 5. Cleanup:
 *    - Removes processed elements from both copy and readback buffers
 *    - Logs completion status
 *
 * @note
 * - Buffer size is fixed at CRV_BUFFER_SIZE (8 bytes)
 * - Elements are processed in reverse order (from highest index to lowest)
 * - Function depends on global_log_file for logging operations
 *
 * @warning
 * - Requires valid global_log_file handle
 * - Requires properly initialized ITCOM module
 * - Assumes both calibration copy and readback buffers are properly maintained
 *
 * @return void
 */

void CRV_vMainFunction(void)
{
    /* Constants for status and validation checks */
    static const uint16_t CRV_ZERO_U16 = 0U;
    static const uint16_t CRV_ONE_U16 = 1U;
    static const uint8_t CRV_COMPARISON_MATCH_U8 = 7U;    /* Status for matching calibration data */
    static const uint8_t CRV_COMPARISON_MISMATCH_U8 = 8U; /* Status for mismatching calibration data */

    /* Data structures and counters for calibration processing */
    stProcessMsgData stCalibCopyData = {0};           /* Holds current calibration data being processed */
    uint16_t u16i = CRV_ZERO_U16;                     /* Loop counter for buffer traversal */
    uint16_t u16MsgId = CRV_ZERO_U16;                 /* Message identifier from calibration data */
    uint16_t u16SequenceNum = CRV_ZERO_U16;           /* Sequence number for message ordering */
    uint16_t u16CalibCopyElements = CRV_ZERO_U16;     /* Count of elements in copy buffer */
    uint16_t u16CalibReadbackElements = CRV_ZERO_U16; /* Count of elements in readback buffer */

    /* Initialize zero-filled reference buffer for data reset */
    const uint8_t au8ZeroBuffer[CRV_BUFFER_SIZE] = {
        CRV_INIT_VALUE, CRV_INIT_VALUE, CRV_INIT_VALUE, CRV_INIT_VALUE,
        CRV_INIT_VALUE, CRV_INIT_VALUE, CRV_INIT_VALUE, CRV_INIT_VALUE};

    /* Working buffers for data comparison */
    uint8_t au8CalibCopyData[CRV_BUFFER_SIZE];     /* Holds original calibration data */
    uint8_t au8CalibReadbackData[CRV_BUFFER_SIZE]; /* Holds readback data for comparison */
    (void)memcpy(au8CalibCopyData, au8ZeroBuffer, sizeof(au8CalibCopyData));
    (void)memcpy(au8CalibReadbackData, au8ZeroBuffer, sizeof(au8CalibReadbackData));

    /* Status indicator for data retrieval operations */
    int16_t s16FoundItemStatus = CRV_NO_VALID_ITEM;

    /* Verify log file availability before processing */
    if (NULL != global_log_file)
    {
        log_message(global_log_file, LOG_INFO, "CRV_vMainFunction: Starting Calibration Readback Verification...");

        /* Get current element counts from both buffers */
        u16CalibCopyElements = ITCOM_u16GetTrackBufferSize(enCalibDataCopyBuffer);
        u16CalibReadbackElements = ITCOM_u16GetTrackBufferSize(enCalibReadbackData);

        /* Convert buffer sizes to boolean status for validation */
        uint16_t u16CopyCheck = (u16CalibCopyElements > CRV_ZERO_U16) ? CRV_TRUE_U16 : CRV_FALSE_U16;
        uint16_t u16ReadbackCheck = (u16CalibReadbackElements > CRV_ZERO_U16) ? CRV_TRUE_U16 : CRV_FALSE_U16;

        /* Proceed only if both buffers contain elements */
        if (u16CopyCheck == u16ReadbackCheck)
        {

            /* Process elements in reverse order to maintain buffer integrity */
            for (u16i = u16CalibCopyElements; ((u16i > CRV_ZERO_U16) && (!get_thread_exit())); u16i--)
            {
                uint16_t u16CurrentIndex;
                /* Calculate zero-based index for buffer access */
                u16CurrentIndex = u16i - CRV_ONE_U16;

                /* Retrieve calibration data at current index */
                ITCOM_vGetCycleSeqElementAtIndex(u16CurrentIndex, (generic_ptr_t)&stCalibCopyData, enCalibDataCopyBuffer);

                /* Extract message identifier and sequence number */
                u16MsgId = stCalibCopyData.stMsgPairData.u16MsgId;
                u16SequenceNum = stCalibCopyData.stMsgPairData.u16SequenceNum;
                uint16_t u16MsdIdEnum = ITCOM_s16GetMessageEnumById(u16MsgId);

                /* Process only valid messages */
                if (u16MsdIdEnum < enTotalMessagesASI)
                {
                    /* Verify buffer capacity before copying data */
                    if ((size_t)sizeof(stCalibCopyData.au8MsgData) <= (size_t)sizeof(au8CalibCopyData))
                    {
                        /* Copy calibration data to working buffer */
                        (void)memcpy(au8CalibCopyData, stCalibCopyData.au8MsgData, sizeof(stCalibCopyData.au8MsgData));

                        /* Attempt to retrieve corresponding readback data */
                        s16FoundItemStatus = ITCOM_s16GetCalibReadbackData(stCalibCopyData, au8CalibReadbackData);

                        /* Process only if readback data was found */
                        if (s16FoundItemStatus >= 0)
                        {
                            uint8_t u8CalibComparisonRes = CRV_COMPARISON_MISMATCH_U8;
                            /* Compare calibration copy against readback data */
                            if (0 == memcmp(au8CalibCopyData, au8CalibReadbackData, sizeof(au8CalibCopyData)))
                            {
                                u8CalibComparisonRes = CRV_COMPARISON_MATCH_U8;
                                ITCOM_vSetCalibComparisonResult(u8CalibComparisonRes);
                                log_message(global_log_file, LOG_INFO, "CRV_vMainFunction: Calibration data match for MsgId: 0x%04X, SequenceNum: %04X", u16MsgId, u16SequenceNum);
                            }
                            else
                            {
                                /* Handle mismatch case */
                                ITCOM_vSetCalibComparisonResult(u8CalibComparisonRes);
                                /* Log error event if successfully recorded */
                                int16_t s16ErrorStatus = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_CAL_READBACK_ERROR);
                                if (s16ErrorStatus == (int16_t)enSuccess_EventAddedToQueue)
                                {
                                    log_message(global_log_file, LOG_ERROR, "CRV_vMainFunction: Calibration data mismatch for MsgId: %04X, SequenceNum: %04X", u16MsgId, u16SequenceNum);
                                }
                            }
                            (void)ITCOM_s8LogNotificationMessage(u16MsgId, u16SequenceNum, u8CalibComparisonRes, (uint8_t)enActionNotification);
                            /* Clean up processed data from both buffers */
                            ITCOM_vSetCalibDataCopy(&stCalibCopyData, REMOVE_ELEMENT);
                            ITCOM_vSetCalibReadbackData(&stCalibCopyData, REMOVE_ELEMENT);
                        }
                    }
                    else
                    {
                        log_message(global_log_file, LOG_ERROR, "CRV_vMainFunction: Data size exceeds buffer limits.");
                    }
                }
                else
                {
                    log_message(global_log_file, LOG_ERROR, "CRV_vMainFunction: Invalid message ID retrieved.");
                }
            }
            log_message(global_log_file, LOG_INFO, "CRV_vMainFunction: Calibration Readback Verification Completed.");
        }
        else
        {
            /* Log when no data is available for processing */
            log_message(global_log_file, LOG_WARNING, "CRV_vMainFunction: No calibration data elements found.");
            log_message(global_log_file, LOG_INFO, "CRV_vMainFunction: Calibration Readback Verification Completed.");
        }
    }
    else
    {
        log_message(global_log_file, LOG_ERROR, "CRV_vMainFunction: Log file handle is invalid.");
    }
}
