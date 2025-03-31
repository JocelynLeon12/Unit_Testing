/**
* @file data_queue.c
*****************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief module to implement data queues
*
* @authors Alejandro Tollola
*
* @date August 08 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 08/08/2024|AT |Initial
*
*/
//*****************************************************************************


/*** Include Files ***/
#include "itcom.h"
#include "storage_handler.h"

#include "data_queue.h"

/*** Module Definitions ***/
#define QUEUE_INIT_VALUE    ((uint16_t)0U)
#define QUEUE_INCREMENT     ((uint16_t)1U)
#define QUEUE_IS_EMPTY      ((uint32_t)0U)


/*** Internal Types ***/



/*** Local Function Prototypes ***/



/*** External Variables ***/



/*** Internal Variables ***/



/*** External Functions ***/

//*****************************************************************************
// FUNCTION NAME : DataQueue_vInit
//*****************************************************************************
/**
*
* @brief Initializes the data queue with the given buffer and size information.
*
* @param [in] q Pointer to the data queue structure
* @param [in] pu8Buffer Pointer to the data buffer
* @param [in] maxSize Maximum size of the data queue
* @param [in] u16BuffSize Size of the data buffer
* @param [in] circularBuffActive Flag indicating if the circular buffer is active
*
* @global none
*
* @return none
*/
void DataQueue_vInit(data_queue_t* q, uint8_t* pu8Buffer, queue_size_t maxSize, queue_size_t u16ElementSize, uint8_t circularBuffActive)
{
    if ((q != NULL) && (pu8Buffer != NULL)) {
        if ((maxSize != QUEUE_INIT_VALUE) && (u16ElementSize != QUEUE_INIT_VALUE)) {
            q->u8CircularBuffActive = circularBuffActive;
            q->u16_qHead = QUEUE_INIT_VALUE;
            q->u16_qTail = QUEUE_INIT_VALUE;
            q->u16_qSize = QUEUE_INIT_VALUE;
            q->u16_qMaxSize = maxSize;
            q->u16_BuffSize = u16ElementSize;
            q->pu8_qData = pu8Buffer;

            const queue_size_t totalSize = maxSize * u16ElementSize;
            uint8_t* const result = memset(pu8Buffer, 0, (size_t)totalSize);
            if (result == pu8Buffer) {
                log_message(global_log_file, LOG_DEBUG, "DataQueue_vInit: Queue initialized...");
            } else {
                log_message(global_log_file, LOG_ERROR, "DataQueue_vInit: Memory initialization failed");
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "DataQueue_vInit: Invalid size parameters");
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "DataQueue_vInit: Invalid queue or buffer pointer");
    }
}

//*****************************************************************************
// FUNCTION NAME : DataQueue_vClear
//*****************************************************************************
/**
*
* @brief Clears all data in the given data queue.
*
* @param [in] q Pointer to the data queue
*
* @global none
*
* @return none
*/
void DataQueue_vClear(data_queue_t* q)
{
    if ((q != NULL) && (q->pu8_qData != NULL)) {
        const queue_size_t clearSize = q->u16_qSize * q->u16_BuffSize;
        if (memset((void*)q->pu8_qData, 0, (size_t)clearSize) == q->pu8_qData) {
            q->u16_qHead = QUEUE_INIT_VALUE;
            q->u16_qTail = QUEUE_INIT_VALUE;
            q->u16_qSize = QUEUE_INIT_VALUE;
        } else {
            log_message(global_log_file, LOG_ERROR, "DataQueue_vClear: Memory clear failed");
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "DataQueue_vClear: Invalid queue pointer or data buffer");
    }
}

//*****************************************************************************
// FUNCTION NAME : DataQueue_bIsEmpty
//*****************************************************************************
/**
*
* @brief Checks if the specified data queue is empty.
*
* @param [in] q Pointer to a data queue structure
*
* @global none
*
* @return Unsigned integer value indicating whether the queue is empty (true) or not (false)
*/
uint8_t DataQueue_u8IsEmpty(const data_queue_t* q)
{
    uint8_t isEmpty = FALSE;
    
    if (q == NULL) {
        log_message(global_log_file, LOG_ERROR, "DataQueue_u8IsEmpty: Invalid queue pointer");
        isEmpty = (uint8_t)TRUE;
    } else {
        isEmpty = (q->u16_qSize == QUEUE_INIT_VALUE) ? (uint8_t)TRUE : (uint8_t)FALSE;
    }
    
    return isEmpty;
}

//*****************************************************************************
// FUNCTION NAME : DataQueue_s8Enqueue
//*****************************************************************************
/**
*
* @brief Enqueues data into the given data queue.
*
* @param [in] q Pointer to the data queue structure
* @param [in] pu8Data Pointer to the data to be enqueued
* @param [in] u16DataSize Size of the data to be enqueued
*
* @global none
*
* @return int8_t Status of the enqueue operation (-1: error, 0: success)
*/
int8_t DataQueue_s8Enqueue(data_queue_t* q, uint8_t* pu8Data, queue_size_t u16DataSize)
{
    int8_t result = QUEUE_ACTION_FAILURE_DEFAULT;

    if ((q != NULL) && (q->pu8_qData != NULL) && (pu8Data != NULL)) {
        /* Check data size */
        if (u16DataSize <= q->u16_BuffSize) {
            /* Check queue capacity */
            if ((!q->u8CircularBuffActive) && (q->u16_qSize >= q->u16_qMaxSize)) {
                result = QUEUE_ACTION_FAILURE_DATAQUEUE_QUEUE_FULL;
                log_message(global_log_file, LOG_WARNING, "DataQueue_s8Enqueue: Queue is full and not active as circular buffer");
            } else {
                const queue_size_t u16StartIndex = q->u16_qTail * q->u16_BuffSize;
                const queue_size_t maxIndex = q->u16_qMaxSize * q->u16_BuffSize;
                
                if (u16StartIndex < maxIndex) {
                    uint8_t* const pu8DestData = &(q->pu8_qData[u16StartIndex]);
                    (void)memcpy(pu8DestData, (const void*)pu8Data, (size_t)u16DataSize);
                    
                    /* Handle circular buffer overflow */
                    if (q->u8CircularBuffActive && (q->u16_qSize == q->u16_qMaxSize)) {
                        q->u16_qHead = (q->u16_qHead + QUEUE_INCREMENT) % q->u16_qMaxSize;
                    } else {
                        q->u16_qSize++;
                    }
                    
                    q->u16_qTail = (q->u16_qTail + QUEUE_INCREMENT) % q->u16_qMaxSize;
                    result = QUEUE_ACTION_SUCCESS;
                } else {
                    result = QUEUE_ACTION_FAILURE_DATAQUEUE_INDEX_OUT_OF_BOUNDS;
                    log_message(global_log_file, LOG_ERROR, "DataQueue_s8Enqueue: Index out of bounds");
                }
            }
        } else {
            result = QUEUE_ACTION_FAILURE_DATAQUEUE_DATA_SIZE_EXCEEDS_BUFFER;
            log_message(global_log_file, LOG_ERROR, "DataQueue_s8Enqueue: Data size exceeds buffer size");
        }
    } else {
        result = QUEUE_ACTION_FAILURE_DATAQUEUE_INVALID_INPUT;
        log_message(global_log_file, LOG_ERROR, "DataQueue_s8Enqueue: Invalid input parameters");
    }

    return result;
}

//*****************************************************************************
// FUNCTION NAME : DataQueue_s8Dequeue
//*****************************************************************************
/**
*
* @brief Dequeues data from the specified data queue and stores it in the provided array.
*
* @param [in] q Pointer to a data_queue_t structure representing the data queue
* @param [out] data Pointer to an array where dequeued data will be stored
* @param [in] u16DataSize Size of data to be dequeued
*
* @global none
*
* @return int8_t Status of the operation (0: success, -1: error)
*/
int8_t DataQueue_s8Dequeue(data_queue_t* q, uint8_t* data, queue_size_t u16DataSize)
{
    int8_t result = QUEUE_ACTION_FAILURE_DEFAULT;

    if ((q != NULL) && (q->pu8_qData != NULL) && (data != NULL)) {
        /* Check if queue is empty */
        if (q->u16_qSize > QUEUE_IS_EMPTY) {
            /* Check data size */
            if (u16DataSize <= q->u16_BuffSize) {
                const queue_size_t u16StartIndex = (q->u16_qHead % q->u16_qMaxSize) * q->u16_BuffSize;
                
                if (u16StartIndex < (q->u16_qMaxSize * q->u16_BuffSize)) {
                    const uint8_t* const pu8SrcData = &q->pu8_qData[u16StartIndex];
                    (void)memcpy(data, (const void*)pu8SrcData, (size_t)u16DataSize);
                    
                    q->u16_qHead = (q->u16_qHead + QUEUE_INCREMENT) % q->u16_qMaxSize;
                    q->u16_qSize--;
                    result = QUEUE_ACTION_SUCCESS;
                } else {
                    result = QUEUE_ACTION_FAILURE_DATAQUEUE_INDEX_OUT_OF_BOUNDS;
                    log_message(global_log_file, LOG_ERROR, "DataQueue_s8Dequeue: Index out of bounds");
                }
            } else {
                result = QUEUE_ACTION_FAILURE_DATAQUEUE_DATA_SIZE_EXCEEDS_BUFFER;
                log_message(global_log_file, LOG_ERROR, "DataQueue_s8Dequeue: Requested data size exceeds buffer size");
            }
        } else {
            result = QUEUE_ACTION_FAILURE_DATAQUEUE_QUEUE_EMPTY;
            log_message(global_log_file, LOG_WARNING, "DataQueue_s8Dequeue: Queue is empty");
        }
    } else {
        result = QUEUE_ACTION_FAILURE_DATAQUEUE_INVALID_INPUT;
        log_message(global_log_file, LOG_ERROR, "DataQueue_s8Dequeue: Invalid input parameters");
    }

    return result;
}

