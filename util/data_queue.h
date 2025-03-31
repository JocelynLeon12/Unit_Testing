//*****************************************************************************
/**
* @file data_queue.h
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

#ifndef DATA_QUEUE_H
#define DATA_QUEUE_H

/*** Include Files ***/
#include "gen_std_types.h"

/*** Definitions Provided to other modules ***/

/*** Type Definitions ***/
/**
 * @def DATA_QUEUE_MAX_SIZE
 * @brief Maximum number of events that can be stored in the event queue.
 *
 * This constant defines the capacity of the event queue used for storing
 * error events before they are processed. It limits the number of pending
 * events to prevent memory overflow in case of rapid event generation.
 *
 * When the queue reaches this size:
 * - New events with higher severity may replace less severe events.
 * - New events with lower or equal severity may be discarded.
 *
 * @note The value of 32 is chosen to balance between having sufficient
 *       capacity for bursts of events and limiting memory usage.
 *
 */
#define DATA_QUEUE_MAX_SIZE (32)

typedef uint32_t queue_size_t;     /* Type for size calculations */
typedef uint32_t queue_index_t;    /* Type for index calculations */

typedef struct {
    uint8_t  u8CircularBuffActive;
    queue_index_t u16_qHead;       /* Changed from uint16_t */
    queue_index_t u16_qTail;       /* Changed from uint16_t */
    queue_size_t u16_qSize;        /* Changed from uint16_t */
    queue_size_t u16_qMaxSize;     /* Changed from uint16_t */
    queue_size_t u16_BuffSize;     /* Changed from uint16_t */
    uint8_t* pu8_qData;
} data_queue_t;

/*** External Variables ***/

/*** External Functions ***/
extern void DataQueue_vInit(data_queue_t* q, uint8_t* pu8Buffer, queue_size_t maxSize, queue_size_t u16ElementSize, uint8_t circularBuffActive);
extern void DataQueue_vClear(data_queue_t* q);
extern uint8_t DataQueue_u8IsEmpty(const data_queue_t* q);
extern int8_t DataQueue_s8Enqueue(data_queue_t* q, uint8_t* pu8Data, queue_size_t u16DataSize);
extern int8_t DataQueue_s8Dequeue(data_queue_t* q, uint8_t* data, queue_size_t u16DataSize);

#endif // DATA_QUEUE_H
