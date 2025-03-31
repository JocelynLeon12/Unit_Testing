//*****************************************************************************
/**
* @file instance_manager.h
*****************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief module to manage instances within buffers
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
* 09/22/2024|TP |Instance Manager Refactored
*
*/
//*****************************************************************************

#ifndef INSTANCE_MANAGER_H
#define INSTANCE_MANAGER_H

/*** Include Files ***/
#include "gen_std_types.h"


/*** Definitions Provided to other modules ***/
#define MAX_BUFFER_CAPACITY 50
#define MIN_BUFFER_CAPACITY 1    // Arbitrary minimum buffer capacity chosen
#define MAX_ELEMENT_SIZE 16

#define REMOVE_ELEMENT			(0)
#define UPDATE_ELEMENT			(1)
#define ADD_ELEMENT				(2)

typedef struct {
    uint8_t  au8_Buffer[MAX_BUFFER_CAPACITY * MAX_ELEMENT_SIZE];  /* Fixed-size buffer array */
    size_t   sz_ElementSize;        /* Size of each element in the buffer */
    uint16_t u16Head;               /* Start of valid data */
    uint16_t u16Tail;               /* End of valid data */
    uint16_t u16Count;              /* Number of active elements */
    uint16_t u16Capacity;           /* Maximum number of elements */
} stIMBuffer;



/*** Type Definitions ***/
typedef uint8_t (*ElementCompareFn)(const_generic_ptr_t element1, const_generic_ptr_t element2);

/*** External Variables ***/



/*** External Functions ***/

extern void InstanceManager_vInitialize(stIMBuffer *cb, size_t elementSize, uint16_t capacity);
extern void InstanceManager_vAddElement(stIMBuffer *cb, const_generic_ptr_t element);
extern int16_t InstanceManager_s8FindElement(const stIMBuffer *cb, const_generic_ptr_t criteria, ElementCompareFn compareFunc, generic_ptr_t result);
extern void InstanceManager_vUpdateElement(stIMBuffer *cb, uint16_t index, const_generic_ptr_t newElement);
extern void InstanceManager_vRemoveElement(stIMBuffer *cb, uint16_t index);


#endif /* INSTANCE_MANAGER_H */
