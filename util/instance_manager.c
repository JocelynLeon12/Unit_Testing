/**
* @file instance_manager.c
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


/*** Include Files ***/
#include "instance_manager.h"



/*** Module Definitions ***/
#define IM_ZERO                   0U
#define IM_INIT_TRUE              1U
#define IM_INIT_FALSE             0U
#define IM_INCREMENT              1U
#define IM_DECREMENT              1U
#define IM_INVALID_INDEX          (-1)


/*** Internal Types ***/



/*** Local Function Prototypes ***/


/*** External Variables ***/



/*** Internal Variables ***/



/*** External Functions ***/

/**
 * @brief Initializes an Instance Manager Buffer (IMBuffer) structure.
 *
 * This function initializes a circular buffer structure used for instance management.
 * It sets up the buffer with the specified element size and capacity, ensuring that
 * the capacity is within valid bounds. The function uses memset to efficiently
 * initialize all structure members to zero, then sets specific non-zero values.
 *
 * @param[out] cb Pointer to the stIMBuffer structure to be initialized.
 *                If NULL, the function returns without performing any operation.
 * @param[in] elementSize Size of each element in the buffer, in bytes.
 *                        Must be non-zero, or the function will return without initialization.
 * @param[in] capacity Desired capacity of the buffer (number of elements).
 *                     Must be positive, or the function will return without initialization.
 *                     If outside the range [MIN_BUFFER_CAPACITY, MAX_BUFFER_CAPACITY],
 *                     it will be adjusted to the nearest valid value.
 *
 * @note The function uses MIN_BUFFER_CAPACITY and MAX_BUFFER_CAPACITY macros to
 *       ensure the buffer capacity is within acceptable limits.
 *
 * @warning This function does not allocate memory for the buffer data.
 *          It only initializes the buffer control structure.
 *
 * @return void This function does not return a value.
 *              It operates directly on the provided buffer structure.
 */
void InstanceManager_vInitialize(stIMBuffer *cb, size_t elementSize, uint16_t capacity) {
    uint8_t proceed = IM_INIT_TRUE;
    uint16_t safeCapacity = IM_ZERO;

    if (!cb || elementSize == IM_ZERO) {
        proceed = IM_INIT_FALSE;
    }

    if (proceed == (uint8_t)IM_INIT_TRUE) {
        safeCapacity = (capacity > (uint16_t)MAX_BUFFER_CAPACITY) ? (uint16_t)MAX_BUFFER_CAPACITY : 
                      (capacity < (uint16_t)MIN_BUFFER_CAPACITY) ? (uint16_t)MIN_BUFFER_CAPACITY : 
                      capacity;

        (void)memset(cb, IM_ZERO, sizeof(*cb));
        cb->sz_ElementSize = elementSize;
        cb->u16Capacity = safeCapacity;
    }
}



/**
 * @brief Adds an element to the circular buffer. Overwrites oldest element if full.
 *
 * This function adds a new element to the circular buffer. If the buffer is full,
 * it overwrites the oldest element. The function handles the circular nature of
 * the buffer, ensuring proper wrapping around the buffer's end.
 *
 * @param[in,out] cb Pointer to the stIMBuffer structure representing the circular buffer.
 * @param[in] element Pointer to the element to be added.
 *
 * @note Circular buffer behavior:
 *       - When the buffer is not full, the new element is added at the tail.
 *       - When the buffer is full, the head is moved to overwrite the oldest element,
 *         and the new element is inserted at the new tail position.
 *
 * @note Memory operations:
 *       - The function uses memcpy to copy the new element into the buffer.
 *       - It's assumed that element points to data of size cb->sz_ElementSize.
 *
 * @warning This function does not perform any bounds checking on the element data.
 *          It's the caller's responsibility to ensure that element points to data
 *          of the correct size (cb->sz_ElementSize).
 * 
 */
void InstanceManager_vAddElement(stIMBuffer *cb, const_generic_ptr_t element) {
    uint8_t proceed = IM_INIT_TRUE;
    uint16_t insertIndex = IM_ZERO;
    size_t elementSize;
    
    if (NULL == cb) {
        proceed = IM_INIT_FALSE;
    } else {
        elementSize = cb->sz_ElementSize;
        if ((NULL == element) || (IM_ZERO == elementSize) || (IM_ZERO == cb->u16Capacity)) {
            proceed = IM_INIT_FALSE;
        }
    }

    if (proceed == (uint8_t)IM_INIT_TRUE) {
        uint32_t tempCalc;
        
        if ((uint16_t)cb->u16Count == (uint16_t)cb->u16Capacity) {
            tempCalc = (uint32_t)cb->u16Head + (uint32_t)IM_INCREMENT;
            cb->u16Head = (uint16_t)(tempCalc % (uint32_t)cb->u16Capacity);
        } else {
            tempCalc = (uint32_t)cb->u16Count + (uint32_t)IM_INCREMENT;
            cb->u16Count = (uint16_t)tempCalc;
        }

        /* Calculate insert position */
        tempCalc = (uint32_t)cb->u16Head + (uint32_t)cb->u16Count - (uint32_t)IM_INCREMENT;
        insertIndex = (uint16_t)(tempCalc % (uint32_t)cb->u16Capacity);
        
        /* Copy element to buffer */
        tempCalc = (uint32_t)insertIndex * (uint32_t)elementSize;
        (void)memcpy(&cb->au8_Buffer[tempCalc], element, elementSize);
        
        /* Update tail */
        tempCalc = (uint32_t)insertIndex + (uint32_t)IM_INCREMENT;
        cb->u16Tail = (uint16_t)(tempCalc % (uint32_t)cb->u16Capacity);
    }
}



/**
 * @brief Finds an element in the circular buffer using a comparison function.
 *
 * This function searches for an element in the circular buffer that matches the given criteria.
 * It uses a user-provided comparison function to determine if an element matches.
 *
 * @param[in] cb Pointer to the stIMBuffer structure representing the circular buffer.
 * @param[in] criteria Pointer to the criteria for searching. The comparison function will use this.
 * @param[in] cmp Pointer to the comparison function. This function should return 0 when a match is found.
 *                The comparison function should have the signature: uint8_t (*)(const_generic_ptr_t , const_generic_ptr_t )
 * @param[out] result Pointer to store the found element (if any). Can be NULL if the caller doesn't need the element data.
 *
 * @return int8_t Returns the index of the found element (0 to cb->u16Count - 1), or -1 if not found or on error.
 *
 * @note Circular buffer traversal:
 *       The function traverses the circular buffer from head to tail, wrapping around if necessary.
 *       It uses the formula (cb->u16Head + i) % cb->u16Capacity to calculate the actual index in the buffer.
 *
 * @note Memory safety:
 *       1. Input pointers (cb, criteria, cmp) are checked for NULL before use.
 *       2. The function checks if the buffer is empty before searching.
 *       3. When copying the found element to result, it uses cb->sz_ElementSize to ensure correct size.
 *
 * @warning The caller must ensure that the criteria and result (if not NULL) point to memory areas 
 *          of at least cb->sz_ElementSize bytes.
 *
 */
int16_t InstanceManager_s8FindElement(const stIMBuffer *cb, const_generic_ptr_t criteria, 
                                    ElementCompareFn compareFunc, generic_ptr_t result) {
    int16_t returnValue = (int16_t)IM_INVALID_INDEX;
    uint8_t proceed = IM_INIT_TRUE;
    size_t elementSize;
    
    if (NULL == cb) {
        proceed = IM_INIT_FALSE;
    } else {
        elementSize = cb->sz_ElementSize;
        if ((NULL == criteria) || (NULL == compareFunc) || (IM_ZERO == cb->u16Count)) {
            proceed = IM_INIT_FALSE;
        }
    }

    if (proceed == (uint8_t)IM_INIT_TRUE) {
        uint16_t i;
        for (i = IM_ZERO; (i < cb->u16Count) && ((returnValue == (int16_t)IM_INVALID_INDEX)); ++i) {
            uint32_t tempCalc = (uint32_t)cb->u16Head + (uint32_t)i;
            uint16_t index = (uint16_t)(tempCalc % (uint32_t)cb->u16Capacity);
            const uint8_t *currentElement;
            
            tempCalc = (uint32_t)index * (uint32_t)elementSize;
            currentElement = &cb->au8_Buffer[tempCalc];

            if (IM_ZERO == compareFunc(currentElement, criteria)) {
                if (NULL != result) {
                    (void)memcpy(result, currentElement, elementSize);
                }
                returnValue = (int16_t)i;
            }
        }
    }

    return returnValue;
}



/**
 * @brief Updates an element in the circular buffer at a specified index.
 *
 * This function updates an existing element in the circular buffer at the given index
 * with new data. It handles the circular nature of the buffer, ensuring proper 
 * index calculation and boundary checks.
 *
 * @param[in,out] cb Pointer to the stIMBuffer structure representing the circular buffer.
 * @param[in] index The index of the element to be updated. This index is relative
 *                  to the current head of the buffer, not the absolute array index.
 * @param[in] newElement Pointer to the new data that will replace the existing element.
 *
 * @note Memory safety:
 *       The function implements several measures to ensure memory safety:
 *       1. Validates input pointers to prevent null pointer dereferences.
 *       2. Checks index bounds to prevent out-of-range access.
 *       3. Verifies that the calculated actual index is within the buffer's capacity.
 *       4. Uses a temporary buffer to limit the size of data copied, preventing buffer overflows.
 *
 * @note Data integrity:
 *       While this function attempts to handle varying sizes of input data safely:
 *       - If newElement contains less data than cb->sz_ElementSize, the remaining space
 *         in the circular buffer will be filled with zeros.
 *       - If newElement contains more data than cb->sz_ElementSize, only the first
 *         cb->sz_ElementSize bytes (up to MAX_ELEMENT_SIZE) will be copied.
 *
 * @warning It is still recommended that the caller ensure newElement points to data
 *          of size cb->sz_ElementSize for optimal operation and data integrity.
 *
 * @return void This function does not return a value.
 */
void InstanceManager_vUpdateElement(stIMBuffer *cb, uint16_t index, const_generic_ptr_t newElement) {
    uint8_t proceed = IM_INIT_TRUE;
    uint16_t actualIndex = IM_ZERO;
    size_t elementSize;
    
    if (NULL == cb) {
        proceed = IM_INIT_FALSE;
    } else {
        elementSize = cb->sz_ElementSize;
        if ((newElement == NULL) || (index >= cb->u16Count)) {
            proceed = IM_INIT_FALSE;
        }
    }

    if (proceed == IM_INIT_TRUE) {
        uint32_t tempCalc = (uint32_t)cb->u16Head + (uint32_t)index;
        uint8_t tempBuffer[MAX_ELEMENT_SIZE] = {IM_ZERO};

        actualIndex = (uint16_t)(tempCalc % (uint32_t)cb->u16Capacity);
        tempCalc = (uint32_t)actualIndex * (uint32_t)elementSize;
        (void)memcpy(tempBuffer, 
                    newElement, 
                    (elementSize <= MAX_ELEMENT_SIZE) ? elementSize : MAX_ELEMENT_SIZE);
        (void)memcpy(&cb->au8_Buffer[tempCalc], tempBuffer, elementSize);
    }
}

/**
 * @brief Removes an element from the circular buffer at a specified index.
 *
 * This function removes an element from the circular buffer at the given index
 * and shifts all subsequent elements to fill the gap. The function handles the
 * circular nature of the buffer, ensuring proper wrapping around the buffer's end.
 *
 * @param[in,out] cb Pointer to the stIMBuffer structure representing the circular buffer.
 * @param[in] index The index of the element to be removed. This index is relative
 *                  to the current head of the buffer, not the absolute array index.
 *
 * @note Wrap-around in circular buffers:
 *       Wrap-around refers to the situation where the logical end of the buffer
 *       extends beyond the physical end of the underlying array and continues at
 *       the beginning. This allows the buffer to use its space efficiently by
 *       treating the array as if it were circular.
 *
 *       Purpose of wrap-around handling:
 *       1. Efficient space utilization: Allows the buffer to use all available space,
 *          even when the logical sequence of elements wraps from the end to the beginning.
 *       2. Continuous operation: Enables the buffer to operate continuously without
 *          needing to reset or move elements when reaching the array's end.
 *       3. Constant-time insertions and deletions: Maintains O(1) time complexity
 *          for insertions and deletions at the ends of the buffer, regardless of
 *          the current head and tail positions.
 *
 *       This function handles wrap-around in two scenarios:
 *       1. When the elements to be moved after removal wrap around the buffer's end.
 *       2. When the actual index of the element to be removed is near the end of
 *          the physical array, but logically in the middle of the buffer's contents.
 *
 * @note The function uses memmove to shift elements, which correctly handles
 *       overlapping memory regions that may occur due to the circular nature of the buffer.
 *
 * @warning This function does not perform any bounds checking on the buffer itself.
 *          It assumes that cb->au8_Buffer is large enough to hold cb->u16Capacity elements.
 *
 * Algorithm:
 * 1. Validate input parameters (cb, index, and buffer state).
 * 2. Calculate the actual array index for the element to be removed.
 * 3. Determine the number of elements that need to be moved.
 * 4. If elements need to be moved:
 *    a. Check if the move can be done without wrap-around.
 *    b. If wrap-around is needed, determine if it's a single or two-part move.
 *    c. Perform the move(s) using memmove, handling wrap-around if necessary.
 * 5. Update the tail pointer and decrement the count.
 *
 * Time Complexity: O(n), where n is the number of elements after the removed element.
 * Space Complexity: O(1), as it operates in-place.
 *
 * @return void This function does not return a value.
 */
void InstanceManager_vRemoveElement(stIMBuffer *cb, uint16_t index) {
    uint8_t proceed = IM_INIT_TRUE;
    size_t elementSize;
    uint32_t tempCalc;
    uint8_t u8i;
    
    if (NULL == cb) {
        proceed = IM_INIT_FALSE;
    } else {
        elementSize = cb->sz_ElementSize;
        if ((index >= cb->u16Count)) {
            proceed = IM_INIT_FALSE;
        }
    }

    if (proceed == IM_INIT_TRUE) {
        // Shift elements back by one to fill the gap
        for(u8i = index; u8i < cb->u16Count -1; u8i++) {
            uint16_t src = (cb->u16Head + u8i +1) % cb->u16Capacity;
            uint16_t dest = (cb->u16Head + u8i) % cb->u16Capacity;
            (void)memcpy(&cb->au8_Buffer[dest * elementSize], &cb->au8_Buffer[src * elementSize], elementSize);
        }

        // Update tail pointer
        tempCalc = (uint32_t)(cb->u16Tail -1 + cb->u16Capacity) % cb->u16Capacity;
        cb->u16Tail = (uint16_t)tempCalc;
        tempCalc = (uint32_t)cb->u16Count - (uint32_t)IM_DECREMENT;
        cb->u16Count = (uint16_t)tempCalc;
        (void)memset(&cb->au8_Buffer[cb->u16Tail * cb->sz_ElementSize], IM_ZERO, cb->sz_ElementSize);
    }
}
