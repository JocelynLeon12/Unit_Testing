/**
* @file itcom.c
*****************************************************************************
* PROJECT NAME: Sonatus Automator Safety Interlock
*
* @brief module to implement inter task communication
*
* @authors Alejandro Tollola, Tusar Palauri
*
* @date August 08 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 08/08/2024|AT |Initial
* 10/03/2024|TP |Refactored for Action Request Timeout
*
*/
//*****************************************************************************


/*** Include Files ***/
#include "process_management.h"
#include "thread_management.h"
#include "storage_handler.h"

#include "itcom.h"



/*** Module Definitions ***/
#define ITCOM_NEG_ONE_INIT                    (-1)        /**< Buffer initialization value */
#define ITCOM_ZERO_INIT_U                     (0U)        /**< Buffer initialization value */
#define ITCOM_ONE_INIT_U                      (1U)        /**< Buffer initialization value */
#define ITCOM_ZERO_INIT_FLOAT                 (0.0f)      /**< Buffer initialization value */

#define SEC_TO_MS                             (1000)      /**< Conversion factor from seconds to milliseconds */
#define NSEC_TO_MS                            (1000000)   /**< Conversion factor from nanoseconds to milliseconds */

#ifndef MAP_ANONYMOUS
    #define MAP_ANONYMOUS                     (0x20)      /**< Fallback definition if not defined in the system headers */
#endif

#define CYCLE_COUNT_INVALID                   (0U)
#define ACTION_REQUEST_NOT_SAVED              (-1)
#define ELEMENT_NOT_FOUND_IN_CIR_BUFFER       (-1)
#define QUEUE_INDEX_INVALID                   (-1)


/*** Internal Types ***/


/*** Local Function Prototypes ***/
static uint8_t itcom_u8CompareMsgIdAndSequence(const_generic_ptr_t a, const_generic_ptr_t b);
static uint8_t itcom_u8CompareCalibData(const_generic_ptr_t a, const_generic_ptr_t b);
static void itcom_vRemoveActionRequestTiming(uint16_t u16MsgId, uint16_t u16SequenceNum);
static void ITCOM_vInit(void);
static struct timespec* ITCOM_pstGetActionRequestStartTime(uint16_t u16MsgId, uint16_t u16SequenceNum);

/*** External Variables ***/



/*** Internal Variables ***/

static DataOnSharedMemory* pstSharedMemData;

/*_______________________SHARED MEMORY VARIABLES_______________________*/



static MessageDictionary_t stMsgDictionary[] = {
    /* DictionaryIndex      MessageID                 MessageType                      MessageEnum                 */
        /*  [00]  */         {0x0000U,           (uint8_t)enActionRequest,        (uint8_t)enHVACFanSpeed            },
        /*  [01]  */         {0x0001U,           (uint8_t)enActionRequest,        (uint8_t)enHVACCabinTemperature    },
        /*  [02]  */         {0x0002U,           (uint8_t)enActionRequest,        (uint8_t)enWindshieldWiperSpeed    },
        /*  [03]  */         {0x0003U,           (uint8_t)enActionRequest,        (uint8_t)enSeatPositionDriver      },
        /*  [04]  */         {0x0004U,           (uint8_t)enActionRequest,        (uint8_t)enSeatPositionPassenger   },
        /*  [05]  */         {0x0005U,           (uint8_t)enActionRequest,        (uint8_t)enSeatHeaterDriver        },
        /*  [06]  */         {0x0006U,           (uint8_t)enActionRequest,        (uint8_t)enSeatHeaterPassenger     },
        /*  [07]  */         {0x0007U,           (uint8_t)enActionRequest,        (uint8_t)enDoorLockState           },
        /*  [08]  */         {0x0008U,           (uint8_t)enActionRequest,        (uint8_t)enTurnSignalState         },
        /*  [09]  */         {0x0009U,           (uint8_t)enActionRequest,        (uint8_t)enAmbientLighting         },
        /*  [10]  */         {0x000AU,           (uint8_t)enActionRequest,        (uint8_t)enTorqueVecMotorCalib     },
        /*  [11]  */         {0x07D0U,           (uint8_t)enActionRequest,        (uint8_t)enRainSensor              },
        /*  [12]  */         {NO_MSG_ID_ASSIGN,  (uint8_t)enAckMessage,           (uint8_t)enAckVAM                  },
        /*  [13]  */         {0x03E8U,           (uint8_t)enStatusMessageCM,      (uint8_t)enPRNDL                   },
        /*  [14]  */         {0x03E9U,           (uint8_t)enStatusMessageCM,      (uint8_t)enVehicleSpeed            },
        /*  [15]  */         {NO_MSG_ID_ASSIGN,  (uint8_t)enCalibReadbackMessage, (uint8_t)enCalibReadback           },
        /*  [16]  */         {NO_MSG_ID_ASSIGN,  (uint8_t)enAckMessage,           (uint8_t)enAckCM                   },
        /*  [17]  */         {0xFF02U,           (uint8_t)enNotificationMessage,  (uint8_t)enNonCriticalFail         },
        /*  [18]  */         {0xFF01U,           (uint8_t)enNotificationMessage,  (uint8_t)enCriticalFail            },
        /*  [19]  */         {NO_MSG_ID_ASSIGN,  (uint8_t)enNotificationMessage,  (uint8_t)enActionNotification      },
        /*  [20]  */         {0x1010U,           (uint8_t)enNotificationMessage,  (uint8_t)enStartUpTestNotification },
        /*  [21]  */         {0x1011U,           (uint8_t)enNotificationMessage,  (uint8_t)enStatusNotificationASI   }
};


static MessageTypeDictionary_t stMsgTypeDictionary[] = {
    /*MessageTypeID        MessageTypeEnum             AssociatedLength */
        {0xFF11U,      (uint8_t)enActionRequest,        {0x02, 0x04, 0x08}},
        {0xFF22U,      (uint8_t)enStatusMessageCM,      {0x02, 0x04}      },
        {0xFF33U,      (uint8_t)enAckMessage,           {0x01}            },
        {0xFF44U,      (uint8_t)enNotificationMessage,  {0x01}            },
        {0xFF55U,      (uint8_t)enCalibReadbackMessage, {0x02, 0x04, 0x08}}
};

/*** External Functions ***/

/**
 * @brief
 *
 * The function is typically called when:
 *      - The program starts and no previous state is available.
 *      - The existing state is corrupted or invalid and needs to be reset.
 *      - A fresh initialization is required for any other reason.
 *
 * @param data Pointer to the DataOnSharedMemory structure to be initialized.
 *
 */
void ITCOM_vSharedMemoryInit(FILE* itcom_log_file, enRestartReason restart_reason) {
    int32_t close_status;
    error_string_t error_str = NULL;
    uint8_t operation_status = ITCOM_OP_SUCCESS;

    /* Allocate shared memory for inter-process communication */
    if (restart_reason == (enRestartReason)enHardRestart) {
        pstSharedMemData = (DataOnSharedMemory*)mmap(NULL, SHARED_BUFFER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        if (pstSharedMemData == MAP_FAILED) {
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(itcom_log_file, LOG_ERROR, "mmap for shared buffer failed: %s", error_str);
            }
            operation_status = ITCOM_OP_FAILURE;
        } else {
            ITCOM_vInit();
            log_message(itcom_log_file, LOG_INFO, "Shared data initialized with default values");
        }
    } else if (restart_reason == (enRestartReason)enSoftRestart) {
        /* Load data from existing storage */
        if (compare_and_load_storage(pstSharedMemData) == -1) {
            log_message(itcom_log_file, LOG_ERROR, "Failed to compare and load storage data");
            operation_status = ITCOM_OP_FAILURE;
        } else {
            log_message(itcom_log_file, LOG_INFO, "Storage data loaded successfully");
        }
    } else {
        /* Intentionally empty else block */
    }

    /* Load event-related data from storage */
    if (operation_status == (uint8_t)ITCOM_OP_SUCCESS) {
        FM_vLoadEventDataFromStorage();
        log_message(itcom_log_file, LOG_INFO, "Event data loaded from storage");

        /* Initialize termination flag and synchronization primitives */
        pstSharedMemData->parent_initiated_termination = ITCOM_ZERO_INIT_U;
        init_mutexes_and_sems(pstSharedMemData);
    }

    /* Close the log file if an error occurred */
    if (operation_status == (uint8_t)ITCOM_OP_FAILURE) {
        close_status = fclose(itcom_log_file);
        if (close_status != (int32_t)ITCOM_ZERO_INIT_U) {
            log_message(global_log_file, LOG_ERROR, "Failed to close log file: error %d", close_status);
        }
    }
}

void ITCOM_vCleanResources(void) {
    // Clean up resources
    destroy_mutexes_and_sems(pstSharedMemData);
    destroy_timers();
    if (munmap(pstSharedMemData, SHARED_BUFFER_SIZE) == -1) {
        error_string_t error_str = strerror(errno);
        if (error_str != NULL) {
            log_message(global_log_file, LOG_ERROR, "munmap failed: %s", error_str);
        }
    }
}

void ITCOM_vWrapperThread_CCU(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_CCU: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_CCU.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_CCU: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_CCU: sem_wait failed: Unknown error");
            }
            break;
        }
        ICM_vCycleCountUpdater();
    }
    log_message(global_log_file, LOG_INFO, "THRD_CCU: Exiting thread");
}

void ITCOM_vWrapperThread_STM(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_STM: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_STM.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_STM: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_STM: sem_wait failed: Unknown error");
            }
            break;
        }
        STM_vMainTask();
    }
    log_message(global_log_file, LOG_INFO, "THRD_STM: Exiting thread");
}

void ITCOM_vWrapperThread_ICM_RX(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_ICM_RX: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_ICM_RX.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_ICM_RX: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_ICM_RX: sem_wait failed: Unknown error");
            }
            break;
        }
        ICM_vReceiveMessage();
    }
    log_message(global_log_file, LOG_INFO, "THRD_ICM_RX: Exiting thread");
}

void ITCOM_vWrapperThread_ARA(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_ARA: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_ARA.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_ARA: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_ARA: sem_wait failed: Unknown error");
            }
            break;
        }
        ARA_vVehicleStatusMonitor();
        ARA_vActionRequestMonitor();
    }
    log_message(global_log_file, LOG_INFO, "THRD_ARA: Exiting thread");
}

void ITCOM_vWrapperThread_ICM_TX(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_ICM_TX: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_ICM_TX.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_ICM_TX: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_ICM_TX: sem_wait failed: Unknown error");
            }
            break;
        }
        ICM_vTransmitMessage();
    }
    log_message(global_log_file, LOG_INFO, "THRD_ICM_TX: Exiting thread");
}

void ITCOM_vWrapperThread_FM(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_FM: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_FM.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_FM: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_FM: sem_wait failed: Unknown error");
            }
            break;
        }
        FM_vMainFunction();
    }
    log_message(global_log_file, LOG_INFO, "THRD_FM: Exiting thread");
}

void ITCOM_vWrapperThread_SD(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;
    bool exit_loop = false;

    log_message(global_log_file, LOG_INFO, "THRD_SD: Entering thread");

    while (!exit_loop) {
        /* Check thread exit and shutdown conditions */
        if (get_thread_exit() || sd_shutdown_initiated) {
            exit_loop = true;
        } else {
            sem_status = sem_wait(&pstSharedMemData->stThread_SD.sem);
            if (sem_status == -1) {
                if (errno == EINTR) {
                    /* Continue waiting if interrupted */
                    continue;
                }
                /* Handle semaphore error */
                error_str = strerror(errno);
                if (error_str != NULL) {
                    log_message(global_log_file, LOG_ERROR, "THRD_SD: sem_wait failed: %s", error_str);
                } else {
                    log_message(global_log_file, LOG_ERROR, "THRD_SD: sem_wait failed with unknown error");
                }
                exit_loop = true;
            } else if (!sd_shutdown_initiated) {
                /* Process main function if no shutdown is initiated */
                SD_vMainFunction();
            } else {
                /* Shutdown was initiated after semaphore acquisition */
                exit_loop = true;
            }
        }
    }
    
    log_message(global_log_file, LOG_INFO, "THRD_SD: Exiting thread");
}

void ITCOM_vWrapperThread_CRV(void) {
    int32_t sem_status;
    error_string_t error_str = NULL;

    log_message(global_log_file, LOG_INFO, "THRD_CRV: Entering thread");

    while (!get_thread_exit()) {
        sem_status = sem_wait(&pstSharedMemData->stThread_CRV.sem);
        if (sem_status == -1) {
            if (errno == EINTR) {
                continue;
            }
            error_str = strerror(errno);
            if (error_str != NULL) {
                log_message(global_log_file, LOG_ERROR, "THRD_CRV: sem_wait failed: %s", error_str);
            } else {
                log_message(global_log_file, LOG_ERROR, "THRD_CRV: sem_wait failed: Unknown error");
            }
            break;
        }
        CRV_vMainFunction();
    }
    log_message(global_log_file, LOG_INFO, "THRD_CRV: Exiting thread");
}

void ITCOM_vChildProcessWrapper(FILE* itcom_log_file, enRestartReason start_reason) {
	child_process(pstSharedMemData, itcom_log_file, start_reason);
}

void ITCOM_vParentdProcessWrapper(FILE* itcom_log_file) {
	parent_process(pstSharedMemData, itcom_log_file);
}

void ITCOM_vSetParentTerminationFlag(uint8_t u8Value) {
	pstSharedMemData->parent_initiated_termination = u8Value;
}

sig_atomic_t ITCOM_vGetParentTerminationFlag(void) {
	return pstSharedMemData->parent_initiated_termination;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vInit
//*****************************************************************************
/**
 * @brief This function is responsible for setting up the initial state of the
 * DataOnSharedMemory structure.
 *
 * The function is typically called when:
 *      - The program starts and no previous state is available.
 *      - The existing state is corrupted or invalid and needs to be reset.
 *      - A fresh initialization is required for any other reason.
 * 
 * @note Data initialization placed here overwrites the data retreived from the storage.
 *
 */
static void ITCOM_vInit(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    generic_ptr_t memory_operation_result = NULL;
    uint8_t initialization_complete = ITCOM_OP_FAILURE;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Init all zeros with result checking */
        memory_operation_result = memset(pstSharedMemData, 0, sizeof(DataOnSharedMemory));
        if ((memory_operation_result != NULL) && (memory_operation_result == pstSharedMemData)) {
            // Sequence Number Record
            InstanceManager_vInitialize(&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack, 
                                     sizeof(stMsgIntegrityData), NUM_TRACKED_ELEMENTS);
            // Calibration Data Copy
            InstanceManager_vInitialize(&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack, 
                                     sizeof(stProcessMsgData), NUM_TRACKED_ELEMENTS);
            // Calibration Readback Data
            InstanceManager_vInitialize(&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, 
                                     sizeof(stProcessMsgData), NUM_TRACKED_ELEMENTS);
            // Data Integrity Queue
            DataQueue_vInit(&pstSharedMemData->stThreadsCommonData.stActionReqQueue, 
                          (uint8_t*)pstSharedMemData->stThreadsCommonData.astDataIntegrityMsgBuffer,
                          (sizeof(pstSharedMemData->stThreadsCommonData.astSSMsgBuffer) / 
                           sizeof(pstSharedMemData->stThreadsCommonData.astSSMsgBuffer[0])),
                          (uint16_t)sizeof(stProcessMsgData), CIRCULAR_BUFF_INACTIVE);
            // Approved Actions Queue
            DataQueue_vInit(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue, 
                          (uint8_t*)pstSharedMemData->stThreadsCommonData.astApprovedMsgBuffer,
                          (sizeof(pstSharedMemData->stThreadsCommonData.astApprovedMsgBuffer) / 
                           sizeof(pstSharedMemData->stThreadsCommonData.astApprovedMsgBuffer[0])),
                          (uint16_t)sizeof(stProcessMsgData), CIRCULAR_BUFF_INACTIVE);
            // Safe State Queue
            DataQueue_vInit(&pstSharedMemData->stThreadsCommonData.stMsgQueueSS, 
                          (uint8_t*)pstSharedMemData->stThreadsCommonData.astSSMsgBuffer,
                          (sizeof(pstSharedMemData->stThreadsCommonData.astSSMsgBuffer) / 
                           sizeof(pstSharedMemData->stThreadsCommonData.astSSMsgBuffer[0])),
                          (uint16_t)sizeof(stProcessMsgData), CIRCULAR_BUFF_INACTIVE);
            
            ///State Machine Initialization
            pstSharedMemData->stThreadsCommonData.u8ASI_State = (uint8_t)STATE_INITIAL;
            pstSharedMemData->stThreadsCommonData.u8CriticalFaultFlag = INACTIVE_FLAG;
            pstSharedMemData->stThreadsCommonData.u8InitFinishFlag = ACTIVE_FLAG;

            ///ICM Initialization
            pstSharedMemData->stThreadsCommonData.u16GnrlCycleCount = ITCOM_ZERO_INIT_U;

            ///ARA Initialization
            pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8ParkStatus = enParkStatus;
            pstSharedMemData->stThreadsCommonData.stVehicleStatus.fVehicleSpeed = ITCOM_ZERO_INIT_FLOAT;
            pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[0] = INFO_OUTDATED;
            pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[1] = INFO_OUTDATED;

            /// FM Initialization
            pstSharedMemData->stThreadsCommonData.Event_Queue_Index = ITCOM_ZERO_INIT_U;
            pstSharedMemData->stThreadsCommonData.u8CriticalFaultFlag = INACTIVE_FLAG;

            /// SD Initialization
            pstSharedMemData->stThreadsCommonData.stStateMonitorData.stCurrentState = STATE_INITIAL;
            pstSharedMemData->stThreadsCommonData.stStateMonitorData.u8StateError = ITCOM_ZERO_INIT_U;
            pstSharedMemData->stThreadsCommonData.enTCPConnectionState[enVAMConnectionTCP] = CONNECTION_STATE_DISCONNECTED;
            pstSharedMemData->stThreadsCommonData.enTCPConnectionState[enCMConnectionTCP] = CONNECTION_STATE_DISCONNECTED;
            
            initialization_complete = ITCOM_OP_SUCCESS;
        } else {
            log_message(global_log_file, LOG_ERROR, "Memory initialization failed");
        }

        /* Always unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vInit failed to unlock mutex: error %d", mutex_unlock_status);
            initialization_complete = ITCOM_OP_FAILURE;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vInit failed to lock mutex: error %d", mutex_lock_status);
    }

    if (initialization_complete == (uint8_t)ITCOM_OP_FAILURE) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vInit failed to complete initialization");
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetASIState
//*****************************************************************************
/**
*
* @brief Sets the ASI state to the specified value.
*
* @param [in] u8Value New ASI state value
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetASIState(uint8_t u8Value) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the ASI state value */
        pstSharedMemData->stThreadsCommonData.u8ASI_State = u8Value;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetASIState failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetASIState failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetASIState
//*****************************************************************************
/**
*
* @brief Retrieves the current ASI state.
*
* @param none
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Current ASI state
*/
uint8_t ITCOM_u8GetASIState(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8Status = ITCOM_ZERO_INIT_U;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Get the ASI state value */
        u8Status = pstSharedMemData->stThreadsCommonData.u8ASI_State;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetASIState failed to unlock mutex: error %d", mutex_unlock_status);
            u8Status = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetASIState failed to lock mutex: error %d", mutex_lock_status);
        u8Status = ITCOM_ZERO_INIT_U;
    }
    return u8Status;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16SetErrorEvent
//*****************************************************************************
/**
 * @brief Adds a new error event to the event queue.
 *
 * This function attempts to add a new error event to the shared event queue.
 * It provides thread-safe access to the queue, handles queue overflow scenarios,
 * and manages event priorities based on severity.
 *
 * @param event_id An unsigned char representing the ID of the error event to be added.
 * @return int Status code:
 *             0 - Success (event added to queue)
 *             1 - Failure (invalid event ID)
 *             2 - Failure (event discarded due to lower severity in a full queue)
 *
 * The function performs the following operations:
 * 1. Validates the event_id against the total number of event IDs.
 * 2. Locks the mutex for the shared event queue.
 * 3. If the queue is not full:
 *    - Captures snapshot data.
 *    - Adds the event to the queue.
 * 4. If the queue is full:
 *    - Finds the least severe event in the queue.
 *    - If the new event is more severe, replaces the least severe event.
 *    - If the new event is less severe, it is discarded.
 * 5. Prints the current event queue for debugging.
 * 6. Unlocks the mutex.
 *
 * @note This function assumes the existence of:
 *       - A global shared data structure (shared_data) containing the event queue and associated mutex.
 *       - A global Error_Events array containing ErrorEvent structures with severity information.
 *       - Global functions for capturing snapshot data and logging.
 *
 * @warning This function modifies the shared event queue. Ensure proper synchronization
 *          when accessing the queue from multiple threads or processes.
 *
 */
enSetErrorEventStatus ITCOM_s16SetErrorEvent(uint8_t u8EventId)
{
    thread_name_t thread_name = get_current_thread_name();
    enSetErrorEventStatus result = enSuccess_EventAddedToQueue;
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    log_message(global_log_file, LOG_DEBUG, "set_errorevent called with event_id: %d by thread: %s", u8EventId, thread_name);

    /* Check for valid event ID first */
    if (u8EventId >= enTotalEventIds)
    {
        log_message(global_log_file, LOG_WARNING, "Thread %s tried adding Invalid Event ID %d to the Event_Queue, but it's discarded", thread_name, u8EventId);
        return enFailure_InvalidEventID;
    }

    /* Attempt to lock mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status != E_OK)
    {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s16SetErrorEvent failed to lock mutex: error %d", mutex_lock_status);
        return enFailure_MutexError;
    }

    /* Process the event */
    if (pstSharedMemData->stThreadsCommonData.Event_Queue_Index < (int32_t)DATA_QUEUE_MAX_SIZE)
    {
        /* Update snapshot data and add event to queue */
        pstSharedMemData->stThreadsCommonData.SystemSnapshotData.ASI_State = pstSharedMemData->stThreadsCommonData.u8ASI_State;
        pstSharedMemData->stThreadsCommonData.SystemSnapshotData.GearShiftPosition = pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8ParkStatus;
        pstSharedMemData->stThreadsCommonData.SystemSnapshotData.VehicleSpeed = pstSharedMemData->stThreadsCommonData.stVehicleStatus.fVehicleSpeed;

        const int32_t current_index = pstSharedMemData->stThreadsCommonData.Event_Queue_Index;
        pstSharedMemData->stThreadsCommonData.Event_Queue[current_index] = u8EventId;
        pstSharedMemData->stThreadsCommonData.Event_Queue_Index++;

        log_message(global_log_file, LOG_INFO, "Thread %s added Event ID %d to the Event_Queue", thread_name, u8EventId);
        result = enSuccess_EventAddedToQueue;
    }
    else
    {
        /* Handle queue full scenario */
        uint32_t least_severe_index = FM_u32FindLeastSevereEvent(pstSharedMemData->stThreadsCommonData.Event_Queue, DATA_QUEUE_MAX_SIZE);
        uint8_t least_severe_event_id = (uint8_t)pstSharedMemData->stThreadsCommonData.Event_Queue[least_severe_index];
        uint8_t u8EventSeverity = FM_u8GetEventSeverity(u8EventId);
        uint8_t u8EventLowestSeverity = FM_u8GetEventSeverity(least_severe_event_id);

        if (u8EventSeverity > u8EventLowestSeverity)
        {
            /* Update snapshot data and replace least severe event */
            pstSharedMemData->stThreadsCommonData.SystemSnapshotData.ASI_State = pstSharedMemData->stThreadsCommonData.u8ASI_State;
            pstSharedMemData->stThreadsCommonData.SystemSnapshotData.GearShiftPosition = pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8ParkStatus;
            pstSharedMemData->stThreadsCommonData.SystemSnapshotData.VehicleSpeed = pstSharedMemData->stThreadsCommonData.stVehicleStatus.fVehicleSpeed;
            log_message(global_log_file, LOG_WARNING, "Event Queue full. Replacing Event ID %d with new Event ID %d", least_severe_event_id, u8EventId);
            pstSharedMemData->stThreadsCommonData.Event_Queue[least_severe_index] = u8EventId;
            result = enSuccess_EventAddedToQueue;
        }
        else
        {
            log_message(global_log_file, LOG_WARNING, 
                      "Event Queue full. New Event ID %d (severity %d) discarded as it's not more severe than existing events",
                      u8EventId, u8EventSeverity);
            result = enFailure_EventDiscarded;
        }
    }

    /* Unlock mutex */
    mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_unlock_status != E_OK)
    {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s16SetErrorEvent failed to unlock mutex: error %d", mutex_unlock_status);
        return enFailure_MutexError;
    }

    return result;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vUpdateCurrentEvent
//*****************************************************************************
/**
*
* @brief
*
* @param
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return none
*/
void ITCOM_vUpdateCurrentEvent(ErrorEvent* pstCurrentEvent) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Input parameter validation */
    if (pstCurrentEvent == NULL) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vUpdateCurrentEvent: NULL pointer received");
        return;
    }

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_FM.mutex);
    if (mutex_lock_status == E_OK) {
        /* Update the current event */
        pstSharedMemData->stThread_FM.current_event = *pstCurrentEvent;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_FM.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vUpdateCurrentEvent failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vUpdateCurrentEvent failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetErrorEvent
//*****************************************************************************
/**
*
* @brief Retrieves logged events from the event queue.
*
* @param [out] pu8LoggedEvents Pointer to store logged events
* @param [out] pu8DequeueStatus Pointer to store dequeue status for each event
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return none
*/
void ITCOM_vGetErrorEvent(ErrorEvent* const pstCurrentEvent)
{
    mutex_status_t mutex_lock_status = 0;
    mutex_status_t mutex_unlock_status = 0;
    bool is_operation_valid = TRUE;

    /* Input parameter validation */
    if ((NULL == pstCurrentEvent) || (NULL == pstSharedMemData))
    {
        log_message(global_log_file, LOG_ERROR, 
                   "ITCOM_vGetErrorEvent: NULL pointer received");
        is_operation_valid = FALSE;
    }

    if (TRUE == is_operation_valid)
    {
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&(pstSharedMemData->stThread_FM.mutex));
        if (E_OK == mutex_lock_status)
        {
            /* Critical section - keep it minimal */
            *pstCurrentEvent = pstSharedMemData->stThread_FM.current_event;

            /* Release mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&(pstSharedMemData->stThread_FM.mutex));
            if (E_OK != mutex_unlock_status)
            {
                log_message(global_log_file, LOG_ERROR, 
                           "ITCOM_vGetErrorEvent mutex unlock failed: %d", 
                           mutex_unlock_status);
                /* Clear the event data if mutex unlock fails */
                (void)memset(pstCurrentEvent, 0, sizeof(ErrorEvent));
            }
        }
        else
        {
            log_message(global_log_file, LOG_ERROR, 
                       "ITCOM_vGetErrorEvent mutex lock failed: %d", 
                       mutex_lock_status);
        }
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetInitFlagStatus
//*****************************************************************************
/**
*
* @brief Sets the status of initialization flag.
*
* @param [in]
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Status of the init flag.
*/
void ITCOM_vSetInitFlagStatus(uint8_t u8FlagValue) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the init flag value */
        pstSharedMemData->stThreadsCommonData.u8InitFinishFlag = u8FlagValue;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetInitFlagStatus failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetInitFlagStatus failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetInitFlagStatus
//*****************************************************************************
/**
*
* @brief Checks the status of initialization flag.
*
* @param [in]
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Status of the init flag.
*/
uint8_t ITCOM_u8GetInitFlagStatus(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8TempInitFlagStatus = INACTIVE_FLAG;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Get the init flag value */
        u8TempInitFlagStatus = pstSharedMemData->stThreadsCommonData.u8InitFinishFlag;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetInitFlagStatus failed to unlock mutex: error %d", mutex_unlock_status);
            u8TempInitFlagStatus = INACTIVE_FLAG;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetInitFlagStatus failed to lock mutex: error %d", mutex_lock_status);
    }
    return u8TempInitFlagStatus;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetCriticalFault
//*****************************************************************************
/**
*
* @brief Sets the critical fault flag to ACTIVE_FLAG.
*
* @param none
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return none
*/
static void ITCOM_vSetCriticalFault(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the critical fault flag to active */
        pstSharedMemData->stThreadsCommonData.u8CriticalFaultFlag = (uint8_t)ACTIVE_FLAG;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCriticalFault failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCriticalFault failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetCriticalFaultStatus
//*****************************************************************************
/**
*
* @brief Checks the status of the critical fault flag.
*
* @param none
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Status of the critical fault flag
*/
uint8_t ITCOM_u8GetCriticalFaultStatus(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8CriticalFaultStatus = ITCOM_ZERO_INIT_U;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Get the critical fault status */
        u8CriticalFaultStatus = pstSharedMemData->stThreadsCommonData.u8CriticalFaultFlag;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetCriticalFaultStatus failed to unlock mutex: error %d", mutex_unlock_status);
            u8CriticalFaultStatus = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetCriticalFaultStatus failed to lock mutex: error %d", mutex_lock_status);
        u8CriticalFaultStatus = ITCOM_ZERO_INIT_U;
    }
    return u8CriticalFaultStatus;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetCycleCountData
//*****************************************************************************
/**
*
* @brief Sets the general cycle count to the specified value.
*
* @param [in] u16Value New cycle count value
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetCycleCountData(uint16_t u16Value) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the cycle count value */
        pstSharedMemData->stThreadsCommonData.u16GnrlCycleCount = u16Value;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCycleCountData failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCycleCountData failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u16GetCycleCountData
//*****************************************************************************
/**
*
* @brief Retrieves the current general cycle count.
*
* @param none
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint16_t Current cycle count
*/
uint16_t ITCOM_u16GetCycleCountData(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint16_t u16CycleCount = CYCLE_COUNT_INVALID;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Get the cycle count value */
        u16CycleCount = pstSharedMemData->stThreadsCommonData.u16GnrlCycleCount;
        
        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetCycleCountData failed to unlock mutex: error %d", mutex_unlock_status);
            u16CycleCount = CYCLE_COUNT_INVALID;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetCycleCountData failed to lock mutex: error %d", mutex_lock_status);
        u16CycleCount = CYCLE_COUNT_INVALID;
    }
    return u16CycleCount;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetMsgCycleCount
//*****************************************************************************
/**
 * @brief Manages message cycle count data in the shared memory structure.
 *
 * This function performs operations on the cycle sequence tracking buffer (stCycleSeqTrack) 
 * based on the specified action. It can add, update, or remove message integrity data elements.
 *
 * @param[in] pstTempMsgTracker Pointer to a stMsgIntegrityData structure containing the message 
 *                              data to be added, updated, or used as a key for removal.
 * @param[in] u8Action An 8-bit unsigned integer specifying the action to perform. Valid values are:
 *                     - ADD_ELEMENT: Adds a new element to the tracking buffer.
 *                     - UPDATE_ELEMENT: Updates an existing element in the buffer.
 *                     - REMOVE_ELEMENT: Removes an element from the buffer.
 *
 * @par Implementation Details
 * The function performs the following steps:
 * 1. Locks the mutex to ensure exclusive access to the shared data.
 * 2. If the action is ADD_ELEMENT, it directly adds the new element to the tracking buffer.
 * 3. For other actions, it first searches for the element in the buffer using InstanceManager_s8FindElement.
 * 4. If found and the action is UPDATE_ELEMENT, it updates the element and logs the update.
 * 5. If found and the action is REMOVE_ELEMENT, it removes the element and logs the removal.
 * 6. If the element is not found or the action is invalid, it logs a debug message.
 * 7. Unlocks the mutex.
 *
 */
void ITCOM_vSetMsgCycleCount(stMsgIntegrityData* pstTempMsgTracker, uint8_t u8Action) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int16_t s16Indx = ITCOM_ZERO_INIT_U;
    stMsgIntegrityData stFoundInstance;
    
    /* Attempt to lock mutex - continue even if logging fails */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status != E_OK) {
        (void)log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMsgCycleCount failed to lock mutex: error %d", 
                   mutex_lock_status);
        return;
    }

    /* Process based on action type */
    if(u8Action == (uint8_t)ADD_ELEMENT) {
        InstanceManager_vAddElement(&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack, pstTempMsgTracker);
        (void)log_message(global_log_file, LOG_DEBUG, "MESSAGE STARTED TRACKING, MSG: 0x%04X, Sequence Num: 0x%04X", 
                   pstTempMsgTracker->stMsgPairData.u16MsgId, 
                   pstTempMsgTracker->stMsgPairData.u16SequenceNum);
    } else {
        s16Indx = InstanceManager_s8FindElement(
            (generic_ptr_t)&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack, 
            (generic_ptr_t)pstTempMsgTracker, 
            (ElementCompareFn)&itcom_u8CompareMsgIdAndSequence, 
            (generic_ptr_t)&stFoundInstance);
        
        if (s16Indx >= (int16_t)ITCOM_ZERO_INIT_U) {
            if (u8Action == (uint8_t)UPDATE_ELEMENT) {
                InstanceManager_vUpdateElement(&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack, 
                                            s16Indx, 
                                            pstTempMsgTracker);
                (void)log_message(global_log_file, LOG_DEBUG, 
                          "MESSAGE UPDATED, TYPE: 0x%04X, MSG: 0x%04X, SEQ NUM: 0x%04X, Clear Condition: %d, Response Cycle: %d", 
                          pstTempMsgTracker->u16Type, 
                          pstTempMsgTracker->stMsgPairData.u16MsgId, 
                          pstTempMsgTracker->stMsgPairData.u16SequenceNum, 
                          pstTempMsgTracker->u8ClearCondition,
                          pstTempMsgTracker->u8ResponseCycleCount);
            } else if (u8Action == (uint8_t)REMOVE_ELEMENT) {
                InstanceManager_vRemoveElement(&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack, s16Indx);
                (void)log_message(global_log_file, LOG_DEBUG, 
                          "REMOVE FROM TRACKING, TYPE: 0x%04X, MSG: 0x%04X, SEQ NUM: 0x%04X, Clear Condition: %d, Response Cycle: %d", 
                          pstTempMsgTracker->u16Type, 
                          pstTempMsgTracker->stMsgPairData.u16MsgId, 
                          pstTempMsgTracker->stMsgPairData.u16SequenceNum, 
                          pstTempMsgTracker->u8ClearCondition,
                          pstTempMsgTracker->u8ResponseCycleCount);
            } else {
                (void)log_message(global_log_file, LOG_DEBUG, 
                          "INVALID ACTION, TYPE: 0x%04X, MSG: 0x%04X, SEQ NUM: 0x%04X, Clear Condition: %d", 
                          pstTempMsgTracker->u16Type, 
                          pstTempMsgTracker->stMsgPairData.u16MsgId, 
                          pstTempMsgTracker->stMsgPairData.u16SequenceNum, 
                          pstTempMsgTracker->u8ClearCondition);
            }
        } else {
            (void)log_message(global_log_file, LOG_DEBUG, 
                      "ELEMENT NOT FOUND, TYPE: 0x%04X, MSG: 0x%04X, SEQ NUM: 0x%04X, Clear Condition: %d", 
                      pstTempMsgTracker->u16Type, 
                      pstTempMsgTracker->stMsgPairData.u16MsgId, 
                      pstTempMsgTracker->stMsgPairData.u16SequenceNum, 
                      pstTempMsgTracker->u8ClearCondition);
        }
    }

    /* Always attempt to unlock mutex */
    mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_unlock_status != E_OK) {
        (void)log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMsgCycleCount failed to unlock mutex: error %d", 
                   mutex_unlock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s8SaveMsgData
//*****************************************************************************
/**
*
* @brief Saves message data and updates instance tracking information.
*
* @param [in] pstTCPMessage Pointer to the TLV message structure
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Status of save operation (-1: error, 0: success)
*/
int8_t ITCOM_s8SaveMsgData(stProcessMsgData* pstMsgPayload, int16_t s16Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int8_t s8Return = ACTION_REQUEST_NOT_SAVED;

    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Enqueue the message data and update sequence number */
        s8Return = DataQueue_s8Enqueue(&pstSharedMemData->stThreadsCommonData.stActionReqQueue, (uint8_t *)pstMsgPayload, sizeof(stProcessMsgData));
        pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[s16Indx].u16SeqNumberSender = pstMsgPayload->stMsgPairData.u16SequenceNum;

        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s8SaveMsgData failed to unlock mutex: error %d", mutex_unlock_status);
            s8Return = ACTION_REQUEST_NOT_SAVED; /* Adjust return in case of unlock failure */
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s8SaveMsgData failed to lock mutex: error %d", mutex_lock_status);
        s8Return = ACTION_REQUEST_NOT_SAVED;
    }
    return s8Return;
}


//*****************************************************************************
// FUNCTION NAME : ITCOM_s8QueueActionReq
//*****************************************************************************
/**
*
* @brief Enqueues an action request into the approved actions queue.
*
* @param [in] pstMsgInfo Pointer to the process message data
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Status of enqueue operation (-1: error, 0: success)
*/
int8_t ITCOM_s8QueueActionReq(stProcessMsgData* pstMsgInfo) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int8_t s8Return = QUEUE_ACTION_FAILURE_DEFAULT;
    int32_t time_status;
    struct timespec end_time;
    const struct timespec* p_start_time = NULL;
    int64_t elapsed_ms = ITCOM_ZERO_INIT_U;
    uint8_t operation_status = ITCOM_OP_FAILURE;

    time_status = clock_gettime(CLOCK_MONOTONIC, &end_time);
    if (time_status != (int32_t)ITCOM_ZERO_INIT_U) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s8QueueActionReq failed to get time: error %d", time_status);
    } else {
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            p_start_time = ITCOM_pstGetActionRequestStartTime(pstMsgInfo->stMsgPairData.u16MsgId, pstMsgInfo->stMsgPairData.u16SequenceNum);

            if (p_start_time != NULL) {
                elapsed_ms = (end_time.tv_sec - p_start_time->tv_sec) * SEC_TO_MS +
                             (end_time.tv_nsec - p_start_time->tv_nsec) / NSEC_TO_MS;

                if (elapsed_ms <= (int64_t)ACTION_REQUEST_PROCESS_TIMEOUT_THRESHOLD) {
                    s8Return = DataQueue_s8Enqueue(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue, (uint8_t *)pstMsgInfo, sizeof(stProcessMsgData));
                } else {
                    log_message(global_log_file, LOG_WARNING, "Action request processing timeout: %ld ms", elapsed_ms);
                    s8Return = QUEUE_ACTION_TIMEOUT;
                }

                itcom_vRemoveActionRequestTiming(pstMsgInfo->stMsgPairData.u16MsgId, pstMsgInfo->stMsgPairData.u16SequenceNum);
                operation_status = ITCOM_OP_SUCCESS;
            } else {
                log_message(global_log_file, LOG_WARNING, "No start time found for Action Request: MsgId 0x%04X, SeqNum %u", 
                            pstMsgInfo->stMsgPairData.u16MsgId, pstMsgInfo->stMsgPairData.u16SequenceNum);
            }

            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_s8QueueActionReq failed to unlock mutex: error %d", mutex_unlock_status);
                s8Return = QUEUE_ACTION_FAILURE_DEFAULT;
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s8QueueActionReq failed to lock mutex: error %d", mutex_lock_status);
        }
    }

    if (operation_status == (uint8_t)ITCOM_OP_FAILURE) {
        s8Return = QUEUE_ACTION_FAILURE_DEFAULT;
    }

    return s8Return;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s8DequeueActionReq
//*****************************************************************************
/**
*
* @brief Dequeues an action request from the specified queue.
*
* @param [out] pstActionReqData Pointer to store dequeued action request data
* @param [in] u8SelectQueue Queue selection flag (DATA_INTEGRITY_QUEUE or APPROVED_ACTIONS_QUEUE)
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return int8_t Status of dequeue operation
*/
int8_t ITCOM_s8DequeueActionReq(stProcessMsgData* pstActionReqData, uint8_t u8SelectQueue) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int8_t s8Return = QUEUE_ACTION_FAILURE_DEFAULT;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Select the appropriate queue based on u8SelectQueue */
        if (u8SelectQueue == (uint8_t)DATA_INTEGRITY_QUEUE) {
            s8Return = DataQueue_s8Dequeue(&pstSharedMemData->stThreadsCommonData.stActionReqQueue, (uint8_t *)pstActionReqData, sizeof(stProcessMsgData));
        } else if (u8SelectQueue == (uint8_t)APPROVED_ACTIONS_QUEUE) {
            s8Return = DataQueue_s8Dequeue(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue, (uint8_t *)pstActionReqData, sizeof(stProcessMsgData));
        } else if (u8SelectQueue == (uint8_t)SAFE_STATE_QUEUE) {
            s8Return = DataQueue_s8Dequeue(&pstSharedMemData->stThreadsCommonData.stMsgQueueSS, (uint8_t *)pstActionReqData, sizeof(stProcessMsgData));
        } else {
            /* Intentionally empty else block */
        }

        /* Log dequeue operation result */
        if (s8Return == (int8_t)ITCOM_ZERO_INIT_U) {
            log_message(global_log_file, LOG_DEBUG, "ITCOM_s8DequeueActionReq: Dequeue operation successful. Message ID: 0x%04X, Sequence Number: %d", 
                        pstActionReqData->stMsgPairData.u16MsgId, pstActionReqData->stMsgPairData.u16SequenceNum);
        } else {
            log_message(global_log_file, LOG_DEBUG, "ITCOM_s8DequeueActionReq: Dequeue operation failed with return code: %d", s8Return);
        }

        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s8DequeueActionReq failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s8DequeueActionReq failed to lock mutex: error %d", mutex_lock_status);
    }
    return s8Return;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s8LogSSMessage
//*****************************************************************************
/**
*
* @brief Logs a safe state message by clearing action request and approved actions queues.
*
* @param none
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Status of enqueue operation for safe state message
*/
int8_t ITCOM_s8LogSSMessage(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    stProcessMsgData stTemp;
    int8_t s8EequeueStatus = ENQUEUE_OPERATION_FAILURE;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Clear the queues if they are not empty */
        if (!DataQueue_u8IsEmpty(&pstSharedMemData->stThreadsCommonData.stActionReqQueue)) {
            DataQueue_vClear(&pstSharedMemData->stThreadsCommonData.stActionReqQueue);
        }
        if (!DataQueue_u8IsEmpty(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue)) {
            DataQueue_vClear(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue);
        }

        /* Set message data */
        stTemp.stMsgPairData.u16SequenceNum = pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[enStatusNotificationASI].u16SeqNumberASI;
        stTemp.stMsgPairData.u16MsgId = stMsgDictionary[enStatusNotificationASI].u16MessageId;
        stTemp.u16Type = stMsgTypeDictionary[enNotificationMessage].u16MessageTypeID;
        stTemp.au8MsgData[0] = (uint8_t)STATE_SAFE_STATE;
        pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[enStatusNotificationASI].u16SeqNumberASI++;

        /* Enqueue the message */
        s8EequeueStatus = DataQueue_s8Enqueue(&pstSharedMemData->stThreadsCommonData.stMsgQueueSS, (uint8_t *)&stTemp, sizeof(stProcessMsgData));

        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status == E_OK) {
            s8EequeueStatus = ENQUEUE_OPERATION_SUCCESS;
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s8LogSSMessage failed to unlock mutex: error %d", mutex_unlock_status);
            s8EequeueStatus = ENQUEUE_OPERATION_FAILURE;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s8LogSSMessage failed to lock mutex: error %d", mutex_lock_status);
        s8EequeueStatus = ENQUEUE_OPERATION_FAILURE;
    }

    return s8EequeueStatus;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s8LogNotificationMessage
//*****************************************************************************
/**
 * @brief Logs a notification message to the approved actions queue.
 *
 * This function creates and enqueues a notification message based on the provided parameters.
 * It handles different types of notifications, including action notifications, startup test
 * notifications, and ASI status notifications.
 *
 * @param u16MsgId The message ID for the notification. This is used directly for action
 *                 notifications, while for other types, it's retrieved from the message dictionary.
 * @param u16SequenceNum The sequence number for the notification. This is used directly for
 *                       action notifications, while for other types, it's retrieved from the
 *                       sequence number register.
 * @param u8Data The data payload for the notification message. This is stored in the first
 *               byte of the message data array.
 * @param u8SelectNotification The type of notification to be logged. This parameter determines
 *                             how the message is constructed and which queue it's added to.
 *                             Possible values are:
 *                             - enActionNotification
 *                             - enStartUpTestNotification
 *                             - enStatusNotificationASI
 *
 * @return int8_t Returns the status of the enqueue operation:
 *         - 0 if the message was successfully enqueued
 *         - -1 if the enqueue operation failed
 *
 * @note This function uses a mutex to ensure thread-safe access to shared data structures.
 *
 * @details The function performs the following steps:
 * 1. Initializes a temporary message data structure with the notification message type and length.
 * 2. Sets the data payload in the message structure.
 * 3. Locks the mutex to ensure exclusive access to shared data.
 * 4. Based on the notification type:
 *    - For action notifications:
 *      - Sets the message ID and sequence number directly from the parameters.
 *      - Updates the sequence number register for the sender.
 *    - For startup test and ASI status notifications:
 *      - Retrieves the message ID from the message dictionary.
 *      - Retrieves the sequence number from the ASI sequence number register.
 * 5. Enqueues the message to the approved actions queue.
 * 6. Unlocks the mutex.
 * 7. Returns the status of the enqueue operation.
 *
 */
int8_t ITCOM_s8LogNotificationMessage(uint16_t u16MsgId, uint16_t u16SequenceNum, uint8_t u8Data, uint8_t u8SelectNotification) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    stProcessMsgData stTempMsgData = {ITCOM_ZERO_INIT_U};
    int8_t s8QueueStatus = QUEUE_ACTION_FAILURE_DEFAULT;

    /* Initialize message data */
    stTempMsgData.u16Type = stMsgTypeDictionary[enNotificationMessage].u16MessageTypeID;
    stTempMsgData.u16Length = stMsgTypeDictionary[enNotificationMessage].au8AssociatedLengths[0];
    stTempMsgData.au8MsgData[0] = u8Data;

    /* Lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Process the notification based on selection */
        if (u8SelectNotification == enActionNotification) {
            stTempMsgData.stMsgPairData.u16MsgId = u16MsgId;
            stTempMsgData.stMsgPairData.u16SequenceNum = u16SequenceNum;
            pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[enActionNotification].u16SeqNumberSender = stTempMsgData.stMsgPairData.u16SequenceNum;
            s8QueueStatus = DataQueue_s8Enqueue(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue, (uint8_t *)&stTempMsgData, sizeof(stProcessMsgData));
        } else if ((u8SelectNotification == enStartUpTestNotification) || (u8SelectNotification == enStatusNotificationASI)) {
            stTempMsgData.stMsgPairData.u16MsgId = stMsgDictionary[u8SelectNotification].u16MessageId;
            stTempMsgData.stMsgPairData.u16SequenceNum = pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[u8SelectNotification].u16SeqNumberASI;
            s8QueueStatus = DataQueue_s8Enqueue(&pstSharedMemData->stThreadsCommonData.stApprovedActionsQueue, (uint8_t *)&stTempMsgData, sizeof(stProcessMsgData));
        } else {
            /* Intentionally empty else block */
        }

        /* Unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s8LogNotificationMessage failed to unlock mutex: error %d", mutex_unlock_status);
            s8QueueStatus = QUEUE_ACTION_FAILURE_DEFAULT;  /* Adjust return value on mutex unlock failure */
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s8LogNotificationMessage failed to lock mutex: error %d", mutex_lock_status);
        s8QueueStatus = QUEUE_ACTION_FAILURE_DEFAULT;
    }
    return s8QueueStatus;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetSeqNumASIRecord
//*****************************************************************************
/**
*
* @brief Sets the sequence number for the ASI record at the specified index.
*
* @param [in] u16SequenceNum Sequence number to set
* @param [in] u8Indx Index of the ASI record to update
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetSeqNumASIRecord(uint16_t u16SequenceNum, uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Update the sequence number if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[u8Indx].u16SeqNumberASI = u16SequenceNum;
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetSeqNumASIRecord failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetSeqNumASIRecord failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u16GetSeqNumASIRecord
//*****************************************************************************
/**
*
* @brief Retrieves the sequence number for the ASI record at the specified index.
*
* @param [in] u8Indx Index of the ASI record
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint16_t Sequence number of the ASI record
*/
uint16_t ITCOM_u16GetSeqNumASIRecord(uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint16_t u16TempSeqNum = ITCOM_ZERO_INIT_U;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Read the sequence number if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            u16TempSeqNum = pstSharedMemData->stThreadsCommonData.stSeqNumberRegister[u8Indx].u16SeqNumberASI;
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetSeqNumASIRecord failed to unlock mutex: error %d", mutex_unlock_status);
            u16TempSeqNum = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetSeqNumASIRecord failed to lock mutex: error %d", mutex_lock_status);
        u16TempSeqNum = ITCOM_ZERO_INIT_U;
    }
    return u16TempSeqNum;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vRecordRC
//*****************************************************************************
/**
*
*
*/
void ITCOM_vRecordRC(uint8_t u8MsgInstance, uint16_t u16RollingCounter, uint8_t u8Direction) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Record the rolling counter if the message instance is valid */
        if (u8MsgInstance < enTotalMessagesASI) {
            if (u8Direction == (uint8_t)ROLLING_COUNT_RX) {
                pstSharedMemData->stThreadsCommonData.stRollingCounterRegister[u8MsgInstance].u16RollingCountRX = u16RollingCounter;
            } else if (u8Direction == (uint8_t)ROLLING_COUNT_TX) {
                pstSharedMemData->stThreadsCommonData.stRollingCounterRegister[u8MsgInstance].u16RollingCountTX = u16RollingCounter;
            } else {
                /* Intentionally empty else block */
            }
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vRecordRC failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vRecordRC failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u16GetRCData
//*****************************************************************************
/**
*
* @brief Retrieves rolling counter data for the specified message instance and direction.
*
* @param [in] u8MsgInstance Message instance to check
* @param [in] u8Direction Direction of rolling count (ROLLING_COUNT_RX or ROLLING_COUNT_TX)
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return int16_t Rolling counter data for the specified direction
*/
int16_t ITCOM_u16GetRCData(uint8_t u8MsgInstance, uint8_t u8Direction) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int16_t u16RC = ITCOM_ZERO_INIT_U;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Retrieve the rolling counter based on the direction */
        if (u8Direction == (uint8_t)ROLLING_COUNT_RX) {
            u16RC = pstSharedMemData->stThreadsCommonData.stRollingCounterRegister[u8MsgInstance].u16RollingCountRX;
        } else if (u8Direction == (uint8_t)ROLLING_COUNT_TX) {
            u16RC = pstSharedMemData->stThreadsCommonData.stRollingCounterRegister[u8MsgInstance].u16RollingCountTX;
        } else {
            /* Intentionally empty else block */
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetRCData failed to unlock mutex: error %d", mutex_unlock_status);
            u16RC = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetRCData failed to lock mutex: error %d", mutex_lock_status);
        u16RC = ITCOM_ZERO_INIT_U;
    }
    /* Return the rolling counter, error handling is managed by operation_status */
    return u16RC;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetParkStatus
//*****************************************************************************
/**
*
*
*/
void ITCOM_vSetParkStatus(uint8_t u8ParkStatus, uint8_t u8Status) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Update the park status and vehicle info status */
        pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8ParkStatus = u8ParkStatus;
        pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[0] = u8Status; /* Index for Park Info Status */

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetParkStatus failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetParkStatus failed to lock mutex: error %d", mutex_lock_status);
    }
}


//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetParkStatus
//*****************************************************************************
/**
*
* @brief Retrieves the current park status and info status.
*
* @param [out] pu8ParkStaus Pointer to store park status
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Info status for the park
*/
uint8_t ITCOM_u8GetParkStatus(uint8_t* pu8ParkStatus) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8InfoStatus = INFO_OUTDATED;

    /* Check for NULL pointer */
    if (pu8ParkStatus != (uint8_t*)NULL) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Retrieve the park status and info status */
            *pu8ParkStatus = pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8ParkStatus;
            u8InfoStatus = pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[0]; /* Index for Park Info Status */

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetParkStatus failed to unlock mutex: error %d", mutex_unlock_status);
                u8InfoStatus = INFO_OUTDATED;
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetParkStatus failed to lock mutex: error %d", mutex_lock_status);
            u8InfoStatus = INFO_OUTDATED;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetParkStatus: NULL pointer provided");
    }
    return u8InfoStatus;
}


//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetVehicleSpeed
//*****************************************************************************
/**
*
*
*/
void ITCOM_vSetVehicleSpeed(float32_t f32VehicleSpeed, uint8_t u8Status) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the vehicle speed and update info status */
        pstSharedMemData->stThreadsCommonData.stVehicleStatus.fVehicleSpeed = f32VehicleSpeed;
        pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[1] = u8Status; // Index for Vehicle Info Status

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetVehicleSpeed failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetVehicleSpeed failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetVehicleSpeed
//*****************************************************************************
/**
*
* @brief Retrieves the current vehicle speed and info status.
*
* @param [out] pf32VehicleSpeed Pointer to store vehicle speed
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return uint8_t Info status for the vehicle speed
*/
uint8_t ITCOM_u8GetVehicleSpeed(float32_t* pf32VehicleSpeed) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8InfoStatus = INFO_OUTDATED;

    if(VALID_PTR(pf32VehicleSpeed)) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Retrieve the vehicle speed and info status */
            *pf32VehicleSpeed = pstSharedMemData->stThreadsCommonData.stVehicleStatus.fVehicleSpeed;
            u8InfoStatus = pstSharedMemData->stThreadsCommonData.stVehicleStatus.u8InfoStatus[1]; // Index for Vehicle Info Status

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetVehicleSpeed failed to unlock mutex: error %d", mutex_unlock_status);
                u8InfoStatus = INFO_OUTDATED;
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetVehicleSpeed failed to lock mutex: error %d", mutex_lock_status);
            u8InfoStatus = INFO_OUTDATED;
        }
    }
    else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetVehicleSpeed: Invalid input parameter pointer pf32VehicleSpeed");
        u8InfoStatus = INFO_OUTDATED;
    }
    /* Return the info status; error handling is managed by operation_status */
    return u8InfoStatus;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vWriteSUTRes
//*****************************************************************************
/**
*
* @brief Writes start-up test results to the SUT results structure.
*
* @param [in] stTestResults Structure containing start-up test results
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vWriteSUTRes(SutTestResults_t stTestResults) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Copy the test results to the shared memory structure */
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < enTotalSUT; i++) {
            pstSharedMemData->stThreadsCommonData.stSUTResults.enRunResult[i] = stTestResults.enRunResult[i];
        }
        pstSharedMemData->stThreadsCommonData.stSUTResults.u8SkippedTests = stTestResults.u8SkippedTests;
        pstSharedMemData->stThreadsCommonData.stSUTResults.enFinalResult = stTestResults.enFinalResult;
        pstSharedMemData->stThreadsCommonData.stSUTResults.u8Completion = stTestResults.u8Completion;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vWriteSUTRes failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vWriteSUTRes failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vRecordSutCompTime
//*****************************************************************************
/**
*
* @brief Records the completion time for start-up tests.
*
* @param [in] u32TimeRegister Completion time to record
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vRecordSutCompTime(DateRecord_t u32TimeRegister) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Record the completion time */
        pstSharedMemData->stThreadsCommonData.stSutTimeRegister = u32TimeRegister;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vRecordSutCompTime failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vRecordSutCompTime failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetActionListTestResult
//*****************************************************************************
/**
*
* @brief Sets the test results for the action list tests.
*
* @param [in] stTestResults Structure containing action list test results
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetActionListTestResult(AraTestResults_t stTestResults) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the test results */
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < enTotalActionListTests; i++) {
            pstSharedMemData->stThreadsCommonData.stActionListTestResults.enSubTestResult[i] = stTestResults.enSubTestResult[i];
        }
        pstSharedMemData->stThreadsCommonData.stActionListTestResults.enGroupResult = stTestResults.enGroupResult;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionListTestResult failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionListTestResult failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetPrecondListTestResult
//*****************************************************************************
/**
*
* @brief Sets the test results for the precondition list tests.
*
* @param [in] stTestResults Structure containing precondition list test results
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetPrecondListTestResult(AraTestResults_t stTestResults) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the precondition test results */
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < enTotalPrecondListTests; i++) {
            pstSharedMemData->stThreadsCommonData.stPrecondTestResults.enSubTestResult[i] = stTestResults.enSubTestResult[i];
        }
        pstSharedMemData->stThreadsCommonData.stPrecondTestResults.enGroupResult = stTestResults.enGroupResult;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetPrecondListTestResult failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetPrecondListTestResult failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetMemoryTestResult
//*****************************************************************************
/**
*
* @brief Sets the test results for memory tests.
*
* @param [in] stTestResults Structure containing memory test results
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetMemoryTestResult(MemTestResult_t stTestResults) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the memory test results */
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < enTotalMemoryTests; i++) {
            pstSharedMemData->stThreadsCommonData.stMemoryTestResults.enSubTestResult[i] = stTestResults.enSubTestResult[i];
        }
        pstSharedMemData->stThreadsCommonData.stMemoryTestResults.enGroupResult = stTestResults.enGroupResult;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMemoryTestResult failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMemoryTestResult failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetCalibReadbackData
//*****************************************************************************
/**
*
* @brief Sets the calibration readback data.
*
* @param [in] pu8Data Pointer to calibration readback data
* @param [in] u8DataSize Size of calibration readback data
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetCalibReadbackData(stProcessMsgData* pstTempMsgDataTracker, uint8_t u8Action) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Handle different actions based on u8Action */
        if (u8Action == (uint8_t)ADD_ELEMENT) {
            InstanceManager_vAddElement(&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, pstTempMsgDataTracker);
        } else {
            int16_t s16Indx = InstanceManager_s8FindElement(
                (generic_ptr_t)&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, 
                (generic_ptr_t)pstTempMsgDataTracker, 
                (ElementCompareFn)&itcom_u8CompareCalibData, 
                NULL);
            if (s16Indx >= (int16_t)ITCOM_ZERO_INIT_U) {
                if (u8Action == (uint8_t)UPDATE_ELEMENT) {
                    InstanceManager_vUpdateElement(&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, s16Indx, pstTempMsgDataTracker);
                } else if (u8Action == (uint8_t)REMOVE_ELEMENT) {
                    InstanceManager_vRemoveElement(&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, s16Indx);
                } else {
                    /* Intentionally empty else block */
                }
            }
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibReadbackData failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibReadbackData failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16GetCalibReadbackData
//*****************************************************************************
/**
*
* @brief Retrieves the calibration readback data.
*
* @param [out] pu8Data Pointer to store calibration readback data
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization}
*
* @return none
*/
int16_t ITCOM_s16GetCalibReadbackData(stProcessMsgData stTempMsgDataTracker, uint8_t* pu8Data) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    stProcessMsgData stFoundInstance = {0};
    int16_t s16Indx = (int16_t)ELEMENT_NOT_FOUND_IN_CIR_BUFFER;
    generic_ptr_t memory_operation_result = NULL;

    /* Validate input parameter */
    if (pu8Data != NULL) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        
        if (mutex_lock_status == E_OK) {
            /* Find the element in the calibration readback track */
            s16Indx = InstanceManager_s8FindElement(
                (generic_ptr_t)&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack, 
                (generic_ptr_t)&stTempMsgDataTracker, 
                (ElementCompareFn)&itcom_u8CompareCalibData, 
                (generic_ptr_t)&stFoundInstance);

            if (s16Indx >= (int16_t)ELEMENT_NOT_FOUND_IN_CIR_BUFFER) {
                /* Perform memory copy operation */
                memory_operation_result = (generic_ptr_t)memcpy((void*)pu8Data, (const void*)stFoundInstance.au8MsgData, sizeof(stFoundInstance.au8MsgData));
                
                if (memory_operation_result == NULL) {
                    log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetCalibReadbackData: Memory copy operation failed");
                    s16Indx = (int16_t)ELEMENT_NOT_FOUND_IN_CIR_BUFFER;
                }
            }

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetCalibReadbackData: Failed to unlock mutex: error %d", (int)mutex_unlock_status);
                s16Indx = (int16_t)ELEMENT_NOT_FOUND_IN_CIR_BUFFER;
            }
        }
        else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetCalibReadbackData: Failed to lock mutex: error %d", (int)mutex_lock_status);
            s16Indx = (int16_t)ELEMENT_NOT_FOUND_IN_CIR_BUFFER;
        }
    }
    else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetCalibReadbackData: NULL pointer received");
    }
    /* Return index or -1 in case of error */
    return s16Indx;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetCalibDataCopy
//*****************************************************************************
/**
*
* @brief Sets the calibration data copy.
*
* @param [in] pu8Data Pointer to calibration data
* @param [in] u8DataSize Size of calibration data
*
* @global {r/w; shared_mutex; shared mutex for thread synchronization},
*         {r/w; shared_cond; shared condition variable for thread signaling}
*
* @return none
*/
void ITCOM_vSetCalibDataCopy(stProcessMsgData* pstTempMsgDataTracker, uint8_t u8Action) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Perform action based on u8Action */
        if (u8Action == (uint8_t)ADD_ELEMENT) {
            InstanceManager_vAddElement(&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack, pstTempMsgDataTracker);
        } else {
            int16_t s16Indx = InstanceManager_s8FindElement(
                (generic_ptr_t)&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack, 
                (generic_ptr_t)pstTempMsgDataTracker, 
                (ElementCompareFn)&itcom_u8CompareCalibData, 
                NULL);
            if (s16Indx >= (int16_t)ITCOM_ZERO_INIT_U) {
                if (u8Action == (uint8_t)UPDATE_ELEMENT) {
                    InstanceManager_vUpdateElement(&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack, s16Indx, pstTempMsgDataTracker);
                } else if (u8Action == (uint8_t)REMOVE_ELEMENT) {
                    InstanceManager_vRemoveElement(&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack, s16Indx);
                    log_message(global_log_file, LOG_DEBUG, "CALIBRATION ELEMENT REMOVED, TYPE: 0x%04X, MSG ID: 0x%04X, Sequence: 0x%04X", 
                        pstTempMsgDataTracker->u16Type, 
                        pstTempMsgDataTracker->stMsgPairData.u16MsgId, 
                        pstTempMsgDataTracker->stMsgPairData.u16SequenceNum);
                } else {
                    /* Intentionally empty else block */
                }
            }
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibDataCopy failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibDataCopy failed to lock mutex: error %d", mutex_lock_status);
    }
}


void ITCOM_vSetCalibComparisonResult(uint8_t u8Result) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the calibration comparison result */
        pstSharedMemData->stThreadsCommonData.u8CalibComparisonResult = u8Result;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibComparisonResult failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCalibComparisonResult failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16GetIndxTableWithMsgType
//*****************************************************************************
/**
* @brief Retrieves the enumeration associated with a message type.
*
* @param [in] u16MsgType The message type (e.g., 0xFF33 for ACK Message).
*
* @global {r; shared_mutex; shared mutex for dictionary access},
*         {r; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Returns the message type enum if found, otherwise returns MESSAGE_TYPE_NOT_FOUND.
*/
int16_t ITCOM_s16GetMessageTypeEnum(uint16_t u16MsgType) {
    uint8_t u8DictionarySize = sizeof(stMsgTypeDictionary) / sizeof(stMsgTypeDictionary[0]);
    int16_t s16Indx = MESSAGE_TYPE_NOT_FOUND;
    uint8_t i;
    for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
        if (stMsgTypeDictionary[i].u16MessageTypeID == u16MsgType) {
            s16Indx = (int16_t)stMsgTypeDictionary[i].u8MessageTypeEnum;
            break;
        }
    }
    
    if (s16Indx == (uint8_t)MESSAGE_TYPE_NOT_FOUND) {
        log_message(global_log_file, LOG_DEBUG, "ITCOM_s16GetMessageTypeEnum: No match found for u16MsgType: 0x%04X", u16MsgType);
    }

    return s16Indx;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16GetMessageEnumById
//*****************************************************************************
/**
* @brief Retrieves the enumeration associated with a message id.
*
* @param [in] u16MsgId The message type ...
*
* @global {r; shared_mutex; shared mutex for dictionary access},
*         {r; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Returns the message type enum if found, otherwise returns MESSAGE_TYPE_NOT_FOUND.
*/
int16_t ITCOM_s16GetMessageEnumById(uint16_t u16MsgId) {
    uint8_t u8DictionarySize = sizeof(stMsgDictionary) / sizeof(stMsgDictionary[0]);
    int16_t s16Indx = MESSAGE_NOT_FOUND;
    uint8_t i;
    for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
        if (stMsgDictionary[i].u16MessageId == u16MsgId) {
            s16Indx = (int16_t)stMsgDictionary[i].u8MessageEnum;
            break;
        }
    }
    
    if (s16Indx == (int16_t)MESSAGE_NOT_FOUND) {
        log_message(global_log_file, LOG_DEBUG, "ITCOM_s16GetMessageEnumById: No match found for u16MsgId: 0x%04X", u16MsgId);
    }
    return s16Indx;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16GetIndxTableWithMsgID
//*****************************************************************************
/**
* @brief Retrieves the index of the message in the dictionary based on message type, ID, and connection source.
*
* @param [in] u16MsgType The message type (e.g., 0xFF11 for Action Request, 0xFF33 for ACK).
* @param [in] u16MsgId The message ID (e.g., 0x0007 for enDoorLockState).
* @param [in] enTCPConn The connection source (enVAMConnectionTCP or enCMConnectionTCP).
*
* @global {r; shared_mutex; shared mutex for dictionary access},
*         {r; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Returns the enum value if found, otherwise returns MESSAGE_NOT_FOUND.
*/
int16_t ITCOM_s16GetMessageEnumFromTypeAndId(uint16_t u16MsgType, uint16_t u16MsgId, enTCPConnectionsASI enTCPConn) {
    uint8_t u8DictionarySize = (uint8_t)(sizeof(stMsgDictionary) / sizeof(stMsgDictionary[0]));
    int16_t s16MsgIdEnum = ITCOM_s16GetMessageEnumById(u16MsgId);
    int16_t s16MsgTypeEnum = ITCOM_s16GetMessageTypeEnum(u16MsgType);
    int16_t s16Result = MESSAGE_NOT_FOUND;

    /* Handle special case for acknowledgment messages */
    if (s16MsgTypeEnum == enAckMessage) {
        s16Result = (enTCPConn == (uint8_t)enVAMConnectionTCP) ? (int16_t)enAckVAM : (int16_t)enAckCM;
    }
    /* Handle notification messages */
    else if (s16MsgTypeEnum == enNotificationMessage) {
    	if((s16MsgIdEnum != (int16_t)enActionNotification) && (s16MsgIdEnum >= (int16_t)enNonCriticalFail && s16MsgIdEnum <= (int16_t)enStatusNotificationASI)) {
            s16Result = s16MsgIdEnum;
        }
        else {
            uint8_t i;
            for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
                if (stMsgDictionary[i].u16MessageId == u16MsgId && stMsgDictionary[i].u16MessageType == enActionRequest) {
                    s16Result = (int16_t)enActionNotification;
                    break;
                }
            }
        }
    }
    /* Handle calibration readback messages */
    else if (s16MsgTypeEnum == enCalibReadbackMessage) {
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
            if (stMsgDictionary[i].u16MessageId == u16MsgId && stMsgDictionary[i].u16MessageType == enActionRequest) {
                s16Result = (int16_t)enCalibReadback;
                break;
            }
        }
    }
    /* General search for message ID and type */
    else {
        uint8_t i;
        for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
            if (stMsgDictionary[i].u16MessageId == u16MsgId && stMsgDictionary[i].u16MessageType == s16MsgTypeEnum) {
                s16Result = (int16_t)stMsgDictionary[i].u8MessageEnum;
                break;
            }
        }
    }

    /* Return the result */
    if (s16Result == (int16_t)MESSAGE_NOT_FOUND) {
        log_message(global_log_file, LOG_DEBUG, "ITCOM_s16GetMessageEnumFromTypeAndId: Message not found for MsgType: 0x%04X, MsgId: 0x%04X", u16MsgType, u16MsgId);
    }

    return s16Result;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetDictionaryTypeDataAtIndx
//*****************************************************************************
/**
 * @brief Retrieves a message type dictionary entry at a specified index.
 *
 * This function fetches a MessageTypeDictionary_t entry from the global stMsgTypeDictionary array
 * at the index specified by u16MsgId. It provides a thread-safe way to access the message type
 * dictionary data.
 *
 * @param[out] pstDictionaryTypeData Pointer to a MessageTypeDictionary_t structure where the
 *                                   retrieved dictionary entry will be stored. This parameter
 *                                   must not be NULL.
 * @param[in] u16MsgId The index of the desired dictionary entry. This value must be less than
 *                     enTotalASIMessageClassification.
 *
 */
void ITCOM_vGetMsgTypeDictionaryEntryAtIndex(MessageTypeDictionary_t* pstDictionaryTypeData, uint16_t u16MsgId) {

    /* Check if pstDictionaryTypeData is NULL */
    if (pstDictionaryTypeData == NULL) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetMsgTypeDictionaryEntryAtIndex: NULL pointer provided for pstDictionaryTypeData");
    }
    /* Check if u16MsgId is valid */
    else if (u16MsgId >= enTotalASIMessageClassification) {
        log_message(global_log_file, LOG_WARNING, "ITCOM_vGetMsgTypeDictionaryEntryAtIndex: Invalid u16MsgId: %u. Must be less than %u", u16MsgId, enTotalASIMessageClassification);
    }
    else {
        /* Successfully retrieve the dictionary entry */
        *pstDictionaryTypeData = stMsgTypeDictionary[u16MsgId];
        log_message(global_log_file, LOG_DEBUG, "ITCOM_vGetMsgTypeDictionaryEntryAtIndex: Successfully retrieved entry for u16MsgId: %u", u16MsgId);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetMsgDictionaryEntryAtIndex
//*****************************************************************************
/**
 *
*/
void ITCOM_vGetMsgDictionaryEntryAtIndex(MessageDictionary_t* pstDictionaryData, uint16_t u16MsgId) {
    generic_ptr_t memory_operation_result = NULL;

    /* Check if pstDictionaryData is NULL */
    if (pstDictionaryData == NULL) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetMsgDictionaryEntryAtIndex: NULL pointer provided for pstDictionaryData");
    }
    /* Check if u16MsgId is valid */
    else if (u16MsgId >= enTotalMessagesASI) {
        log_message(global_log_file, LOG_WARNING, "ITCOM_vGetMsgDictionaryEntryAtIndex: Invalid u16MsgId: %u. Must be less than %u", u16MsgId, enTotalMessagesASI);

        /* Initialize the structure with zeros to indicate an invalid entry */
        memory_operation_result = memset(pstDictionaryData, 0, sizeof(MessageDictionary_t));
        if ((memory_operation_result == NULL) || (memory_operation_result != pstDictionaryData)) {
            log_message(global_log_file, LOG_ERROR, "Memory initialization failed");
        }
    }
    /* Valid case: retrieve the dictionary entry */
    else {
        *pstDictionaryData = stMsgDictionary[u16MsgId];
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_s16CheckAssociatedLength
//*****************************************************************************
/**
* @brief Checks if the given length is valid for the specified message type.
*
* @param [in] u16MsgType The message type (e.g., 0xFF11 for Action Request).
* @param [in] u8Length The length to check (e.g., 16, 32).
*
* @global {r; shared_mutex; shared mutex for dictionary access},
*         {r; shared_cond; shared condition variable for thread signaling}
*
* @return int8_t Returns E_OK if the length is found, otherwise returns ASSOCIATED_LENGTH_NOT_FOUND.
*/
int8_t ITCOM_s8ValidateMessageTypeLength(uint16_t u16MsgType, uint8_t u8Length) {
    const uint8_t u8DictionarySize = (uint8_t)(sizeof(stMsgTypeDictionary) / sizeof(stMsgTypeDictionary[0]));
    int8_t s8Result = MESSAGE_TYPE_NOT_FOUND;

    /* Iterate over the message type dictionary */
    uint8_t i;
    for (i = ITCOM_ZERO_INIT_U; i < u8DictionarySize; i++) {
        if (stMsgTypeDictionary[i].u16MessageTypeID == (uint16_t)u16MsgType) {
            /* Check against the associated lengths */
            uint8_t j;
            for (j = ITCOM_ZERO_INIT_U; j < (uint8_t)NUM_ASSOCIATED_LENGHTS; j++) {
                if (stMsgTypeDictionary[i].au8AssociatedLengths[j] == u8Length) {
                    s8Result = E_OK;
                    break;
                }
            }

            /* If the length was not found, record an error */
            if (s8Result != (int8_t)E_OK) {
                enSetErrorEventStatus error_status = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_MSG_TYPE_LENGTH);
                if (error_status != (enSetErrorEventStatus)enSuccess_EventAddedToQueue) {
                    log_message(global_log_file, LOG_ERROR, "Failed to set error event: status %d", error_status);
                }
                log_message(global_log_file, LOG_WARNING, "ITCOM_s8ValidateMessageTypeLength: Length %u not found for message type 0x%04X", u8Length, u16MsgType);
                s8Result = ASSOCIATED_LENGTH_NOT_FOUND;
            }
            break;
        }
    }

    /* Log a message type not found warning if applicable */
    if (s8Result == (int8_t)MESSAGE_TYPE_NOT_FOUND) {
        log_message(global_log_file, LOG_WARNING, "ITCOM_s8ValidateMessageTypeLength: Message type 0x%04X not found in dictionary", u16MsgType);
    }

    return s8Result;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u16GetTrackBufferSize
//*****************************************************************************
/**
 * @brief Retrieves the size of a specified tracking buffer.
 *
 * This function returns the current count of elements in one of three tracking buffers:
 * Action Message Buffer, Calibration Data Copy Buffer, or Calibration Readback Data Buffer.
 * The specific buffer is selected using the input parameter.
 *
 * @param[in] u8SelectBuffer An 8-bit unsigned integer that specifies which buffer's size to retrieve.
 *                           Valid values are:
 *                           - enActionMsgBuffer: For the Action Message Buffer
 *                           - enCalibDataCopyBuffer: For the Calibration Data Copy Buffer
 *                           - enCalibReadbackData: For the Calibration Readback Data Buffer
 *
 * @return uint16_t The number of elements currently in the selected buffer.
 *                  Returns 0 if pstSharedMemData is NULL or if an invalid buffer type is specified.
 *
 */
uint16_t ITCOM_u16GetTrackBufferSize(uint8_t u8SelectBuffer) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint16_t u16BufferTrackSize = ITCOM_ZERO_INIT_U;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Select the appropriate buffer */
        switch (u8SelectBuffer) {
            case enActionMsgBuffer:
                u16BufferTrackSize = pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.u16Count;
                break;

            case enCalibDataCopyBuffer:
                u16BufferTrackSize = pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.u16Count;
                break;

            case enCalibReadbackData:
                u16BufferTrackSize = pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.u16Count;
                break;

            default:
                log_message(global_log_file, LOG_WARNING, "ITCOM_u16GetTrackBufferSize: Invalid buffer type: %u", u8SelectBuffer);
                break;
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetTrackBufferSize: Failed to unlock mutex: error %d", mutex_unlock_status);
            u16BufferTrackSize = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u16GetTrackBufferSize: Failed to lock mutex: error %d", mutex_lock_status);
        u16BufferTrackSize = ITCOM_ZERO_INIT_U;
    }
    return u16BufferTrackSize;
}


//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetElementAtIndex
//*****************************************************************************
/**
 * @brief Retrieves an element from the cycle sequence tracking buffer at a specified index.
 *
 * This function copies an element from the cycle sequence tracking buffer (stCycleSeqTrack) 
 * to a provided memory location. It handles the circular buffer logic and ensures thread-safe 
 * access to the shared data structure.
 *
 * @param[in] u16Indx The logical index of the element to retrieve from the buffer.
 *                    This index is relative to the current head of the circular buffer.
 * @param[out] pvElement Pointer to the memory location where the retrieved element will be copied.
 *                       The caller must ensure that this pointer is valid and points to a memory 
 *                       location large enough to hold the element.
 *
 * @par Implementation Details
 * The function performs the following steps:
 * 1. Checks for NULL pointers in the input parameters.
 * 2. Locks the mutex to ensure exclusive access to the shared data.
 * 3. Checks if the provided index is within the valid range.
 * 4. If valid, calculates the actual index in the circular buffer.
 * 5. Copies the element from the buffer to the provided memory location.
 * 6. Unlocks the mutex.
 *
 */

void ITCOM_vGetCycleSeqElementAtIndex(uint16_t u16Indx, generic_ptr_t pvElement, uint8_t u8SelectBuffer) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint16_t u16ActualIndex = ITCOM_ZERO_INIT_U;
    generic_ptr_t memory_operation_result = NULL;
    size_t buffer_element_size = ITCOM_ZERO_INIT_U;
    uint16_t buffer_count = ITCOM_ZERO_INIT_U;
    uint8_t* source_buffer = NULL;

    /* Check for NULL pointer */
    if (pvElement == NULL) {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: NULL pointer provided for pvElement");
    } else {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Select the appropriate buffer based on u8SelectBuffer */
            switch (u8SelectBuffer) {
                case enActionMsgBuffer:
                    buffer_element_size = pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.sz_ElementSize;
                    buffer_count = pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.u16Count;
                    if (u16Indx < buffer_count) {
                        u16ActualIndex = (pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.u16Head + u16Indx) % 
                                         pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.u16Capacity;
                        source_buffer = (uint8_t*)&pstSharedMemData->stThreadsCommonData.stCycleSeqTrack.au8_Buffer[(u16ActualIndex) * (buffer_element_size)];
                        memory_operation_result = memcpy(pvElement, source_buffer, buffer_element_size);
                        if (memory_operation_result != pvElement) {
                            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Memory copy operation failed for ActionMsgBuffer");
                        }
                    }
                    break;

                case enCalibDataCopyBuffer:
                    buffer_element_size = pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.sz_ElementSize;
                    buffer_count = pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.u16Count;
                    if (u16Indx < buffer_count) {
                        u16ActualIndex = (pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.u16Head + u16Indx) % 
                                         pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.u16Capacity;
                        source_buffer = (uint8_t*)&pstSharedMemData->stThreadsCommonData.stCalibrationDataCopyTrack.au8_Buffer[(u16ActualIndex) * (buffer_element_size)];
                        memory_operation_result = memcpy(pvElement, source_buffer, buffer_element_size);
                        if (memory_operation_result != pvElement) {
                            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Memory copy operation failed for CalibDataCopyBuffer");
                        }
                    }
                    break;

                case enCalibReadbackData:
                    buffer_element_size = pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.sz_ElementSize;
                    buffer_count = pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.u16Count;
                    if (u16Indx < buffer_count) {
                        u16ActualIndex = (pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.u16Head + u16Indx) % 
                                         pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.u16Capacity;
                        source_buffer = (uint8_t*)&pstSharedMemData->stThreadsCommonData.stCalibrationReadbackTrack.au8_Buffer[(u16ActualIndex) * (buffer_element_size)];
                        memory_operation_result = memcpy(pvElement, source_buffer, buffer_element_size);
                        if (memory_operation_result != pvElement) {
                            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Memory copy operation failed for CalibReadbackData");
                        }
                    }
                    break;

                default:
                    log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Invalid buffer type: %u", u8SelectBuffer);
                    break;
            }

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Failed to unlock mutex: error %d", mutex_unlock_status);
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetCycleSeqElementAtIndex: Failed to lock mutex: error %d", mutex_lock_status);
        }
    }
}



//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetCrcErrorCount
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetCrcErrorCount(uint8_t u8Indx, uint8_t u8Value) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_RX.mutex);
    if (mutex_lock_status == E_OK) {
        /* Update the CRC error counter if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            pstSharedMemData->stThread_ICM_RX.u8CrcErrorCounter[u8Indx] = u8Value;
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_RX.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCrcErrorCount: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetCrcErrorCount: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetCrcErrorCount
//*****************************************************************************
/**
 *
 *
*/
uint8_t ITCOM_u8GetCrcErrorCount(uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8CrcErrorCount = ITCOM_ZERO_INIT_U;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_RX.mutex);
    if (mutex_lock_status == E_OK) {
        /* Retrieve the CRC error count if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            u8CrcErrorCount = pstSharedMemData->stThread_ICM_RX.u8CrcErrorCounter[u8Indx];
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_RX.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetCrcErrorCount: Failed to unlock mutex: error %d", mutex_unlock_status);
            u8CrcErrorCount = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetCrcErrorCount: Failed to lock mutex: error %d", mutex_lock_status);
        u8CrcErrorCount = ITCOM_ZERO_INIT_U;
    }
    return u8CrcErrorCount;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetRollingCountError
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetRollingCountError(uint8_t u8Indx, uint8_t u8Value) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_RX.mutex);
    if (mutex_lock_status == E_OK) {
        /* Update the rolling count error if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            pstSharedMemData->stThread_ICM_RX.u8RollingCounterError[u8Indx] = u8Value;
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_RX.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetRollingCountError: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetRollingCountError: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_u8GetRollingCountError
//*****************************************************************************
/**
 *
 *
*/
uint8_t ITCOM_u8GetRollingCountError(uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    uint8_t u8RollingCounterError = ITCOM_ZERO_INIT_U;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_RX.mutex);
    if (mutex_lock_status == E_OK) {
        /* Retrieve the rolling count error if the index is valid */
        if (u8Indx < enTotalMessagesASI) {
            u8RollingCounterError = pstSharedMemData->stThread_ICM_RX.u8RollingCounterError[u8Indx];
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_RX.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetRollingCountError: Failed to unlock mutex: error %d", mutex_unlock_status);
            u8RollingCounterError = ITCOM_ZERO_INIT_U;
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_u8GetRollingCountError: Failed to lock mutex: error %d", mutex_lock_status);
        u8RollingCounterError = ITCOM_ZERO_INIT_U;
    }
    return u8RollingCounterError;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetMsgRateLimiter
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetMsgRateLimiter(RateLimiter_t* pstRateLimiter) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    if(VALID_PTR(pstRateLimiter)) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_TX.mutex);
        if (mutex_lock_status == E_OK) {
            /* Update the rate limiter parameters */
            pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16AllowedMessages = pstRateLimiter->u16AllowedMessages;
            pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16TimeWindowMs = pstRateLimiter->u16TimeWindowMs;
            pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16MessageCount = pstRateLimiter->u16MessageCount;
            pstSharedMemData->stThread_ICM_TX.stRateLimiter.stStartTime = pstRateLimiter->stStartTime;

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_TX.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMsgRateLimiter: Failed to unlock mutex: error %d", mutex_unlock_status);
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMsgRateLimiter: Failed to lock mutex: error %d", mutex_lock_status);
        }
    }
    else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetMsgRateLimiter: Invalid input parameter pointer pstRateLimiter");
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetMsgRateLimiter
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vGetMsgRateLimiter(RateLimiter_t* pstRateLimiter) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    if(VALID_PTR(pstRateLimiter)) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_ICM_TX.mutex);
        if (mutex_lock_status == E_OK) {
            /* Retrieve the rate limiter parameters */
            pstRateLimiter->u16AllowedMessages = pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16AllowedMessages;
            pstRateLimiter->u16TimeWindowMs = pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16TimeWindowMs;
            pstRateLimiter->u16MessageCount = pstSharedMemData->stThread_ICM_TX.stRateLimiter.u16MessageCount;
            pstRateLimiter->stStartTime = pstSharedMemData->stThread_ICM_TX.stRateLimiter.stStartTime;

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_ICM_TX.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vGetMsgRateLimiter: Failed to unlock mutex: error %d", mutex_unlock_status);
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetMsgRateLimiter: Failed to lock mutex: error %d", mutex_lock_status);
        }
    }
    else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetMsgRateLimiter: Invalid input parameter pointer pstRateLimiter");
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetErrorProcessingFlag
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetErrorProcessingFlag(int16_t s16Value) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThread_FM.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the error processing flag */
        pstSharedMemData->stThread_FM.processing = s16Value;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThread_FM.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetErrorProcessingFlag: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetErrorProcessingFlag: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetErrorProcessingFlag
//*****************************************************************************
/**
 *
 *
*/
int16_t ITCOM_s16GetProcessingFlag(void)
{
    mutex_status_t mutex_lock_status = 0;
    mutex_status_t mutex_unlock_status = 0;
    int16_t s16ErrorProcessing = (int16_t)ITCOM_NEG_ONE_INIT;
    bool is_operation_valid = TRUE;
    
    /* Validate shared memory */
    if (NULL == pstSharedMemData)
    {
        log_message(global_log_file, LOG_ERROR, 
                   "ITCOM_s16GetProcessingFlag: Invalid shared memory pointer");
        is_operation_valid = FALSE;
    }
    
    if (TRUE == is_operation_valid)
    {
        /* Try to acquire mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&(pstSharedMemData->stThread_FM.mutex));
        if (E_OK == mutex_lock_status)
        {
            /* Critical section - keep it minimal */
            s16ErrorProcessing = pstSharedMemData->stThread_FM.processing;
            
            /* Release mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&(pstSharedMemData->stThread_FM.mutex));
            if (E_OK != mutex_unlock_status)
            {
                log_message(global_log_file, LOG_ERROR, 
                           "ITCOM_s16GetProcessingFlag: Failed to unlock mutex: error %d", 
                           (int)mutex_unlock_status);
                is_operation_valid = FALSE;
            }
        }
        else
        {
            log_message(global_log_file, LOG_ERROR, 
                       "ITCOM_s16GetProcessingFlag: Failed to lock mutex: error %d", 
                       (int)mutex_lock_status);
            is_operation_valid = FALSE;
        }
    }
    
    /* Reset to error value if any operation failed */
    if (FALSE == is_operation_valid)
    {
        s16ErrorProcessing = (int16_t)ITCOM_NEG_ONE_INIT;
    }
    
    return s16ErrorProcessing;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetEventQueueIndx
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetEventQueueIndx(uint8_t u8IndxValue) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the event queue index */
        pstSharedMemData->stThreadsCommonData.Event_Queue_Index = u8IndxValue;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetEventQueueIndx: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetEventQueueIndx: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetErrorProcessingFlag
//*****************************************************************************
/**
 *
 *
*/
int16_t ITCOM_s16GetEventQueueIndx(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int16_t s16EventQueueIndx = QUEUE_INDEX_INVALID;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Retrieve the event queue index */
        s16EventQueueIndx = (int16_t)pstSharedMemData->stThreadsCommonData.Event_Queue_Index;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetEventQueueIndx: Failed to unlock mutex: error %d", mutex_unlock_status);
            s16EventQueueIndx = QUEUE_INDEX_INVALID;  /* Set to error value if unlocking fails */
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_s16GetEventQueueIndx: Failed to lock mutex: error %d", mutex_lock_status);
        s16EventQueueIndx = QUEUE_INDEX_INVALID;
    }
    return s16EventQueueIndx;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetEventQueueId
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetEventQueueId(uint8_t u8EventQueue, uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the event queue ID at the specified index */
        pstSharedMemData->stThreadsCommonData.Event_Queue[u8Indx] = u8EventQueue;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetEventQueueId: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetEventQueueId: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetEventQueueId
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vGetEventQueueId(uint8_t* pu8EventQueue, uint8_t u8Indx) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Retrieve the event queue ID at the specified index */
        *pu8EventQueue = pstSharedMemData->stThreadsCommonData.Event_Queue[u8Indx];

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetEventQueueId: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetEventQueueId: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vRemoveProcessedEvent
//*****************************************************************************
/**
 * @brief Removes the first event from the event queue after processing.
 *
 * This function is responsible for removing the first event from the shared
 * event queue after it has been processed. It ensures thread-safe operation
 * by using mutex locks when accessing the shared data.
 *
 * The function performs the following operations:
 * 1. Locks the mutex for the shared event queue.
 * 2. Checks if there are any events in the queue.
 * 3. If events exist, it removes the first event by shifting all subsequent
 *    events one position forward in the queue.
 * 4. Decrements the event queue index.
 * 5. Unlocks the mutex.
 *
 * @note This function assumes the existence of a global shared data structure
 *       (shared_data) containing the event queue and associated mutex.
 *
 * @warning This function does not return any status about whether an event
 *          was actually removed. Callers should ensure that an event is
 *          available for removal before calling this function.
 *
 */
void ITCOM_vRemoveProcessedEvent(void) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    generic_ptr_t move_result = NULL;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Check if there is an event to remove */
        if (pstSharedMemData->stThreadsCommonData.Event_Queue_Index > 0) {
            move_result = memmove(pstSharedMemData->stThreadsCommonData.Event_Queue,
                                  pstSharedMemData->stThreadsCommonData.Event_Queue + 1,
                                  (pstSharedMemData->stThreadsCommonData.Event_Queue_Index - 1) * sizeof(unsigned char));

            /* Check for success of memmove operation */
            if ((move_result != NULL) && (move_result == pstSharedMemData->stThreadsCommonData.Event_Queue)) {
                pstSharedMemData->stThreadsCommonData.Event_Queue_Index--;
            } else {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vRemoveProcessedEvent: Memory move operation failed");
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vRemoveProcessedEvent: No events to remove");
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vRemoveProcessedEvent: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vRemoveProcessedEvent: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSemaphoreTryWait
//*****************************************************************************
/**
 *
 *
*/
int16_t ITCOM_vSemaphoreTryWait(void) {
    int16_t s16SemTryWait = -1;
    s16SemTryWait = sem_trywait(&pstSharedMemData->stThread_FM.sem);
    return s16SemTryWait;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vNotification_SM
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vNotification_SM(void)
{
    log_message(global_log_file, LOG_INFO, "SM_Notification called");
    ITCOM_vSetCriticalFault();
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vExtSysNotification
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vExtSysNotification(void)
{
    log_message(global_log_file, LOG_INFO, "External_System_Notification called");
    (void)ITCOM_s8LogNotificationMessage(0U, 0U, 0U, (uint8_t)enStatusNotificationASI);
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetTCPConnectionState
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetTCPConnectionState(enTCPConnectionsASI enConnection, TCPConnectionState_t enState) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Validate connection index */
    if (enConnection >= (enTCPConnectionsASI)enTotalTCPConnections) {
        log_message(global_log_file, LOG_ERROR, "Invalid connection index %d", enConnection);
    } else {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Set the TCP connection state */
            pstSharedMemData->stThreadsCommonData.enTCPConnectionState[enConnection] = enState;

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vSetTCPConnectionState: Failed to unlock mutex: error %d", mutex_unlock_status);
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetTCPConnectionState: Failed to lock mutex: error %d", mutex_lock_status);
        }
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_enGetTCPConnectionState
//*****************************************************************************
/**
 *
 *
*/
TCPConnectionState_t ITCOM_enGetTCPConnectionState(enTCPConnectionsASI enConnection) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    TCPConnectionState_t enState = CONNECTION_STATE_DISCONNECTED;

    /* Validate connection index */
    if (enConnection >= (enTCPConnectionsASI)enTotalTCPConnections) {
        log_message(global_log_file, LOG_ERROR, "Invalid connection index %d", enConnection);
    } else {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Retrieve the TCP connection state */
            enState = pstSharedMemData->stThreadsCommonData.enTCPConnectionState[enConnection];

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_enGetTCPConnectionState: Failed to unlock mutex: error %d", mutex_unlock_status);
                enState = CONNECTION_STATE_ERROR;  /* Set error state on unlock failure */
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_enGetTCPConnectionState: Failed to lock mutex: error %d", mutex_lock_status);
        }
    }
    return enState;
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vSetStateMonitorTestData
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vSetStateMonitorTestData(StateMonitor_t stStateMonitorData) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        /* Set the state monitor data */
        pstSharedMemData->stThreadsCommonData.stStateMonitorData.stCurrentState = stStateMonitorData.stCurrentState;
        pstSharedMemData->stThreadsCommonData.stStateMonitorData.u8StateError = stStateMonitorData.u8StateError;

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetStateMonitorTestData: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetStateMonitorTestData: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

//*****************************************************************************
// FUNCTION NAME : ITCOM_vGetStateMonitorTestData
//*****************************************************************************
/**
 *
 *
*/
void ITCOM_vGetStateMonitorTestData(StateMonitor_t* pstStateMonitorData) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;

    /* Check if the pointer is valid */
    if (VALID_PTR(pstStateMonitorData)) {
        /* Attempt to lock the mutex */
        mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_lock_status == E_OK) {
            /* Retrieve the state monitor data */
            pstStateMonitorData->stCurrentState = pstSharedMemData->stThreadsCommonData.stStateMonitorData.stCurrentState;
            pstStateMonitorData->u8StateError = pstSharedMemData->stThreadsCommonData.stStateMonitorData.u8StateError;

            /* Attempt to unlock the mutex */
            mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
            if (mutex_unlock_status != E_OK) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vGetStateMonitorTestData: Failed to unlock mutex: error %d", mutex_unlock_status);
            }
        } else {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vGetStateMonitorTestData: Failed to lock mutex: error %d", mutex_lock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vGetStateMonitorTestData: Invalid pointer for pstStateMonitorData");
    }
}


/*** Private Functions ***/

static uint8_t itcom_u8CompareMsgIdAndSequence(const_generic_ptr_t a, const_generic_ptr_t b) {
    const stMsgIntegrityData* const element = (const stMsgIntegrityData* const)a;
    const stMsgIntegrityData* const criteria = (const stMsgIntegrityData* const)b;
    uint8_t result = ITCOM_ONE_INIT_U;

    if ((bool)(element != (stMsgIntegrityData*)NULL) && (bool)(criteria != (stMsgIntegrityData*)NULL)) {
        if (((uint16_t)element->stMsgPairData.u16MsgId == (uint16_t)criteria->stMsgPairData.u16MsgId) && 
            ((uint16_t)element->stMsgPairData.u16SequenceNum == (uint16_t)criteria->stMsgPairData.u16SequenceNum) && 
            ((uint8_t)element->u8ClearCondition == (uint8_t)criteria->u8ClearCondition)) {
            result = ITCOM_ZERO_INIT_U;
        }
    }
    return result;
}

static uint8_t itcom_u8CompareCalibData(const_generic_ptr_t a, const_generic_ptr_t b) {
    const stProcessMsgData* const element = (const stProcessMsgData* const)a;
    const stProcessMsgData* const criteria = (const stProcessMsgData* const)b;
    uint8_t result = ITCOM_ONE_INIT_U;

    if ((element != (stProcessMsgData*)NULL) && (criteria != (stProcessMsgData*)NULL)) {
        if (((uint16_t)element->stMsgPairData.u16MsgId == (uint16_t)criteria->stMsgPairData.u16MsgId) && 
            ((uint16_t)element->stMsgPairData.u16SequenceNum == (uint16_t)criteria->stMsgPairData.u16SequenceNum)) {
            result = ITCOM_ZERO_INIT_U;
        }
    }
    return result;
}

static void itcom_vRemoveActionRequestTiming(uint16_t u16MsgId, uint16_t u16SequenceNum) {
    uint8_t actionRequestCount = pstSharedMemData->stThreadsCommonData.u8ActionRequestTimingCount;
    uint8_t i;
    for (i = ITCOM_ZERO_INIT_U; i < actionRequestCount; i++) {
        if (pstSharedMemData->stThreadsCommonData.astActionRequestTiming[i].u16MsgId == u16MsgId &&
            pstSharedMemData->stThreadsCommonData.astActionRequestTiming[i].u16SequenceNum == u16SequenceNum) {
            // Remove this entry by shifting the remaining entries
            uint8_t j;
            for (j = i; j < actionRequestCount - 1; j++) {
                pstSharedMemData->stThreadsCommonData.astActionRequestTiming[j] = pstSharedMemData->stThreadsCommonData.astActionRequestTiming[j+1];
            }
            pstSharedMemData->stThreadsCommonData.u8ActionRequestTimingCount--;
            break;
        }
    }
}

void ITCOM_vSetActionRequestStartTime(uint16_t u16MsgId, uint16_t u16SequenceNum) {
    mutex_status_t mutex_lock_status;
    mutex_status_t mutex_unlock_status;
    int32_t time_status;
    generic_ptr_t move_result = NULL;
    uint8_t operation_status = ITCOM_OP_FAILURE;

    /* Attempt to lock the mutex */
    mutex_lock_status = (mutex_status_t)pthread_mutex_lock(&pstSharedMemData->stThreadsCommonData.mutex);
    if (mutex_lock_status == E_OK) {
        uint8_t index = pstSharedMemData->stThreadsCommonData.u8ActionRequestTimingCount;

        /* Check if the timing count exceeds the limit */
        if (index >= (uint8_t)MAX_PENDING_ACTION_REQUESTS) {
            move_result = memmove(&pstSharedMemData->stThreadsCommonData.astActionRequestTiming[0],
                                  &pstSharedMemData->stThreadsCommonData.astActionRequestTiming[1],
                                  sizeof(ActionRequestTiming_t) * (MAX_PENDING_ACTION_REQUESTS - 1));

            /* Check if the memmove operation was successful */
            if ((move_result != NULL) && (move_result == &pstSharedMemData->stThreadsCommonData.astActionRequestTiming[0])) {
                index = MAX_PENDING_ACTION_REQUESTS - 1;
                operation_status = ITCOM_OP_SUCCESS;
            } else {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionRequestStartTime: Memory move operation failed");
            }
        } else {
            pstSharedMemData->stThreadsCommonData.u8ActionRequestTimingCount++;
            operation_status = ITCOM_OP_SUCCESS;
        }

        /* Proceed if memory operation or index update was successful */
        if (operation_status == (uint8_t)ITCOM_ONE_INIT_U) {
            pstSharedMemData->stThreadsCommonData.astActionRequestTiming[index].u16MsgId = u16MsgId;
            pstSharedMemData->stThreadsCommonData.astActionRequestTiming[index].u16SequenceNum = u16SequenceNum;

            /* Attempt to set the start time */
            time_status = clock_gettime(CLOCK_MONOTONIC, &pstSharedMemData->stThreadsCommonData.astActionRequestTiming[index].start_time);
            if (time_status != (int32_t)ITCOM_ZERO_INIT_U) {
                log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionRequestStartTime: Failed to get time: error %d", time_status);
                operation_status = ITCOM_OP_FAILURE;
            }
        }

        /* Attempt to unlock the mutex */
        mutex_unlock_status = (mutex_status_t)pthread_mutex_unlock(&pstSharedMemData->stThreadsCommonData.mutex);
        if (mutex_unlock_status != E_OK) {
            log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionRequestStartTime: Failed to unlock mutex: error %d", mutex_unlock_status);
        }
    } else {
        log_message(global_log_file, LOG_ERROR, "ITCOM_vSetActionRequestStartTime: Failed to lock mutex: error %d", mutex_lock_status);
    }
}

static struct timespec* ITCOM_pstGetActionRequestStartTime(uint16_t u16MsgId, uint16_t u16SequenceNum) {
    struct timespec* pstStartTime = NULL;
    uint8_t maxTimingCount = pstSharedMemData->stThreadsCommonData.u8ActionRequestTimingCount;

    /* Iterate through the action request timing records */
    uint8_t i;
    for (i = ITCOM_ZERO_INIT_U; i < maxTimingCount; i++) {
        if ((pstSharedMemData->stThreadsCommonData.astActionRequestTiming[i].u16MsgId == u16MsgId) &&
            (pstSharedMemData->stThreadsCommonData.astActionRequestTiming[i].u16SequenceNum == u16SequenceNum)) {
            pstStartTime = &pstSharedMemData->stThreadsCommonData.astActionRequestTiming[i].start_time;
            break;
        }
    }

    return pstStartTime;
}
