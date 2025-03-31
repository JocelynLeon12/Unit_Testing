//*****************************************************************************
/**
* @file itcom.h
*****************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief module to implement state machine
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

#ifndef ITCOM_H
#define ITCOM_H

/*** Include Files ***/
#include "gen_std_types.h"

#include "data_queue.h"
#include "instance_manager.h"
#include "util_time.h"

#include "action_request_approver.h"
#include "state_machine.h"
#include "fault_manager.h"
#include "start_up_test.h"
#include "icm.h"
#include "system_diagnostics.h"
#include "crv.h"




/*** Definitions Provided to other modules ***/

#define SHARED_BUFFER_SIZE (64 * 1024)
#define THREAD_MAX_RESTART_THRESOLD 5
#define THREAD_CRASH_MONITORING_INTERVAL 5
#define STORAGE_WRITE_INTERVAL 2

// Log levels
#define LOG_ERROR 0
#define LOG_WARNING 1
#define LOG_INFO 2
#define LOG_DEBUG 3

/*temporary include while other components are added*/
#define CIRCULAR_BUFF_INACTIVE	(0)

#define DATA_INTEGRITY_QUEUE	(0)
#define APPROVED_ACTIONS_QUEUE	(1)
#define SAFE_STATE_QUEUE		(2)

#define NUM_ASSOCIATED_LENGHTS  (3)

#define MAX_PENDING_ACTION_REQUESTS 10
#define ACTION_REQUEST_PROCESS_TIMEOUT_THRESHOLD 50

#define QUEUE_ACTION_SUCCESS                                                 ((int8_t)0)
#define QUEUE_ACTION_FAILURE_DATAQUEUE_INVALID_INPUT                         ((int8_t)-1)
#define QUEUE_ACTION_FAILURE_DATAQUEUE_DATA_SIZE_EXCEEDS_BUFFER              ((int8_t)-2)
#define QUEUE_ACTION_FAILURE_DATAQUEUE_QUEUE_FULL                            ((int8_t)-3) 
#define QUEUE_ACTION_FAILURE_DEFAULT                                         ((int8_t)-4)
#define QUEUE_ACTION_TIMEOUT                                                 ((int8_t)-5)
#define QUEUE_ACTION_FAILURE_DATAQUEUE_INDEX_OUT_OF_BOUNDS                   ((int8_t)-6)
#define QUEUE_ACTION_FAILURE_DATAQUEUE_QUEUE_EMPTY                           ((int8_t)-7)

#define ITCOM_OP_SUCCESS                      1U               /**< Operation is successful*/
#define ITCOM_OP_FAILURE                      0U               /**< Operation is failed*/

#define ENQUEUE_OPERATION_SUCCESS             ((int8_t)1)      /**< Enqueue operation is successful*/
#define ENQUEUE_OPERATION_FAILURE             ((int8_t)0)      /**< Enqueue operation is failed*/

/*** Type Definitions ***/

/**
 * @brief Structure to store the start time of an action request.
 */
typedef struct {
    uint16_t u16MsgId;
    uint16_t u16SequenceNum;
    struct timespec start_time;
} ActionRequestTiming_t;

/**
 * @brief Structure defining the private data for Thread CCU.
 */
typedef struct {
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_CCU_Private_Data_t;

/**
 * @brief Structure defining the private data for Thread STM.
 */
typedef struct {
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_STM_Private_Data_t;

/**
 * @brief Structure defining the private data for Thread ICM_RX.
 */
typedef struct {
	TLVMessage_t stReceivedTCPMsg;
	uint8_t u8CrcErrorCounter[enTotalMessagesASI];
	uint8_t u8RollingCounterError[enTotalMessagesASI];
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_ICM_RX_Private_Data_t;

/**
 * @brief Structure defining the private data for Thread ARA.
 */
typedef struct {
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_ARA_Private_Data_t;

/**
 * @brief Structure defining the private data for Thread ICM_TX.
 */
typedef struct {
	RateLimiter_t stRateLimiter;
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_ICM_TX_Private_Data_t;

/**
 * @struct SM_THRD_FM_Private_Data
 * @brief Private data structure for the Fault Management thread.
 *
 * Holds the current event being processed, processing flag, and synchronization primitives.
 */
typedef struct
{
    ErrorEvent current_event;
    int16_t processing;
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_FM_Private_Data;

/**
 * @brief Structure defining the private data for Thread SD.
 */
typedef struct {
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_SD_Private_Data_t;

/**
 * @brief Structure defining the private data for Thread CRV.
 */
typedef struct {
    pthread_mutex_t mutex;
    sem_t sem;
} SM_THRD_CRV_Private_Data_t;

/**
 * @brief Structure defining the common data shared by all threads and processes.
 */
typedef struct {
	/// STATE MACHINE
    uint8_t u8ASI_State;
    uint8_t u8CriticalFaultFlag;
    uint8_t u8InitFinishFlag;
    /// START-UP TEST
    SutTestResults_t stSUTResults;
    DateRecord_t stSutTimeRegister;
    AraTestResults_t stActionListTestResults;
    AraTestResults_t stPrecondTestResults;
    MemTestResult_t stMemoryTestResults;
    /// ICM
    uint16_t u16GnrlCycleCount;
    stRollingCountData_t stRollingCounterRegister[enTotalMessagesASI];
    stSequenceNumberData_t stSeqNumberRegister[enTotalMessagesASI];
    stIMBuffer stCycleSeqTrack;
    stIMBuffer stCalibrationDataCopyTrack;
    stIMBuffer stCalibrationReadbackTrack;
    stProcessMsgData astDataIntegrityMsgBuffer[MSG_QUEUE_BUFFER_SIZE]; //Arbitrarily selected buffer size
    data_queue_t stActionReqQueue;
    stProcessMsgData astApprovedMsgBuffer[MSG_QUEUE_BUFFER_SIZE]; //Arbitrarily selected buffer size
    data_queue_t stApprovedActionsQueue;
    stProcessMsgData astSSMsgBuffer[MSG_QUEUE_BUFFER_SIZE]; //Arbitrarily selected buffer size
    data_queue_t stMsgQueueSS;
    /// ARA
    stVehicleStatusInfo_t stVehicleStatus;
    ActionRequestTiming_t astActionRequestTiming[MAX_PENDING_ACTION_REQUESTS];
    uint8_t u8ActionRequestTimingCount; // To keep track of the number of entries
    /// FM
    uint8_t Event_Queue[DATA_QUEUE_MAX_SIZE];
    int8_t Event_Queue_Index;
    SystemSnapshot_t SystemSnapshotData;
    /// SD
    StateMonitor_t stStateMonitorData;
    TCPConnectionState_t enTCPConnectionState[enTotalTCPConnections];
    /// CRV
    uint8_t u8CalibComparisonResult;
    /// POSIX HANDLER
    pthread_mutex_t mutex;
} SM_Common_Public_Data;

/**
 * @brief Main structure defining all data shared in the application.
 * It includes private data for each thread and common public data.
 */
typedef struct {
    SM_THRD_CCU_Private_Data_t stThread_CCU;
    SM_THRD_STM_Private_Data_t stThread_STM;
    SM_THRD_ICM_RX_Private_Data_t stThread_ICM_RX;
    SM_THRD_ARA_Private_Data_t stThread_ARA;
    SM_THRD_ICM_TX_Private_Data_t stThread_ICM_TX;
    SM_THRD_FM_Private_Data stThread_FM;
    SM_THRD_SD_Private_Data_t stThread_SD;
    SM_THRD_CRV_Private_Data_t stThread_CRV;
    SM_Common_Public_Data stThreadsCommonData;
    volatile sig_atomic_t parent_initiated_termination;
} DataOnSharedMemory;


//############################################################################
typedef enum
{
    enSoftRestart = 0,
    enHardRestart,
    enTotalRestartReasons
}enRestartReason;

typedef enum
{
    enSuccess_EventAddedToQueue = 0,
    enFailure_InvalidEventID,
    enFailure_EventDiscarded,
    enFailure_MutexError,
}enSetErrorEventStatus;

typedef struct {
    uint16_t u16MessageId;
    uint16_t u16MessageType;
    uint8_t u8MessageEnum;
} MessageDictionary_t;

typedef struct {
    uint16_t u16MessageTypeID;
    uint8_t u8MessageTypeEnum;
    uint8_t au8AssociatedLengths[NUM_ASSOCIATED_LENGHTS];
} MessageTypeDictionary_t;


/*** External Variables ***/


/*** Functions Provided to other modules ***/
extern void ITCOM_vSharedMemoryInit(FILE* itcom_log_file, enRestartReason restart_reason);
extern void ITCOM_vCleanResources(void);
extern void ITCOM_vWrapperThread_CCU(void);
extern void ITCOM_vWrapperThread_STM(void);
extern void ITCOM_vWrapperThread_ICM_RX(void);
extern void ITCOM_vWrapperThread_ARA(void);
extern void ITCOM_vWrapperThread_ICM_TX(void);
extern void ITCOM_vWrapperThread_FM(void);
extern void ITCOM_vWrapperThread_SD(void);
extern void ITCOM_vWrapperThread_CRV(void);
extern void ITCOM_vChildProcessWrapper(FILE* itcom_log_file, enRestartReason start_reason);
extern void ITCOM_vParentdProcessWrapper(FILE* itcom_log_file);
extern void ITCOM_vSetParentTerminationFlag(uint8_t u8Value);
extern sig_atomic_t ITCOM_vGetParentTerminationFlag(void);

extern void ITCOM_vSetASIState(uint8_t u8Value);
extern uint8_t ITCOM_u8GetASIState(void);
extern enSetErrorEventStatus ITCOM_s16SetErrorEvent(uint8_t u8EventId);
extern void ITCOM_vUpdateCurrentEvent(ErrorEvent* pstCurrentEvent);
extern void ITCOM_vGetErrorEvent(ErrorEvent* const pstCurrentEvent);
extern void ITCOM_vSetInitFlagStatus(uint8_t u8FlagValue);
extern uint8_t ITCOM_u8GetInitFlagStatus(void);
extern uint8_t ITCOM_u8GetCriticalFaultStatus(void);
extern void ITCOM_vSetCycleCountData(uint16_t u16Value);
extern uint16_t ITCOM_u16GetCycleCountData(void);
extern void ITCOM_vSetMsgCycleCount(stMsgIntegrityData* pstTempMsgTracker, uint8_t u8Action);
extern int8_t ITCOM_s8SaveMsgData(stProcessMsgData* pstMsgPayload, int16_t s16Indx);
extern int8_t ITCOM_s8QueueActionReq(stProcessMsgData* pstMsgInfo);
extern int8_t ITCOM_s8DequeueActionReq(stProcessMsgData* pstActionReqData, uint8_t u8SelectQueue);
extern int8_t ITCOM_s8LogSSMessage(void);
extern int8_t ITCOM_s8LogNotificationMessage(uint16_t u16MsgId, uint16_t u16SequenceNum , uint8_t u8Data, uint8_t u8SelectNotification);
extern void ITCOM_vSetSeqNumASIRecord(uint16_t u16SequenceNum, uint8_t u8Indx);
extern uint16_t ITCOM_u16GetSeqNumASIRecord(uint8_t u8Indx);
extern void ITCOM_vRecordRC(uint8_t u8MsgInstance, uint16_t u16RollingCounter, uint8_t u8Direction);
extern int16_t ITCOM_u16GetRCData(uint8_t u8MsgInstance, uint8_t u8Direction);
extern void ITCOM_vSetParkStatus(uint8_t u8ParkStatus, uint8_t u8Status);
extern uint8_t ITCOM_u8GetParkStatus(uint8_t* pu8ParkStatus);
extern void ITCOM_vSetVehicleSpeed(float32_t f32VehicleSpeed, uint8_t u8Status);
extern uint8_t ITCOM_u8GetVehicleSpeed(float32_t* pf32VehicleSpeed);
extern void ITCOM_vWriteSUTRes(SutTestResults_t stTestResults);
extern void ITCOM_vRecordSutCompTime(DateRecord_t u32TimeRegister);
extern void ITCOM_vSetActionListTestResult(AraTestResults_t stTestResults);
extern void ITCOM_vSetPrecondListTestResult(AraTestResults_t stTestResults);
extern void ITCOM_vSetMemoryTestResult(MemTestResult_t stTestResults);
extern int16_t ITCOM_s16GetMessageTypeEnum(uint16_t u16MsgType);
extern int16_t ITCOM_s16GetMessageEnumById(uint16_t u16MsgId);
extern int16_t ITCOM_s16GetMessageEnumFromTypeAndId(uint16_t u16MsgType, uint16_t u16MsgId, enTCPConnectionsASI enTCPConn);
extern void ITCOM_vGetMsgTypeDictionaryEntryAtIndex(MessageTypeDictionary_t* pstDictionaryTypeData, uint16_t u16MsgId);
extern void ITCOM_vGetMsgDictionaryEntryAtIndex(MessageDictionary_t* pstDictionaryData, uint16_t u16MsgId);
extern int8_t ITCOM_s8ValidateMessageTypeLength(uint16_t u16MsgType, uint8_t u8Length);
extern uint16_t ITCOM_u16GetTrackBufferSize(uint8_t u8SelectBuffer);
extern void ITCOM_vGetCycleSeqElementAtIndex(uint16_t u16Indx, generic_ptr_t pvElement, uint8_t u8SelectBuffer);
extern void ITCOM_vSetCrcErrorCount(uint8_t u8Indx, uint8_t u8Value);
extern uint8_t ITCOM_u8GetCrcErrorCount(uint8_t u8Indx);
extern void ITCOM_vSetRollingCountError(uint8_t u8Indx, uint8_t u8Value);
extern uint8_t ITCOM_u8GetRollingCountError(uint8_t u8Indx);
extern void ITCOM_vSetMsgRateLimiter(RateLimiter_t* pstRateLimiter);
extern void ITCOM_vGetMsgRateLimiter(RateLimiter_t* pstRateLimiter);
extern void ITCOM_vSetErrorProcessingFlag(int16_t s16Value);
extern int16_t ITCOM_s16GetProcessingFlag(void);
extern void ITCOM_vSetEventQueueIndx(uint8_t u8IndxValue);
extern int16_t ITCOM_s16GetEventQueueIndx(void);
extern void ITCOM_vSetEventQueueId(uint8_t u8EventQueue, uint8_t u8Indx);
extern void ITCOM_vGetEventQueueId(uint8_t* pu8EventQueue, uint8_t u8Indx);
extern void ITCOM_vRemoveProcessedEvent(void);
extern int16_t ITCOM_vSemaphoreTryWait(void);
extern void ITCOM_vNotification_SM(void);
extern void ITCOM_vExtSysNotification(void);
extern void ITCOM_vSetTCPConnectionState(enTCPConnectionsASI enConnection, TCPConnectionState_t enState);
extern TCPConnectionState_t ITCOM_enGetTCPConnectionState(enTCPConnectionsASI enConnection);
extern void ITCOM_vSetStateMonitorTestData(StateMonitor_t stStateMonitorData);
extern void ITCOM_vGetStateMonitorTestData(StateMonitor_t* pstStateMonitorData);

extern void ITCOM_vSetCalibDataCopy(stProcessMsgData* pstTempMsgDataTracker, uint8_t u8Action);
extern void ITCOM_vSetCalibReadbackData(stProcessMsgData* pstTempMsgDataTracker, uint8_t u8Action);
extern int16_t ITCOM_s16GetCalibReadbackData(stProcessMsgData stTempMsgDataTracker, uint8_t* pu8Data);
extern void ITCOM_vSetCalibComparisonResult(uint8_t u8Result);

extern void ITCOM_vSetActionRequestStartTime(uint16_t u16MsgId, uint16_t u16SequenceNum);

#endif // ITCOM_H
