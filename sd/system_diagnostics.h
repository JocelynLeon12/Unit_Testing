//****************************************************************************
/**
* @file system_diagnostics.h
******************************************************************************
* PROJECT NAME: Sonatus Automator
*
* @brief Module to implement System Diagnostics Communication Test
*
* @authors Tusar Palauri, Alejandro Tollola
*
* @date September 13 2024
*
* HISTORY:
* DATE BY DESCRIPTION
* date      |IN |Description
* ----------|---|-----------
* 09/13/2024|TP |Initial Implementation
*
*/
//*****************************************************************************

#ifndef SYSTEM_DIAGNOSTICS_H
#define SYSTEM_DIAGNOSTICS_H

/*** Include Files ***/
#include "gen_std_types.h"
#include "icm.h"
#include "state_machine.h"

/*** Definitions Provided to other modules ***/


/*** Type Definitions ***/
typedef enum {
    CONNECTION_STATE_DISCONNECTED = 0U,
    CONNECTION_STATE_CONNECTING,
    CONNECTION_STATE_CONNECTED,
    CONNECTION_STATE_ERROR
} TCPConnectionState_t;

typedef enum {
    enVAMConnectionTCP = 0U,
    enCMConnectionTCP,
    enTotalTCPConnections
} enTCPConnectionsASI;

typedef int32_t     sd_socket_t;
typedef int32_t     sd_result_t;
typedef int32_t     sd_flags_t;
typedef float32_t   sd_latency_t;
typedef uint8_t     sd_connected_t;
typedef char        sd_char_t;

typedef struct {
    sd_char_t* pchServerIp;
    uint16_t u16Port;
    sd_socket_t s16Socket;
    TCPConnectionState_t enState;
    TCPConnectionState_t enPreviousState;
    uint8_t u8ConnectedCycleCount;
} TCPConnectionConfig_t;

typedef struct {
    states_t stCurrentState;
    uint8_t u8StateError;
} StateMonitor_t;

/*** Functions Provided to other modules ***/
extern void SD_vTCPConnectionsInit(void);
extern void SD_vMainFunction(void);
extern void SD_vCloseTCPConnection(enTCPConnectionsASI enConnection);
extern const TCPConnectionConfig_t* SD_GetTCPConnectionConfig(enTCPConnectionsASI enConnection);

/*** Variables Provided to other modules ***/
extern volatile sig_atomic_t sd_shutdown_initiated;

#endif // SYSTEM_DIAGNOSTICS_H
