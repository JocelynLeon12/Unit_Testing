//****************************************************************************
/**
 * @file system_diagnostics.c
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

#include "system_diagnostics.h"
#include "storage_handler.h"

/*** Module Definitions ***/
/**
 * @def NETWORK_CONFIG_FILE
 * @brief File path for the network configuration file.
 *
 * Specifies the location and name of the network configuration file.
 * This file is used to indicate the ip addresses and ports to which the
 * ASI connects.
 *
 */
#define TEST_TIMEOUT_MS                   ((uint32_t)100U)
#define MAX_LATENCY_MS                    ((uint32_t)500U)

#define MAX_RECONNECT_ATTEMPTS            ((uint8_t)3U)
#define RECONNECT_DELAY_MS                ((uint32_t)100U)
#define CONNECTION_TIMEOUT_SEC            ((uint32_t)1U)
#define MAX_CONNECTED_CYCLES_BEFORE_CHECK ((uint8_t)25U)

#define MS_TO_USEC                        ((uint32_t)1000U)
#define SEC_TO_MS                         ((float32_t)1000.0f)
#define NSEC_TO_MS                        ((float32_t)1000000.0f)

#define VAM_IP_ADDR                       ((sd_char_t *)"192.168.0.246")
#define CM_IP_ADDR                        ((sd_char_t *)"192.168.0.246")
#define DEFAULT_VAM_PORT_NUMBER           ((uint16_t)8080U)
#define DEFAULT_CM_PORT_NUMBER            ((uint16_t)9090U)
#define INVALID_SOCKET                    ((sd_socket_t) - 1)
#define DEFAULT_CYCLE_COUNT               ((uint8_t)0U)

#define STATE_MONITOR_INIT_VALUE          ((uint8_t)0U)

/*** Internal Types ***/

/*** Local Function Prototypes ***/
static void SD_vStateMonitorTest(StateMonitor_t *pstStateMonitor, states_t stASIState);
static sd_socket_t sd_InitClientConnection(const sd_char_t *s8ServerIP, uint16_t u16Port);
static int8_t sd_ManageConnection(enTCPConnectionsASI enConnection);
static int8_t sd_s8TCPConnectionTest(enTCPConnectionsASI enConnection);
static void sd_EvaluateConnectionStatus(enTCPConnectionsASI enConnection, TCPConnectionState_t connectionState);
static void sd_vEvaluateStateTransitions(StateMonitor_t *pstStateMonitor, states_t stASIState);
static void sd_vEvaluateStateFaultMismatch(StateMonitor_t *pstStateMonitor, states_t stASIState);

/*** External Variables ***/
volatile sig_atomic_t sd_shutdown_initiated = 0;

/*** Internal Variables ***/
static TCPConnectionConfig_t stTCPConnectionConfigs[enTotalTCPConnections] = {
    {VAM_IP_ADDR, DEFAULT_VAM_PORT_NUMBER, INVALID_SOCKET, CONNECTION_STATE_DISCONNECTED, CONNECTION_STATE_DISCONNECTED, DEFAULT_CYCLE_COUNT},
    {CM_IP_ADDR, DEFAULT_CM_PORT_NUMBER, INVALID_SOCKET, CONNECTION_STATE_DISCONNECTED, CONNECTION_STATE_DISCONNECTED, DEFAULT_CYCLE_COUNT}};

/*** External Functions ***/

/**
 * @brief Main function for System Diagnostics module
 *
 * @details Performs the following operations:
 * 1. Checks ASI state, skipping diagnostics if in safe state
 * 2. Verifies system shutdown status
 * 3. Manages and checks TCP connections (VAM and CM)
 *
 * @note Designed for periodic execution in the main THRD_SD loop
 *
 */
void SD_vMainFunction(void)
{
    if (!sd_shutdown_initiated)
    {
        enTCPConnectionsASI enConnection;
        log_message(global_log_file, LOG_INFO, "Starting System Diagnostics...");
        uint8_t u8ASI_State = ITCOM_u8GetASIState();
        StateMonitor_t stStateMonitorData = {STATE_MONITOR_INIT_VALUE, STATE_MONITOR_INIT_VALUE};
        ITCOM_vGetStateMonitorTestData(&stStateMonitorData);
        SD_vStateMonitorTest(&stStateMonitorData, u8ASI_State);
        ITCOM_vSetStateMonitorTestData(stStateMonitorData);
        for (enConnection = 0; enConnection < enTotalTCPConnections; enConnection++)
        {
            if (sd_ManageConnection((enTCPConnectionsASI)enConnection) != E_OK)
            {
                log_message(global_log_file, LOG_WARNING, "Connection check failed for %s. Will retry in next cycle.",
                            enConnection == enVAMConnectionTCP ? "VAM" : "CM");
            }
        }
        log_message(global_log_file, LOG_INFO, "Completed System Diagnostics.");
    }
    else
    {
        log_message(global_log_file, LOG_INFO,
                    "TCP Connections are closing down. Exiting System Diagnostics...");
    }
}

/**
 * @brief Initializes TCP connections for the System Diagnostics module
 *
 * @details
 * - Iterates through all defined TCP connections (VAM and CM)
 * - Attempts to establish each connection using sd_InitClientConnection()
 * - Sets connection state based on the result of the connection attempt
 * - Updates the global connection state using ITCOM_vSetTCPConnectionState()
 * - Logs the outcome of each connection attempt
 *
 * @note Handles connection errors and successful connections separately
 *
 */
void SD_vTCPConnectionsInit(void)
{
    uint8_t u8InitFlagStatus = INACTIVE_FLAG;
    log_message(global_log_file, LOG_INFO, "Initializing TCP Connections...");

    enTCPConnectionsASI enConnection;
    for (enConnection = 0; enConnection < enTotalTCPConnections; enConnection++)
    {
        const sd_char_t *connectionName = (enConnection == enVAMConnectionTCP) ? (const sd_char_t *)"VAM" : (const sd_char_t *)"CM";
        log_message(global_log_file, LOG_DEBUG, "Initializing connection for %s", connectionName);

        stTCPConnectionConfigs[enConnection].s16Socket = sd_InitClientConnection(
            stTCPConnectionConfigs[enConnection].pchServerIp,
            stTCPConnectionConfigs[enConnection].u16Port);

        if (stTCPConnectionConfigs[enConnection].s16Socket >= 0)
        {
            stTCPConnectionConfigs[enConnection].enState = CONNECTION_STATE_CONNECTED;
            stTCPConnectionConfigs[enConnection].enPreviousState = CONNECTION_STATE_DISCONNECTED;
            stTCPConnectionConfigs[enConnection].u8ConnectedCycleCount = 0U;
            ITCOM_vSetTCPConnectionState((enTCPConnectionsASI)enConnection, CONNECTION_STATE_CONNECTED);
            sd_EvaluateConnectionStatus((enTCPConnectionsASI)enConnection, CONNECTION_STATE_CONNECTED);
            log_message(global_log_file, LOG_INFO, "Connection established for %s", connectionName);
            u8InitFlagStatus = ACTIVE_FLAG;
        }
        else
        {
            stTCPConnectionConfigs[enConnection].enState = CONNECTION_STATE_ERROR;
            stTCPConnectionConfigs[enConnection].enPreviousState = CONNECTION_STATE_DISCONNECTED;
            stTCPConnectionConfigs[enConnection].u8ConnectedCycleCount = 0U;
            sd_EvaluateConnectionStatus((enTCPConnectionsASI)enConnection, CONNECTION_STATE_ERROR);
            log_message(global_log_file, LOG_ERROR, "Failed to establish connection for %s", connectionName);
            u8InitFlagStatus = INACTIVE_FLAG;
        }
        u8InitFlagStatus = (u8InitFlagStatus == ACTIVE_FLAG ? u8InitFlagStatus : INACTIVE_FLAG);
    }

    ITCOM_vSetInitFlagStatus(u8InitFlagStatus);
    log_message(global_log_file, LOG_INFO, "TCP Connections initialization complete.");
}

/**
 * @brief Closes a specific TCP connection in the System Diagnostics module
 *
 * @param enConnection The connection to be closed (enTCPConnectionsASI type)
 *
 * @details
 * - Validates the connection index
 * - Closes the socket if it's open
 * - Resets connection state to DISCONNECTED
 * - Updates global connection state via ITCOM_vSetTCPConnectionState()
 * - Logs the closure or attempts to close an already closed connection
 *
 * @note Handles invalid connection indices and already closed connections
 *
 */
void SD_vCloseTCPConnection(enTCPConnectionsASI enConnection)
{
    sd_shutdown_initiated = 1;

    if ((enTCPConnectionsASI)enConnection < (enTCPConnectionsASI)enTotalTCPConnections)
    {
        log_message(global_log_file, LOG_INFO, "Initiating TCP Connection close down for : %s",
                    ((enTCPConnectionsASI)enConnection == (enTCPConnectionsASI)enVAMConnectionTCP) ? "VAM" : "CM");

        if (stTCPConnectionConfigs[enConnection].s16Socket != -1)
        {
            (void)close(stTCPConnectionConfigs[enConnection].s16Socket);
            log_message(global_log_file, LOG_INFO, "Closed TCP Connection for %s.",
                        ((enTCPConnectionsASI)enConnection == (enTCPConnectionsASI)enVAMConnectionTCP) ? "VAM" : "CM");

            stTCPConnectionConfigs[enConnection].s16Socket = -1;
            stTCPConnectionConfigs[enConnection].enState = CONNECTION_STATE_DISCONNECTED;
            stTCPConnectionConfigs[enConnection].enPreviousState = CONNECTION_STATE_DISCONNECTED;
            stTCPConnectionConfigs[enConnection].u8ConnectedCycleCount = 0U;
            ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_DISCONNECTED);
            sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_DISCONNECTED);
        }
        else
        {
            log_message(global_log_file, LOG_WARNING, "Attempted to close already closed connection %s.",
                        ((enTCPConnectionsASI)enConnection == (enTCPConnectionsASI)enVAMConnectionTCP) ? "VAM" : "CM");
        }
    }
    else
    {
        log_message(global_log_file, LOG_ERROR, "Invalid connection index: %d", (enTCPConnectionsASI)enConnection);
    }
}

/**
 * @brief Retrieves the configuration of a specific TCP connection
 *
 * @param enConnection The connection to retrieve the configuration for (enTCPConnectionsASI type)
 *
 * @return const TCPConnectionConfig_t* Pointer to the connection configuration structure,
 *         or NULL if the connection index is invalid
 *
 * @details
 * - Validates the connection index
 * - Returns a pointer to the corresponding TCPConnectionConfig_t structure if valid
 * - Returns NULL for invalid connection indices
 *
 * @note This function provides read-only access to the connection configuration
 */
const TCPConnectionConfig_t *SD_GetTCPConnectionConfig(enTCPConnectionsASI enConnection)
{
    const TCPConnectionConfig_t *result = NULL;

    if ((enTCPConnectionsASI)enConnection < (enTCPConnectionsASI)enTotalTCPConnections)
    {
        result = &stTCPConnectionConfigs[enConnection];
    }

    return result;
}

/*** Local Function Implementations ***/

/**
 * @brief Evaluates
 *
 * @param
 *
 * @details
 * - Determines the ...
 *
 */
// Update the state and check if the transition is valid
static void SD_vStateMonitorTest(StateMonitor_t *pstStateMonitor, states_t stASIState)
{
    if (VALID_PTR(pstStateMonitor))
    {
        sd_vEvaluateStateFaultMismatch(pstStateMonitor, stASIState);
        sd_vEvaluateStateTransitions(pstStateMonitor, stASIState);
        if (pstStateMonitor->u8StateError != 0U)
        {
            enSetErrorEventStatus result = ITCOM_s16SetErrorEvent(EVENT_ID_FAULT_SM_TRANSITION_ERROR);
            if (result != enSuccess_EventAddedToQueue)
            {
                log_message(global_log_file, LOG_ERROR, "Failed to set error event for State Machine Transition Error.");
            }
            ITCOM_vSetASIState((uint8_t)STATE_SAFE_STATE);
        }
        else
        {
            pstStateMonitor->stCurrentState = stASIState;
        }
    }
    else
    {
        log_message(global_log_file, LOG_WARNING, "Invalid pointer passed down.");
    }
}

/**
 * @brief Initializes a non-blocking client TCP connection
 *
 * @param s8ServerIP The IP address of the server to connect to
 * @param u16Port The port number to connect to
 *
 * @return sd_socket_t The socket file descriptor if successful, -1 on failure
 *
 * @details
 * - Creates a new socket
 * - Sets the socket to non-blocking mode
 * - Attempts to connect to the specified server
 * - Uses select() to wait for connection with a timeout
 * - Checks for successful connection or errors
 *
 */
static sd_socket_t sd_InitClientConnection(const sd_char_t *s8ServerIP, uint16_t u16Port)
{
    sd_socket_t sockfd = -1;
    sd_result_t result = -1;
    struct sockaddr_in server_addr;
    sd_flags_t socketFlags = 0;
    int32_t connection_status = 0;

    sockfd = (sd_socket_t)socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd >= 0)
    {
        socketFlags = fcntl(sockfd, F_GETFL, 0);
        (void)fcntl(sockfd, F_SETFL, socketFlags | O_NONBLOCK);

        (void)memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(u16Port);

        if (inet_pton(AF_INET, s8ServerIP, &server_addr.sin_addr) > 0)
        {
            connection_status = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));

            if ((connection_status < 0) && (errno == EINPROGRESS))
            {
                fd_set write_fds;
                struct timeval tv;
                FD_ZERO(&write_fds);
                FD_SET((uint32_t)sockfd, &write_fds);
                tv.tv_sec = (time_t)CONNECTION_TIMEOUT_SEC;
                tv.tv_usec = 0;

                if (select(sockfd + 1, NULL, &write_fds, NULL, &tv) == 1)
                {
                    sd_result_t socketErrorStatus = 0;
                    socklen_t len = sizeof(socketErrorStatus);

                    if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &socketErrorStatus, &len) == 0)
                    {
                        if (socketErrorStatus == 0)
                        {
                            log_message(global_log_file, LOG_INFO,
                                        "Successfully connected to %s:%u", s8ServerIP, u16Port);
                            result = sockfd;
                        }
                    }
                }
            }
        }

        if (result == -1)
        {
            log_message(global_log_file, LOG_ERROR,
                        "Connection failed/timed out to %s:%u", s8ServerIP, u16Port);
            (void)close(sockfd);
        }
    }
    else
    {
        (void)log_message(global_log_file, LOG_ERROR,
                          "Socket creation error for %s:%u - %s", s8ServerIP, u16Port, strerror(errno));
    }

    return result;
}

/**
 * @brief Manages the state and health of a specific TCP connection
 *
 * @param enConnection The connection to manage (enTCPConnectionsASI type)
 *
 * @return int8_t E_OK on successful management, E_NOT_OK on failure
 *
 * @details
 * - Handles connection based on its current state:
 *   - DISCONNECTED/ERROR: Attempts to reconnect
 *   - CONNECTED: Performs periodic health checks
 *   - CONNECTING: Monitors ongoing connection attempts
 * - Updates connection state and triggers appropriate actions
 * - Manages reconnection attempts with backoff
 * - Logs connection status changes
 *
 */
static int8_t sd_ManageConnection(enTCPConnectionsASI enConnection)
{
    int8_t result = E_NOT_OK;

    if (sd_shutdown_initiated == 0U)
    {
        if ((enTCPConnectionsASI)enConnection < (enTCPConnectionsASI)enTotalTCPConnections)
        {
            TCPConnectionConfig_t *config = &stTCPConnectionConfigs[enConnection];
            TCPConnectionState_t currentState = config->enState;
            const sd_char_t *connectionName =
                (enConnection == enVAMConnectionTCP) ? "VAM" : "CM";

            switch (currentState)
            {
            case CONNECTION_STATE_DISCONNECTED:
            case CONNECTION_STATE_ERROR:
            {
                uint8_t attempt;
                sd_connected_t connected = 0U;

                log_message(global_log_file, LOG_INFO, "Attempting to connect to %s...", connectionName);
                config->enState = CONNECTION_STATE_CONNECTING;
                ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_CONNECTING);
                sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_CONNECTING);

                for (attempt = 1U; (attempt <= MAX_RECONNECT_ATTEMPTS) && (connected == 0U); attempt++)
                {
                    sd_socket_t sockfd = sd_InitClientConnection(config->pchServerIp, config->u16Port);

                    if (sockfd != -1)
                    {
                        config->s16Socket = sockfd;
                        config->enState = CONNECTION_STATE_CONNECTED;
                        ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_CONNECTED);
                        sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_CONNECTED);
                        log_message(global_log_file, LOG_INFO,
                                    "Connection %s established on attempt %u.", connectionName, attempt);
                        result = E_OK;
                        connected = 1U;
                    }
                    else
                    {
                        log_message(global_log_file, LOG_WARNING,
                                    "Reconnect attempt %u for %s failed.", attempt, connectionName);
                        (void)usleep(RECONNECT_DELAY_MS * MS_TO_USEC);
                    }
                }

                if (connected == 0U)
                {
                    config->enState = CONNECTION_STATE_ERROR;
                    ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_ERROR);
                    sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_ERROR);
                    log_message(global_log_file, LOG_ERROR,
                                "Failed to establish connection %s after %u attempts.",
                                connectionName, MAX_RECONNECT_ATTEMPTS);
                }
                break;
            }

            case CONNECTION_STATE_CONNECTED:
            {
                log_message(global_log_file, LOG_DEBUG,
                            "Managing %s connection in CONNECTED state.", connectionName);

                if ((config->enPreviousState != CONNECTION_STATE_CONNECTED) ||
                    (config->u8ConnectedCycleCount >= MAX_CONNECTED_CYCLES_BEFORE_CHECK))
                {
                    log_message(global_log_file, LOG_DEBUG,
                                "Performing health check for %s connection.", connectionName);

                    if (sd_s8TCPConnectionTest(enConnection) == E_OK)
                    {
                        ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_CONNECTED);
                        sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_CONNECTED);
                        config->u8ConnectedCycleCount = 0U;
                        result = E_OK;
                    }
                    else
                    {
                        config->enState = CONNECTION_STATE_ERROR;
                        ITCOM_vSetTCPConnectionState(enConnection, CONNECTION_STATE_ERROR);
                        sd_EvaluateConnectionStatus(enConnection, CONNECTION_STATE_ERROR);
                        config->u8ConnectedCycleCount = 0U;
                    }
                }
                else
                {
                    config->u8ConnectedCycleCount++;
                    log_message(global_log_file, LOG_DEBUG,
                                "%s connection stable. Cycles since last check: %u",
                                connectionName, config->u8ConnectedCycleCount);
                    result = E_OK;
                }
                break;
            }

            case CONNECTION_STATE_CONNECTING:
                log_message(global_log_file, LOG_DEBUG,
                            "Connection %s is already attempting to connect.", connectionName);
                result = E_OK;
                break;

            default:
                log_message(global_log_file, LOG_WARNING,
                            "Connection %s is in an unknown state.", connectionName);
                sd_EvaluateConnectionStatus(enConnection, currentState);
                break;
            }

            config->enPreviousState = currentState;
        }
        else
        {
            log_message(global_log_file, LOG_ERROR, "Invalid connection index: %u", (enTCPConnectionsASI)enConnection);
        }
    }
    else
    {
        result = E_OK;
    }

    return result;
}

/**
 * @brief Tests the health of a specific TCP connection
 *
 * @param enConnection The connection to test (enTCPConnectionsASI type)
 *
 * @return int8_t E_OK if the connection is healthy, E_NOT_OK otherwise
 *
 * @details
 * - Validates the connection index and socket
 * - Uses select() to check if the socket is writable
 * - Sends a test packet (TEST_PACKET) to the connection
 * - Measures the latency of the send operation
 * - Considers the connection healthy if:
 *   1. The full test packet is sent successfully
 *   2. The latency is within the defined limit (MAX_LATENCY_MS)
 * - Updates the connection state based on the test result
 * - Logs the test outcome and any errors encountered
 *
 */
static int8_t sd_s8TCPConnectionTest(enTCPConnectionsASI enConnection)
{
    static const sd_char_t TEST_PACKET[] = "PING";
    static const size_t TEST_PACKET_SIZE = 4U;
    int8_t result = E_NOT_OK;

    if ((enTCPConnectionsASI)enConnection < (enTCPConnectionsASI)enTotalTCPConnections)
    {
        TCPConnectionConfig_t *config = &stTCPConnectionConfigs[enConnection];
        const sd_char_t *connectionName =
            (enConnection == enVAMConnectionTCP) ? "VAM" : "CM";

        if (config->s16Socket >= 0)
        {
            fd_set write_fds;
            struct timeval timeout = {0, (suseconds_t)(TEST_TIMEOUT_MS * MS_TO_USEC)};
            FD_ZERO(&write_fds);
            FD_SET((uint32_t)config->s16Socket, &write_fds);

            int32_t select_result = select(config->s16Socket + 1, NULL, &write_fds, NULL, &timeout);

            if ((select_result > 0) && (FD_ISSET((uint32_t)config->s16Socket, &write_fds) != 0))
            {
                struct timespec start, end;
                (void)clock_gettime(CLOCK_MONOTONIC, &start);

                ssize_t sent = send(config->s16Socket, TEST_PACKET, TEST_PACKET_SIZE, MSG_NOSIGNAL);

                (void)clock_gettime(CLOCK_MONOTONIC, &end);
                sd_latency_t latency = (sd_latency_t)(((end.tv_sec - start.tv_sec) * SEC_TO_MS) +
                                                      ((end.tv_nsec - start.tv_nsec) / NSEC_TO_MS));

                if (sent == (ssize_t)TEST_PACKET_SIZE)
                {
                    if (latency <= (sd_latency_t)MAX_LATENCY_MS)
                    {
                        log_message(global_log_file, LOG_DEBUG,
                                    "%s TCPConnectionTest successful. Latency: %.2f ms", connectionName, latency);
                        config->enState = CONNECTION_STATE_CONNECTED;
                        result = E_OK;
                    }
                    else
                    {
                        log_message(global_log_file, LOG_WARNING,
                                    "%s TCPConnectionTest failed. High latency: %.2f ms", connectionName, latency);
                    }
                }
                else
                {
                    (void)log_message(global_log_file, LOG_WARNING,
                                      "TCPConnectionTest failed to send test packet on %s socket %d: %s",
                                      connectionName, (int32_t)config->s16Socket, strerror(errno));
                }
            }
            else if (select_result == 0)
            {
                (void)log_message(global_log_file, LOG_WARNING,
                                  "%s TCPConnectionTest timed out after %u ms", connectionName, TEST_TIMEOUT_MS);
            }
            else
            {
                (void)log_message(global_log_file, LOG_WARNING,
                                  "%s TCPConnectionTest select() failed: %s", connectionName, strerror(errno));
            }

            if (result != E_OK)
            {
                config->enState = CONNECTION_STATE_ERROR;
            }
        }
        else
        {
            log_message(global_log_file, LOG_WARNING,
                        "%s socket %d is invalid", connectionName, (int32_t)config->s16Socket);
        }
    }
    else
    {
        log_message(global_log_file, LOG_ERROR, "Invalid TCPConnection under Test: %u", (enTCPConnectionsASI)enConnection);
    }

    return result;
}

/**
 * @brief Evaluates and logs the status of a TCP connection
 *
 * @param enConnection The connection to evaluate (enTCPConnectionsASI type)
 * @param connectionState The current state of the connection (TCPConnectionState_t type)
 *
 * @details
 * - Determines the connection name (CM or VAM) based on the enConnection parameter
 * - Logs appropriate messages based on the connection state:
 *   - CONNECTED: Logs that the connection is stable
 *   - DISCONNECTED or ERROR: Logs connection loss and sets an error event
 *   - CONNECTING: Logs that a reconnection attempt is in progress
 *   - Unknown states: Logs a warning
 * - Uses ITCOM_s16SetErrorEvent() to set an error event for lost connections
 *
 */
static void sd_EvaluateConnectionStatus(enTCPConnectionsASI enConnection, TCPConnectionState_t connectionState)
{
    const sd_char_t *connectionName = ((enTCPConnectionsASI)enConnection == enCMConnectionTCP) ? "CM" : "VAM";

    switch (connectionState)
    {
    case CONNECTION_STATE_CONNECTED:
        log_message(global_log_file, LOG_INFO, "System Diagnostics: Connection %s is stable.", connectionName);
        break;
    case CONNECTION_STATE_DISCONNECTED:
    case CONNECTION_STATE_ERROR:
        log_message(global_log_file, LOG_ERROR, "System Diagnostics: Connection %s lost.", connectionName);
        enSetErrorEventStatus result = ITCOM_s16SetErrorEvent(EVENT_ID_INFO_LOSS_COMM);
        if (result != enSuccess_EventAddedToQueue)
        {
            log_message(global_log_file, LOG_ERROR, "Failed to set error event for Connection Loss.");
        }
        break;
    case CONNECTION_STATE_CONNECTING:
        log_message(global_log_file, LOG_INFO, "System Diagnostics: Connection %s is attempting to reconnect.", connectionName);
        break;
    default:
        log_message(global_log_file, LOG_WARNING, "System Diagnostics: Connection %s is in an unknown state.", connectionName);
        break;
    }
}

/**
 * @brief Evaluates the state transitions
 *
 * @param pstStateMonitor Pointer to the StateMonitor_t structure
 * @param stASIState The current state of the ASI
 */
static void sd_vEvaluateStateTransitions(StateMonitor_t *pstStateMonitor, states_t stASIState)
{
    if (pstStateMonitor != NULL)
    {
        switch (pstStateMonitor->stCurrentState)
        {
        case STATE_INITIAL:
            if ((stASIState != STATE_STARTUP_TEST) && (stASIState != STATE_SAFE_STATE))
            {
                pstStateMonitor->u8StateError = STATE_INVALID;
                log_message(global_log_file, LOG_DEBUG, "Invalid transition from initial state.");
            }
            break;

        case STATE_STARTUP_TEST:
            if ((stASIState != STATE_SAFE_STATE) && (stASIState != STATE_NORM_OP) && (stASIState != STATE_STARTUP_TEST))
            {
                pstStateMonitor->u8StateError = STATE_INVALID;
                log_message(global_log_file, LOG_DEBUG, "Invalid transition from start-up test.");
            }
            break;
        case STATE_NORM_OP:
            if ((stASIState != STATE_SAFE_STATE) && (stASIState != STATE_NORM_OP))
            {
                pstStateMonitor->u8StateError = STATE_INVALID;
                log_message(global_log_file, LOG_DEBUG, "Invalid transition from normal operation.");
            }
            break;

        case STATE_SAFE_STATE:
            if (stASIState != STATE_SAFE_STATE)
            {
                pstStateMonitor->u8StateError = STATE_INVALID;
                log_message(global_log_file, LOG_DEBUG, "No escape from safe state.");
            }
            break;

        default:
            pstStateMonitor->u8StateError = STATE_INVALID;
            break;
        }
    }
}

/**
 * @brief Evaluates the state and fault mismatch
 *
 * @param pstStateMonitor Pointer to the StateMonitor_t structure
 * @param stASIState The current state of the ASI
 */
static void sd_vEvaluateStateFaultMismatch(StateMonitor_t *pstStateMonitor, states_t stASIState)
{
    if (pstStateMonitor != NULL)
    {
        uint8_t u8CriticalFaultFlag = ITCOM_u8GetCriticalFaultStatus();
        if ((u8CriticalFaultFlag == ACTIVE_FLAG) &&
            (stASIState != STATE_SAFE_STATE))
        {
            pstStateMonitor->u8StateError = STATE_INVALID;
            log_message(global_log_file, LOG_DEBUG, "State-Fault Mismatch.");
        }
    }
}