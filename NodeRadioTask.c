/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* Standard C Libraries */
#include <stdlib.h>

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/* Application Header files */
#include "RadioProtocol.h"
#include "NodeRadioTask.h"
#include "NodeTask.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
#include DeviceFamily_constructPath(driverlib/trng.h)

#ifdef FEATURE_BLE_ADV
#include "ble_adv/BleAdv.h"
#endif

#ifdef USE_DMM
#include <dmm/dmm_policy_blesp_wsnnode.h>
#ifndef USE_DMM_BLE_SP
#include <ble_remote_display/app/remote_display.h>
#endif /* USE_DMM_BLE_SP */
#endif /* USE_DMM */

/***** Defines *****/
#define NODERADIO_TASK_STACK_SIZE 1024
#define NODERADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                 0xFFFFFFFF
#define RADIO_EVENT_SEND_ADC_DATA       (uint32_t)(1 << 0)
#define RADIO_EVENT_DATA_ACK_RECEIVED   (uint32_t)(1 << 1)
#define RADIO_EVENT_ACK_TIMEOUT         (uint32_t)(1 << 2)
#define RADIO_EVENT_SEND_FAIL           (uint32_t)(1 << 3)
#ifdef FEATURE_BLE_ADV
#define NODE_EVENT_UBLE                 (uint32_t)(1 << 4)
#endif

#define NODERADIO_MAX_RETRIES 2
#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)


/***** Type declarations *****/
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};


/***** Variable declarations *****/
static Task_Params nodeRadioTaskParams;
Task_Struct nodeRadioTask;        /* not static so you can see in ROV */
#ifdef USE_DMM
static Task_Handle nodeRadioTaskHndl;
#endif
static uint8_t nodeRadioTaskStack[NODERADIO_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;
Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;
Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;
static struct RadioOperation currentRadioOperation;
static uint16_t adcData;
static uint8_t nodeAddress = 0;
static struct DualModeSensorPacket dmSensorPacket;

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
RemoteDisplay_nodeWsnStats_t radioStats = {0};
uint32_t rxErrorFlg = 0;
#endif /* USE_DMM && !USE_DMM_BLE_SP */

#ifdef USE_DMM_BLE_SP
uint32_t rxErrorFlg = 0;
uint32_t txError = 0;
uint32_t rxError = 0;
uint32_t rxTimeout = 0;
uint32_t rxSchError = 0;
#endif /* USE_DMM_BLE_SP */

/* previous Tick count used to calculate uptime */
static uint32_t prevTicks;

/* Pin driver handle */
extern PIN_Handle ledPinHandle;

/***** Prototypes *****/
static void nodeRadioTaskFunction(UArg arg0, UArg arg1);
static void returnRadioOperationStatus(enum NodeRadioOperationStatus status);
static void sendDmPacket(struct DualModeSensorPacket sensorPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs);
static void resendPacket(void);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);

#ifdef FEATURE_BLE_ADV
static void bleAdv_eventProxyCB(void);
static void bleAdv_updateTlmCB(uint16_t *pVbatt, uint16_t *pTemp, uint32_t *pTime100MiliSec);
static void bleAdv_updateMsButtonCB(uint8_t *pButton);
#endif

/***** Function definitions *****/
#ifndef USE_DMM
void NodeRadioTask_init(void) {
#else
Task_Handle* NodeRadioTask_init(void) {
#endif /* !USE_DMM */

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem, 0, &semParam);
    radioResultSemHandle = Semaphore_handle(&radioResultSem);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the radio protocol task */
    Task_Params_init(&nodeRadioTaskParams);
    nodeRadioTaskParams.stackSize = NODERADIO_TASK_STACK_SIZE;
    nodeRadioTaskParams.priority = NODERADIO_TASK_PRIORITY;
    nodeRadioTaskParams.stack = &nodeRadioTaskStack;
    Task_construct(&nodeRadioTask, nodeRadioTaskFunction, &nodeRadioTaskParams, NULL);

#ifdef USE_DMM
    nodeRadioTaskHndl = (Task_Handle) &nodeRadioTask;
    return &nodeRadioTaskHndl;
#endif /* USE_DMM */
}

uint8_t nodeRadioTask_getNodeAddr(void)
{
    return nodeAddress;
}

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
void nodeRadioTask_setNodeAddr(uint8_t newNodeAddress)
{
    if(nodeAddress != newNodeAddress)
    {
        nodeAddress = newNodeAddress;

        /* Setup ADC sensor packet */
        dmSensorPacket.header.sourceAddress = nodeAddress;
        dmSensorPacket.header.packetType = RADIO_PACKET_TYPE_DM_SENSOR_PACKET;

        /* Set the filter to the generated random address. Note that we may miss
         * an Ack*/
        EasyLink_enableRxAddrFilter(&nodeAddress, 1, 1);
    }
}

void nodeRadioTask_setConcLedToggle(bool concLedToggle)
{
    dmSensorPacket.concLedToggle = concLedToggle;

    if(concLedToggle)
    {
        /* Raise RADIO_EVENT_SEND_ADC_DATA event to send data msg with toggle*/
        Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_ADC_DATA);
    }

}
#endif /* USE_DMM && !USE_DMM_BLE_SP */

static void nodeRadioTaskFunction(UArg arg0, UArg arg1)
{
#ifdef FEATURE_BLE_ADV
    BleAdv_Params_t bleAdv_Params;
    /* Set mulitclient mode for EasyLink */
    EasyLink_setCtrl(EasyLink_Ctrl_MultiClient_Mode, 1);

#endif

    /* Initialize the EasyLink parameters to their default values */
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

#ifdef DEFINED_RADIO_EASYLINK_MODULATION
    /* Change the modulation from the default found in easylink_config.h */
    easyLink_params.ui32ModType = DEFINED_RADIO_EASYLINK_MODULATION;
#endif

    uint32_t noRadioAckTimeoutTimeMs = NORERADIO_ACK_TIMEOUT_TIME_MS;

#ifdef USE_DMM
    EasyLink_setCtrl(EasyLink_Ctrl_MultiClient_Mode, true);

    if (easyLink_params.ui32ModType == EasyLink_PHY_CUSTOM)
    {
        noRadioAckTimeoutTimeMs = 10;
    }
    else if (easyLink_params.ui32ModType == EasyLink_PHY_5KBPSSLLR)
    {
        noRadioAckTimeoutTimeMs = 100;
    }
    else
    {
        System_abort("Unsupported PHY Mode.");
    }
#endif /* USE_DMM */


    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success){
        System_abort("EasyLink_init failed");
    }

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */
    //EasyLink_setFrequency(868300000);////////////////change here

    /* Use the True Random Number Generator to generate sensor node address randomly */;
    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    /* Do not accept the same address as the concentrator, in that case get a new random value */
    do
    {
        while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
        {
            //wait for random number generator
        }
        nodeAddress = (uint8_t)TRNGNumberGet(TRNG_LOW_WORD);
    } while (nodeAddress == RADIO_CONCENTRATOR_ADDRESS);
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    /* Set the filter to the generated random address */
    if (EasyLink_enableRxAddrFilter(&nodeAddress, 1, 1) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_enableRxAddrFilter failed");
    }

    /* Setup ADC sensor packet */
    dmSensorPacket.header.sourceAddress = nodeAddress;
    dmSensorPacket.header.packetType = RADIO_PACKET_TYPE_DM_SENSOR_PACKET;
#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
    dmSensorPacket.concLedToggle = false;
    RemoteDisplay_updateNodeWsnStats(&radioStats);
#endif /* USE_DMM && !USE_DMM_BLE_SP */

    /* Initialise previous Tick count used to calculate uptime for the TLM beacon */
    prevTicks = Clock_getTicks();

#ifdef FEATURE_BLE_ADV
    /* Initialize the Simple Beacon module wit default params */
    BleAdv_Params_init(&bleAdv_Params);
    bleAdv_Params.pfnPostEvtProxyCB = bleAdv_eventProxyCB;
    bleAdv_Params.pfnUpdateTlmCB = bleAdv_updateTlmCB;
    bleAdv_Params.pfnUpdateMsButtonCB = bleAdv_updateMsButtonCB;
    bleAdv_Params.pfnAdvStatsCB = NodeTask_advStatsCB;
    BleAdv_init(&bleAdv_Params);

    /* initialize BLE advertisements to default to MS */
    BleAdv_setAdvertiserType(BleAdv_AdertiserMs);
#endif

    /* Enter main task loop */
    while (1)
    {
        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we should send ADC data */
        if (events & RADIO_EVENT_SEND_ADC_DATA)
        {
            uint32_t currentTicks;

            currentTicks = Clock_getTicks();
            //check for wrap around
            if (currentTicks > prevTicks)
            {
                //calculate time since last reading in 0.1s units
                dmSensorPacket.time100MiliSec += ((currentTicks - prevTicks) * Clock_tickPeriod) / 100000;
            }
            else
            {
                //calculate time since last reading in 0.1s units
                dmSensorPacket.time100MiliSec += ((prevTicks - currentTicks) * Clock_tickPeriod) / 100000;
            }
            prevTicks = currentTicks;

            dmSensorPacket.batt = AONBatMonBatteryVoltageGet();
            dmSensorPacket.adcValue = adcData;
            dmSensorPacket.button = !PIN_getInputValue(Board_PIN_BUTTON0);

            sendDmPacket(dmSensorPacket, NODERADIO_MAX_RETRIES, noRadioAckTimeoutTimeMs);
        }

        /* If we get an ACK from the concentrator */
        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            returnRadioOperationStatus(NodeRadioStatus_Success);
#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
            radioStats.dataSendSuccess++;
            RemoteDisplay_updateNodeWsnStats(&radioStats);
#endif /* USE_DMM && !USE_DMM_BLE_SP */
        }

        /* If we get an ACK timeout */
        if (events & RADIO_EVENT_ACK_TIMEOUT)
        {

            /* If we haven't resent it the maximum number of times yet, then resend packet */
            if (currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries)
            {
                resendPacket();
            }
            else
            {
                /* Else return send fail */
                Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_FAIL);
            }
        }

        /* If send fail */
        if (events & RADIO_EVENT_SEND_FAIL)
        {
            returnRadioOperationStatus(NodeRadioStatus_Failed);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
            radioStats.dataSendFail++;
            RemoteDisplay_updateNodeWsnStats(&radioStats);
#endif /* USE_DMM && !USE_DMM_BLE_SP */
        }

#ifdef FEATURE_BLE_ADV
        if (events & NODE_EVENT_UBLE)
        {
            uble_processMsg();
        }
#endif
    }
}

enum NodeRadioOperationStatus NodeRadioTask_sendAdcData(uint16_t data)
{
    enum NodeRadioOperationStatus status;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    /* Save data to send */
    adcData = data;

    /* Raise RADIO_EVENT_SEND_ADC_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_ADC_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}

static void returnRadioOperationStatus(enum NodeRadioOperationStatus result)
{
#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
    /* update DMM policy */
    DMMPolicy_updateStackState(DMMPolicy_StackRole_WsnNode, DMMPOLICY_STACKSTATE_WSNNODE_SLEEPING);
    /* Clear toggle in case it was set */
    if ((result == NodeRadioStatus_Success) && (dmSensorPacket.concLedToggle))
    {
        dmSensorPacket.concLedToggle = false;
        NodeTask_disableConcLedToggle();
    }
#endif /* USE_DMM && !USE_DMM_BLE_SP */

    /* Save result */
    currentRadioOperation.result = result;

    /* Post result semaphore */
    Semaphore_post(radioResultSemHandle);
}

static void sendDmPacket(struct DualModeSensorPacket sensorPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs)
{
    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;

    /* Copy ADC packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    currentRadioOperation.easyLinkTxPacket.payload[0] = dmSensorPacket.header.sourceAddress;
    currentRadioOperation.easyLinkTxPacket.payload[1] = dmSensorPacket.header.packetType;
    currentRadioOperation.easyLinkTxPacket.payload[2] = (dmSensorPacket.adcValue & 0xFF00) >> 8;
    currentRadioOperation.easyLinkTxPacket.payload[3] = (dmSensorPacket.adcValue & 0xFF);
    currentRadioOperation.easyLinkTxPacket.payload[4] = (dmSensorPacket.batt & 0xFF00) >> 8;
    currentRadioOperation.easyLinkTxPacket.payload[5] = (dmSensorPacket.batt & 0xFF);
    currentRadioOperation.easyLinkTxPacket.payload[6] = (dmSensorPacket.time100MiliSec & 0xFF000000) >> 24;
    currentRadioOperation.easyLinkTxPacket.payload[7] = (dmSensorPacket.time100MiliSec & 0x00FF0000) >> 16;
    currentRadioOperation.easyLinkTxPacket.payload[8] = (dmSensorPacket.time100MiliSec & 0xFF00) >> 8;
    currentRadioOperation.easyLinkTxPacket.payload[9] = (dmSensorPacket.time100MiliSec & 0xFF);
    currentRadioOperation.easyLinkTxPacket.payload[10] = dmSensorPacket.button;
#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
    currentRadioOperation.easyLinkTxPacket.payload[11] = dmSensorPacket.concLedToggle;
#endif /* USE_DMM && !USE_DMM_BLE_SP */

    currentRadioOperation.easyLinkTxPacket.len = sizeof(struct DualModeSensorPacket);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
    currentRadioOperation.ackTimeoutMs = ackTimeoutMs;
    currentRadioOperation.retriesDone = 0;
    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(ackTimeoutMs));
#ifdef USE_DMM
    /* update DMM policy */
    DMMPolicy_updateStackState(DMMPolicy_StackRole_WsnNode, DMMPOLICY_STACKSTATE_WSNNODE_TX);
#endif /* USE_DMM */

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.dataTxSchError++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
        /* Return before entering Rx */
        return;
#elif USE_DMM_BLE_SP
        txError++;
        /* Return before entering Rx */
        return;
#endif /* USE_DMM && !USE_DMM_BLE_SP */

    }

#ifdef USE_DMM
    /* update DMM policy */
    DMMPolicy_updateStackState(DMMPolicy_StackRole_WsnNode, DMMPOLICY_STACKSTATE_WSNNODE_ACK);
#endif /* USE_DMM */

    /* Enter RX */
    if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
    {
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.ackRxSchError++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
#elif USE_DMM_BLE_SP
        rxSchError++;
#endif /* USE_DMM && !USE_DMM_BLE_SP */
    }
}

static void resendPacket(void)
{
#ifdef USE_DMM
    /* update DMM policy */
    DMMPolicy_updateStackState(DMMPolicy_StackRole_WsnNode, DMMPOLICY_STACKSTATE_WSNNODE_TX);

#endif /* USE_DMM */
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.dataTxSchError++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
        /* Return before entering Rx */
        return;
#elif USE_DMM_BLE_SP
        txError++;
        /* Return before entering Rx */
        return;
#endif /* USE_DMM && !USE_DMM_BLE_SP */

    }

#ifdef USE_DMM
    /* Update DMM policy */
    DMMPolicy_updateStackState(DMMPolicy_StackRole_WsnNode, DMMPOLICY_STACKSTATE_WSNNODE_ACK);
#endif /* USE_DMM */

    /* Enter RX and wait for ACK with timeout */
    if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
    {
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.ackRxSchError++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
#elif USE_DMM_BLE_SP
        rxSchError++;
#endif /* USE_DMM && !USE_DMM_BLE_SP */
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}

#ifdef FEATURE_BLE_ADV
/*********************************************************************
* @fn      bleAdv_eventProxyCB
*
* @brief   Post an event to the application so that a Micro BLE Stack internal
*          event is processed by Micro BLE Stack later in the application
*          task's context.
*
* @param   None
*
* @return  None
*/
static void bleAdv_eventProxyCB(void)
{
    /* Post event */
    Event_post(radioOperationEventHandle, NODE_EVENT_UBLE);
}

/*********************************************************************
* @fn      bleAdv_updateTlmCB

* @brief Callback to update the TLM data
*
* @param pvBatt Battery level
* @param pTemp Current temperature
* @param pTime100MiliSec time since boot in 100ms units
*
* @return  None
*/
static void bleAdv_updateTlmCB(uint16_t *pvBatt, uint16_t *pTemp, uint32_t *pTime100MiliSec)
{
    uint32_t currentTicks = Clock_getTicks();

    //check for wrap around
    if (currentTicks > prevTicks)
    {
        //calculate time since last reading in 0.1s units
        *pTime100MiliSec += ((currentTicks - prevTicks) * Clock_tickPeriod) / 100000;
    }
    else
    {
        //calculate time since last reading in 0.1s units
        *pTime100MiliSec += ((prevTicks - currentTicks) * Clock_tickPeriod) / 100000;
    }
    prevTicks = currentTicks;

    *pvBatt = AONBatMonBatteryVoltageGet();
    // Battery voltage (bit 10:8 - integer, but 7:0 fraction)
    *pvBatt = (*pvBatt * 125) >> 5; // convert V to mV

    *pTemp = adcData;
}

/*********************************************************************
* @fn      bleAdv_updateMsButtonCB
*
* @brief Callback to update the MS button data
*
* @param pButton Button state to be added to MS beacon Frame
*
* @return  None
*/
static void bleAdv_updateMsButtonCB(uint8_t *pButton)
{
    *pButton = !PIN_getInputValue(Board_PIN_BUTTON0);
}
#endif

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    struct PacketHeader* packetHeader;

    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket->payload;

        /* Check if this is an ACK packet */
        if (packetHeader->packetType == RADIO_PACKET_TYPE_ACK_PACKET)
        {
            /* Signal ACK packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
        else
        {
            /* Packet Error, treat as a Timeout and Post a RADIO_EVENT_ACK_TIMEOUT
               event */
            Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.ackRxTimeout++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
#elif USE_DMM_BLE_SP
        rxTimeout++;
#endif /* USE_DMM && !USE_DMM_BLE_SP */
        }
    }
    /* did the Rx timeout */
    else if(status == EasyLink_Status_Rx_Timeout)
    {
        /* Post a RADIO_EVENT_ACK_TIMEOUT event */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);

#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        radioStats.ackRxTimeout++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
#elif USE_DMM_BLE_SP
        rxTimeout++;
#endif /* USE_DMM && !USE_DMM_BLE_SP */
    }
    else
    {
        /* The Ack reception may have been corrupted causing an error.
         * Treat this as a timeout
         */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
#if defined(USE_DMM) && !defined(USE_DMM_BLE_SP)
        rxErrorFlg |= status;
        radioStats.ackRxSchError++;
        RemoteDisplay_updateNodeWsnStats(&radioStats);
#elif USE_DMM_BLE_SP
        rxErrorFlg |= status;
        rxError++;
#endif /* USE_DMM && !USE_DMM_BLE_SP */
    }
}
