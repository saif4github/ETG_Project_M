/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/qspi/qspi.h>
#include <ti/drivers/qspiflash/qspiflash.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/include/pinmux_xwr18xx.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/include/rl_driver.h>

#include <ti/drivers/canfd/canfd.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/utils/testlogger/logger.h>

/* Application Include Files: */
#include "mss_mrr.h"
#include "../common/mrr_config_consts.h"
#include "../common/mmw_messages.h"
#include <ti/utils/cli/cli.h>
#include <ti/drivers/cbuff/cbuff.h>
#include "BSP/Inc/battery.h"
#include "BSP/Inc/i2c_driver.h"
#include "BSP/Inc/rtc.h"
/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/


//#define SERIAL_ENABLE
#define MMWDEMO_OUTPUT_MSG_CLUSTERS        2
#define MMWDEMO_OUTPUT_MSG_TRACKED_OBJECTS 3
#define MMWDEMO_OUTPUT_MSG_PARKING_ASSIST  4
#define MMWDEMO_HEADER              0x5U
#define MMWDEMO_PADDING             0x6U
uint8_t powerOffCounter = 0;
uint8_t powerOffFlag = 0;
volatile uint8_t powerBtn = 0;
uint8_t va_LEDCounter = 0;
volatile uint8_t testBtn = 1;
volatile uint8_t runBtn = 1;
uint8_t BatMonitorCounter = 0;
volatile uint8_t rangeBtn = 1;
extern uint8_t lowBatteryCounter;
extern uint8_t lowBatteryFlag;
uint8_t selfTestCounter = 0;
uint16_t adc_val;
uint8_t selfTestFlag = 1;
uint8_t rangeRadarFlag = FALSE;
extern volatile rlUInt8_t isGetGpAdcMeasData;
extern rlRecvdGpAdcData_t rcvGpAdcData;
uint8_t rangeCounter = 0;
uint8_t rangeFlag = 0;
extern uint8_t get_time[NO_OF_REG];
uint8_t logDataCounter = 0;
uint8_t flashDataFlag = 0;
uint8_t dataExtractCounter = 0;
uint8_t lowBatteryCounter = 0;
uint8_t bootFlag = 0;
uint8_t bootTimeCounter = 0;
char startOfFrame[] = "Start";
char endOfFrame[] = "Stop";
/**
 * @brief
 *  Global Variable for tracking information required by the MRR TI Design
 */
#pragma DATA_ALIGN(gMrrMSSMCB, 16);
Mrr_MSS_MCB    gMrrMSSMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
static void MRR_MSS_chirpIntCallback(uintptr_t arg);
static void MRR_MSS_frameStartIntCallback(uintptr_t arg);
static int32_t MRR_MSS_eventFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
static void MRR_MSS_cfgFxn(MMWave_CtrlCfg* ptrCtrlCfg);
static void MRR_MSS_startFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MRR_MSS_stopFxn(void);
static void MRR_MSS_initTask (UArg arg0, UArg arg1);
static void MRR_MSS_mmWaveCtrlTask (UArg arg0, UArg arg1);
void GPIO_allpins();
void seflTestFunction();


/*
 *  QSPI Flash Related Init
 */

#define LOG_DATA_ADDR           0xC0201000U   //  Start logging Radar Data at this location
#define LOG_DATA_OFFSET         0x201000U     //  Start logging Radar Data at this location
#define LAST_DATA_LOG_OFFSET    0x200000U     // for Storing last location of the flash
#define LAST_DATA_LOG_ADDR      0xC0200000U   // for Storing last location of the flash
#define FLASH_END_ADDR_8MB      0xC07FEFFFU   // end address for 8MB flash
#define FLASH_END_ADDR_64MB     0xC3FFFFFFU   // end address for 64MB flash

QSPI_Handle   gQSPIDrv = (QSPI_Handle)NULL;
QSPIFlash_Handle QSPIFlashHandle = NULL;

volatile uint32_t flashAddr = 0U;
uint32_t lastLogDataPointer = 0; // for storing Last location of the data written
static uint32_t totalSizeWritten = 0;

void QSPIFlash_4KBFlashErase(uint32_t flashAddrTemp)
{
    uint32_t tempFlashAddr = (uint32_t)flashAddrTemp;
    QSPIFlash_sectorErase(QSPIFlashHandle, tempFlashAddr);
}


uint32_t QSPIFlash_Write(uint32_t tempFlashAddrTx, uint8_t *dataBuffer, uint32_t dataLen)
{

    QSPIFlash_singleWrite(QSPIFlashHandle, tempFlashAddrTx, dataLen, (uint8_t*)dataBuffer);
    tempFlashAddrTx = tempFlashAddrTx + dataLen;
    return tempFlashAddrTx;
}
void processFlashDataThroughUart()
{
    uint32_t tempFlashAddrRx = (uint32_t)LOG_DATA_ADDR;
    uint32_t tempData = 0;
    do
    {
        QSPIFlash_singleRead(QSPIFlashHandle, tempFlashAddrRx, sizeof(tempData), (uint8_t*)&tempData);
        if(tempData != 0xFFFFFFFF)
            UART_writePolling(gMrrMSSMCB.loggingUartHandle, (uint8_t *)&tempData, sizeof(tempData));
        tempFlashAddrRx = tempFlashAddrRx + (uint32_t)sizeof(tempData);
    }while(tempFlashAddrRx <= 0xC07FEFFF);
    QSPIFlash_sectorErase(QSPIFlashHandle, LAST_DATA_LOG_ADDR);
    QSPIFlash_Write(LAST_DATA_LOG_ADDR, (uint8_t *)&flashAddr, sizeof(flashAddr));
    GPIO_write(SOC_XWR18XX_GPIO_18, 0); //  Power IND led off
    GPIO_write(SOC_XWR18XX_GPIO_12, 0); // Power Enable off

}
void QSPIFlash_Init (void)
{
    int32_t         retVal = 0;
    QSPI_Params     QSPIParams;

    /* Initialize the QSPI Driver */
    QSPI_init();

    /* Initialize the QSPI Flash */
    QSPIFlash_init();

    /* Open QSPI driver */
    QSPI_Params_init(&QSPIParams);

    /* Set the QSPI peripheral clock to 200MHz  */
    QSPIParams.qspiClk = 200 * 1000000U;

    /* Running at 40MHz QSPI bit rate*/
    QSPIParams.bitRate = 40 * 1000000U;

    QSPIParams.clkMode = QSPI_CLOCK_MODE_0;

    gQSPIDrv = QSPI_open(&QSPIParams, &retVal);

    if(gQSPIDrv == NULL)
    {
        printf("QSPI_open failed with error=%d\n", retVal);
        return;
    }


    QSPIFlashHandle = QSPIFlash_open(gQSPIDrv, &retVal);

    if (QSPIFlashHandle == NULL )
    {
        System_printf("QSPIFlash Open API ", MCPI_TestResult_FAIL);
        return;
    }
    else
    {

        System_printf("QSPIFlash Open API ", MCPI_TestResult_PASS);
    }

    return;
}


/**************************************************************************
 ********************** MSS MRR TI Design Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the ADC Driver which is invoked
 *      when a chirp is available. This is executed in the ISR context.
 *      
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_chirpIntCallback(uintptr_t arg)
{
    gMrrMSSMCB.chirpInt++;
    gMrrMSSMCB.chirpIntcumSum++;

    /* The different subframes in the MRR configuration have different chirp sizes. 
     * These need to be configured before the start of the next subframe, at the 
     * end of the previous subframes last chirp. */
    if (gMrrMSSMCB.chirpInt == gMrrMSSMCB.numChirpsPerSubframe[gMrrMSSMCB.subframeId]) 
    {
        gMrrMSSMCB.subframeCntFromChirpInt++;
        gMrrMSSMCB.subframeId = gMrrMSSMCB.subframeId + 1;
        if (gMrrMSSMCB.subframeId == NUM_SUBFRAMES)
        {
            gMrrMSSMCB.subframeId = 0;
        }
        gMrrMSSMCB.chirpInt = 0; 
    }
}

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the ADC Driver which is invoked
 *      when a frame is started. This is executed in the ISR context.
 *
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_frameStartIntCallback(uintptr_t arg)
{
    gMrrMSSMCB.frameStartToken++;
    gMrrMSSMCB.subframeCntFromFrameStart++;
    /* Check if the frames are coming correctly, and no chirps have been missed.         */
    if (gMrrMSSMCB.frameStartToken == 1) 
    {
        if (gMrrMSSMCB.chirpInt == 0)
        {
            gMrrMSSMCB.frameStartToken = 0;
        }
        else
        {
            //DebugP_assert (0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
static int32_t MRR_MSS_eventFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
    case RL_RF_ASYNC_EVENT_MSG:
    {
        /* Received Asychronous Message: */
        switch (asyncSB)
        {
        case RL_RF_AE_CPUFAULT_SB:
        {
            /* Post event to datapath task notify BSS events */
            //Event_post(gMrrMSSMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
            break;
        }
        case RL_RF_AE_ESMFAULT_SB:
        {
            /* Post event to datapath task notify BSS events */
            //Event_post(gMrrMSSMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
            break;
        }
        case RL_RF_AE_INITCALIBSTATUS_SB:
        {
            /* This event should be handled by mmwave internally, ignore the event here */
            break;
        }
        case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
        {
            break;
        }
        case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
        {
            break;
        }
        case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
        {
            break;
        }
        case RL_RF_AE_GPADC_MEAS_DATA_SB:
        {
            isGetGpAdcMeasData = 1U;
            memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));

            adc_val = rcvGpAdcData.sensor[3].avg;
            break;
        }

        default:
        {
            System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
            break;
        }
        }
        break;
    }
    default:
    {
        System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
        break;
    }
    }
    return 0;
}

/**
 *  @b DescriptionTask
 *  @n
 *      This is the application registered callback function which is invoked after
 *      the configuration has been used to configure the mmWave link and the BSS.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_cfgFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* The MRR TI Design operates in MINIMAL mode and so the configuration 
     * callback function is never invoked. The assertion will ensure that 
     * this is never invoked */
    //DebugP_assert (0);
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibrat    ion configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_startFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    /* Post an event to main data path task. 
       This function in only called when mmwave_start() is called on DSS */
    gMrrMSSMCB.stats.datapathStartEvt ++;

    /* Post event to start is done */
    Event_post(gMrrMSSMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
}

/**
 *  @b Description
 *  @n
 *      This is the application registered callback function which is invoked after the
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      mmWave link on BSS has been stopped.
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_stopFxn(void)
{
    /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS 
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMrrMSSMCB.stats.datapathStopEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_openFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    return;
}


/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_closeFxn(void)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
    int32_t                  retVal = -1;

    retVal = Mailbox_write (gMrrMSSMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from 
 *      Mailbox virtual channel.  
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    bool bAlertWarning = FALSE;
    bool bLEDWarning = FALSE;
    uint8_t count = 0;


    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gMrrMSSMCB.mboxSemHandle, BIOS_WAIT_FOREVER);
        powerBtn = GPIO_read(SOC_XWR18XX_GPIO_1);
        testBtn = GPIO_read(SOC_XWR18XX_GPIO_2);
        runBtn = GPIO_read(SOC_XWR18XX_GPIO_0);
        RTC_get_time(); // Working

        if(BatMonitorCounter > 4)
        {
            BatMonitorCounter = 0;
            if(lowBatteryFlag == 1)
            {
                GPIO_toggle(SOC_XWR18XX_GPIO_47); // R - Bat
                GPIO_write(SOC_XWR18XX_GPIO_23, 0); // G - Bat
                GPIO_write(SOC_XWR18XX_GPIO_31, 0); // B - Bat
            }
            BATTERY_charge_status();
        }
        if(runBtn == 1 && testBtn == 0)
        {
            selfTestFlag = 1;
        }
        seflTestFunction();
        rangeBtn = GPIO_read(SOC_XWR18XX_GPIO_38);
        if(rangeFlag == 1)
        {
            rangeRadarFlag ^= 1U;
            GPIO_toggle(SOC_XWR18XX_GPIO_20);
            rangeFlag = 0;
        }
        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMrrMSSMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMrrMSSMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {

            case MMWDEMO_DSS2MSS_DETOBJ_READY:
                /* Got detected objects , to be shipped out shipped out through UART */

            {
                if (selfTestFlag == 0)
                {
                    uint32_t itemIdx;
                    if(logDataCounter > 3)  // Every 500ms Store the Radar Data into Flash
                    {
                        logDataCounter = 0;
                        uint32_t flashAddrTx;
                        uint32_t totalSize = 0;
                        uint32_t paddingBytes = 0;
                        flashAddrTx = flashAddr;
                        totalSize = sizeof(rangeRadarFlag);
                        flashAddrTx = QSPIFlash_Write(flashAddrTx,(uint8_t*)&rangeRadarFlag, sizeof(rangeRadarFlag));
                        totalSize = totalSize + sizeof(lowBatteryFlag);
                        flashAddrTx = QSPIFlash_Write(flashAddrTx, (uint8_t *)&lowBatteryFlag, sizeof(lowBatteryFlag)); //  RTC Data Storing
                        totalSize = totalSize + sizeof(get_time);
                        flashAddrTx = QSPIFlash_Write(flashAddrTx, (uint8_t *)get_time, sizeof(get_time)); //  RTC Data Storing
                        totalSize = totalSize + sizeof(MmwDemo_output_message_header);
                        flashAddrTx = QSPIFlash_Write(flashAddrTx, (uint8_t *)&message.body.detObj.header, sizeof(MmwDemo_output_message_header)); // Radar Msg header
                        for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                        {

                            flashAddrTx = QSPIFlash_Write(flashAddrTx, (uint8_t *)&message.body.detObj.tlv[itemIdx], sizeof(MmwDemo_output_message_tl));
                            totalSize = totalSize + sizeof(MmwDemo_output_message_tl);
                            flashAddrTx = QSPIFlash_Write(flashAddrTx,(uint8_t *)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address, SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL),
                                                          message.body.detObj.tlv[itemIdx].length);
                            totalSize = totalSize + message.body.detObj.tlv[itemIdx].length;

                        }
                        paddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalSize & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                        if (paddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                        {
                            uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
                            memset(&padding, 0xf, MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                            flashAddrTx = QSPIFlash_Write(flashAddrTx, (uint8_t*)&padding, paddingBytes);
                            totalSize = totalSize + paddingBytes;
                        }
                        totalSizeWritten = totalSizeWritten + totalSize;
                        if(totalSizeWritten >= 4096) //  Check if 4096 bytes is written if it so erased next 4Killo bytes flash
                        {
                            totalSizeWritten = 0;
                            QSPIFlash_4KBFlashErase(flashAddrTx);
                        }
                        if(flashAddrTx >= FLASH_END_ADDR_8MB) // Check whether full flash is written if it so erase the flash and point to starting
                        {
                            flashAddrTx = LOG_DATA_ADDR;
                            QSPIFlash_4KBFlashErase(flashAddrTx);
                        }
                        flashAddr = flashAddrTx;

                    }
                    /* Send header */
#ifdef SERIAL_ENABLE
                    UART_writePolling (gMrrMSSMCB.loggingUartHandle, (uint8_t*)get_time, sizeof(get_time));
                    totalPacketLen = sizeof(MmwDemo_output_message_header) + sizeof(get_time);
                    UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                       (uint8_t*)&message.body.detObj.header,
                                       sizeof(MmwDemo_output_message_header));
#endif
                    /* Send TLVs */
                    for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                    {
#ifdef SERIAL_ENABLE
                        UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                           (uint8_t*)&message.body.detObj.tlv[itemIdx],
                                           sizeof(MmwDemo_output_message_tl));


                        UART_writePolling (gMrrMSSMCB.loggingUartHandle,
                                           (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address, SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                           message.body.detObj.tlv[itemIdx].length);


                        totalPacketLen += sizeof(MmwDemo_output_message_tl) + message.body.detObj.tlv[itemIdx].length;
#endif
                        if(((message.body.detObj.tlv[itemIdx].type & 0x000B) == 0x000B) && (bAlertWarning != TRUE) )
                        {
                            bAlertWarning = TRUE;
                            bLEDWarning = FALSE; //  vehicle in TTC
                            count = 40;
                        }
                        else if(((message.body.detObj.tlv[itemIdx].type & 0x0013) == 0x0013) && (bLEDWarning != TRUE) )
                        {
                            bLEDWarning = TRUE;             //not in collision course but in primary zone
                            bAlertWarning = FALSE;
                        }
                        else
                        {
                            bAlertWarning = FALSE;
                            bLEDWarning = FALSE;
                        }
                    }

                    /* Alert the System based on the TCC and Target in the collision Course */
                    if(bAlertWarning != FALSE)
                    {
                        if(va_LEDCounter > 1)
                        {
                            GPIO_toggle(SOC_XWR18XX_GPIO_34); //  V-Alert
                            va_LEDCounter = 0;
                        }
                        GPIO_write(SOC_XWR18XX_GPIO_37, 1); // Siren

                    }
                    else if(bLEDWarning != FALSE)
                    {
                        if(va_LEDCounter > 3)
                        {
                            GPIO_toggle(SOC_XWR18XX_GPIO_34);
                            va_LEDCounter = 0;
                        }
                        GPIO_write(SOC_XWR18XX_GPIO_37, 0);
                    }
                    else
                    {
                        if(va_LEDCounter > 7)
                        {
                            GPIO_toggle(SOC_XWR18XX_GPIO_34);
                            va_LEDCounter = 0;
                        }
                        if(count >= 1)
                        {
                            count--;
                            if(count == 0)
                            {
                                GPIO_write(SOC_XWR18XX_GPIO_37, 0);
                            }
                        }
                    }
#ifdef SERIAL_ENABLE
                    /* Send padding to make total packet length multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
                    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalPacketLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                    {
                        uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
                        /*DEBUG:*/ memset(&padding, 0xf, MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                        UART_writePolling (gMrrMSSMCB.loggingUartHandle, padding, numPaddingBytes);
                    }
#endif
                }
            }
            /* Send a message to MSS to log the output data */
            memset((void *)&message, 0, sizeof(MmwDemo_message));

            message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;
            message.body.dataLogger = (uint8_t)rangeRadarFlag;
            retVal = MmwDemo_mboxWrite(&message);

            if (retVal != 0)
            {
                System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
            }

            break;

            case MMWDEMO_DSS2MSS_ASSERT_INFO:
                /* Send the received DSS assert info through CLI */
//                CLI_write ("DSS Exception: %s, line %d.\n", message.body.assertInfo.file,
//                           message.body.assertInfo.line);
                break;
            default:
            {
                /* Message not support */
                System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                break;
            }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
        Mbox_Handle  handle,
        Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMrrMSSMCB.mboxSemHandle);
}


/**
 *  @b Description
 *  @n
 *      This is the task which provides an execution context for
 *      the mmWave control module.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_mmWaveCtrlTask (UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {

        /* Execute the mmWave control module: */
        if (MMWave_execute (gMrrMSSMCB.ctrlHandle, &errCode) < 0)
        {
            System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Callback function invoked when the GPIO switch is pressed.
 *      This is invoked from interrupt context.
 *
 *  @param[in]  index
 *      GPIO index configured as input
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_switchPressFxn(unsigned int index)
{
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MRR_MSS_mssProcessDataPathTask(UArg arg0, UArg arg1)
{
    UInt          event = 0;

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Input : Configure pin J13 as GPIO_1 input
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINJ13_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINJ13_PADAC, SOC_XWR18XX_PINJ13_PADAC_GPIO_1);
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINK13_PADAZ, SOC_XWR18XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the SW1 switch on the EVM connected to GPIO_1
     * - This is used as an input
     * - Enable interrupt to be notified on a switch press
     **********************************************************************/
    GPIO_setConfig (SOC_XWR18XX_GPIO_1, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING | GPIO_CFG_IN_INT_LOW);
    GPIO_setCallback (SOC_XWR18XX_GPIO_1, MRR_MSS_switchPressFxn);
    GPIO_enableInt (SOC_XWR18XX_GPIO_1);

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (SOC_XWR18XX_GPIO_2, GPIO_CFG_OUTPUT);

    System_printf ("Debug: Data Path management main loop. \n");
    /* Data Path management task Main loop */
    while (1)
    {
        event = Event_pend(gMrrMSSMCB.eventHandle,
                           Event_Id_NONE,
                           MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS,
                           BIOS_WAIT_FOREVER);

        /************************************************************************
         * BSS event:: CPU fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_CPUFAULT_EVT)
        {
            break;
        }

        /************************************************************************
         * BSS event:: ESM fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_ESMFAULT_EVT)
        {
            break;
        }
    }

    System_printf("Debug: MMWDemoDSS Data path exit\n");
}
void GPIO_allpins (void)
{
    GPIO_init();

    /*Power on enable*/
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP13_PADAA,PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP13_PADAA,SOC_XWR18XX_PINP13_PADAA_GPIO_12);
    GPIO_setConfig (SOC_XWR18XX_GPIO_12, GPIO_CFG_OUTPUT);


    /*FAULT LED */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PIND13_PADAD,PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PIND13_PADAD,SOC_XWR18XX_PIND13_PADAD_GPIO_19);
    GPIO_setConfig (SOC_XWR18XX_GPIO_19, GPIO_CFG_OUTPUT);


    /*Siren */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINR8_PADBL, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINR8_PADBL, SOC_XWR18XX_PINR8_PADBL_GPIO_37);
    GPIO_setConfig (SOC_XWR18XX_GPIO_37, GPIO_CFG_OUTPUT);

    /*  Battery R LED */

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN15_PADBV, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN15_PADBV, SOC_XWR18XX_PINN15_PADBV_GPIO_47);
    GPIO_setConfig (SOC_XWR18XX_GPIO_47, GPIO_CFG_OUTPUT);

    /*  Battery G LED */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINR11_PADAW, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINR11_PADAW, SOC_XWR18XX_PINR11_PADAW_GPIO_23);
    GPIO_setConfig (SOC_XWR18XX_GPIO_23, GPIO_CFG_OUTPUT);


    /* Battery B LED */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINR4_PADBF, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINR4_PADBF, SOC_XWR18XX_PINR4_PADBF_GPIO_31);
    GPIO_setConfig (SOC_XWR18XX_GPIO_31, GPIO_CFG_OUTPUT);

    /* Power IND LED */

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN10_PADAV, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN10_PADAV, SOC_XWR18XX_PINN10_PADAV_GPIO_18);
    GPIO_setConfig (SOC_XWR18XX_GPIO_18, GPIO_CFG_OUTPUT);

    /*   Status Ring R */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINR5_PADBH, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINR5_PADBH, SOC_XWR18XX_PINR5_PADBH_GPIO_33);
    GPIO_setConfig (SOC_XWR18XX_GPIO_33, GPIO_CFG_OUTPUT);

    /*  Status Ring G */

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP5_PADBG, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP5_PADBG, SOC_XWR18XX_PINP5_PADBG_GPIO_32);
    GPIO_setConfig (SOC_XWR18XX_GPIO_32, GPIO_CFG_OUTPUT);

    /*  Status Ring B LED*/

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINE15_PADAG, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINE15_PADAG, SOC_XWR18XX_PINE15_PADAG_GPIO_30);
    GPIO_setConfig (SOC_XWR18XX_GPIO_30, GPIO_CFG_OUTPUT);


    /*VA */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP6_PADBI, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP6_PADBI, SOC_XWR18XX_PINP6_PADBI_GPIO_34);
    GPIO_setConfig (SOC_XWR18XX_GPIO_34, GPIO_CFG_OUTPUT);


    /* Range Radar LED     */

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINE14_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINE14_PADAE, SOC_XWR18XX_PINE14_PADAE_GPIO_20);
    GPIO_setConfig (SOC_XWR18XX_GPIO_20, GPIO_CFG_OUTPUT);

    /*Power Input Button*/
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINJ13_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINJ13_PADAC, SOC_XWR18XX_PINJ13_PADAC_GPIO_1);
    GPIO_setConfig (SOC_XWR18XX_GPIO_1, GPIO_CFG_INPUT);


    /*Range Button */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP8_PADBM, SOC_XWR18XX_PINP8_PADBM_GPIO_38);
    GPIO_setConfig (SOC_XWR18XX_GPIO_38, GPIO_CFG_INPUT);

    /* Run Button */

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINH13_PADAB, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINH13_PADAB, SOC_XWR18XX_PINH13_PADAB_GPIO_0);
    GPIO_setConfig (SOC_XWR18XX_GPIO_0, GPIO_CFG_INPUT);

    /* Test button     */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINK13_PADAZ, SOC_XWR18XX_PINK13_PADAZ_GPIO_2);
    GPIO_setConfig (SOC_XWR18XX_GPIO_2, GPIO_CFG_INPUT);

    GPIO_write(SOC_XWR18XX_GPIO_34, 0);
    GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
    GPIO_write(SOC_XWR18XX_GPIO_19, 0);
    GPIO_write(SOC_XWR18XX_GPIO_31, 0);
    GPIO_write(SOC_XWR18XX_GPIO_47, 0);
    GPIO_write(SOC_XWR18XX_GPIO_23, 0);
    GPIO_write(SOC_XWR18XX_GPIO_33, 0); // R Status Ring
    GPIO_write(SOC_XWR18XX_GPIO_32, 0); // G Status Ring
    GPIO_write(SOC_XWR18XX_GPIO_30, 0); // B Status Ring
    GPIO_write(SOC_XWR18XX_GPIO_20, 1);
}

void seflTestFunction()
{

    if(selfTestFlag)
    {
        if (selfTestCounter <= 4)
        {
            GPIO_write(SOC_XWR18XX_GPIO_30, 1); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE

            GPIO_write(SOC_XWR18XX_GPIO_34, 1);
            GPIO_write(SOC_XWR18XX_GPIO_37, 1); // Siren
        }
        else if (selfTestCounter > 4 && selfTestCounter <= 8 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE

            GPIO_write(SOC_XWR18XX_GPIO_34, 0);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 8 && selfTestCounter <= 12 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 1); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE

            GPIO_write(SOC_XWR18XX_GPIO_34, 1);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 12 && selfTestCounter <= 16 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE
            GPIO_write(SOC_XWR18XX_GPIO_34, 0);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 16 && selfTestCounter <= 20 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 1); //BLUE


            GPIO_write(SOC_XWR18XX_GPIO_34, 1);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 20 && selfTestCounter <= 24 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE

            GPIO_write(SOC_XWR18XX_GPIO_34, 0);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 24 && selfTestCounter <= 28 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_32, 1);
            GPIO_write(SOC_XWR18XX_GPIO_33, 1);
            GPIO_write(SOC_XWR18XX_GPIO_30, 1);

            GPIO_write(SOC_XWR18XX_GPIO_34, 1);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren
        }
        else if (selfTestCounter > 28 && selfTestCounter <= 32 )
        {

            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //RED
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //Green
            GPIO_write(SOC_XWR18XX_GPIO_33, 0); //BLUE

            GPIO_write(SOC_XWR18XX_GPIO_34, 0);
            GPIO_write(SOC_XWR18XX_GPIO_37, 0); // Siren

        }
        else if (selfTestCounter > 32)
        {
            GPIO_write(SOC_XWR18XX_GPIO_32, 1); //Green
            selfTestCounter = 0;
            selfTestFlag = 0;
        }

    }

}
void timerISR(UArg arg0)
{
    bootTimeCounter++;
    if(bootFlag)
    {
        logDataCounter++;
        BatMonitorCounter++;
        va_LEDCounter++;
        if(runBtn == 0 && testBtn == 0 && ++dataExtractCounter > 15)
        {
            flashAddr = QSPIFlash_Write(flashAddr, (uint8_t*)endOfFrame, strlen(endOfFrame));
            GPIO_write(SOC_XWR18XX_GPIO_32, 0); //BLUE);
            GPIO_write(SOC_XWR18XX_GPIO_30, 0); //BLUE);
            GPIO_write(SOC_XWR18XX_GPIO_33, 1); //BLUE);

            processFlashDataThroughUart();
        }
        if(testBtn == 1 || runBtn == 1)
            dataExtractCounter = 0;
        if(lowBatteryFlag == 1)
        {
            lowBatteryCounter++;
        }
        if(rangeBtn == 0 && ++rangeCounter > 2)
        {
            rangeFlag = 1;
            rangeCounter = 0;
        }
        if(rangeBtn == 1)
            rangeCounter = 0;
        /*2 Sec power off condition */
        if(powerBtn == 1 && ++powerOffCounter > 15)
        {
            flashAddr = QSPIFlash_Write(flashAddr, (uint8_t*)endOfFrame, strlen(endOfFrame));
            QSPIFlash_sectorErase(QSPIFlashHandle, LAST_DATA_LOG_ADDR);
            QSPIFlash_Write(LAST_DATA_LOG_ADDR, (uint8_t *)&flashAddr, sizeof(flashAddr));
            GPIO_write(SOC_XWR18XX_GPIO_18, 0); //  Power IND led off
            GPIO_write(SOC_XWR18XX_GPIO_12, 0); // Power Enable off
            powerOffCounter = 0;
        }
        if(powerBtn == 0)
        {
            powerOffCounter = 0;
            powerOffFlag = 0;
        }
        if(selfTestFlag == 1)
        {
            selfTestCounter++;
        }
    }
}
/**
 *  @b Description
 *  @n
 *      MSS Initialization Task which initializes the various
 *      components in the MSS subsystem.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_MSS_initTask (UArg arg0, UArg arg1)
{
    int32_t                 errCode;
    MMWave_InitCfg          initCfg;
    UART_Params             uartParams;
    Task_Params                taskParams;
    SOC_SysIntListenerCfg   listenerCfg;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN5_PADBE, SOC_XWR18XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINN4_PADBD, SOC_XWR18XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINP10_PADAU, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINP10_PADAU, SOC_XWR18XX_PINP10_PADAU_MSS_UARTB_TX);


    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Open UART Driver: */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = MSS_SYS_VCLK;
    uartParams.baudRate       = 115200;
    uartParams.isPinMuxDone   = 1;

    /* Open the Command UART Instance */
    gMrrMSSMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMrrMSSMCB.commandUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = MSS_SYS_VCLK;
    uartParams.baudRate       = 115200*8;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMrrMSSMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMrrMSSMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }
    while(bootTimeCounter <= 16);
    GPIO_write(SOC_XWR18XX_GPIO_12, 1);
    GPIO_write(SOC_XWR18XX_GPIO_18, 1); //  Power IND led ON
    flashAddr = QSPIFlash_Write(flashAddr, (uint8_t*)startOfFrame, strlen(startOfFrame));
    selfTestFlag = 1;
    bootFlag = 1;
    i2c_pins_init();
    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMrrMSSMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMrrMSSMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMrrMSSMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 24*1024;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /* Register Chirp Available Listener */
    memset ((void*)&listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
    listenerCfg.systemInterrupt   = SOC_XWR18XX_MSS_CHIRP_AVAIL_IRQ;
    listenerCfg.listenerFxn       = MRR_MSS_chirpIntCallback;
    listenerCfg.arg               = 0;
    gMrrMSSMCB.chirpIntHandle = SOC_registerSysIntListener (gMrrMSSMCB.socHandle, &listenerCfg, &errCode);
    if (gMrrMSSMCB.chirpIntHandle == NULL)
    {
        System_printf ("Error: Unable to register the Chirp Available Listener [Error code %d]\n", errCode);
        return;
    }

    /* Register Frame Start Listener */
    memset ((void*)&listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
    listenerCfg.systemInterrupt   = SOC_XWR18XX_MSS_FRAME_START_INT;
    listenerCfg.listenerFxn       = MRR_MSS_frameStartIntCallback;
    listenerCfg.arg               = 0;
    gMrrMSSMCB.frameStartIntHandle = SOC_registerSysIntListener (gMrrMSSMCB.socHandle, &listenerCfg, &errCode);

    if (gMrrMSSMCB.frameStartIntHandle == NULL)
    {
        System_printf("Error: Unable to register the Frame start Listener [Error code %d]\n", errCode);
        return ;
    }

    /*****************************************************************************
     * Initialize the mmWave module:
     *****************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Populate the initialization configuration:
     * The MMWAve is configured in minimal isolation mode. */
    initCfg.domain                      = MMWave_Domain_MSS;
    initCfg.socHandle                   = gMrrMSSMCB.socHandle;
    initCfg.eventFxn                    = &MRR_MSS_eventFxn;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_MINIMAL;
    initCfg.executionMode               = MMWave_ExecutionMode_ISOLATION;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cooperativeModeCfg.cfgFxn   = &MRR_MSS_cfgFxn;
    initCfg.cooperativeModeCfg.startFxn = &MRR_MSS_startFxn;
    initCfg.cooperativeModeCfg.stopFxn  = &MRR_MSS_stopFxn;
    initCfg.cooperativeModeCfg.openFxn  = &MRR_MSS_openFxn;
    initCfg.cooperativeModeCfg.closeFxn = &MRR_MSS_closeFxn;

    /* Initialize and setup the mmWave Control module */
    gMrrMSSMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMrrMSSMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Initialized the mmWave module\n");

    /*****************************************************************************
     * Synchronize the mmWave module:
     *****************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMrrMSSMCB.ctrlHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

    System_printf ("Debug: Synchronized the mmWave module\n");

#ifdef SUBFRAME_CONF_MRR_USRR
    gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
    gMrrMSSMCB.numChirpsPerSubframe[1] = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
#else
#ifdef SUBFRAME_CONF_MRR
    gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
#endif
#ifdef SUBFRAME_CONF_USRR
    gMrrMSSMCB.numChirpsPerSubframe[0] = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
#endif
#endif

#ifdef USE_LVDS_INTERFACE_FOR_OBJECT_DATA_TX
    /* Configure the LVDS streaming interface.*/
    {
        int32_t streamConfiguration = MRR_MSS_configureStreaming() ;

        if (streamConfiguration != 0)
        {
            System_printf ("Error: Unable to activate the Streaming Session [Error code %d]\n", streamConfiguration);

            DebugP_assert (0);
        }
    }
#endif

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 6;
    taskParams.stackSize = 3*1024;
    Task_create(MRR_MSS_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Setup the CLI
     *****************************************************************************/
    MRR_MSS_CLIInit ();
}

/**
 *  @b Description
 *  @n
 *      Entry point into the MSS TI Design
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    Task_Params        taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);
    /*
     * Initialization of All GPIO pins
     */
    GPIO_allpins();

    /* Initialize the global variables */
    memset ((void*)&gMrrMSSMCB, 0, sizeof(Mrr_MSS_MCB));

    /* Initialize the SOC configuration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMrrMSSMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gMrrMSSMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Check if the SOC is a secure device */
    if (SOC_isSecureDevice(gMrrMSSMCB.socHandle, &errCode))
    {
        /* Disable firewall for JTAG and LOGGER (UART) which is needed by the demo */
        SOC_controlSecureFirewall(gMrrMSSMCB.socHandle,
                                  (uint32_t)(SOC_SECURE_FIREWALL_JTAG | SOC_SECURE_FIREWALL_LOGGER),
                                  SOC_SECURE_FIREWALL_DISABLE,
                                  &errCode);
    }
    /*  QSPI Flash Init  */
    QSPIFlash_Init();

    /*  Get Base Address of Flash Adsress  */
    flashAddr = QSPIFlash_getExtFlashAddr(QSPIFlashHandle); // 0xC0000000

    QSPIFlash_singleRead(QSPIFlashHandle, (uint32_t)LAST_DATA_LOG_ADDR, sizeof(lastLogDataPointer), (uint8_t*)&lastLogDataPointer);
    if(lastLogDataPointer == 0xFFFFFFFF)
    {
        flashAddr = flashAddr + LOG_DATA_OFFSET; // 0xC0000000 +
    }
    else
    {
        flashAddr = lastLogDataPointer;
    }
    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MRR_MSS_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

#ifdef USE_LVDS_INTERFACE_FOR_OBJECT_DATA_TX

/**
 *  @b Description
 *  @n
 *      The function is used to configure the CBUFF/LVDS Streaming
 *
 *  @param[in]  ptrDataPathObj
 *      Handle to data Path object
 *
 *  @retval
 *      Success  - 0
 *  @retval
 *      Error    - <0
 */
static int32_t MRR_MSS_configureStreaming()
{
    CBUFF_InitCfg       initCfg;
    CBUFF_SessionCfg    sessionCfg;
    rlDevHsiClk_t       hsiClkgs;
    int32_t             errCode;
    int32_t             retVal = -1;

    /* Initialize the CBUFF Initialization configuration: */
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.socHandle                = gMrrMSSMCB.socHandle;
    initCfg.enableECC                = 0U;
    initCfg.crcEnable                = 1U;
    initCfg.maxSessions              = 1U;
    initCfg.enableDebugMode          = false;
    initCfg.interface                = CBUFF_Interface_LVDS;
    initCfg.u.lvdsCfg.crcEnable      = 0U;
    initCfg.u.lvdsCfg.msbFirst       = 1U;
    initCfg.u.lvdsCfg.lvdsLaneEnable = 0x3U;
    initCfg.u.lvdsCfg.ddrClockMode   = 1U;
    initCfg.u.lvdsCfg.ddrClockModeMux= 1U;


    /*****************************************************************************
     * Initialize EDMA driver
     *****************************************************************************/
    if (MRR_MSS_initEDMA(CBUFF_EDMA_INSTANCE) < 0)
    {
        return -1;
    }

    /*****************************************************************************
     * Open EDMA driver:
     *****************************************************************************/
    if( (gMrrMSSMCB.dmaHandle = MRR_MSS_openEDMA(CBUFF_EDMA_INSTANCE)  )== NULL)
    {
        return -2;
    }

    /* Translate the data format from the mmWave link to the CBUFF: */
    /* 16bit Data Output Format: */
    initCfg.outputDataFmt = CBUFF_OutputDataFmt_16bit;

    /* Initialize the CBUFF Driver: */
    gMrrMSSMCB.cbuffHandle = CBUFF_init (&initCfg, &errCode);
    if (gMrrMSSMCB.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF; report the error */
        System_printf ("Error: CBUFF Driver initialization failed [Error code %d]\n", errCode);
        goto exit;
    }


    /********************************************************************************
     * Software Triggered Session:
     ********************************************************************************/
    {
        /* Initialize the session configuration: */
        memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));

        /* Populate the configuration: */
        sessionCfg.executionMode                     = CBUFF_SessionExecuteMode_SW;
        sessionCfg.edmaHandle                        = gMrrMSSMCB.dmaHandle;
        sessionCfg.allocateEDMAChannelFxn            = MRR_MSS_EDMAAllocateCBUFFChannel;
        sessionCfg.freeEDMAChannelFxn                = MRR_MSS_EDMAFreeCBUFFChannel;
        sessionCfg.frameDoneCallbackFxn              = NULL;
        sessionCfg.dataType                          = CBUFF_DataType_REAL;
        sessionCfg.u.swCfg.headerMode                = CBUFF_HeaderMode_NONE;
        sessionCfg.u.swCfg.userBufferInfo[0].size    = sizeof(gSwUserBuffer)/2;
        sessionCfg.u.swCfg.userBufferInfo[0].address = (uint32_t)&gSwUserBuffer[0];

        /* Create the session: */
        gMrrMSSMCB.swSessionHandle = CBUFF_createSession (gMrrMSSMCB.cbuffHandle, &sessionCfg, &errCode);
        if (gMrrMSSMCB.swSessionHandle == NULL)
        {
            System_printf ("Error: Unable to create the CBUFF Session [Error code %d]\n", errCode);
            retVal = -3;
            goto exit;
        }
    }

    /*************************************************************************************
     * Setup the HSI Clock through the mmWave Link:
     *************************************************************************************/
    memset ((void*)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));

    /* Setup the HSI Clock as per the Radar Interface Document: This is set to 600Mhtz DDR Mode */
    hsiClkgs.hsiClk = 0x9;

    /* Setup the HSI in the radar link: */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1, &hsiClkgs);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Unable to set the HSI clock */
        System_printf ("Error: Setting up the HSI Clock in BSS Failed [Error %d]\n", retVal);
        goto exit;
    }

    {
        /******************************************************************************
         * ----------------------------------------------------------------------------
         * Use Case 3: Hardware sessions is DISABLED and Software session is ENABLED
         * ----------------------------------------------------------------------------
         *    1) Activate SW Session
         *    2) Frame Done on SW Session
         *       -> Do nothing.
         *  This is to simulate a use case where only the application debug data is
         *  being streamed out. So we kaunch the software session trigger task which
         *  will send out the data periodically. Activate the software session
         ******************************************************************************/
        if (CBUFF_activateSession (gMrrMSSMCB.swSessionHandle, &errCode) < 0)
        {
            System_printf ("Error: Unable to activate the CBUFF Session [Error code %d]\n", errCode);
            retVal = -4;
            goto exit;
        }


    }

    /* Setup the return value as the CBUFF streaming has been configured successfully */
    retVal = 0;

    exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function which allocates
 *      EDMA channels for HW Triggered sessions
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MRR_MSS_EDMAAllocateCBUFFChannel (CBUFF_EDMAInfo* ptrEDMAInfo, CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    /* Use the DMA Information to perform the CBUFF EDMA allocations */
    if (ptrEDMAInfo->dmaNum == 0U)
    {
        ptrEDMACfg->chainChannelsId = MRR_CBUFF_EDMA_CH;
        ptrEDMACfg->shadowLinkChannelsId = MRR_CBUFF_EDMA_SHADOW_CH;
    }
    else
    {
        /* Error: MRR only supports a single session (i.e. DMA-0)*/
        DebugP_assert (0);
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees allocated
 *      EDMA channels for HW Triggered sessions
 *
 *  @retval
 *      Not applicable
 */
static void MRR_MSS_EDMAFreeCBUFFChannel(CBUFF_EDMAChannelCfg* ptrEDMACfg)
{

    return;

}


/**
 *  @b Description
 *  @n
 *      Open EDMA driver Instance for ADCBUF data buffer copy.
 *
 *  @param[in]  instance
 *      EDMA driver instance.
 *
 *  @retval
 *      Success     - EDMA handle
 *      Fail        - NULL pointer
 */
static EDMA_Handle MRR_MSS_openEDMA(uint8_t instance)
{
    EDMA_errorInfo_t              EDMAErrorInfo;
    EDMA_transferControllerErrorInfo_t EDMATransferControllerErrorInfo;
    EDMA_Handle                   EdmaHandle = NULL;
    EDMA_instanceInfo_t           instanceInfo;
    EDMA_errorConfig_t            errorConfig;
    int32_t                       retVal = 0;

    memset(&EDMAErrorInfo, 0, sizeof(EDMAErrorInfo));
    memset(&EDMATransferControllerErrorInfo, 0, sizeof(EDMATransferControllerErrorInfo));

    /* Open the EDMA Instance */
    EdmaHandle = EDMA_open(instance, &retVal, &instanceInfo);
    if (EdmaHandle == NULL)
    {
        System_printf("Error: Unable to open the edma Instance(%d), errorCode = %d\n",instance, retVal);
        return NULL;
    }

    /* Configurate EDMA Error Monitor */
    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = MRR_MSS_edmaErrorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MRR_MSS_edmaTransferControllerErrorCallbackFxn;
    if ((retVal = EDMA_configErrorMonitoring(EdmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        System_printf("Debug: EDMA_configErrorMonitoring() failed with errorCode = %d\n", retVal);
        return NULL;
    }

    return EdmaHandle;
}


/**
 *  @b Description
 *  @n
 *      EDMA driver instance Initialization. Application is responsible for EDMA instance
 *  management.
 *
 *  @param[in]  instance
 *      EDMA driver instance.
 *
 *  @retval
 *      Success     - 0
 *      Fail        - -1
 */
int32_t MRR_MSS_initEDMA(uint8_t instance)
{
    int32_t    retVal = 0;

    retVal = EDMA_init(instance);
    if (retVal != EDMA_NO_ERROR)
    {
        System_printf ("Debug: EDMA instance %d initialization returned error %d\n", instance, retVal);
        return -1;
    }
    System_printf ("Debug: EDMA instance %d has been initialized\n", instance);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 *
 *  @param[in]  handle
 *      EDMA driver instance Handle
 *  @param[in]  errorInfo
 *      EDMA Channle id used to copy data buffer
 *
 *  @retval
 *      None
 */
static void MRR_MSS_edmaErrorCallbackFxn
(
        EDMA_Handle         handle,
        EDMA_errorInfo_t*   errorInfo
)
{
    /* EDMA CC Error reported, Assert ? */
    DebugP_assert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 *
 *  @param[in]  handle
 *      EDMA driver instance Handle
 *  @param[in]  errorInfo
 *      EDMA Channle id used to copy data buffer
 *
 *  @retval
 *      None
 */
static void MRR_MSS_edmaTransferControllerErrorCallbackFxn
(
        EDMA_Handle                         handle,
        EDMA_transferControllerErrorInfo_t* errorInfo
)
{
    /* EDMA TC Error reported, Assert ? */
    DebugP_assert(0);
}
#endif
