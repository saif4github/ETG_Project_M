/*
 *   @file  battery.c
 *
 *   @brief
 *      The file contains common functions which test the mmWave Link API
 *
 *   */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include "BSP/Inc/battery.h"
#include <stdint.h>
#include <xdc/runtime/System.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_driver.h>
#include <ti/sysbios/knl/Task.h>


#define MAX_SAMPLE 10
uint8_t lowBatteryFlag = 0;
/*I2C driver configuration */
extern I2C_Transaction i2cTransaction;
extern I2C_Handle      i2cHandle;
extern I2C_Params      i2cParams;

/*@ Description
 * global declarations
 */
volatile rlUInt8_t isGetGpAdcMeasData = 0U;
rlRecvdGpAdcData_t rcvGpAdcData;


uint16_t largest(uint16_t arr[], uint8_t n)
{
    uint8_t i;
    // Initialize maximum element
    uint16_t max = arr[0];

    // Traverse array elements from second and
    // compare every element with current max
    for (i = 1; i < n; i++)
        if (arr[i] > max)
            max = arr[i];
    return max;
}


/*@ Description
 *         This structure used to Configure number of sample to be collected
 *
 */
const rlGpAdcCfg_t gpAdcCfg =
{
 .enable = 0x3F,
 .bufferEnable = 0x3F,
 .numOfSamples[0].sampleCnt = 20,
 .numOfSamples[0].settlingTime = 3,
 .numOfSamples[1].sampleCnt = 14,
 .numOfSamples[1].settlingTime = 3,
 .numOfSamples[2].sampleCnt = 14,
 .numOfSamples[2].settlingTime = 3,
 .numOfSamples[3].sampleCnt = 14,
 .numOfSamples[3].settlingTime = 3,
 .numOfSamples[4].sampleCnt = 14,
 .numOfSamples[4].settlingTime = 3,
 .numOfSamples[5].sampleCnt = 14,
 .numOfSamples[5].settlingTime = 3,
 .reserved0 = 0,
 .reserved1[0] = 0,
 .reserved1[1] = 0,
 .reserved1[2] = 0
};


/**
 *  @ Description
 *      The function is used to Configure GP ADC data parameters and read data
 *      using the mmWave link API.
 *
 *  @ retval
 *      Success -   0
 *  @ retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setGpAdcConfig (void)
{
    int32_t         retVal;


    for(int i = 0; i < 1; i++)
    {
        /* Set GPADC configuration */
        retVal = rlSetGpAdcConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlGpAdcCfg_t*)&gpAdcCfg);

        /* Check for mmWaveLink API call status */
        if(retVal != 0)
        {
            /* Error: Link reported an issue. */
            System_printf("Error: rlSetGpAdcConfig retVal=%d\n", retVal);
            return -1;
        }

        System_printf("Debug: Finished rlSetGpAdcConfig\n");

        if(isGetGpAdcMeasData == 0)
        {
            /* Sleep and poll again: */
            Task_sleep(1);

        }
        isGetGpAdcMeasData=0;

    }
    return 0;
}
/*@ Description
 *        This Function used to read external voltage
  @ retval
 *      Success -   0
 *  @ retval
 *      Error   -   <0
 */
int32_t BATTERY_charge_status(void)
{
    uint32_t batt_value = 0;
    static uint16_t battArr[MAX_SAMPLE] = {0};
    static uint8_t i = 0;

    /****GPADC configuration call****/

    if (MmwaveLink_setGpAdcConfig() < 0)
    {
        System_printf ("Chris Error: MmwaveLink_setGpAdcConfig\n");
        return -1;
    }

    /**Storing Battery value**/
    if(i == MAX_SAMPLE)
        i = 0;
    battArr[i] = rcvGpAdcData.sensor[RL_SENSOR_ANALOGTEST_FOUR].avg;
    i++;
    if(i == MAX_SAMPLE)
    {
        batt_value = largest(battArr, MAX_SAMPLE);
        /**Battery charge less than or equal to 20%**/

        if(batt_value <= BATT_20)
        {
            lowBatteryFlag = 1;
            lowBatteryStatusWrite(35);
        }
        /**Battery charge greater than 20% and less than or equal to 30%**/
        else if((batt_value > BATT_20) && (batt_value <= BATT_30))
        {
            lowBatteryFlag = 2;
            lowBatteryStatusWrite(127);
            GPIO_write(SOC_XWR18XX_GPIO_47, 1); // R - Bat
            GPIO_write(SOC_XWR18XX_GPIO_23, 0); // G - Bat
            GPIO_write(SOC_XWR18XX_GPIO_31, 0); // B - Bat


        }
        /**Battery charge greater than 30% and less than or equal to 50%**/
        else if((batt_value > BATT_30) && (batt_value <= BATT_50))
        {
            lowBatteryStatusWrite(127);
            GPIO_write(SOC_XWR18XX_GPIO_47, 0); // R - Bat
            GPIO_write(SOC_XWR18XX_GPIO_23, 0); // G - Bat
            GPIO_write(SOC_XWR18XX_GPIO_31, 1); // B - Bat
            lowBatteryFlag = 3;
        }
        /**Battery charge greater than 50% and less than or equal to 100%**/
        else if((batt_value > BATT_50) && (batt_value <= BATT_100))
        {
            lowBatteryStatusWrite(127);
            GPIO_write(SOC_XWR18XX_GPIO_47, 0); // R - Bat
            GPIO_write(SOC_XWR18XX_GPIO_23, 1); // G - Bat
            GPIO_write(SOC_XWR18XX_GPIO_31, 0); // B - Bat
            lowBatteryFlag = 4;
        }
    }
    return 0;
}
bool lowBatteryStatusWrite(uint8_t potValue)
{
    bool retVal;
    uint8_t tempBuf[2] = {0};
    tempBuf[0] = 0x00;
    tempBuf[1] = potValue;
    i2cTransaction.slaveAddress = DIGI_POT_ADDR;
    i2cTransaction.writeBuf = &tempBuf;
    i2cTransaction.writeCount = 2;
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == FALSE)
        return retVal;
    return 1;
}
