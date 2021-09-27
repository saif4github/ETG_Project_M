
#include "../BSP/Inc/rtc.h"
#include <stdio.h>
#include <ti/drivers/i2c/I2C.h>
/*Enable RTC after configuring pin SOC_XWR18XX_PINF13_PADAH and SOC_XWR18XX_PING14_PADAI as I2C functionality */

/*temporary buffer for RTC registers */
static uint8_t txBuffer[3];
static uint8_t rxBuffer[2];


/*Array to store timestamp*/
uint8_t get_time[NO_OF_REG] = {0};  // 64 bytes
uint8_t get_lux[2] = {0};

/*Register address of RTC registers*/
uint8_t RTC_reg_add_buff[NO_OF_REG] = {SEC, MINUTE, HOUR, DOW, DOM, MONTH, YEAR};
uint8_t Amibent[2] = {0,0};

/*I2C driver configuration */
extern I2C_Transaction i2cTransaction;
extern I2C_Handle      i2cHandle;
extern I2C_Params      i2cParams;
extern volatile float Lux;
//extern volatile uint8_t Lux2;

extern volatile  uint8_t slaveaddr[];

/*
 * @brief - Function to Check i2C handle
 *
 * @return    value = 0 - success;
 *            Value < 0 - I2C handle error
 */
int32_t RTC_init(void)
{
    if (i2cHandle == NULL)
        return RTC_FAILURE;
    else
        return RTC_SUCCESS;
}

/*
 * @brief - Function to convert decimal to bcd for RTC registers
 *
 * @return    value >= 0;
 */
uint8_t RTC_decToBcd(int val)
{
    return (uint8_t)( (val/10*16) + (val%10) );
}


/*
 * @brief - Function to convert bcd to decimal for reading timestamp in human readable format
 *
 * @return    value >= 0;
 */
uint8_t RTC_bcdToDec(uint8_t val)
{
    return (int)( (val/16*10) + (val%16) );
}


/*
 * @brief - Function to get timestamp parameters from RTC
 *
 * @return    Value = 0 - in case of error
 *            Value = 1 - success
 */
bool RTC_get_time()
{
    bool retVal = 0;

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = RTC_reg_add_buff;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = get_time;
    i2cTransaction.readCount = NO_OF_REG;

    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == FALSE)
        return retVal;
    get_time[SEC] = RTC_bcdToDec(get_time[SEC] & 0x7f); // 1000 0000

    get_time[MINUTE] = RTC_bcdToDec(get_time[MINUTE]);

    get_time[HOUR] = RTC_bcdToDec(get_time[HOUR]);

    get_time[DOW] = RTC_bcdToDec(get_time[DOW]);

    get_time[DOM] = RTC_bcdToDec(get_time[DOM]);

    get_time[MONTH] = RTC_bcdToDec(get_time[MONTH]);

    get_time[YEAR] = RTC_bcdToDec(get_time[YEAR]);

    return retVal;
}
/*
 * @brief - Function to set time stamp parameters from RTC
 *
 * @return    Value = 0 - in case of error
 *            Value = 1 - success
 */
bool RTC_set_time(uint8_t sec, uint8_t min, uint8_t hour,uint8_t dow, uint8_t dom, uint8_t month, uint8_t year )
{
    uint8_t retVal = 0;

    /*  Seconds */
    txBuffer[0] = SEC;
    txBuffer[1] = RTC_decToBcd(sec);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;

    /*end of Seconds */

    /*Minute */
    txBuffer[0] = MINUTE;
    txBuffer[1] = RTC_decToBcd(min);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;


    /*end of Minute */


    /*Hour */
    txBuffer[0] = HOUR;
    txBuffer[1] = RTC_decToBcd(hour);
    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;

    /*end of Hour */


    /*Day Of the Week */
    txBuffer[0] = DOW;
    txBuffer[1] = RTC_decToBcd(dow);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;

    /*end of DOW */

    /*Day Of the Month */
    txBuffer[0] = DOM;
    txBuffer[1] = RTC_decToBcd(dom);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;


    /*end of dom */

    /*Month */
    txBuffer[0] = MONTH;
    txBuffer[1] = RTC_decToBcd(month);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;


    /*end of Month */

    /*Year */
    txBuffer[0] = YEAR;
    txBuffer[1] = RTC_decToBcd(year);

    i2cTransaction.slaveAddress = SLAVE_ADDR_RTC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;

    return retVal;

    /*end of Year */

}
bool Ambient_init()
{
    bool retVal = 0;

    txBuffer[0] = 0x01;
    txBuffer[1] = 0xC6;
    txBuffer[2] = 0x10;

    i2cTransaction.slaveAddress = 0x45;;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0 ;
    /* Writing to slave address */
    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == false)
        return retVal;

    return retVal;

}


bool get_ambient_lux()
{
    bool retVal = 0;
    Amibent[0] = 0x00;

    i2cTransaction.slaveAddress = 0x45;
    i2cTransaction.writeBuf = Amibent;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = get_lux;
    i2cTransaction.readCount = 2;

    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    if(retVal == FALSE)
        return retVal;

    int Lux_raw = (get_lux[0] << 8) | get_lux[1];

        float iMantissa = Lux_raw & 0x0FFF;                 // Extract Mantissa
        float iExponent = (Lux_raw & 0xF000) >> 12;         // Extract Exponent
        Lux =   iMantissa * (0.01 * pow(2, iExponent));

    return retVal;

}




