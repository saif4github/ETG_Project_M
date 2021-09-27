/*
 * rtc.h
 *
 */

#ifndef BOARD_INCLUDE_RTC_H_
#define BOARD_INCLUDE_RTC_H_


#include "BSP/Inc/i2c_driver.h"
#include <math.h>
#include <string.h>


/*Register Address for ds3231 */
#define         SEC                 (0U)
#define         MINUTE              (1U)
#define         HOUR                (2U)
#define         DOW                 (3U)
#define         DOM                 (4U)
#define         MONTH               (5U)
#define         YEAR                (6U)
#define         NO_OF_REG           (7U)

#define         RTC_SUCCESS         0U
#define         RTC_FAILURE         (-1)


#define         RTC_ENABLE          1U
#define         RTC_DISABLE         0U

#define         SLAVE_ADDR_RTC      0x6F

/*
 * @brief - Function to Che1ck i2C handle
 *
 * @return    value = 0 - success;
 *            Value < 0 - I2C handle error
 */
int32_t RTC_init(void);

/*
 * @brief - Function to set time stamp parameters from RTC
 *
 * @return    Value = 0 - in case of error
 *            Value = 1 - success
 */
bool RTC_set_time(uint8_t sec, uint8_t min, uint8_t hour,uint8_t dow, uint8_t dom, uint8_t mon, uint8_t year);

/*
 * @brief - Function to get timestamp parameters from RTC
 *
 * @return    Value = 0 - in case of error
 *            Value = 1 - success
 */
bool RTC_get_time(void);

/*
 * @brief - Function to convert bcd to decimal for reading timestamp in human readable format
 *
 * @return    value >= 0;
 */
uint8_t RTC_bcdToDec(uint8_t val);

/*
 * @brief - Function to convert decimal to bcd for RTC registers
 *
 * @return    value >= 0;
 */
uint8_t RTC_decToBcd(int val);
//
//static int32_t i2cReadAddress(I2C_Handle i2cHandle, uint8_t i2cSlaveAddress, uint8_t i2cSlaveRegAddress);
//
//int32_t i2cScanModeTest();
bool Ambient_init();


bool get_ambient_lux();

#endif /* BOARD_INCLUDE_RTC_H_ */
