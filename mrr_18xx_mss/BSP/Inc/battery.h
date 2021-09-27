/*
 * battery.h
 *
 *  Created on: 16-Aug-2021
 *      Author: dubey
 */

#ifndef BOARD_INCLUDE_BATTERY_H_
#define BOARD_INCLUDE_BATTERY_H_



#ifdef __cplusplus
extern "C" {
#endif

#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/drivers/i2c/I2C.h>
#include <ti/drivers/gpio/gpio.h>
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
extern volatile uint32_t runMode;
extern volatile uint32_t setupMode;
extern int32_t MmwaveLink_setGpAdcConfig (void);
#define DIGI_POT_ADDR   0x2E

/*@ Description
 *        This Function used to read external voltage
  @ retval
 *      Success -   0
 *  @ retval
 *      Error   -   <0
 */
int32_t BATTERY_charge_status(void);
bool lowBatteryStatusWrite(uint8_t potValue);
extern const rlGpAdcCfg_t gpAdcCfg;

/*
 * ADC values 10-bit
 */
#define BATT_05 (550)
#define BATT_20 (625U)
#define BATT_30 (645U)
#define BATT_50 (670U)
#define BATT_100 (750U)

#ifdef __cplusplus

}
#endif


#endif /* BOARD_INCLUDE_BATTERY_H_ */
