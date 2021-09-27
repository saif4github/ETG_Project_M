/*
 * i2c_driver.h
 *
 */

#ifndef BOARD_INCLUDE_I2C_DRIVER_H_
#define BOARD_INCLUDE_I2C_DRIVER_H_


#include <stdint.h>
#include <ti/drivers/i2c/I2C.h>
#include <xdc/std.h>
#include <stdbool.h>
#include <ti/drivers/pinmux/pinmux.h>


#define     I2C_SDA                     SOC_XWR18XX_PINF13_PADAH
#define     I2C_SDA_PIN                 SOC_XWR18XX_PINF13_PADAH_I2C_SDA

#define     I2C_SCL                     SOC_XWR18XX_PING14_PADAI
#define     I2C_SCL_PIN                 SOC_XWR18XX_PING14_PADAI_I2C_SCL


/***********  Configurations to be used for output/input control **************/

#define     OUT_CTRL_HW_EN              2U
#define     OUT_CTRL_DIS                0U
#define     IN_CTRL_HW_EN               2U
#define     IN_CTRL_DIS                 0U


#define     I2C_FAILURE                 0U
#define     I2C_SUCCESS                 1U

/*
 * @brief - Initialize I2C pins
 *
 * @return    Value < 0 - in case of error
 *            Value > 0 - success; outputCtrl and inputCtrl have valid values,value read from the Func_Sel bits of a given valid pin and
 *
 */

int32_t i2c_pins_init(void);


#endif /* BOARD_INCLUDE_I2C_DRIVER_H_ */
