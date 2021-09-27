/*
 * i2c_driver.c
 *
 */

#include "BSP/Inc/i2c_driver.h"


/******************** I2C driver configuration ******************************/
I2C_Transaction i2cTransaction;
I2C_Handle      i2cHandle;
I2C_Params      i2cParams;

/*
 * @brief - Initialize I2C pins
 *
 * @return    Value < 0 - in case of error
 *            Value > 0 - success; outputCtrl and inputCtrl have valid values,value read from the Func_Sel bits of a given valid pin and
 *
 */
int32_t i2c_pins_init(void)
{
    int32_t ret = 0;

    //MUXing I2C functionality for Serial Clock
    if((ret = Pinmux_Set_OverrideCtrl(I2C_SCL, OUT_CTRL_HW_EN, IN_CTRL_HW_EN)) < 0)
    {
        return ret;
    }

    if((ret = Pinmux_Set_FuncSel(I2C_SCL, I2C_SCL_PIN)) < 0)
    {
        return ret;
    }

    //MUXing I2C functionality for Serial Data
    if((ret = Pinmux_Set_OverrideCtrl(I2C_SDA, OUT_CTRL_HW_EN, IN_CTRL_HW_EN)) < 0)
    {
        return ret;
    }

    if((ret = Pinmux_Set_FuncSel(I2C_SDA, I2C_SDA_PIN)) < 0)
    {
        return ret;
    }

    /* Initializa the I2C driver */
        I2C_init();

        /* Initialize the I2C driver default parameters */
        I2C_Params_init(&i2cParams);

        i2cParams.transferMode = I2C_MODE_BLOCKING;
        i2cParams.bitRate = I2C_100kHz;

        /* Open the I2C driver */
        i2cHandle = I2C_open(0, &i2cParams);

        if (i2cHandle == NULL)
               return I2C_FAILURE;
           else
               return I2C_SUCCESS;
}

