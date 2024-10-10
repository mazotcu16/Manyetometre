/*
 * BMM150.c
 *
 *  Created on: Sep 11, 2024
 *      Author: Furkan Gundogdu
 */

#include "BMM150.h"


//#include "Inc/BMM150.h"


extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
/************************** Internal macros *******************************/
/* Sensor ODR, Repetition and axes enable/disable settings */
#define MODE_SETTING_SEL                ( (uint16_t)(0x000F) )

/* Interrupt pin settings like polarity,latch and int_pin enable */
#define INTERRUPT_PIN_SETTING_SEL       ( (uint16_t)(0x01F0) )

/* Settings to enable/disable interrupts */
#define INTERRUPT_CONFIG_SEL            ( (uint16_t)(0x1E00) )

/* Interrupt settings for configuring threshold values */
#define INTERRUPT_THRESHOLD_CONFIG_SEL  ( (uint16_t)(0x6000) )




static int8_t Set_Odr(BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static int8_t Null_Ptr_Check(BMM150_Typedef_t *dev);
static int8_t Set_AxisXY_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static int8_t Set_AxisZ_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static int8_t Set_Odr_XYZ_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static uint8_t check_powerControl_Bit(BMM150_Typedef_t *dev);
static int8_t Read_Trim_Registers(BMM150_Typedef_t *dev);
static int8_t Int_Pin_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static int8_t Mode_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
static uint8_t are_settings_changed(uint16_t sub_settings, uint16_t settings_parameters_user);
static int8_t Interrupt_Config(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
int8_t bmm150_get_interrupt_status(BMM150_Typedef_t *dev);
static int16_t compensate_x(int16_t mag_data_x, uint16_t data_rhall, BMM150_Typedef_t *dev);
static int16_t compensate_y(int16_t mag_data_y, uint16_t data_rhall, BMM150_Typedef_t *dev);
static int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall,BMM150_Typedef_t *dev);


/**
  * @brief  Initializes the BMM_150 Sensor
  * @param  BMM150_Typedef_t *dev: Sensor struct pointer for passing peripheral base and store mag_data
  * @param  I2C_HandleTypeDef *I2CHandle: Peripheral instance
  *
  * @retval uint8_t
  */

uint8_t BMM150_Initialise(BMM150_Typedef_t *dev, I2C_HandleTypeDef *I2CHandle, BMM150_Settings_t *settings)
{
	uint8_t checkfreq = 0;
	uint8_t count_error = 0;
    HAL_StatusTypeDef status;
    uint8_t regData = (1 << 0x0);

	dev->I2CHandle  = I2CHandle;

	dev->mag_data[0] = 0.0;
	dev->mag_data[1] = 0.0;
	dev->mag_data[2] = 0.0;


	/****************************!< Chip ID Oku ve Interrupt Başlat ****************************/

	status =HAL_I2C_Mem_Write(&hi2c1, BMM150_I2C_ADDR, BMM150_REG_POWER_CONTROL, I2C_MEMADD_SIZE_8BIT, &regData, 1, 100);

	if (HAL_I2C_IsDeviceReady(dev->I2CHandle, BMM150_I2C_ADDR, 1, 100) != HAL_OK)
	{

	    return HAL_BUSY;  // Cihaz hazır değilse çık
	}

	status = ReadRegister(dev, BMM150_ChipID_ADDR, &regData );
	count_error += (status != HAL_OK);

		if(regData != BMM150_ChipID_ReturnValue_ADDR)
		{
			return 255;
		}
		dev->chip_ID = regData;

		regData = 0xC0;
		status = WriteRegister(dev, BMM150_REG_AXES_ENABLE, &regData);

	/*!< Read Trim Registers  			*/

	status=Read_Trim_Registers(dev);

	/*!< Operation Mode and Output Data Rate Select(With Preset)   */


	Write_Op_Mode(BMM150_POWERMODE_NORMAL,dev );
	settings->Preset_Mode = BMM150_PRESETMODE_HIGHACCURACY;
    BMM150_ConfigPreset(dev,settings);
//	settings->Data_Rate = BMM150_DATA_RATE_25HZ;
//	Set_Odr(dev,settings);
//	status = ReadRegister(dev, BMM150_REG_OP_MODE, &regData);


	/*<!  AXIS ENABLE	*/

	settings->Int_Settings.drdy_pin_en = 0x01;
	status = BMM150_Set_Sensor_Settings(BMM150_SEL_DRDY_PIN_EN, dev, settings);

	regData = (0x80);
	status = WriteRegister(dev, BMM150_REG_AXES_ENABLE, &regData);
	count_error += (status != HAL_OK);

	status= ReadRegister(dev, BMM150_REG_OP_MODE, &checkfreq);	//TEST

	return count_error;
}



int8_t drdy_overflow_int_setting(BMM150_Typedef_t *dev,BMM150_Settings_t *settings)
{
	int8_t status = 0;
	uint16_t settings_parameters_user;

	/* Enable the desired settings to be set in the sensor */
	settings_parameters_user = BMM150_SEL_DRDY_PIN_EN | BMM150_SEL_OVERFLOW_INT |
		BMM150_SEL_INT_PIN_EN | BMM150_SEL_INT_LATCH;

	/* Set the desired configuration */
	/* Enable the drdy interrupt  */
	dev->settings.Int_Settings.drdy_pin_en = BMM150_INT_ENABLE;
	/* Enable the mapping of interrupt to the interrupt pin */
	dev->settings.Int_Settings.int_pin_en = BMM150_INT_ENABLE;
	/* Enable the overflow interrupt */
	dev->settings.Int_Settings.overflow_int_en = BMM150_INT_ENABLE;
	/* Set the interrupt in non latched mode */
	dev->settings.Int_Settings.int_latch = BMM150_NON_LATCHED;

	/* Set the configurations in the sensor */
	status = BMM150_Set_Sensor_Settings(settings_parameters_user,dev,settings);

	return status;
}

int8_t BMM150_Set_Sensor_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    int8_t status = 0 ;

        if (are_settings_changed(MODE_SETTING_SEL, settings_parameters_user))
        {
            /* ODR, Control measurement, XY,Z repetition values */
        	status = Mode_Settings(settings_parameters_user, dev, settings);
        }

        if ((!status) && are_settings_changed(INTERRUPT_PIN_SETTING_SEL, settings_parameters_user))
        {
            /* Interrupt pin settings */
        	status = Int_Pin_Settings(settings_parameters_user, dev, settings);
        }

        if ((!status) && are_settings_changed(INTERRUPT_CONFIG_SEL, settings_parameters_user))
        {
            /* Interrupt configuration settings */
        	status = Interrupt_Config(settings_parameters_user, dev, settings);
        }

       // if ((!status) && are_settings_changed(INTERRUPT_THRESHOLD_CONFIG_SEL, settings_parameters_user))
       // {
            /* Interrupt threshold settings */
        //	status = interrupt_threshold_settings(settings_parameters_user, settings, dev);
       // }

        return status;
   }


static int8_t Interrupt_Config(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    int8_t status;
    uint8_t reg_data;
    BMM150_Int_Ctrl_Settings_t int_settings;

    status = ReadRegister(dev, BMM150_REG_INT_CONFIG, &reg_data);

    if (status == BMM150_OK)
    {
        int_settings = settings->Int_Settings;
        if (settings_parameters_user & BMM150_SEL_DATA_OVERRUN_INT)
        {
            /* Sets Data overrun interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DATA_OVERRUN_INT, int_settings.data_overrun_en);
        }

        if (settings_parameters_user & BMM150_SEL_OVERFLOW_INT)
        {
            /* Sets Data overflow interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_OVERFLOW_INT, int_settings.overflow_int_en);
        }

        if (settings_parameters_user & BMM150_SEL_HIGH_THRESHOLD_INT)
        {
            /* Sets high threshold interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_HIGH_THRESHOLD_INT, int_settings.high_int_en);
        }

        if (settings_parameters_user & BMM150_SEL_LOW_THRESHOLD_INT)
        {
            /* Sets low threshold interrupt */
            reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_LOW_THRESHOLD_INT, int_settings.Low_Int_En);
        }

        /* Set the interrupt configurations in the 0x4D register */
        status = WriteRegister(dev, BMM150_REG_INT_CONFIG, &reg_data);
    }

    return status;
}



static uint8_t are_settings_changed(uint16_t sub_settings, uint16_t settings_parameters_user)
{
    uint8_t settings_changed;

    if (sub_settings & settings_parameters_user)
    {
        /* User wants to modify this particular settings */
        settings_changed = BMM150_TRUE;
    }
    else
    {
        /* User don't want to modify this particular settings */
        settings_changed = BMM150_FALSE;
    }

    return settings_changed;
}

/**
  * @brief  Reads the sensor register
  * @param  BMM150_Typedef_t *dev: Sensor struct pointer for passing peripheral base and store mag_data
  * @param  uint8_t reg: Sensor register address for read the registers
  * @param  uint8_t *data: Rx Buffer for storing the data from the sensor register
  *
  * @retval HAL_StatusTypeDef
  */

HAL_StatusTypeDef ReadRegister(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status;
	volatile uint8_t i2c_busy = 0;

	while(i2c_busy);
	i2c_busy =1;

	status = HAL_I2C_Mem_Read(dev->I2CHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

	    i2c_busy = 0; // Mark the bus as free

	    return status;

	return HAL_I2C_Mem_Read(dev->I2CHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef ReadRegisters(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->I2CHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}

/**
  * @brief  Writes the sensor registers given data
  * @param  BMM150_Typedef_t *dev: Sensor struct pointer for passing peripheral base and store mag_data
  * @param  uint8_t reg: Sensor register address for read the registers
  * @param  uint8_t *data: Rx Buffer for storing the data from the sensor register
  *
  * @retval HAL_StatusTypeDef
  */

HAL_StatusTypeDef WriteRegister(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->I2CHandle, BMM150_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}


HAL_StatusTypeDef BMM150_ReadMagneto(BMM150_Typedef_t *dev, int16_t *dataforglobal)
{
    uint8_t reg_Data[8];  // rhall değerini de okur
    HAL_StatusTypeDef status = 0;

    status = ReadRegisters(dev, BMM150_REG_DATA_X_LSB, reg_Data, 8);

    int16_t mag_Axis_X = ((int16_t)((int8_t)reg_Data[1]) << 5) | ((reg_Data[0] & 0xF8) >> 3);
    int16_t mag_Axis_Y = ((int16_t)((int8_t)reg_Data[3]) << 5) | ((reg_Data[2] & 0xF8) >> 3);
    int16_t mag_Axis_Z = ((int16_t)((int8_t)reg_Data[5]) << 7) | ((reg_Data[4] & 0xFE) >> 1);
    int16_t rhall      = ((int16_t)((int8_t)reg_Data[7]) << 6) | ((reg_Data[6] & 0xFC) >> 2);

    dataforglobal[0] = mag_Axis_X;
    dataforglobal[1] = mag_Axis_Y;
    dataforglobal[2] = mag_Axis_Z;

    dev->mag_data[0] = mag_Axis_X;
    dev->mag_data[1] = mag_Axis_Y;
    dev->mag_data[2] = mag_Axis_Z;

    dev->rhall = rhall;

    /* Compensated Mag X data in int16_t format */
    dev->compensated_data->x = compensate_x(mag_Axis_X, rhall, dev);

    /* Compensated Mag Y data in int16_t format */
    dev->compensated_data->y = compensate_y(mag_Axis_Y, rhall, dev);

    /* Compensated Mag Z data in int16_t format */
    dev->compensated_data->z = compensate_z(mag_Axis_Z,rhall, dev);


    return status;
}

static int16_t compensate_x(int16_t mag_data_x, uint16_t data_rhall, BMM150_Typedef_t *dev)
{
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;

    /* Overflow condition check */
    if (mag_data_x != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP)
    {
        if (data_rhall != 0)
        {
            /* Availability of valid data */
            process_comp_x0 = data_rhall;
        }
        else if (dev->trimData.dig_xyz1 != 0)
        {
            process_comp_x0 = dev->trimData.dig_xyz1;
        }
        else
        {
            process_comp_x0 = 0;
        }

        if (process_comp_x0 != 0)
        {
            /* Processing compensation equations */
            process_comp_x1 = ((int32_t)dev->trimData.dig_xyz1) * 16384;
            process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_x2);
            process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4 = (((int32_t)dev->trimData.dig_xy2) * (process_comp_x3 / 128));
            process_comp_x5 = (int32_t)(((int16_t)dev->trimData.dig_xy1) * 128);
            process_comp_x6 = ((int32_t)retval) * process_comp_x5;
            process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8 = ((int32_t)(((int16_t)dev->trimData.dig_x2) + ((int16_t)0xA0)));
            process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval = ((int16_t)(process_comp_x10 / 8192));
            retval = (retval + (((int16_t)dev->trimData.dig_x1) * 8)) / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
    return retval;
}

static int16_t compensate_y(int16_t mag_data_y, uint16_t data_rhall, BMM150_Typedef_t *dev)
{
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;

    /* Overflow condition check */
    if (mag_data_y != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP)
    {
        if (data_rhall != 0)
        {
            /* Availability of valid data */
            process_comp_y0 = data_rhall;
        }
        else if (dev->trimData.dig_xyz1 != 0)
        {
            process_comp_y0 = dev->trimData.dig_xyz1;
        }
        else
        {
            process_comp_y0 = 0;
        }

        if (process_comp_y0 != 0)
        {
            /* Processing compensation equations */
            process_comp_y1 = (((int32_t)dev->trimData.dig_xyz1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t) retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)dev->trimData.dig_xy2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)dev->trimData.dig_xy1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = ((int32_t)(((int16_t)dev->trimData.dig_y2) + ((int16_t)0xA0)));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval = (int16_t)(process_comp_y9 / 8192);
            retval = (retval + (((int16_t)dev->trimData.dig_y1) * 8)) / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }
    return retval;
}

static int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall,BMM150_Typedef_t *dev)
{
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;

    if (mag_data_z != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL)
    {
        if ((dev->trimData.dig_z2 != 0) && (dev->trimData.dig_z1 != 0) && (data_rhall != 0) &&
            (dev->trimData.dig_xyz1 != 0))
        {
            /*Processing compensation equations */
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t) dev->trimData.dig_xyz1);
            process_comp_z1 = (((int32_t)dev->trimData.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - dev->trimData.dig_z4)) * 32768);
            process_comp_z3 = ((int32_t)dev->trimData.dig_z1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval = ((process_comp_z2 - process_comp_z1) / (dev->trimData.dig_z2 + process_comp_z4));

            /* Saturate result to +/- 2 micro-tesla */
            if (retval > BMM150_POSITIVE_SATURATION_Z)
            {
                retval = BMM150_POSITIVE_SATURATION_Z;
            }
            else if (retval < BMM150_NEGATIVE_SATURATION_Z)
            {
                retval = BMM150_NEGATIVE_SATURATION_Z;
            }

            /* Conversion of LSB to micro-tesla */
            retval = retval / 16;
        }
        else
        {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    }
    else
    {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return (int16_t)retval;
}



/*!
 * @brief  This API writes OP_Mode appropriate value for register(0x4C) Op_Mode bits. (Bits 1 and 2)
 *
 * @param uint8_t Op_Mode: Operation Mode for sensor
 * @param BMM150_Typedef_t *dev: Device struct pointer
 *
 * @def_group OP_Mode
 */
int8_t Write_Op_Mode(uint8_t Op_Mode, BMM150_Typedef_t *dev )
{
    int8_t status;
    uint8_t reg_data = 0;

    /* Read the 0x4C register */
    status = ReadRegisters(dev, BMM150_REG_OP_MODE, &reg_data, 1);

    if (status == HAL_OK)
    {
        /* Set the op_mode value in Opmode bits of 0x4C */
        reg_data = BMM150_SET_BITS(reg_data, BMM150_OP_MODE, Op_Mode);
        status = WriteRegister(dev, BMM150_REG_OP_MODE, &reg_data);
    }

    return status;
}


static int8_t Set_Odr(BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    HAL_StatusTypeDef status;
    uint8_t reg_data = 0;

    /* Read the 0x4C register */
    status = ReadRegisters(dev, BMM150_REG_OP_MODE, &reg_data, 1);
    if (status == HAL_OK)
    {
        /* Set the ODR value */
        reg_data = BMM150_SET_BITS(reg_data, BMM150_ODR, settings->Data_Rate);
        status = WriteRegister(dev, BMM150_REG_OP_MODE, &reg_data);
    }
    return status;
}


/* Koruma olarak kullanılacak */
static int8_t Null_Ptr_Check(BMM150_Typedef_t *dev)
{
	int8_t status = 0;

    if ( (dev == NULL) )

    {
        /* NOT VALID */
    	status = BMM150_E_NULL_PTR;
    }
    else
    {
        /* VALID */
        status = BMM150_OK;
    }

    return status;
}


static int8_t Set_AxisXY_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
	HAL_StatusTypeDef status;
    uint8_t Rep_XY;

    /* Set the xy repetition */
    Rep_XY = settings->XY_Repetitions;
    status = WriteRegister(dev, BMM150_REG_REP_XY, &Rep_XY);

    return status;
}

static int8_t Set_AxisZ_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    uint8_t Rep_Z;
    HAL_StatusTypeDef status;

    /* Set the z repetition */
    Rep_Z = settings->Z_Repetitions;
    status = WriteRegister(dev, BMM150_REG_REP_Z , &Rep_Z);

    return status;
}




static int8_t Set_Odr_XYZ_Repetition(BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    int8_t status = 0;

    /* Set the ODR */
    status = Set_Odr(dev, settings);

    if (status == BMM150_OK)
    {
        /* Set the XY-repetitions number */
    	status = Set_AxisXY_Repetition(dev, settings);

        if (status == BMM150_OK)
        {
            /* Set the Z-repetitions number */
        	status = Set_AxisZ_Repetition(dev, settings);
        }
    }
    return status;
}


int8_t BMM150_ConfigPreset(BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    uint8_t data = 0;
    HAL_StatusTypeDef status;

    status = ReadRegister(dev, BMM150_REG_POWER_CONTROL, &data);


    if(status == HAL_OK)
    {
    // Set power mode to normal
    	if((data & BMM150_POWER_CNTRL_ENABLE) != 0x01U )
    	{
    	 	data = BMM150_POWER_CNTRL_ENABLE;
    	    HAL_I2C_Mem_Write(dev->I2CHandle, BMM150_I2C_ADDR, BMM150_REG_POWER_CONTROL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    	}
    }
    // Set the preset mode
    data = settings->Preset_Mode;

    switch (data)
            {
                case BMM150_PRESETMODE_LOWPOWER:

                    /* Set the data rate x,y,z repetition
                     * for Low Power mode
                     */
                    settings->Data_Rate = BMM150_DATA_RATE_10HZ;
                    settings->XY_Repetitions = BMM150_REPXY_LOWPOWER;
                    settings->Z_Repetitions = BMM150_REPZ_LOWPOWER;
                    status = Set_Odr_XYZ_Repetition(dev, settings);
                    break;
                case BMM150_PRESETMODE_REGULAR:

                    /* Set the data rate x,y,z repetition
                     * for Regular mode
                     */
                    settings->Data_Rate = BMM150_DATA_RATE_10HZ;
                    settings->XY_Repetitions = BMM150_REPXY_REGULAR;
                    settings->Z_Repetitions = BMM150_REPZ_REGULAR;
                    status = Set_Odr_XYZ_Repetition(dev, settings);
                    break;
                case BMM150_PRESETMODE_HIGHACCURACY:

                    /* Set the data rate x,y,z repetition
                     * for High Accuracy mode *
                     */
                    settings->Data_Rate = BMM150_DATA_RATE_20HZ;
                    settings->XY_Repetitions = BMM150_REPXY_HIGHACCURACY;
                    settings->Z_Repetitions = BMM150_REPZ_HIGHACCURACY;
                    status = Set_Odr_XYZ_Repetition(dev, settings);
                    break;
                case BMM150_PRESETMODE_ENHANCED:

                    /* Set the data rate x,y,z repetition
                     * for Enhanced Accuracy mode
                     */
                    settings->Data_Rate = BMM150_DATA_RATE_10HZ;
                    settings->XY_Repetitions = BMM150_REPXY_ENHANCED;
                    settings->Z_Repetitions = BMM150_REPZ_ENHANCED;
                    status = Set_Odr_XYZ_Repetition(dev, settings);
                    break;
                default:
                	status = BMM150_E_INVALID_CONFIG;
                    break;
            }

    return status;
}




int8_t bmm150_soft_reset(BMM150_Typedef_t *dev)
{
    HAL_StatusTypeDef status;
    uint8_t reg_Data  = 0;

    // Read the Power Control Register
   // status = ReadRegister(dev, BMM150_REG_POWER_CONTROL, &reg_Data);
    //status = HAL_I2C_Mem_Read(dev->I2CHandle, BMM150_I2C_ADDR, BMM150_REG_POWER_CONTROL, I2C_MEMADD_SIZE_8BIT, &reg_Data, 1, 100);

        // Set the Soft Reset Bit
    	reg_Data |= BMM150_SET_SOFT_RESET;
        // Write back to the Power Control Register
        status = WriteRegister(dev, BMM150_REG_POWER_CONTROL, &reg_Data);


    // Wait for the Reset to Complete
    HAL_Delay(1); // Delay for 1 ms

    return status;
}


static uint8_t check_powerControl_Bit(BMM150_Typedef_t *dev)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t regData = 0;
    uint8_t regData_Mask = 0;

    status = ReadRegister(dev, BMM150_REG_POWER_CONTROL, &regData);
    if (status == HAL_BUSY)
    {
    	HAL_Delay(20);
    	status = ReadRegister(dev, BMM150_REG_POWER_CONTROL, &regData);
    }

    regData_Mask = regData & BMM150_POWER_CONTROL_BIT_MSK;

    if (status == HAL_OK && regData_Mask != 0x01U)
    {
        regData |= (BMM150_POWER_CNTRL_ENABLE << BMM150_POWER_CONTROL_BIT_POS);
        status = WriteRegister(dev, BMM150_REG_POWER_CONTROL, &regData);
        if (status == HAL_BUSY)
        {
            // Handle the busy status, maybe retry after a delay
        	HAL_Delay(20);
        	status = WriteRegister(dev, BMM150_REG_POWER_CONTROL, &regData);
        }
    }

    return status;
}

void I2C_ResetBus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Adjust pin number as your configuration
    // Configure SCL as GPIO output
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Toggle SCL Line
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    // Reinitialize I2C
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

/*!
 * @brief This internal API reads the trim registers of the sensor and stores
 * the trim values in the "trim_data" of device structure.
 */
static int8_t Read_Trim_Registers(BMM150_Typedef_t *dev)
{
    int8_t status;
    uint8_t trim_x1y1[2] = { 0 };
    uint8_t trim_xyz_data[4] = { 0 };
    uint8_t trim_xy1xy2[10] = { 0 };
    uint16_t temp_msb = 0;

    /* Trim register value is read */
    status = ReadRegisters(dev, BMM150_DIG_X1, trim_x1y1, 2);

    if (status == BMM150_OK)
    {
    	status = ReadRegisters(dev, BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
        if (status == BMM150_OK)
        {
        	status = ReadRegisters(dev, BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
            if (status == BMM150_OK)
            {
                /* Trim data which is read is updated
                 * in the device structure
                 */
                dev->trimData.dig_x1 = (int8_t)trim_x1y1[0];
                dev->trimData.dig_y1 = (int8_t)trim_x1y1[1];
                dev->trimData.dig_x2 = (int8_t)trim_xyz_data[2];
                dev->trimData.dig_y2 = (int8_t)trim_xyz_data[3];
                temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
                dev->trimData.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
                temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
                dev->trimData.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
                temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
                dev->trimData.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
                temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
                dev->trimData.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
                dev->trimData.dig_xy1 = trim_xy1xy2[9];
                dev->trimData.dig_xy2 = (int8_t)trim_xy1xy2[8];
                temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
                dev->trimData.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);
            }
        }
    }

    return status;
}


/** @brief This function convert raw sensor data to gauss data
 *
 * @param  int16_t *raw_data: Address of the raw sensor values ​​read from the sensor
 * @param  float *gauss_data: Converted values from raw data(destination).
 *
 * @retval none
 */

void convert_to_gauss(int16_t *raw_data, float *gauss_data)
{
    // Sensitivity factor for BMM150(16 LSB/µT)
    const float sensitivity = 16.0;

    // Convert raw sensor data to microTesla (µT)
    float microtesla_data[3];
    for (int i = 0; i < 3; i++)
    {
        microtesla_data[i] = raw_data[i] / sensitivity;
    }

    // Convert microTesla (µT) to Gauss ( 100 µT = 1 Gauss)
    for (int i = 0; i < 3; i++)
    {
        gauss_data[i] = microtesla_data[i] / 100.0;
    }
}





static int8_t Int_Pin_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    int8_t status;
    uint8_t reg_data = 0;
    BMM150_Int_Ctrl_Settings_t int_settings;


   status = ReadRegister(dev, BMM150_REG_AXES_ENABLE, &reg_data);

    if (status == BMM150_OK)
    {
        int_settings = settings->Int_Settings;
        if (settings_parameters_user & BMM150_SEL_DRDY_PIN_EN)
        {
            /* Enables the Data ready interrupt and
             * maps it to the DRDY pin of the sensor
             */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DRDY_EN, int_settings.drdy_pin_en);
        }

        if (settings_parameters_user & BMM150_SEL_INT_PIN_EN)
        {
            /* Sets interrupt pin enable */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_INT_PIN_EN, int_settings.int_pin_en);
        }

        if (settings_parameters_user & BMM150_SEL_DRDY_POLARITY)
        {
            /* Sets Data ready pin's polarity */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DRDY_POLARITY, int_settings.drdy_polarity);
        }

        if (settings_parameters_user & BMM150_SEL_INT_LATCH)
        {
            /* Sets Interrupt in latched or non-latched mode */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_INT_LATCH, int_settings.int_latch);
        }

        if (settings_parameters_user & BMM150_SEL_INT_POLARITY)
        {
            /* Sets Interrupt pin's polarity */
            reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_INT_POLARITY, int_settings.int_polarity);
        }

        /* Set the interrupt configurations in the 0x4E register */
        status = WriteRegister(dev, BMM150_REG_AXES_ENABLE, &reg_data);
    }

    return status;
}



static int8_t Mode_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings)
{
    int8_t status = BMM150_E_INVALID_CONFIG;

    if (settings_parameters_user & BMM150_SEL_DATA_RATE)
    {
        /* Sets the ODR */
    	status = Set_Odr(dev, settings);
    }

    if (settings_parameters_user & BMM150_SEL_CONTROL_MEASURE)
    {
        /* Enables/Disables the control measurement axes */
    	status = Set_Odr_XYZ_Repetition(dev, settings);
    }

    if (settings_parameters_user & BMM150_SEL_XY_REP)
    {
        /* Sets the XY repetition */
    	status = Set_AxisXY_Repetition(dev, settings);
    }

    if (settings_parameters_user & BMM150_SEL_Z_REP)
    {
        /* Sets the Z repetition */
    	status = Set_AxisZ_Repetition(dev, settings);
    }

    return status;
}

int8_t drdy_overflow_int_handling(BMM150_Typedef_t *dev)
{
	int8_t status;
	uint8_t control = 0;

	status = bmm150_get_interrupt_status(dev);
	if (status == BMM150_OK)
	{
		/* Multiple interrupt assertion can be checked as follows */
		if ((dev->int_status & BMM150_INT_ASSERTED_DRDY) || (dev->int_status & BMM150_INT_DATA_OVERFLOW))
		{
			/* Either data ready or overflow has occurred */
			control = 1;
		}
		else
		{
			control = 0;
		}
	}

	return status;
}

int8_t bmm150_get_interrupt_status(BMM150_Typedef_t *dev)
{
    int8_t status;
    uint8_t interrupt_status;
    uint8_t data_ready_status;

    /* Read the data ready status from the register 0x48 */
    status = ReadRegister(dev, BMM150_REG_DATA_RDY_STATUS_LSB, &data_ready_status);

    if (status == BMM150_OK)
    {
        /* Read the interrupt status from the register 0x50 */
    	status = ReadRegister(dev, BMM150_REG_INTERRUPT_STATUS, &interrupt_status);
        if (status == BMM150_OK)
        {
            /* Mask and store the data ready status bit */
            data_ready_status = BMM150_GET_BITS_POS_0(data_ready_status, BMM150_DRDY_STATUS);

            /* Store the entire interrupt status in dev */
            dev->int_status = (data_ready_status << 8) | interrupt_status;
        }
    }
    return status;
}


bool magneto_collect_gauss(CircularBuffer_t *cbuffer, BMM150_Typedef_t *dev)
{
    int16_t mag_raw[3];
    HAL_StatusTypeDef status = BMM150_ReadMagneto(dev, mag_raw);
    if (status == HAL_OK)
    {
        // Ham veriyi ölçeklendir ve Gauss cinsine çevir
        float sensitivity = 16.0f; // 16 LSB/µT
        float microtesla_to_gauss = 1.0f / 100.0f; // 100 µT = 1 Gauss

        MagData_Gauss_t newData;
//       newData.x = ((float)dev->compensated_data->x ) * microtesla_to_gauss;
//       newData.y = ((float)dev->compensated_data->y ) * microtesla_to_gauss;

        newData.x = ((float)mag_raw[0] / sensitivity );
        newData.y = ((float)mag_raw[1] / sensitivity );
        newData.z = ((float)mag_raw[2] / sensitivity );

        // Buffer'a ekle
        CircularBuffer_Enqueue(cbuffer, &newData);
        return true;
    }
    else
    {
        return false;
    }
}


/************** CIRCULAR BUFFER FUNCTIONS ******************************************* */


void CircularBuffer_Init(CircularBuffer_t *cb)
{
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}


bool CircularBuffer_IsFull(CircularBuffer_t *cbuffer)
{
    return cbuffer->count == SAMPLE_COUNT;
}

bool CircularBuffer_IsEmpty(CircularBuffer_t *cbuffer)
{
    return cbuffer->count == 0;
}

uint16_t CircularBuffer_Count(CircularBuffer_t *cbuffer)
{
    return cbuffer->count;
}

bool CircularBuffer_Enqueue(CircularBuffer_t *cbuffer, MagData_Gauss_t *item)
{
    if (CircularBuffer_IsFull(cbuffer))
    {

        cbuffer->data[cbuffer->head] = *item;
        cbuffer->head = (cbuffer->head + 1) % SAMPLE_COUNT;
        cbuffer->tail = cbuffer->head;
        return false; // Buffer dolu olduğu için overwrite yapıldı
    }
    else
    {
    	cbuffer->data[cbuffer->head] = *item;
    	cbuffer->head = (cbuffer->head + 1) % SAMPLE_COUNT;
    	cbuffer->count++;
        return true; // Başarılı enqueue
    }
}

bool CircularBuffer_Dequeue(CircularBuffer_t *cbuffer, MagData_Gauss_t *item)
{
    if (CircularBuffer_IsEmpty(cbuffer))
    {
        return false; // Buffer boş, dequeue yapılamaz
    } else
    {
        *item = cbuffer->data[cbuffer->tail];
        cbuffer->tail = (cbuffer->tail + 1) % SAMPLE_COUNT;
        cbuffer->count--;
        return true; // Başarılı dequeue
    }
}

/***************************************************************************************************************************************	*/


void ApplyMagCalibration(float raw_measurement[3], float bias[3], float calibration_matrix[3][3], float corrected_measurement[3])
{
    // 1. Bias'ı kaldırılması (hard-iron düzeltmesi)
    float temp[3];
    temp[0] = raw_measurement[0] - bias[0];
    temp[1] = raw_measurement[1] - bias[1];
    temp[2] = raw_measurement[2] - bias[2];

    // 2. Kalibrasyon matrisini uygulanması (soft-iron düzeltmesi)
    for (int i = 0; i < 3; i++)
    {
        corrected_measurement[i] = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            corrected_measurement[i] += calibration_matrix[i][j] * temp[j];
        }
    }
}


float CalculateHeading(float x, float y, float declination_deg)
{
    // Açıyı radyan cinsinden hesapla
    float heading_rad = atan2f(y, x);

    // Açıyı dereceye çevir
    float heading_deg = heading_rad * (180.0f / M_PI);

    // Açı değeri negatifse 360 ekle
    if (heading_deg < 0) {
        heading_deg += 360.0f;
    }

    // Manyetik deklinasyonu ekle
    heading_deg += declination_deg;

    // Açı değerini 0-360 derece aralığına getir
    if (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    } else if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    return heading_deg;
}
HAL_StatusTypeDef ResetDevice(BMM150_Typedef_t *dev)
{
    uint8_t reset_cmd = 0x82; // Reset komutu
    return HAL_I2C_Mem_Write(dev->I2CHandle, BMM150_I2C_ADDR, 0x4B, I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, 100);
}

