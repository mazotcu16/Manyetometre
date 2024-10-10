/*
 * BMM150.h
 *
 *  Created on: Sep 11, 2024
 *      Author: Furkan Gundogdu
 */


#ifndef INC_BMM150_H_
#define INC_BMM150_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define BMM150_ChipID_ADDR								(0x40U)
#define BMM150_ChipID_ReturnValue_ADDR					(0x32U)
#define BMM150_I2C_ADDR									(0x13U<<1)

#define TRUE											(1U)
#define FALSE											(!TRUE)

#define MAX_MATRIX_SIZE 10
#define MAX_SIZE 10

#define DECLINATION_DEG 5.0f

/*! @name Macro to SET and GET BITS of the Register*/

#define BMM150_SET_BITS(REGDATA, BIT, DATA) 	  		 ((REGDATA & ~(BIT##_MSK)) |  ( (DATA << BIT##_POS) & BIT##_MSK) )

#define BMM150_GET_BITS(REGDATA, BIT)            		 ( (REGDATA & (BIT##_MSK)) >>  (BIT##_POS) )

#define BMM150_SET_BITS_POS_0(REGDATA, BIT, DATA) 		 ( (REGDATA & ~(BIT##_MSK)) | (DATA & BIT##_MSK))

#define BMM150_GET_BITS_POS_0(REGDATA, BIT)  		     (REGDATA & (BIT##_MSK) )

#define SAMPLE_COUNT 250

#define EPSILON 1e-8      // Sayısal hassasiyet için tolerans değeri

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period           : The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*BMM150_delay_us_fptr_t)(uint16_t period);

/**
 * @brief Interface Selection
 *
 */
typedef enum
{
    BMM150_SPI_INTF,									/*! SPI interface */
    BMM150_I2C_INTF										/*! I2C interface */
}BMM150_SelectInterface_Enum;


/*
 * BMM150 Interrupt Control Settings TypeDef_Structure
 *
 */

typedef struct
{
    uint8_t drdy_pin_en;								 /*!< Data ready interrupt EN 												 	 */
    uint8_t int_pin_en;									 /*!< Threshold and overflow interrupts EN 										 */
    uint8_t drdy_polarity;								 /*!< Data ready interrupt polarity Active HIGH/LOW 							 */
    uint8_t int_latch;									 /*!< Interrupt pin - Latched or Non-latched 									 */
    uint8_t int_polarity;								 /*!< Interrupt polarity Active high/low 										 */
    uint8_t data_overrun_en;							 /*!< Data overrun interrupt enable 											 */
    uint8_t overflow_int_en;							 /*!< Overflow interrupt enable 												 */
    uint8_t high_int_en;								 /*!< High interrupt enable/disable axis selection							 	 */
    uint8_t Low_Int_En;									 /*!< low interrupt enable/disable axis selection 						 		 */
    uint8_t Low_Threshold;								 /*!< Low threshold limit 														 */
    uint8_t High_Threshold;								 /*!< High threshold limit 														 */
}BMM150_Int_Ctrl_Settings_t;

/*!
 * @brief bmm150 sensor settings
 */

typedef struct BMM150_Settings
{

	uint8_t PowerMode;									   /*!< Power Control  Settings																*/
    uint8_t AxesControl_XYZ;							   /*!< Control measurement of XYZ axes Settings 											*/
    uint8_t Data_Rate;									   /*!<	Output data rate Settings															*/
    uint8_t XY_Repetitions;								   /*! XY Repetitions Settings																*/
    uint8_t Z_Repetitions;								   /*! Z Repetitions Settings																*/
    uint8_t Preset_Mode;								   /*! Preset Mode Settings 																*/
    /*! Interrupt configuration settings */
   BMM150_Int_Ctrl_Settings_t Int_Settings;    	       // struct bmm150_int_ctrl_settings int_settings;
}BMM150_Settings_t;





/*
 * BMM150 Sensor Trim Registers TypeDef_Structure
 *
 */

typedef struct
{
    int8_t   dig_x1;												 /*!< Trim x1 data																 */
    int8_t   dig_y1;												 /*!< Trim y1 data 																 */
    int8_t   dig_x2;												 /*!< Trim x2 data 																 */
    int8_t   dig_y2;												 /*!< Trim y2 data 																 */
    uint16_t dig_z1;											     /*!< Trim z1 data 																 */
    int16_t  dig_z2;												 /*!< Trim z2 data 																 */
    int16_t  dig_z3;												 /*!< Trim z3 data 																 */
    int16_t  dig_z4;												 /*!< Trim z4 data 																 */
    uint8_t  dig_xy1;											     /*!< Trim xy1 data 															 */
    int8_t   dig_xy2;												 /*!< Trim xy2 data 															 */
    uint16_t dig_xyz1;											     /*!< Trim xyz1 data 															 */
}BMM150_TrimReg_t;


typedef struct
{
    /*! compensated mag X data */
    float x;

    /*! compensated mag Y data */
    float y;

    /*! compensated mag Z data */
    float z;
}BMM150_compensated_data_t;

// Kalibrasyon parametreleri
typedef struct {
    float bias[3];
    float soft_iron_matrix[3][3];
} MagCalibrationParams_t;




/*
 * Sensor Struct Prototype
 *
 */

typedef struct
{
	uint8_t chip_ID;

	I2C_HandleTypeDef *I2CHandle;

	int16_t mag_data[3];

	uint8_t Pwr_Cntrl_Bit;

	BMM150_TrimReg_t trimData;

	BMM150_delay_us_fptr_t delay_us;

	BMM150_Settings_t settings;

	uint16_t int_status;

	int16_t rhall;

	BMM150_compensated_data_t *compensated_data;
}BMM150_Typedef_t;



typedef struct
{
    float x;
    float y;
    float z;
} MagData_Gauss_t;



typedef struct
{
	uint8_t onhz;
	uint8_t yirmibeshz;
	uint8_t kirkhz;
	uint8_t ellihz;
	uint8_t seksenhz;
}Sayac_t;

/* Circular Buffer Structure																															*/
typedef struct
{
    MagData_Gauss_t data[SAMPLE_COUNT];
    uint16_t head; // Yeni verilerin eklendiği indeks
    uint16_t tail; // Verilerin okunduğu indeks
    uint16_t count; // Buffer'daki mevcut veri sayısı
} CircularBuffer_t;

/* Calibration State Machine */
typedef enum
{
    CALIBRATION_IDLE,
    CALIBRATION_COLLECTING,
    CALIBRATION_CALCULATING,
    CALIBRATION_COMPLETE
} CalibrationState_t;

typedef enum {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_NORMAL
} SystemState_t;

/******************************************************************************/
/*! @name        General Macro Definitions                */
/******************************************************************************/
/*! @name API success code */
#define BMM150_OK                                  (uint8_t)(0U)

/*! @name To define TRUE or FALSE */
#define BMM150_TRUE                                (uint8_t)(1U)
#define BMM150_FALSE                               (uint8_t)(0U)

/*! @name API error codes */
#define BMM150_E_NULL_PTR                          (uint8_t)(-1)
#define BMM150_E_DEV_NOT_FOUND                     (uint8_t)(-2)
#define BMM150_E_INVALID_CONFIG                    (uint8_t)(-3)
#define BMM150_E_COM_FAIL                          (uint8_t)(-4)

/*! @name API warning codes */
#define BMM150_W_NORMAL_SELF_TEST_YZ_FAIL          (uint8_t)(1U)
#define BMM150_W_NORMAL_SELF_TEST_XZ_FAIL          (uint8_t)(2U)
#define BMM150_W_NORMAL_SELF_TEST_Z_FAIL           (uint8_t)(3U)
#define BMM150_W_NORMAL_SELF_TEST_XY_FAIL          (uint8_t)(4U)
#define BMM150_W_NORMAL_SELF_TEST_Y_FAIL           (uint8_t)(5U)
#define BMM150_W_NORMAL_SELF_TEST_X_FAIL           (uint8_t)(6U)
#define BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL         (uint8_t)(7U)
#define BMM150_W_ADV_SELF_TEST_FAIL                (uint8_t)(8U)

/*! @name CHIP ID & SOFT RESET VALUES      */
#define BMM150_CHIP_ID                             (uint8_t)(0x32U)
#define BMM150_SET_SOFT_RESET                      (uint8_t)(0x82U)

/*! @name POWER MODE DEFINITIONS      */
/*
 * @def_group OP_Mode
 */
#define BMM150_POWERMODE_NORMAL                    (uint8_t)(0x00U)			/*!< Magnetometer Operation Mode Normal  		*/
#define BMM150_POWERMODE_FORCED                    (uint8_t)(0x01U)			/*!< Magnetometer Operation Mode Forced  		*/
#define BMM150_POWERMODE_SLEEP                     (uint8_t)(0x03U)			/*!< Magnetometer Operation Mode Sleep  		*/
#define BMM150_POWERMODE_SUSPEND                   (uint8_t)(0x04U)			/*!< Magnetometer Operation Mode Suspend  		*/

/*! @name Power mode settings  */
#define BMM150_POWER_CNTRL_DISABLE                 (uint8_t)(0x00U)
#define BMM150_POWER_CNTRL_ENABLE                  (uint8_t)(0x01U)

/*! @name I2C ADDRESS       */
#define BMM150_DEFAULT_I2C_ADDRESS                 (uint8_t)(0x10U)
#define BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH        (uint8_t)(0x11U)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW        (uint8_t)(0x12U)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH       (uint8_t)(0x13U)

/*! @name Sensor delay time settings  */
#define BMM150_DELAY_SOFT_RESET                    (uint16_t)(1000)
#define BMM150_DELAY_NORMAL_SELF_TEST              (uint16_t)(2000)
#define BMM150_START_UP_TIME                       (uint16_t)(3000)
#define BMM150_DELAY_ADV_SELF_TEST                 (uint16_t)(4000)

/*! @name ENABLE/DISABLE DEFINITIONS  */
#define BMM150_XYZ_CHANNEL_ENABLE                  (uint8_t)(0x00U)			//Enable  option  for  user  macro
#define BMM150_XYZ_CHANNEL_DISABLE                 (uint8_t)(0x07U)			//Disable option for user macro


/*! @name BMM 150 Sensor Registers Address */
#define BMM150_REG_CHIP_ID                       (uint8_t)(0x40U)	/*!< Chip identification number,which is 0x32.Can only be read if the power control bit (Register 0x4B bit0) is enabled */
#define BMM150_REG_DATA_X_LSB                    (uint8_t)(0x42U)	/*!< Contains the LSB part of X-axis magnetic field data and the self test result flag for the X axis 					*/
#define BMM150_REG_DATA_X_MSB                    (uint8_t)(0x43U)	/*!< Contains the MSB part of X-axis magnetic field data and the self test result flag for the X axis 					*/
#define BMM150_REG_DATA_Y_LSB                    (uint8_t)(0x44U)	/*!< Contains the LSB part of Y-axis magnetic field data and the self test result flag for the Y axis 					*/
#define BMM150_REG_DATA_Y_MSB                    (uint8_t)(0x45U)	/*!< Contains the MSB part of Y-axis magnetic field data and the self test result flag for the Y axis 					*/
#define BMM150_REG_DATA_Z_LSB                    (uint8_t)(0x46U)	/*!< Contains the LSB part of Z-axis magnetic field data and the self test result flag for the Z axis 					*/
#define BMM150_REG_DATA_Z_MSB                    (uint8_t)(0x47U)	/*!< Contains the MSB part of Z-axis magnetic field data and the self test result flag for the Z axis 					*/
#define BMM150_REG_DATA_RDY_STATUS_LSB           (uint8_t)(0x48U)	/*!< Contains the LSB part of hall resistance and the Data Ready (DRDY) status bit*/
#define BMM150_REG_DATA_RDY_STATUS_MSB           (uint8_t)(0x49U)	/*!< Contains the MSB part of hall resistance and the Data Ready (DRDY) status bit										*/
#define BMM150_REG_INTERRUPT_STATUS              (uint8_t)(0x4AU)	/*!< Contains the states of all interrupts																				*/
#define BMM150_REG_POWER_CONTROL                 (uint8_t)(0x4BU) 	/*!< Contains control bits for power control, soft reset and interface SPI mode selection								*/
#define BMM150_REG_OP_MODE                       (uint8_t)(0x4CU)	/*!< Contains the control bits for operation mode, output data rate and self test										*/
#define BMM150_REG_INT_CONFIG                    (uint8_t)(0x4DU)	/*!< Contains the control bits for interrupt settings																	*/
#define BMM150_REG_AXES_ENABLE                   (uint8_t)(0x4EU)	/*!< Contains the control bits interrupt settings and axes enables bits.	 			     							*/
#define BMM150_REG_LOW_THRESHOLD                 (uint8_t)(0x4FU)	/*!< Contains the LOW-Threshold interrupt threshold settings															*/
#define BMM150_REG_HIGH_THRESHOLD                (uint8_t)(0x50U)	/*!< Contains the HIGH-Threshold interrupt threshold settings															*/
#define BMM150_REG_REP_XY                        (uint8_t)(0x51U)	/*!< Contains the number of repetitions for X/Y axis																	*/
#define BMM150_REG_REP_Z                         (uint8_t)(0x52U)	/*!< Contains the number of repetitions for Z axis																		*/



/*! @name TRIM REGISTERS Non Volatile Memory      */
/* Trim Extended Registers */
#define BMM150_DIG_X1                             (uint8_t)(0x5D)
#define BMM150_DIG_Y1                             (uint8_t)(0x5E)
#define BMM150_DIG_Z4_LSB                         (uint8_t)(0x62)
#define BMM150_DIG_Z4_MSB                         (uint8_t)(0x63)
#define BMM150_DIG_X2                             (uint8_t)(0x64)
#define BMM150_DIG_Y2                             (uint8_t)(0x65)
#define BMM150_DIG_Z2_LSB                         (uint8_t)(0x68)
#define BMM150_DIG_Z2_MSB                         (uint8_t)(0x69)
#define BMM150_DIG_Z1_LSB                         (uint8_t)(0x6A)
#define BMM150_DIG_Z1_MSB                         (uint8_t)(0x6B)
#define BMM150_DIG_XYZ1_LSB                       (uint8_t)(0x6C)
#define BMM150_DIG_XYZ1_MSB                       (uint8_t)(0x6D)
#define BMM150_DIG_Z3_LSB                         (uint8_t)(0x6E)
#define BMM150_DIG_Z3_MSB                         (uint8_t)(0x6F)
#define BMM150_DIG_XY2                            (uint8_t)(0x70)
#define BMM150_DIG_XY1                            (uint8_t)(0x71)



/*! @name BMM_150 Sensor Bit Definitions */

/*! @name DATA RATE DEFINITIONS for user */
#define BMM150_DATA_RATE_10HZ                     (uint8_t)(0x00U)					/*!< Output Data Rate(User Configurable) for 10Hz																		*/
#define BMM150_DATA_RATE_02HZ                     (uint8_t)(0x01U)					/*!< Output Data Rate(User Configurable) for 2Hz																		*/
#define BMM150_DATA_RATE_06HZ                     (uint8_t)(0x02U)					/*!< Output Data Rate(User Configurable) for 6Hz																		*/
#define BMM150_DATA_RATE_08HZ                     (uint8_t)(0x03U)					/*!< Output Data Rate(User Configurable) for 8Hz																		*/
#define BMM150_DATA_RATE_15HZ                     (uint8_t)(0x04U)					/*!< Output Data Rate(User Configurable) for 15Hz																		*/
#define BMM150_DATA_RATE_20HZ                     (uint8_t)(0x05U)					/*!< Output Data Rate(User Configurable) for 20Hz																		*/
#define BMM150_DATA_RATE_25HZ                     (uint8_t)(0x06U)					/*!< Output Data Rate(User Configurable) for 25Hz																		*/
#define BMM150_DATA_RATE_30HZ                     (uint8_t)(0x07U)					/*!< Output Data Rate(User Configurable) for 30Hz																		*/
#define BMM150_ODR_MAX                            (uint8_t)(0x07U)					/*!< Output Data Rate bit position max value (111)																		*/
#define BMM150_ODR_MSK                            (uint8_t)(0x38U)					// 3 4 ve 5. biti 1 olan sayı maske değeri.
#define BMM150_ODR_POS                            (uint8_t)(0x03U)					/*!< BMM150 Output Data Rate bit positions 																				*/

/*! @name Threshold interrupt setting macros for x,y,z axes selection */
#define BMM150_THRESHOLD_X                         (uint8_t)(0x06)
#define BMM150_THRESHOLD_Y                         (uint8_t)(0x05)
#define BMM150_THRESHOLD_Z                         (uint8_t)(0x03)

#define BMM150_HIGH_THRESHOLD_INT_MSK              (uint8_t)(0x38)
#define BMM150_HIGH_THRESHOLD_INT_POS              (uint8_t)(0x03)
#define BMM150_LOW_THRESHOLD_INT_MSK               (uint8_t)(0x07)

/*! @name User configurable interrupt setting macros 					*/
/* @def_group User_Config_Interrupt Macros								*/

#define BMM150_INT_ENABLE                          (uint8_t)(0x01)
#define BMM150_INT_DISABLE                         (uint8_t)(0x00)
#define BMM150_ACTIVE_HIGH_POLARITY                (uint8_t)(0x01)
#define BMM150_ACTIVE_LOW_POLARITY                 (uint8_t)(0x00)
#define BMM150_LATCHED                             (uint8_t)(0x01)
#define BMM150_NON_LATCHED                         (uint8_t)(0x00)

/*! @name Interrupt status */
#define BMM150_INT_THRESHOLD_X_LOW                 (uint8_t)(0x0001)
#define BMM150_INT_THRESHOLD_Y_LOW                 (uint8_t)(0x0002)
#define BMM150_INT_THRESHOLD_Z_LOW                 (uint8_t)(0x0004)
#define BMM150_INT_THRESHOLD_X_HIGH                (uint8_t)(0x0008)
#define BMM150_INT_THRESHOLD_Y_HIGH                (uint8_t)(0x0010)
#define BMM150_INT_THRESHOLD_Z_HIGH                (uint8_t)(0x0020)
#define BMM150_INT_DATA_OVERFLOW                   (uint8_t)(0x0040)
#define BMM150_INT_DATA_OVERRUN                    (uint8_t)(0x0080)
#define BMM150_INT_DATA_READY                      (uint8_t)(0x0100)

#define BMM150_DRDY_EN_MSK                         (uint8_t)(0x80)
#define BMM150_DRDY_EN_POS                         (uint8_t)(0x07)
#define BMM150_DRDY_POLARITY_MSK                   (uint8_t)(0x04)
#define BMM150_DRDY_POLARITY_POS                   (uint8_t)(0x02)
#define BMM150_INT_PIN_EN_MSK                      (uint8_t)(0x40)
#define BMM150_INT_PIN_EN_POS                      (uint8_t)(0x06)
#define BMM150_INT_LATCH_MSK                       (uint8_t)(0x02)
#define BMM150_INT_LATCH_POS                       (uint8_t)(0x01)
#define BMM150_INT_POLARITY_MSK                    (uint8_t)(0x01)
#define BMM150_DRDY_STATUS_MSK                     (uint8_t)(0x01)

/*! @name Interrupt status macros */
#define BMM150_INT_ASSERTED_DRDY                  (uint16_t)(0x0100)
#define BMM150_INT_ASSERTED_LOW_THRES             (uint16_t)(0x0007)
#define BMM150_INT_ASSERTED_HIGH_THRES            (uint16_t)(0x0380)

/*! @name Power control bit macros */
#define BMM150_PWR_CNTRL_MSK                      (uint8_t)(0x01)
#define BMM150_CONTROL_MEASURE_MSK                (uint8_t)(0x38)
#define BMM150_CONTROL_MEASURE_POS                (uint8_t)(0x03)
#define BMM150_POWER_CONTROL_BIT_MSK              (uint8_t)(0x01)
#define BMM150_POWER_CONTROL_BIT_POS              (uint8_t)(0x00)

/*! @name Data macros */
#define BMM150_DATA_X_MSK                         (uint8_t)(0xF8)
#define BMM150_DATA_X_POS                         (uint8_t)(0x03)

#define BMM150_DATA_Y_MSK                         (uint8_t)(0xF8)
#define BMM150_DATA_Y_POS                         (uint8_t)(0x03)

#define BMM150_DATA_Z_MSK                         (uint8_t)(0xFE)
#define BMM150_DATA_Z_POS                         (uint8_t)(0x01)

#define BMM150_DATA_RHALL_MSK                     (uint8_t)(0xFC)
#define BMM150_DATA_RHALL_POS                     (uint8_t)(0x02)

#define BMM150_DATA_OVERRUN_INT_MSK               (uint8_t)(0x80)
#define BMM150_DATA_OVERRUN_INT_POS               (uint8_t)(0x07)

#define BMM150_OVERFLOW_INT_MSK                   (uint8_t)(0x40)
#define BMM150_OVERFLOW_INT_POS                   (uint8_t)(0x06)

/*! @name OVERFLOW DEFINITIONS  */
#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        (uint16_t)(-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL         (uint16_t)(-16384)
#define BMM150_OVERFLOW_OUTPUT                    (uint16_t)(-32768)
#define BMM150_NEGATIVE_SATURATION_Z              (uint16_t)(-32767)
#define BMM150_POSITIVE_SATURATION_Z              (uint16_t)(32767)
#ifdef BMM150_USE_FLOATING_POINT
#define BMM150_OVERFLOW_OUTPUT_FLOAT              0.0f
#endif



/* @def_group settings_parameters_user																		*/

#define BMM150_SEL_DATA_RATE                       ( (uint16_t)(1 << 0)  )
#define BMM150_SEL_CONTROL_MEASURE                 ( (uint16_t)(1 << 1)  )
#define BMM150_SEL_XY_REP                          ( (uint16_t)(1 << 2)  )
#define BMM150_SEL_Z_REP                           ( (uint16_t)(1 << 3)  )
#define BMM150_SEL_DRDY_PIN_EN                     ( (uint16_t)(1 << 4)  )
#define BMM150_SEL_INT_PIN_EN                      ( (uint16_t)(1 << 5)  )
#define BMM150_SEL_DRDY_POLARITY                   ( (uint16_t)(1 << 6)  )
#define BMM150_SEL_INT_LATCH                       ( (uint16_t)(1 << 7)  )
#define BMM150_SEL_INT_POLARITY                    ( (uint16_t)(1 << 8)  )
#define BMM150_SEL_DATA_OVERRUN_INT                ( (uint16_t)(1 << 9)  )
#define BMM150_SEL_OVERFLOW_INT                    ( (uint16_t)(1 << 10) )
#define BMM150_SEL_HIGH_THRESHOLD_INT              ( (uint16_t)(1 << 11) )
#define BMM150_SEL_LOW_THRESHOLD_INT               ( (uint16_t)(1 << 12) )
#define BMM150_SEL_LOW_THRESHOLD_SETTING           ( (uint16_t)(1 << 13) )
#define BMM150_SEL_HIGH_THRESHOLD_SETTING          ( (uint16_t)(1 << 14) )




/*! @name PRESET MODE DEFINITIONS  */
#define BMM150_PRESETMODE_LOWPOWER                (uint8_t)(0x01)
#define BMM150_PRESETMODE_REGULAR                 (uint8_t)(0x02)
#define BMM150_PRESETMODE_HIGHACCURACY            (uint8_t)(0x03)
#define BMM150_PRESETMODE_ENHANCED                (uint8_t)(0x04)

#define BMM150_OP_MODE_MSK                        (uint8_t)(0x06)
#define BMM150_OP_MODE_POS                        (uint8_t)(0x01)

/*! @name PRESET MODES - REPETITIONS-XY RATES */
#define BMM150_REPXY_LOWPOWER                     (uint8_t)(0x01)
#define BMM150_REPXY_REGULAR                      (uint8_t)(0x04)
#define BMM150_REPXY_ENHANCED                     (uint8_t)(0x07)
#define BMM150_REPXY_HIGHACCURACY                 (uint8_t)(0x17)

/*! @name PRESET MODES - REPETITIONS-Z RATES */
#define BMM150_REPZ_LOWPOWER                      (uint8_t)(0x01)
#define BMM150_REPZ_REGULAR                       (uint8_t)(0x07)
#define BMM150_REPZ_ENHANCED                      (uint8_t)(0x0D)
#define BMM150_REPZ_HIGHACCURACY                  (uint8_t)(0x29)

/*! @name Self test settings */
#define BMM150_DISABLE_XY_AXIS                    (uint8_t)(0x03)
#define BMM150_SELF_TEST_REP_Z                    (uint8_t)(0x04)

/*! @name Self test selection macros */
#define BMM150_SELF_TEST_NORMAL                   (uint8_t)(0)
#define BMM150_SELF_TEST_ADVANCED                 (uint8_t)(1)

/*! @name Advanced self-test current settings */
#define BMM150_DISABLE_SELF_TEST_CURRENT          (uint8_t)(0x00)
#define BMM150_ENABLE_NEGATIVE_CURRENT            (uint8_t)(0x02)
#define BMM150_ENABLE_POSITIVE_CURRENT            (uint8_t)(0x03)

/*! @name Normal self-test status */
#define BMM150_SELF_TEST_STATUS_XYZ_FAIL          (uint8_t)(0x00)
#define BMM150_SELF_TEST_STATUS_SUCCESS           (uint8_t)(0x07)

#define BMM150_SELF_TEST_MSK                      (uint8_t)(0x01)
#define BMM150_ADV_SELF_TEST_MSK                  (uint8_t)(0xC0)
#define BMM150_ADV_SELF_TEST_POS                  (uint8_t)(0x06)

/*! @name Register read lengths  */
#define BMM150_LEN_SELF_TEST                      (uint8_t)(5)
#define BMM150_LEN_SETTING_DATA                   (uint8_t)(8)
#define BMM150_LEN_XYZR_DATA                      (uint8_t)(8)

/*! @name Boundary check macros */
#define BMM150_BOUNDARY_MAXIMUM                   (uint8_t)(0)
#define BMM150_BOUNDARY_MINIMUM                   (uint8_t)(1)







uint8_t BMM150_Initialise(BMM150_Typedef_t *dev, I2C_HandleTypeDef *I2CHandle, BMM150_Settings_t *settings);

HAL_StatusTypeDef BMM150_ReadMagneto(BMM150_Typedef_t *dev,int16_t *dataforglobal);

HAL_StatusTypeDef ReadRegister(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ReadRegisters(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data,uint8_t length);
HAL_StatusTypeDef WriteRegister(BMM150_Typedef_t *dev, uint8_t reg, uint8_t *data);
int8_t Write_Op_Mode(uint8_t Op_Mode, BMM150_Typedef_t *dev);
int8_t BMM150_ConfigPreset(BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
int8_t bmm150_soft_reset(BMM150_Typedef_t *dev);
void reset_I2C(void);
void I2C_ResetBus(I2C_HandleTypeDef *hi2c);
void delay_us (uint16_t us);
void convert_to_gauss(int16_t *raw_data, float *gauss_data);
HAL_StatusTypeDef Set_Pwr_Ctrl_Bit(uint8_t PwrCntrl_Bit, BMM150_Typedef_t *dev);
int8_t BMM150_Set_Sensor_Settings(uint16_t settings_parameters_user,BMM150_Typedef_t *dev, BMM150_Settings_t *settings);
int8_t drdy_overflow_int_setting(BMM150_Typedef_t *dev,BMM150_Settings_t *settings);

bool magneto_collect_gauss(CircularBuffer_t *cb, BMM150_Typedef_t *dev);
void CalculateCovarianceMatrix(CircularBuffer_t *cb, float mean[3], float covMatrix[3][3]);
bool PerformCalibration(CircularBuffer_t *cbuffer, float bias[3], float scale[3]);
float CalculateHeading(float x, float y, float declination_deg);
bool PerformCalibration_LSE(CircularBuffer_t *cbuffer, float bias_out[3], float calibration_matrix[3][3]);
bool QRDecomposition(float A[3][3], float Q[3][3], float R[3][3]);
void ApplyMagCalibration(float raw_measurement[3], float bias[3], float calibration_matrix[3][3], float corrected_measurement[3]);

/* CIRCULAR BUFFER FUNCTIONS */
void CircularBuffer_Init(CircularBuffer_t *cbuffer);
bool CircularBuffer_Enqueue(CircularBuffer_t *cbuffer, MagData_Gauss_t *item);
bool CircularBuffer_Dequeue(CircularBuffer_t *cbuffer, MagData_Gauss_t *item);
bool CircularBuffer_IsFull(CircularBuffer_t *cbuffer);
bool CircularBuffer_IsEmpty(CircularBuffer_t *cbuffer);
uint16_t CircularBuffer_Count(CircularBuffer_t *cbuffer);
HAL_StatusTypeDef ResetDevice(BMM150_Typedef_t *dev);



#endif /* INC_BMM150_H_ */
