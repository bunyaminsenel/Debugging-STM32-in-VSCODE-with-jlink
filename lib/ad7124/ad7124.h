/************************************************************************
 *
* FILENAME: 	ad7124.h
* DATE: 		10.07.2020
* DESCRIPTION: 	AD7124 library header
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
#ifndef AD7124_H
#define AD7124_H
/************************************************************************
 *
 * INCLUDES
 *
************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "ad7124_drv.h"
#include "ad7124_regs.h"
/************************************************************************
 *
 * DEFINES AND MACROS
 *
************************************************************************/
#define AD7124RegisterTable ad7124_regs

/* default register values */
#define AD7124_CONFIG_DEFAULT_VALUE     0x0860u
#define AD7124_FILTER_DEFAULT_VALUE     0x060180u
#define AD7124_OFFSET_DEFAULT_VALUE     0x800000u
#define AD7124_GAIN_DEFAULT_VALUE       0x500000u

/* known devide IDs */
#define AD7124_ID_AD7124_8              0x14u
#define AD7124_ID_AD7124_8_B_GRADE      0x16u

/* reference voltages in microvolts */
#define AD7124_INTERNAL_REFERENCE_UV    2500000ULL
#define AD7124_AVDD_REFERENCE_UV        3300000ULL
#define AD7124_REFIN1_REFERENCE_UV      2500000ULL
#define AD7124_REFIN2_REFERENCE_UV      0000000ULL	// needs to be set correctly when used

/************************************************************************
 *
 * ENUMS AND STRUCTS
 *
************************************************************************/
typedef enum ad7124_registers AD7124_RegisterIdx_T;

typedef enum
{
	AD7124_OK,
	AD7124_NOK,
	AD7124_DATA_PENDING,
	AD7124_WRONG_PARAMS,
} AD7124_ErrorCode_T;

typedef enum
{
	AD7124_CHANNEL_0 = 0x00u,
	AD7124_CHANNEL_1,
	AD7124_CHANNEL_2,
	AD7124_CHANNEL_3,
	AD7124_CHANNEL_4,
	AD7124_CHANNEL_5,
	AD7124_CHANNEL_6,
	AD7124_CHANNEL_7,
	AD7124_CHANNEL_8,
	AD7124_CHANNEL_9,
	AD7124_CHANNEL_10,
	AD7124_CHANNEL_11,
	AD7124_CHANNEL_12,
	AD7124_CHANNEL_13,
	AD7124_CHANNEL_14,
	AD7124_CHANNEL_15,
	AD7124_CHANNEL_INVALID = 0xFF

} AD7124_Channel_T;

typedef enum
{
	AD7124_POWER_MODE_LOW = 0x00u,
	AD7124_POWER_MODE_MID,
	AD7124_POWER_MODE_HIGH,
} AD7124_AdcControlReg_PowerMode_T;

typedef enum
{
	AD7124_OPERATING_MODE_CONTINUOUS = 0x00u,
	AD7124_OPERATING_MODE_SINGLE,
	AD7124_OPERATING_MODE_STANDBY,
	AD7124_OPERATING_MODE_POWER_DOWN,
	AD7124_OPERATING_MODE_IDLE,
	AD7124_OPERATING_MODE_INTERNAL_OFFSET_CALIBRATION,
	AD7124_OPERATING_MODE_INTERNAL_GAIN_CALIBRATION,
	AD7124_OPERATING_MODE_SYSTEM_OFFSET_CALIBRATION,
	AD7124_OPERATING_MODE_SYSTEM_GAIN_CALIBRATION
} AD7124_AdcControlReg_OperatingMode_T;

typedef enum
{
	AD7124_CLKSEL_INTERNAL_614_4_KHZ_NOT_AVAILABLE_ON_CLK = 0x00u,
	AD7124_CLKSEL_INTERNAL_614_4_KHZ_AVAILABLE_ON_CLK,
	AD7124_CLKSEL_EXTERNAL_614_4_KHZ,
	AD7124_CLKSEL_EXTERNAL
} AD7124_AdcControlReg_ClkSel_T;

typedef enum
{
	AD7124_AIN_AIN0 = 0x00u,
	AD7124_AIN_AIN1,
	AD7124_AIN_AIN2,
	AD7124_AIN_AIN3,
	AD7124_AIN_AIN4,
	AD7124_AIN_AIN5,
	AD7124_AIN_AIN6,
	AD7124_AIN_AIN7,
	AD7124_AIN_AIN8,
	AD7124_AIN_AIN9,
	AD7124_AIN_AIN10,
	AD7124_AIN_AIN11,
	AD7124_AIN_AIN12,
	AD7124_AIN_AIN13,
	AD7124_AIN_AIN14,
	AD7124_AIN_AIN15,
	AD7124_AIN_TEMPERATURE_SENSOR,
	AD7124_AIN_AVSS,
	AD7124_AIN_INTERNAL_REFERENCE,
	AD7124_AIN_DGND,
	AD7124_AIN_AVDD_AVSS_6P,
	AD7124_AIN_AVDD_AVSS_6M,
	AD7124_AIN_IOVDD_DGND_6P,
	AD7124_AIN_IOVDD_DGND_6M,
	AD7124_AIN_ALDO_AVSS_6P,
	AD7124_AIN_ALDO_AVSS_6M,
	AD7124_AIN_DLDO_DGND_6P,
	AD7124_AIN_DLDO_DGND_6M,
	AD7124_AIN_V_20MV_P,
	AD7124_AIN_V_20MV_M
} AD7124_ChannelReg_Ain_T;

typedef enum
{
	AD7124_SETUP_0 = 0x00u,
	AD7124_SETUP_1,
	AD7124_SETUP_2,
	AD7124_SETUP_3,
	AD7124_SETUP_4,
	AD7124_SETUP_5,
	AD7124_SETUP_6,
	AD7124_SETUP_7

} AD7124_ChannelReg_Setup_T;

typedef enum
{
	AD7124_BURNOUT_OFF = 0x00u,
	AD7124_BURNOUT_0_5_UA,
	AD7124_BURNOUT_2_UA,
	AD7124_BURNOUT_4_UA
} AD7124_ConfigurationReg_Burnout_T;

/**
 * Reference source select bits. These bits select the reference source to use when
 * converting on any channels using this configuration register.
 */
typedef enum
{
	AD7124_REFSEL_REFIN1 = 0x00u, /* REFIN1(+)/REFIN1(−) */
	AD7124_REFSEL_REFIN2, /* REFIN2(+)/REFIN2(−) */
	AD7124_REFSEL_INTERNAL, /* internal reference */
	AD7124_REFSEL_AVDD /* AVDD */
} AD7124_ConfigurationReg_RefSel_T;

typedef enum
{
	AD7124_PGA_1 = 0x00u,
	AD7124_PGA_2,
	AD7124_PGA_4,
	AD7124_PGA_8,
	AD7124_PGA_16,
	AD7124_PGA_32,
	AD7124_PGA_64,
	AD7124_PGA_128
} AD7124_ConfigurationReg_PGA_T;

typedef enum
{
	AD7124_FILTER_SINC_4 				= 0x00u,
	AD7124_FILTER_SINC_3 				= 0x02u,
	AD7124_FILTER_SINC_4_FAST_SETTLING 	= 0x04u,
	AD7124_FILTER_SINC_3_FAST_SETTLING 	= 0x05u,
	AD7124_FILTER_POST_FILTER 			= 0x07u
} AD7124_FilterReg_Filter_T;

typedef union
{
	struct
	{
		uint32_t PGA : 3;          // Gain select bits
		uint32_t REF_SEL : 2;      // Reference select bits
		uint32_t AIN_BUFM : 1;     //
		uint32_t AIN_BUFP : 1;     // Pseudo differential analog inputs
		uint32_t REF_BUFM : 1;     //
		uint32_t REF_BUFP : 1;     // Reference select bits
		uint32_t BURNOUT : 2;      //
		uint32_t BIPOLAR : 1;      // Polarity select bit(0-bipolar, 1-unipolar)
		uint32_t RESERVED : 4;     // Reserved
		uint32_t RESERVED2 : 16;   // Reserved
	} B;
	uint32_t D;

} AD7124_ConfigurationRegister_T;

typedef union
{
	struct
	{
		uint32_t FS : 11;          //
		uint32_t RESERVED : 5;     // Reserved
		uint32_t SINGLE_CYCLE : 1; //
		uint32_t POST_FILTER : 3;  //
		uint32_t REJ60 : 1;        //
		uint32_t FILTER : 3;       //
		uint32_t RESERVED2 : 8;    // Reserved
	} B;
	uint32_t D;

} AD7124_FilterRegister_T;

typedef union
{
	struct
	{
		uint32_t CLK_SEL : 2;
		uint32_t MODE : 4;
		uint32_t POWER_MODE : 2;
		uint32_t REF_EN : 1;
		uint32_t CS_EN : 1;
		uint32_t DATA_STATUS : 1;
		uint32_t CONT_READ : 1;
		uint32_t DOUT_RDY_DEL : 1;
		uint32_t RESERVED : 3;
		uint32_t RESERVED2 : 16;
	} B;
	uint32_t D;

} AD7124_AdcControlRegister_T;

typedef uint32_t AD7124_OffsetRegister_T;
typedef uint32_t AD7124_GainRegister_T;

/************************************************************************
 *
 * STATIC FUNCTIONS DECLARATIONS
 *
************************************************************************/

/************************************************************************
 *
 * FUNCTIONS
 *
************************************************************************/


/************************************************************************
*
* FUNCTION: 	AD7124_Init
* PARAMS: 		none
* RETVAL: 		Error Code
* DESCRIPTION:	AD7124 init function
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_Init(void);

/************************************************************************
*
* FUNCTION: 	AD7124_Deinit
* PARAMS: 		none
* RETVAL: 		Error Code
* DESCRIPTION:	AD7124 deinit function
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_Deinit(void);

/************************************************************************
*
* FUNCTION: 	AD7124_ReadData
* PARAMS: 		[out] channel - channel read
				[out] data - output buffer for data from ADC
                [in] raw - if true then raw ADC data will be received, otherwise in millivolts
* RETVAL: 		Error Code
* DESCRIPTION:	Reads data register from AD7124. If data is not
* 				ready yet it returns AD7124_DATA_PENDING
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ReadData(AD7124_Channel_T* channel, int32_t* data, bool raw);

/************************************************************************
*
* FUNCTION: 	AD7124_WriteRegister
* PARAMS: 		[in] regIdx - index of register to be written
* 				[out] data - data to write to register
* RETVAL: 		Error Code
* DESCRIPTION:	Writes single register to AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_WriteRegister(const AD7124_RegisterIdx_T regIdx, const uint32_t data);

/************************************************************************
*
* FUNCTION: 	AD7124_ReadRegister
* PARAMS: 		[in] regIdx - index of register to be read
* 				[out] data - output buffer for register data
* RETVAL: 		Error Code
* DESCRIPTION:	Reads single register from AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ReadRegister(const AD7124_RegisterIdx_T regIdx, uint32_t* data);

/************************************************************************
*
* FUNCTION: 	AD7124_SetChannel
* PARAMS: 		[in] channel - channel to be set
* 				[in] ainP - analog input positive
* 				[in] ainM - analog input negative
* RETVAL: 		Error Code
* DESCRIPTION:	Setups given ADC channel on AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetChannel(const AD7124_Channel_T channel, const AD7124_ChannelReg_Ain_T ainP,
									 const AD7124_ChannelReg_Ain_T ainM, const AD7124_ChannelReg_Setup_T setup);

/************************************************************************
*
* FUNCTION: 	AD7124_EnableChannel
* PARAMS: 		[in] channel - channel to be enabled
* RETVAL: 		Error Code
* DESCRIPTION:	Enables given channel on AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_EnableChannel(const AD7124_Channel_T channel);

/************************************************************************
*
* FUNCTION: 	AD7124_DisableChannel
* PARAMS: 		[in] channel - channel to be disabled
* RETVAL: 		Error Code
* DESCRIPTION:	Disables given channel on AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_DisableChannel(const AD7124_Channel_T channel);

/************************************************************************
*
* FUNCTION: 	AD7124_SetPowerMode
* PARAMS: 		[in] pwr_mode - power mode to be set
* RETVAL: 		Error Code
* DESCRIPTION:	Sets power mode on AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetPowerMode(const AD7124_AdcControlReg_PowerMode_T pwr_mode);

/************************************************************************
*
* FUNCTION: 	AD7124_SetOperatingMode
* PARAMS: 		[in] op_mode - operating mode to be set
* RETVAL: 		Error Code
* DESCRIPTION:	Sets operating mode on AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetOperatingMode(const AD7124_AdcControlReg_OperatingMode_T op_mode);

/************************************************************************
*
* FUNCTION: 	AD7124_PowerOff
* PARAMS: 		none
* RETVAL: 		Error Code
* DESCRIPTION:	Puts AD7124 into power down mode
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_PowerOff(void);

/************************************************************************
*
* FUNCTION: 	AD7124_PowerOn
* PARAMS: 		none
* RETVAL: 		Error Code
* DESCRIPTION:	Wakes the AD7142 from power down mode
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_PowerOn(void);

/************************************************************************
*
* FUNCTION: 	AD7124_SetClockSelection
* PARAMS: 		[in] clk_sel - clock source to be set
* RETVAL: 		Error Code
* DESCRIPTION:	Sets clock source of AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetClockSelection(const AD7124_AdcControlReg_ClkSel_T clk_sel);

/************************************************************************
*
* FUNCTION: 	AD7124_ConfigureSetup
* PARAMS: 		[in] setupIdx - setup numer to be set
* 				[in] cfgReg - corresponding configuration register to be set
* 				[in] filterReg - corresponding filter register to be set
* RETVAL: 		Error Code
* DESCRIPTION:	Sets given setup
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ConfigureSetup(const AD7124_ChannelReg_Setup_T setupIdx, const AD7124_ConfigurationRegister_T cfgReg,
										 const AD7124_FilterRegister_T filterReg);

/************************************************************************
*
* FUNCTION: 	AD7124_WriteAdcControlReg
* PARAMS: 		[in] ctrlReg - ADC configuration register structure to be written
* RETVAL: 		Error Code
* DESCRIPTION:	Writes ADC configuration register to AD7124
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
AD7124_ErrorCode_T AD7124_WriteAdcControlReg(const AD7124_AdcControlRegister_T ctrlReg);

#endif // AD7124_H