/************************************************************************
 *
* FILENAME: 	ad7124.c
* DATE: 		10.07.2020
* DESCRIPTION: 	AD7124 library
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/

/************************************************************************
 *
 * INCLUDES
 *
************************************************************************/
#include "ad7124.h"
#include "log.h"
/************************************************************************
 *
 * DEFINES AND MACROS
 *
************************************************************************/

/* start addresses of setup registers */
#define AD7124_CHANNEL_REGISTER_OFFSET						0x09u
#define AD7124_CHANNEL_CONFIGURATION_REGISTER_OFFSET		0x19u
#define AD7124_CHANNEL_FILTER_REGISTER_OFFSET				0x21u
#define AD7124_CHANNEL_OFFSET_REGISTER_OFFSET				0x29u
#define AD7124_CHANNEL_GAIN_REGISTER_OFFSET					0x31u

/* max poll count in synchronous mode */
#define AD7124_TIMOUT_POLL_COUNT							1000

/************************************************************************
 *
 * STATIC VARIABLES
 *
************************************************************************/

/* structures handling AD7124 device */
static struct ad7124_dev Ad7124Device;
static struct ad7124_init_param Ad7124InitParams;

/************************************************************************
 *
 * STATIC FUNCTIONS DECLARATIONS
 *
************************************************************************/
/************************************************************************
*
* FUNCTION: 	AD7124_ModifyRegisterBits
* PARAMS: 		[in] regIdx - index of register to be modified
* 				[in] val - value to be modified
* 				[in] mask - bits that shall be changed
* RETVAL: 		Error Code
* DESCRIPTION:	Modify single register bits
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
static AD7124_ErrorCode_T AD7124_ModifyRegisterBits(const AD7124_RegisterIdx_T regIdx, const uint32_t val, const uint32_t mask);

/************************************************************************
*
* FUNCTION: 	AD7124_ConvertRawAdcToMicrovolts
* PARAMS: 		[in] channel - channel index
* 				[in] data - data to be converted
* RETVAL: 		Voltage in microvolts
* DESCRIPTION:	Converts raw ADC data register value into microvolts
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
static int32_t AD7124_ConvertRawAdcToMicrovolts(const AD7124_Channel_T channel, const uint32_t data);

/************************************************************************
 *
 * FUNCTIONS
 *
************************************************************************/

/************************************************************************
*
* FUNCTION: 	AD7124_ModifyRegisterBits
*
************************************************************************/
static AD7124_ErrorCode_T AD7124_ModifyRegisterBits(const AD7124_RegisterIdx_T regIdx, const uint32_t val, const uint32_t mask)
{
	AD7124_ErrorCode_T ret;
	uint32_t regVal = 0u;

	ret = AD7124_ReadRegister(regIdx, &regVal);

	if (AD7124_SUCCESS == ret)
	{
		/* modify only bits set in mask */
		regVal = (regVal & ~mask) | (val & mask);
		ret = AD7124_WriteRegister(regIdx, regVal);
	}

	return ret;
}

/************************************************************************
*
* FUNCTION: 	AD7124_ReadData
*
************************************************************************/
static int32_t AD7124_ConvertRawAdcToMicrovolts(const AD7124_Channel_T channel, const uint32_t data)
{
	if ((channel > AD7124_CHANNEL_15) || (data > 0xFFFFFFu)) return AD7124_WRONG_PARAMS;

	int32_t aIn = 0;

	/* given channel, obtain a configuraion setup for that channel */
	const AD7124_ChannelReg_Setup_T channelSetup = AD7124_CH_MAP_REG_SETUP((uint32_t) Ad7124Device.regs[AD7124_CHANNEL_REGISTER_OFFSET + (uint8_t) channel].value);

	/* having configuration setup, we can obtain rest of data needed to calculate voltage: Vref and gain */
	const AD7124_ConfigurationRegister_T cfgReg = {.D = (uint32_t) Ad7124Device.regs[AD7124_CHANNEL_CONFIGURATION_REGISTER_OFFSET + (uint8_t) channelSetup].value };

	uint64_t Vref = 0u;

	/* set correct reference voltage, needed when calculating ADC output */
	switch (cfgReg.B.REF_SEL)
	{
		case AD7124_REFSEL_REFIN1:
		{
			Vref = AD7124_REFIN1_REFERENCE_UV;
			break;
		}

		case AD7124_REFSEL_REFIN2:
		{
			Vref = AD7124_REFIN2_REFERENCE_UV;
			break;
		}

		case AD7124_REFSEL_INTERNAL:
		{
			Vref = AD7124_INTERNAL_REFERENCE_UV;
			break;
		}

		case AD7124_REFSEL_AVDD:
		{
			Vref = AD7124_AVDD_REFERENCE_UV;
			break;
		}

		default:
		{
			LOG_ERROR("Unknown voltage reference source!");
			break;
		}
	}

	/* data coding differs in unipolar and bipolar modes.
	 * these calculations can be found in datasheet PDF page 48 */
	if (cfgReg.B.BIPOLAR)
	{
		aIn = ((uint64_t)(data * Vref)/(1UL << 23UL)) - Vref;
	}
	else
	{
		aIn = ((uint64_t)(data * Vref))/((1UL << 24UL));
	}

	return aIn;
}

/************************************************************************
*
* FUNCTION: 	AD7124_Init
*
************************************************************************/
AD7124_ErrorCode_T AD7124_Init(void)
{
	Ad7124InitParams.regs = ad7124_regs;
	Ad7124InitParams.spi_rdy_poll_cnt = AD7124_TIMOUT_POLL_COUNT;
	int32_t ret = 0;

	Spi_Init();

	ret = ad7124_setup(&Ad7124Device, Ad7124InitParams);

	return (ret < 0) ? AD7124_NOK : AD7124_OK;
}

/************************************************************************
*
* FUNCTION: 	AD7124_Deinit
*
************************************************************************/
AD7124_ErrorCode_T AD7124_Deinit(void)
{
	int32_t ret = 0;

	ret = ad7124_remove(&Ad7124Device);

	Spi_Deinit();

	return (ret < 0) ? AD7124_NOK : AD7124_OK;
}

/************************************************************************
*
* FUNCTION: 	AD7124_ReadData
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ReadData(AD7124_Channel_T* channel, int32_t* data, bool raw)
{
	if ((NULL == data) || (NULL == channel)) return AD7124_WRONG_PARAMS;

	AD7124_ErrorCode_T retVal = AD7124_NOK;
	int32_t ret = ad7124_wait_for_conv_ready(&Ad7124Device, 1u);
	uint32_t rawAdc = 0;

	if (AD7124_SUCCESS == ret)
	{
		*channel = (AD7124_Channel_T) AD7124_STATUS_REG_CH_ACTIVE(Ad7124Device.regs[AD7124_Status].value);

		ret = ad7124_read_data(&Ad7124Device, (int32_t *) &rawAdc);

		if (AD7124_SUCCESS == ret)
		{
			if (!raw)
            {
				*data = (int32_t) AD7124_ConvertRawAdcToMicrovolts(*channel, rawAdc);
            }
            else
            {
				*data = rawAdc;
			}
			
			retVal = AD7124_OK;
		}
	}
	else if (AD7124_TIMEOUT == ret)
	{
		retVal = AD7124_DATA_PENDING;
	}
	else
	{
		retVal = AD7124_NOK;
	}

	return retVal;
}

/************************************************************************
*
* FUNCTION: 	AD7124_WriteAdcControlReg
*
************************************************************************/
AD7124_ErrorCode_T AD7124_WriteAdcControlReg(const AD7124_AdcControlRegister_T ctrlReg)
{
	if (ctrlReg.D > 0xFFFF) return AD7124_WRONG_PARAMS;

	return AD7124_WriteRegister(AD7124_ADC_Control, ctrlReg.D);
}

/************************************************************************
*
* FUNCTION: 	AD7124_SetChannel
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetChannel(const AD7124_Channel_T channel, const AD7124_ChannelReg_Ain_T ainP,
								     const AD7124_ChannelReg_Ain_T ainM, const AD7124_ChannelReg_Setup_T setup)
{
	if ((channel > AD7124_CHANNEL_15) || (ainP > AD7124_AIN_V_20MV_M) ||
		(ainM > AD7124_AIN_V_20MV_M) || (setup > AD7124_SETUP_7))
	{
		return AD7124_WRONG_PARAMS;
	}

	AD7124_ErrorCode_T ret;
	const AD7124_RegisterIdx_T regIdx = (AD7124_RegisterIdx_T) ((AD7124_CHANNEL_REGISTER_OFFSET) + (uint8_t) channel);

	ret = AD7124_ModifyRegisterBits(regIdx,
								   ((AD7124_CH_MAP_REG_AINP((uint8_t) ainP) | AD7124_CH_MAP_REG_AINM((uint8_t) ainM)) | AD7124_CH_MAP_REG_SETUP((uint8_t) setup)),
								   (AD7124_CH_MAP_REG_AINP(0xFFu) | AD7124_CH_MAP_REG_AINM(0xFFu) | AD7124_CH_MAP_REG_SETUP(0xFFu)));

	return ret;
}

/************************************************************************
*
* FUNCTION: 	AD7124_EnableChannel
*
************************************************************************/
AD7124_ErrorCode_T AD7124_EnableChannel(const AD7124_Channel_T channel)
{
	if (channel > AD7124_CHANNEL_15) return AD7124_WRONG_PARAMS;

	const AD7124_RegisterIdx_T regIdx = (AD7124_RegisterIdx_T) (AD7124_CHANNEL_REGISTER_OFFSET + (uint8_t) channel);

	return AD7124_ModifyRegisterBits(regIdx, AD7124_CH_MAP_REG_CH_ENABLE, AD7124_CH_MAP_REG_CH_ENABLE);
}

/************************************************************************
*
* FUNCTION: 	AD7124_DisableChannel
*
************************************************************************/
AD7124_ErrorCode_T AD7124_DisableChannel(const AD7124_Channel_T channel)
{
	if (channel > AD7124_CHANNEL_15) return AD7124_WRONG_PARAMS;

	const AD7124_RegisterIdx_T regIdx = (AD7124_RegisterIdx_T) (AD7124_CHANNEL_REGISTER_OFFSET + (uint8_t) channel);

	return AD7124_ModifyRegisterBits(regIdx, ~AD7124_CH_MAP_REG_CH_ENABLE, AD7124_CH_MAP_REG_CH_ENABLE);
}

/************************************************************************
*
* FUNCTION: 	AD7124_SetPowerMode
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetPowerMode(const AD7124_AdcControlReg_PowerMode_T pwr_mode)
{
	if (pwr_mode > AD7124_POWER_MODE_HIGH) return AD7124_WRONG_PARAMS;

	return AD7124_ModifyRegisterBits(AD7124_ADC_Control, AD7124_ADC_CTRL_REG_POWER_MODE(pwr_mode), AD7124_ADC_CTRL_REG_POWER_MODE(0xFFu));
}

/************************************************************************
*
* FUNCTION: 	AD7124_SetOperatingMode
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetOperatingMode(const AD7124_AdcControlReg_OperatingMode_T op_mode)
{
	if (op_mode > AD7124_OPERATING_MODE_SYSTEM_GAIN_CALIBRATION) return AD7124_WRONG_PARAMS;

	return AD7124_ModifyRegisterBits(AD7124_ADC_Control, AD7124_ADC_CTRL_REG_MODE(op_mode), AD7124_ADC_CTRL_REG_MODE(0xFFu));
}

/************************************************************************
*
* FUNCTION: 	AD7124_PowerOff
*
************************************************************************/
AD7124_ErrorCode_T AD7124_PowerOff(void)
{
	AD7124_ErrorCode_T ret = AD7124_NOK;

	/* according to datasheet, to enter power down mode we must first enter standby mode */
	ret = AD7124_ModifyRegisterBits(AD7124_ADC_Control, AD7124_ADC_CTRL_REG_MODE((uint8_t) AD7124_OPERATING_MODE_STANDBY), AD7124_ADC_CTRL_REG_MODE(0xFFu));
	if (AD7124_SUCCESS != ret) return ret;

	return AD7124_ModifyRegisterBits(AD7124_ADC_Control, AD7124_ADC_CTRL_REG_MODE((uint8_t) AD7124_OPERATING_MODE_POWER_DOWN), AD7124_ADC_CTRL_REG_MODE(0xFFu));
}

/************************************************************************
*
* FUNCTION: 	AD7124_PowerOn
*
************************************************************************/
AD7124_ErrorCode_T AD7124_PowerOn(void)
{
	volatile AD7124_AdcControlRegister_T ctrlReg = {0u};

	/* set mode to continuous on powerup */
	ctrlReg.D = (uint32_t) Ad7124Device.regs[AD7124_ADC_Control].value;
	ctrlReg.B.MODE = (uint8_t) AD7124_OPERATING_MODE_CONTINUOUS;
	Ad7124Device.regs[AD7124_ADC_Control].value = (int32_t) ctrlReg.D;

	/* this covers whole wake-up procedure and inits ADC registers with those cached with structure in RAM */
	int32_t ret = ad7124_setup(&Ad7124Device, Ad7124InitParams);

	return (ret < 0) ? AD7124_NOK : AD7124_OK;
}

/************************************************************************
*
* FUNCTION: 	AD7124_SetClockSelection
*
************************************************************************/
AD7124_ErrorCode_T AD7124_SetClockSelection(const AD7124_AdcControlReg_ClkSel_T clk_sel)
{
	if (clk_sel > AD7124_CLKSEL_EXTERNAL) return AD7124_WRONG_PARAMS;

	return AD7124_ModifyRegisterBits(AD7124_ADC_Control, AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel), AD7124_ADC_CTRL_REG_CLK_SEL(0xFFu));
}

/************************************************************************
*
* FUNCTION: 	AD7124_ConfigureSetup
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ConfigureSetup(const AD7124_ChannelReg_Setup_T setupIdx, const AD7124_ConfigurationRegister_T cfgReg,
										 const AD7124_FilterRegister_T filterReg)
{
	if ((setupIdx > AD7124_SETUP_7) || (cfgReg.D > 0xFFFFu) || (filterReg.D > 0xFFFFFFu))
	{
		return AD7124_WRONG_PARAMS;
	}

	AD7124_ErrorCode_T ret = AD7124_NOK;

	const AD7124_RegisterIdx_T configRegIdx = (AD7124_RegisterIdx_T) (AD7124_CHANNEL_CONFIGURATION_REGISTER_OFFSET + (uint8_t) setupIdx);
	const AD7124_RegisterIdx_T filterRegIdx = (AD7124_RegisterIdx_T) (AD7124_CHANNEL_FILTER_REGISTER_OFFSET + (uint8_t) setupIdx);

	ret = AD7124_WriteRegister(configRegIdx, cfgReg.D);
	if (AD7124_SUCCESS != ret) return ret;

	ret = AD7124_WriteRegister(filterRegIdx, filterReg.D);
	if (AD7124_SUCCESS != ret) return ret;

	return ret;
}

/************************************************************************
*
* FUNCTION: 	AD7124_WriteRegister
*
************************************************************************/
AD7124_ErrorCode_T AD7124_WriteRegister(const AD7124_RegisterIdx_T regIdx, const uint32_t data)
{
	if (regIdx > AD7124_Gain_7) return AD7124_WRONG_PARAMS;

	Ad7124Device.regs[regIdx].value = (int32_t) data;

	int32_t ret = ad7124_write_register(&Ad7124Device, Ad7124Device.regs[regIdx]);

	return (ret < 0) ? AD7124_NOK : AD7124_OK;
}

/************************************************************************
*
* FUNCTION: 	AD7124_ReadRegister
*
************************************************************************/
AD7124_ErrorCode_T AD7124_ReadRegister(const AD7124_RegisterIdx_T regIdx, uint32_t* data)
{
	if ((NULL == data) || (regIdx > AD7124_Gain_7)) return AD7124_WRONG_PARAMS;

	int32_t ret = ad7124_read_register(&Ad7124Device, &Ad7124Device.regs[regIdx]);

	if (AD7124_SUCCESS == ret)
	{
		*data = (uint32_t) Ad7124Device.regs[regIdx].value;
	}

	return (ret < 0) ? AD7124_NOK : AD7124_OK;
}

