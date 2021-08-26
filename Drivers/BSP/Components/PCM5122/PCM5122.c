/*
 * PCM5122.c
 *
 *  Created on: Aug 18, 2021
 *      Author: Eissa NEMATI
 */

#include "PCM5122.h"
#include <math.h>

typePCM5122 PCM5122;


/**
 * @fn uint8_t PCM5122_I2C_Write(uint8_t, uint8_t, uint8_t)
 * @brief
 *
 * @param Addr
 * @param Reg
 * @param Value
 * @return
 */
static uint8_t PCM5122_I2C_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	uint32_t result = 0;
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(PCM5122_I2C_HANDLE, (uint16_t) Addr, (uint16_t) (Reg | 0X80), (uint16_t) I2C_MEMADD_SIZE_8BIT, &Value, 1, PCM5122_I2C_TIMEOUT_MAX);

	//---Check the communication status
	if (status != HAL_OK)
	{
		//---Execute user timeout callback
		I2C_Error(PCM5122_I2C_HANDLE);
	}

	return result;
}



/**
 * @fn uint8_t PCM5122_I2C_Read(uint8_t, uint8_t, uint8_t*)
 * @brief
 *
 * @param Addr
 * @param Reg
 * @param Value
 * @return
 */
static uint8_t PCM5122_I2C_Read(uint8_t Addr, uint8_t Reg, uint8_t *Value)
{
	uint8_t result = 0;
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(PCM5122_I2C_HANDLE, Addr, (uint16_t)(Reg|0X80), I2C_MEMADD_SIZE_8BIT, Value, 1, PCM5122_I2C_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2C_Error(PCM5122_I2C_HANDLE);
	}

	return result;
}

/**
 * @fn uint8_t PCM5122_SelectPage(typeRegister)
 * @brief Called when reading/writing a register to select the correct page
 *
 * @param RegAddr
 * @return
 */
uint8_t PCM5122_SelectPage(typeRegister RegAddr)
{
	uint8_t CurrentPage;
	uint8_t Result = 0;
	uint8_t RequestPage = ((RegAddr >> 8) & 0xff);

	if (PCM5122_I2C_Read(PCM5122_AUDIO_I2C_ADDRESS, 0, &CurrentPage) == HAL_OK)
	{
		if (RequestPage != CurrentPage)
		{
			Result = PCM5122_I2C_Write(PCM5122_AUDIO_I2C_ADDRESS, 0, RequestPage);
		}
	}

	return Result;
}

/**
 * @fn uint8_t PCM5122_WriteRegister(typeRegister, uint8_t)
 * @brief Write a single register
 *
 * @param RegAddr
 * @param data
 * @return
 */
uint8_t PCM5122_WriteRegister(typeRegister RegAddr, uint8_t data)
{
	uint8_t Result = 0;

	if (PCM5122_SelectPage(RegAddr) == 0)
		Result += PCM5122_I2C_Write(PCM5122_AUDIO_I2C_ADDRESS, RegAddr, data);

	return Result;
}


/**
 * @fn uint8_t PCM5122_ReadRegister(typeRegister)
 * @brief Read a single register
 *
 * @param RegAddr
 * @return
 */
uint8_t PCM5122_ReadRegister(typeRegister RegAddr)
 {
	uint8_t value = 0;

	PCM5122_SelectPage(RegAddr);

	PCM5122_I2C_Read(PCM5122_AUDIO_I2C_ADDRESS, RegAddr, &value);

	return value;
}

/**
 * @fn bool PCM5122_RegisterInitialize(typeSamplingRate, typeBitDepth)
 * @brief
 *
 * @param rate
 * @param bps
 * @return
 */
bool PCM5122_RegisterInitialize(typeSamplingRate rate, typeBitDepth bps)
{
	PCM5122_Reset();

	PCM5122_WriteRegister(PCM5122_REG_GPIO_EN, 0x24);

	PCM5122_WriteRegister((PCM5122_REG_GPIOn_OUTPUT_SEL + 2), 0x02);
	PCM5122_WriteRegister((PCM5122_REG_GPIOn_OUTPUT_SEL + 5), 0x02);

	PCM5122_SetGpioLow(PCM5122_GPIO_ALL);
	PCM5122_SetGpioHigh(PCM5122_GPIO_6);
	PCM5122_SetGpioLow(PCM5122_GPIO_ALL);
	PCM5122_SetGpioHigh(PCM5122_GPIO_3);

	PCM5122_WriteRegister(PCM5122_REG_GPIO_EN, 0x2C);			// LED Blue = OFF / OSCs = ENABLE
	PCM5122_WriteRegister((PCM5122_REG_GPIOn_OUTPUT_SEL + 3), 0x02);// (Output Selection) GPIO4=Register GPIO4 output (Page 0 / Register 86, bit 3)
	PCM5122_LED_BLUE_ON();

	PCM5122_ConfigurePLL(rate, bps);
	PCM5122_ConfigSpeedMode(rate);
	PCM5122_SetVolume(255);										// Volume = Mute
	PCM5122_SetPowerMode(PCM5122_POWER_MODE_STANDBY);
	PCM5122_SetPowerMode(PCM5122_POWER_MODE_POWERDOWN_STANDBY);
	PCM5122_SetVolume(0);										// Volume = +24.0 dB
	PCM5122_ConfigI2SFormat(bps);
	HAL_Delay(10); 												// Wait for calibration, startup, etc

	PCM5122_LED_RED_ON();

	return (PCM5122_GetPowerState() == PCM5122_POWER_STATE_RUN);
}

/**
 * @fn uint32_t PCM5122_Setup(void)
 * @brief Initializes the audio codec and the control interface.
 *
 * @return
 */
uint32_t PCM5122_Setup(void)
{
  uint32_t counter = 0;

  counter += PCM5122_RegisterInitialize(PCM5122_SAMPLE_RATE_48K, PCM5122_BITS_PER_SAMPLE_16);//PCM5122_BITS_PER_SAMPLE_32);

  return counter;	// Return communication control value
}

/**
 * @fn uint8_t PCM5122_SetPowerMode(typePowerMode)
 * @brief Set power mode
 *
 * @param Mode:	PCM5122_POWER_MODE_ACTIVE
 * 				PCM5122_POWER_MODE_STANDBY
 * 				PCM5122_POWER_MODE_POWERDOWN
 * 				PCM5122_POWER_MODE_POWERDOWN_STANDBY
 * @return
 */
uint8_t PCM5122_SetPowerMode(typePowerMode Mode)
{
	return PCM5122_WriteRegister(PCM5122_REG_STANDBY_POWERDOWN, Mode);
}

/**
 * @fn uint8_t PCM5122_SetGpioHigh(typeGpio)
 * @brief
 *
 * @param Gpio
 * @return
 */
uint8_t PCM5122_SetGpioHigh(typeGpio Gpio)
{
	uint8_t Result = 0;
	uint8_t RegValue = 0;

	RegValue = PCM5122_ReadRegister(PCM5122_REG_GPIO_OUTPUT_CTRL);
	RegValue |= (uint8_t)Gpio;
	Result += PCM5122_WriteRegister(PCM5122_REG_GPIO_OUTPUT_CTRL, RegValue);

	return Result;
}

/**
 * @fn uint8_t PCM5122_SetGpioLow(typeGpio)
 * @brief
 *
 * @param Gpio
 * @return
 */
uint8_t PCM5122_SetGpioLow(typeGpio Gpio)
{
	uint8_t Result = 0;
	uint8_t RegValue = 0;

	RegValue = PCM5122_ReadRegister(PCM5122_REG_GPIO_OUTPUT_CTRL);
	RegValue &= ~(uint8_t)Gpio;
	Result += PCM5122_WriteRegister(PCM5122_REG_GPIO_OUTPUT_CTRL, RegValue);

	return Result;
}

/**
 * @fn uint8_t PCM5122_SetGpioToggle(typeGpio)
 * @brief
 *
 * @param Gpio
 * @return
 */
uint8_t PCM5122_SetGpioToggle(typeGpio Gpio)
{
	uint8_t Result = 0;
	uint8_t RegValue = 0;

	RegValue = PCM5122_ReadRegister(PCM5122_REG_GPIO_OUTPUT_CTRL);
	RegValue ^= (uint8_t)Gpio;
	Result += PCM5122_WriteRegister(PCM5122_REG_GPIO_OUTPUT_CTRL, RegValue);

	return Result;
}

/**
 * @fn bool PCM5122_ConfigurePLL(typeSamplingRate, typeBitDepth)
 * @brief
 *
 * @param rate
 * @param bps
 * @return
 */
bool PCM5122_ConfigurePLL(typeSamplingRate rate, typeBitDepth bps)
{
	// See here : https://e2e.ti.com/support/data_converters/audio_converters/f/64/t/428281
	// for a config example

	//---Check that the bit clock (PLL input) is between 1MHz and 50MHz
	uint32_t bckFreq = rate * bps * 2;
	if (bckFreq < 1000000 || bckFreq > 50000000)
		return FALSE;

	//---24 bits is not supported for 44.1kHz and 48kHz.
	if ((rate == PCM5122_SAMPLE_RATE_44_1K || rate == PCM5122_SAMPLE_RATE_48K) && bps == PCM5122_BITS_PER_SAMPLE_24)
		return FALSE;

	//---Initialize system clock from the I2S BCK input
	PCM5122_WriteRegister(PCM5122_REG_IGNORE_ERRORS, 0x1A); 	// Disable clock autoset and ignore SCK detection
	PCM5122_WriteRegister(PCM5122_REG_PLL_CLOCK_SOURCE, 0x10); 	// Set PLL clock source to BCK
	PCM5122_WriteRegister(PCM5122_REG_DAC_CLOCK_SOURCE, 0x10); 	// Set DAC clock source to PLL output

	//---PLL configuration
	int p, j, d, r;

	//---Clock dividers
	int nmac, ndac, ncp, dosr, idac;

	if (rate == PCM5122_SAMPLE_RATE_11_025K
			|| rate == PCM5122_SAMPLE_RATE_22_05K
			|| rate == PCM5122_SAMPLE_RATE_44_1K) {
		//44.1kHz and derivatives.
		//P = 1, R = 2, D = 0 for all supported combinations.
		//Set J to have PLL clk = 90.3168 MHz
		p = 1;
		r = 2;
		j = 90316800 / bckFreq / r;
		d = 0;

		//Derive clocks from the 90.3168MHz PLL
		nmac = 2;
		ndac = 16;
		ncp = 4;
		dosr = 8;
		idac = 1024; // DSP clock / sample rate
	}
	else
	{
		//---8kHz and multiples.
		//   PLL config for a 98.304 MHz PLL clk
		if ((bps == PCM5122_BITS_PER_SAMPLE_24) && (bckFreq > 1536000))
			p = 3;
		else if (bckFreq > 12288000)
			p = 2;
		else
			p = 1;

		r = 2;
		j = 98304000 / (bckFreq / p) / r;
		d = 0;

		//---Derive clocks from the 98.304MHz PLL
		switch (rate)
		{
			case PCM5122_SAMPLE_RATE_16K:
			{
				nmac = 6;
				break;
			}
			case PCM5122_SAMPLE_RATE_32K:
			{
				nmac = 3;
				break;
			}
			default:
			{
				nmac = 2;
				break;
			}
		}

		ndac = 16;
		ncp = 4;
		dosr = (384000 / rate);
		idac = (98304000 / nmac / rate); // DSP clock / sample rate
	}

	//---Configure PLL
	PCM5122_WriteRegister(PCM5122_REG_PLL_P, (p - 1));
	PCM5122_WriteRegister(PCM5122_REG_PLL_J, j);
	PCM5122_WriteRegister(PCM5122_REG_PLL_D_MSB, ((d >> 8) & 0x3F));
	PCM5122_WriteRegister(PCM5122_REG_PLL_D_LSB, (d & 0xFF));
	PCM5122_WriteRegister(PCM5122_REG_PLL_R, (r - 1));

	//---Clock dividers
	PCM5122_WriteRegister(PCM5122_REG_DSP_CLOCK_DIV, (nmac - 1));
	PCM5122_WriteRegister(PCM5122_REG_DAC_CLOCK_DIV, (ndac - 1));
	PCM5122_WriteRegister(PCM5122_REG_NCP_CLOCK_DIV, (ncp - 1));
	PCM5122_WriteRegister(PCM5122_REG_OSR_CLOCK_DIV, (dosr - 1));

	//---IDAC (nb of DSP clock cycles per sample)
	PCM5122_WriteRegister(PCM5122_REG_IDAC_MSB, ((idac >> 8) & 0xFF));
	PCM5122_WriteRegister(PCM5122_REG_IDAC_LSB, (idac & 0xFF));

	return TRUE;
}

/**
 * @fn uint8_t PCM5122_ConfigSpeedMode(typeSamplingRate)
 * @brief
 *
 * @param Rate
 * @return
 */
uint8_t PCM5122_ConfigSpeedMode(typeSamplingRate Rate)
{
	uint8_t Status = 0;

	int SpeedMode;

	if (Rate <= PCM5122_SAMPLE_RATE_48K)
		SpeedMode = 0;
	else if (Rate <= PCM5122_SAMPLE_RATE_96K)
		SpeedMode = 1;
	else if (Rate <= PCM5122_SAMPLE_RATE_192K)
		SpeedMode = 2;
	else
		SpeedMode = 3;

	Status+= PCM5122_WriteRegister(PCM5122_REG_FS_SPEED_MODE, SpeedMode);

	return Status;
}

/**
 * @fn uint8_t PCM5122_ConfigI2SFormat(typeBitDepth)
 * @brief
 *
 * @param bps
 * @return
 */
uint8_t PCM5122_ConfigI2SFormat(typeBitDepth bps)
{
	uint8_t Status = 0;
	uint8_t Config = 0;

	switch (bps)
	{
		case PCM5122_BITS_PER_SAMPLE_16:
		{
			Config = 0x00;
			break;
		}
		case PCM5122_BITS_PER_SAMPLE_24:
		{
			Config = 0x02;
			break;
		}
		case PCM5122_BITS_PER_SAMPLE_32:
		{
			Config = 0x03;
			break;
		}
	}

	Status += PCM5122_WriteRegister(PCM5122_REG_I2S_FORMAT, Config);

	return Status;
}

/**
 * @fn uint8_t PCM5122_Reset(void)
 * @brief Reset internal modules and registers
 *
 * @return
 */
uint32_t PCM5122_Reset(void)
{
	uint32_t counter = 0;

	counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_STANDBY);
	counter += PCM5122_WriteRegister(PCM5122_REG_RESET, 0x11);
	HAL_Delay(100);
	counter += PCM5122_WriteRegister(PCM5122_REG_RESET, 0);
	counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_ACTIVE);

	return counter;
}

/**
 * @fn typePowerState PCM5122_GetPowerState()
 * @brief Get current power state
 *
 * @return
 */
typePowerState PCM5122_GetPowerState(void)
{
  uint8_t regValue = PCM5122_ReadRegister(PCM5122_REG_DSP_BOOT_POWER_STATE);

  return (typePowerState)(regValue & 0x0F);
}

/**
 * @fn uint8_t PCM5122_SetVolume(uint8_t)
 * @brief Set volume for left and right channels.
 *
 * @param Volume:	0 = loudest (+24dB)
 * 					1 = +23.5dB
 * 					...
 * 					254 = most quiet (-103dB)
 * 					255 = mute
 * @return			0 if correct communication, else wrong communication
 */
uint32_t PCM5122_SetVolume(uint8_t Volume)
{
	uint32_t Counter = 0;

	Counter += PCM5122_WriteRegister(PCM5122_REG_DIGITAL_VOLUME_L, Volume);
	Counter += PCM5122_WriteRegister(PCM5122_REG_DIGITAL_VOLUME_R, Volume);

	return Counter;
}

/**
 * @fn uint32_t PCM5122_Play(uint16_t, uint16_t*, uint16_t)
 * @brief Start the audio Codec play feature. For this codec no Play options are required.
 *
 * @return				0 if correct communication, else wrong communication
 */

uint32_t PCM5122_Play(void)
{
	uint32_t counter = 0;

	if (PCM5122.Status == PCM5122_STOP)
	{
		counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_STANDBY);
		counter += PCM5122_WriteRegister(PCM5122_REG_GPIO_OUTPUT_CTRL, 0x28);

		//---Sync request = 1: Halt DAC, CP and OSR clocks as the beginning of resynchronization process
		counter += PCM5122_WriteRegister(PCM5122_REG_CLOCK_SYNC_REQUEST, 0x11);

		//---Sync request = 0: Resume DAC, CP and OSR clocks synchronized to the beginning of audio frame
		counter += PCM5122_WriteRegister(PCM5122_REG_CLOCK_SYNC_REQUEST, 0x10);

		counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_ACTIVE);
		counter += PCM5122_SetMute(PCM5122_AUDIO_MUTE_OFF);

		PCM5122.Status = PCM5122_PLAY;

		PCM5122.SilentCnt = 0;
		PCM5122.Ready = TRUE;
	}

	return counter;
}

/**
  * @brief Pauses playing on the audio codec.
  *
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t PCM5122_Pause(void)
{
	uint32_t counter = 0;

	//---Pause the audio file playing
	//   Mute the output first
	counter += PCM5122_SetMute(PCM5122_AUDIO_MUTE_ON);

	//---Put the Codec in Power save mode
	counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_STANDBY);

	counter += PCM5122_LED_RED_ON();

	return counter;
}

/**
  * @brief Resumes playing on the audio codec.
  *
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t PCM5122_Resume(void)
{
	uint32_t counter = 0;

	//---Resumes the audio file playing
	//   Unmute the output first
	counter += PCM5122_SetMute(PCM5122_AUDIO_MUTE_OFF);

	//---Exit the Power save mode
	counter += PCM5122_SetPowerMode(PCM5122_POWER_MODE_ACTIVE);

	counter += PCM5122_LED_BLUE_ON();

	return counter;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param DeviceAddr: Device address on communication Bus.
  * @param CodecPdwnMode: selects the  power down mode.
  *          - CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *                           mode, the codec is set to default configuration
  *                           (user should re-Initialize the codec in order to
  *                            play again the audio stream).
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t PCM5122_Stop(void)
{
	uint32_t counter = 0;

	counter += PCM5122_SetMute(PCM5122_AUDIO_MUTE_ON);
	counter += PCM5122_WriteRegister(PCM5122_REG_GPIO_OUTPUT_CTRL, 0x20);
	PCM5122.Status = PCM5122_STOP;

	return counter;
}

/**
 * @fn uint32_t PCM5122_SetMute(uint16_t, uint32_t)
 * @brief Enables or disables the mute feature on the audio codec.
 *
 * @param Command:		AUDIO_MUTE_ON = Enable the mute.
 * 						AUDIO_MUTE_OFF = Disable the mute mode.
 * @return 0 if correct communication, else wrong communication
 */
uint32_t PCM5122_SetMute(uint32_t Command)
{
	uint32_t counter = 0;

	if (Command == PCM5122_AUDIO_MUTE_ON)
		counter += PCM5122_WriteRegister(PCM5122_REG_MUTE, 0x11);	// Mute Enable
	else
		counter += PCM5122_WriteRegister(PCM5122_REG_MUTE, 0);		// Mute Disable

	return counter;
}

/**
 * @fn int16_t PCM5122_GenerateSineWave(float, int16_t*, int16_t)
 * @brief
 *
 * @param freq
 * @param outBuffer
 * @param outBuffLen
 * @return
 */
int16_t PCM5122_GenerateSineWave(float freq, int16_t *outBuffer, int16_t outBuffLen)
{
	int16_t Result;
	float sineVal;
	float sample = 50000.0f;
	float sampleDt = freq / sample;
	float sampleN = sample / freq;

	if (sampleN > outBuffLen)
		sampleN = outBuffLen;

	for (uint16_t i = 0; i < sampleN; i++)
	{
		sineVal = sinf(i * 2 * PI * sampleDt);
		outBuffer[i * 2] = (sineVal) * 8000;
		outBuffer[i * 2 + 1] = (sineVal) * 8000;
	}

	Result = sampleN;

	return Result;
}

/**
 * @fn void PCM5122_PlaySineWaveSample(uint8_t)
 * @brief
 *
 * @param Volume
 */
void PCM5122_PlaySineWaveSample(uint8_t Volume)
{
	PCM5122_SetVolume(Volume);
	PCM5122.SampleLen = PCM5122_GenerateSineWave(1500.0f, PCM5122.SampleBuff, 100);
	PCM5122_Play();
	HAL_I2S_Transmit_DMA(PCM5122_I2S_HANDLE, (uint16_t *)PCM5122.SampleBuff, PCM5122.SampleLen*2);
}


