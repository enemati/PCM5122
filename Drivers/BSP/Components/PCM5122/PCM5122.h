/*
 * PCM5122.h
 *
 *  Created on: Aug 18, 2021
 *      Author: Eissa NEMATI
 */

#ifndef BSP_COMPONENTS_PCM5122_PCM5122_H_
#define BSP_COMPONENTS_PCM5122_PCM5122_H_

#include "PCM5122Config.h"

#define PI 								3.14159f

#define PCM5122_LED_RED_ON()			PCM5122_SetGpioLow(PCM5122_GPIO_4)
#define PCM5122_LED_BLUE_ON()			PCM5122_SetGpioHigh(PCM5122_GPIO_4)

//---MUTE commands
typedef enum
{
	PCM5122_AUDIO_MUTE_OFF,
	PCM5122_AUDIO_MUTE_ON,
} typePcm5122MuteStatus;

//---Supported sampling rates
typedef enum
{
	PCM5122_SAMPLE_RATE_8K = 8000,
	PCM5122_SAMPLE_RATE_11_025K = 11025,
	PCM5122_SAMPLE_RATE_16K = 16000,
	PCM5122_SAMPLE_RATE_22_05K = 22050,
	PCM5122_SAMPLE_RATE_32K = 32000,
	PCM5122_SAMPLE_RATE_44_1K = 44100,
	PCM5122_SAMPLE_RATE_48K = 48000,
	PCM5122_SAMPLE_RATE_96K = 96000,
	PCM5122_SAMPLE_RATE_192K = 192000,
	PCM5122_SAMPLE_RATE_384K = 384000
}typeSamplingRate;

//---Supported bit depth (bits per sample)
typedef enum
{
	PCM5122_BITS_PER_SAMPLE_16 = 16,
	PCM5122_BITS_PER_SAMPLE_24 = 24,
	PCM5122_BITS_PER_SAMPLE_32 = 32,
}typeBitDepth;

//---Power modes
typedef enum
{
	PCM5122_POWER_MODE_ACTIVE = 0x00,
	PCM5122_POWER_MODE_STANDBY = 0x10,
	PCM5122_POWER_MODE_POWERDOWN = 0x01,
	PCM5122_POWER_MODE_POWERDOWN_STANDBY = 0x11
}typePowerMode;

//---Power states
typedef enum
{
	PCM5122_POWER_STATE_POWERDOWN = 0x00,
	PCM5122_POWER_STATE_WAIT_FOR_CP = 0x01,
	PCM5122_POWER_STATE_CALIB_1 = 0x02,
	PCM5122_POWER_STATE_CALIB_2 = 0x03,
	PCM5122_POWER_STATE_VOL_RAMP_UP = 0x04,
	PCM5122_POWER_STATE_RUN = 0x05,
	PCM5122_POWER_STATE_LINE_SHORT = 0x06,
	PCM5122_POWER_STATE_VOL_RAMP_DN = 0x07,
	PCM5122_POWER_STATE_STANDBY = 0x08,
	PCM5122_POWER_STATE_I2C_FAILURE = 0x0F
}typePowerState;

//---Register addresses. Set as 16-bit values (the high byte sets the page number)
typedef enum
{
	//---Page 0
	PCM5122_REG_RESET 					= 0x0000 + 1,   	// Reset
	PCM5122_REG_STANDBY_POWERDOWN 		= 0x0000 + 2,   	// Standby, Powerdown requests
	PCM5122_REG_MUTE 					= 0x0000 + 3,   	// Mute
	PCM5122_REG_PLL_LOCK_PLL_ENABLE 	= 0x0000 + 4,   	// PLL Lock Flag, PLL enable
	PCM5122_REG_SPI_MISO_FUNCTION 		= 0x0000 + 6,   	// SPI MISO function select
	PCM5122_REG_DEEMP_EN_SDOUT_SEL 		= 0x0000 + 7, 		// De-emphasis enable, SDOUT select
	PCM5122_REG_GPIO_EN 				= 0x0000 + 8,   	// GPIO enables
	PCM5122_REG_BCK_LRCK_CONFIG 		= 0x0000 + 9,   	// BCK, LRCLK configuration
	PCM5122_REG_DSG_GPIO_INPUT 			= 0x0000 + 10, 	 	// DSP GPIO Input
	PCM5122_REG_MASTER_BCK_LRCK_RESET 	= 0x0000 + 12, 		// Master mode BCK, LRCLK reset
	PCM5122_REG_PLL_CLOCK_SOURCE 		= 0x0000 + 13,  	// PLL Clock source selection
	PCM5122_REG_DAC_CLOCK_SOURCE 		= 0x0000 + 14,  	// DAC clock source selection
	PCM5122_REG_GPIO_SRC_FOR_PLL 		= 0x0000 + 18,  	// GPIO source for PLL reference
	PCM5122_REG_CLOCK_SYNC_REQUEST 		= 0x0000 + 19,  	// C
	PCM5122_REG_PLL_P 					= 0x0000 + 20,  	// PLL divider P factor
	PCM5122_REG_PLL_J 					= 0x0000 + 21, 		// PLL J part of the overall PLL multiplication factor J.D * R
	PCM5122_REG_PLL_D_MSB 				= 0x0000 + 22, 		// PLL D part (MSB) of the overall PLL multiplication factor J.D * R
	PCM5122_REG_PLL_D_LSB 				= 0x0000 + 23, 		// PLL D part (LSB) of the overall PLL multiplication factor J.D * R
	PCM5122_REG_PLL_R 					= 0x0000 + 24, 		// PLL R part of the overall PLL multiplication factor J.D * R
	PCM5122_REG_DSP_CLOCK_DIV 			= 0x0000 + 27,  	// DSP clock divider
	PCM5122_REG_DAC_CLOCK_DIV 			= 0x0000 + 28,  	// DAC clock divider
	PCM5122_REG_NCP_CLOCK_DIV 			= 0x0000 + 29,  	// CP clock divider
	PCM5122_REG_OSR_CLOCK_DIV 			= 0x0000 + 30,  	// OSR clock divider
	PCM5122_REG_MASTER_BCK_DIV 			= 0x0000 + 32,  	// Master mode BCK divider
	PCM5122_REG_MASTER_LRCK_DIV 		= 0x0000 + 33,  	// Master mode LRCK divider
	PCM5122_REG_FS_SPEED_MODE 			= 0x0000 + 34,  	// 16x interpolation, FS speed mode
	PCM5122_REG_IDAC_MSB 				= 0x0000 + 35, 		// IDAC MSB (number of DSP clock cycles available in one audio frame)
	PCM5122_REG_IDAC_LSB 				= 0x0000 + 36, 		// IDAC LSB (number of DSP clock cycles available in one audio frame)
	PCM5122_REG_IGNORE_ERRORS 			= 0x0000 + 37,  	// Ignore various errors
	PCM5122_REG_I2S_FORMAT 				= 0x0000 + 40,  	// I2S data format, word length
	PCM5122_REG_I2S_SHIFT 				= 0x0000 + 41,  	// I2S shift
	PCM5122_REG_DAC_DATA_PATH 			= 0x0000 + 42,  	// DAC data path
	PCM5122_REG_DSP_PROGRAM_SEL 		= 0x0000 + 43,  	// DSP program selection
	PCM5122_REG_CLK_MISSING_DETECTION 	= 0x0000 + 44, 		// Clock missing detection period
	PCM5122_REG_AUTO_MUTE_TIME 			= 0x0000 + 59,  	// Auto mute time
	PCM5122_REG_DIGITAL_VOLUME_CTRL 	= 0x0000 + 60,  	// Digital volume control
	PCM5122_REG_DIGITAL_VOLUME_L 		= 0x0000 + 61, 		// Digital volume for left channel
	PCM5122_REG_DIGITAL_VOLUME_R 		= 0x0000 + 62, 		// Digital volume for right channel
	PCM5122_REG_DIGITAL_VOLUME_RAMP 	= 0x0000 + 63, 		// Digital volume ramp up/down behavior
	PCM5122_REG_DIGITAL_VOLUME_EMERG 	= 0x0000 + 64, 		// Digital volume emergency ramp down behavior
	PCM5122_REG_AUTO_MUTE 				= 0x0000 + 65,  	// Auto mute
	PCM5122_REG_GPIOn_OUTPUT_SEL 		= 0x0000 + 80, 		// GPIOn output selection. GPIO1 - GPIO6 are available (80 - 85)
	PCM5122_REG_GPIO_OUTPUT_CTRL 		= 0x0000 + 86,  	// GPIO output value
	PCM5122_REG_GPIO_OUTPUT_INVERT 		= 0x0000 + 87,  	// GPIO output inversion
	PCM5122_REG_DSP_OVERFLOW 			= 0x0000 + 90,  	// DSP overflow status
	PCM5122_REG_DET_FS_SCK_RATIO 		= 0x0000 + 91,  	// Detected FS and SCK ratio
	PCM5122_REG_DET_BCK_RATIO_MSB 		= 0x0000 + 92,  	// Detected BCK ratio (MSB)
	PCM5122_REG_DET_BCK_RATIO_LSB 		= 0x0000 + 93,  	// Detected BCK ratio (LSB)
	PCM5122_REG_CLOCK_STATUS 			= 0x0000 + 94, 		// Various clock and sample rate status
	PCM5122_REG_CLOCK_ERROR 			= 0x0000 + 95,  	// Critical clock failures reporting
	PCM5122_REG_ANALOG_MUTE_MON 		= 0x0000 + 108, 	// Analog mute monitor
	PCM5122_REG_SHORT_DETECT_MON 		= 0x0000 + 109, 	// Line output short monitor
	PCM5122_REG_XSMUTE_MON 				= 0x0000 + 114, 	// XSMUTE pin level monitor
	PCM5122_REG_FS_SPEED_MODE_MON 		= 0x0000 + 115, 	// FS speed mode monitor
	PCM5122_REG_DSP_BOOT_POWER_STATE 	= 0x0000 + 118, 	// DSP boot done flag & power state
	PCM5122_REG_GPIO_INPUT 				= 0x0000 + 119, 	// GPIO input
	PCM5122_REG_AUTO_MUTE_FLAGS 		= 0x0000 + 120, 	// Auto Mute flags

	//---Page 1
	PCM5122_REG_OUTPUT_AMPL_TYPE 		= 0x0100 + 1,   	// Output amplitude type
	PCM5122_REG_ANALOG_GAIN_CTRL 		= 0x0100 + 2,   	// Analog gain control
	PCM5122_REG_UV_PROTECTION 			= 0x0100 + 5,   	// Undervoltage protection
	PCM5122_REG_ANALOG_MUTE_CTRL 		= 0x0100 + 6,   	// Analog mute control
	PCM5122_REG_ANALOG_GAIN_BOOST 		= 0x0100 + 7,   	// Analog gain boost
	PCM5122_REG_VCOM_RAMP_SPEED 		= 0x0100 + 8,   	// VCOM ref ramp up speed
	PCM5122_REG_VCOM_POWERDOWN 			= 0x0100 + 9,   	// VCOM powerdown control

	//---Page 44
	PCM5122_REG_CRAM_CONTROL 			= 0x2C00 + 1,   	// Coefficient memory (CRAM) control

	//---Page 253
	PCM5122_REG_CLOCK_FLEX_1 			= 0xFD00 + 63,  	// Clock flex register #1
	PCM5122_REG_CLOCK_FLEX_2 			= 0xFD00 + 64,  	// Clock flex register #2
}typeRegister;

typedef enum
{
	PCM5122_GPIO_1		= 0x01,
	PCM5122_GPIO_2		= 0x02,
	PCM5122_GPIO_3		= 0x04,
	PCM5122_GPIO_4		= 0x08,
	PCM5122_GPIO_5		= 0x10,
	PCM5122_GPIO_6		= 0x20,
	PCM5122_GPIO_ALL	= 0x3f,
}typeGpio;

typedef enum
{
	PCM5122_STOP,
	PCM5122_PLAY,
}typePCM5122Status;

typedef struct
{
	uint32_t SilentCnt;
	bool Ready;
	typePCM5122Status Status;
	int16_t SampleBuff[100];
	int16_t SampleLen;
} typePCM5122;

extern typePCM5122 PCM5122;

uint8_t PCM5122_SelectPage(typeRegister);
uint8_t PCM5122_WriteRegister(typeRegister, uint8_t);
uint8_t PCM5122_ReadRegister(typeRegister);
bool PCM5122_RegisterInitialize(typeSamplingRate, typeBitDepth);
uint32_t PCM5122_Setup(void);
uint32_t PCM5122_Reset(void);
typePowerState PCM5122_GetPowerState(void);
uint32_t PCM5122_SetVolume(uint8_t);
uint8_t PCM5122_SetPowerMode(typePowerMode);

uint32_t PCM5122_Play(void);
uint32_t PCM5122_Pause(void);
uint32_t PCM5122_Resume(void);
uint32_t PCM5122_Stop(void);
uint32_t PCM5122_SetMute(uint32_t);

bool PCM5122_ConfigurePLL(typeSamplingRate, typeBitDepth);
uint8_t PCM5122_ConfigSpeedMode(typeSamplingRate);
uint8_t PCM5122_ConfigI2SFormat(typeBitDepth);
uint8_t PCM5122_SetGpioHigh(typeGpio);
uint8_t PCM5122_SetGpioLow(typeGpio);
uint8_t PCM5122_SetGpioToggle(typeGpio);

int16_t PCM5122_GenerateSineWave(float freq, int16_t *outBuffer, int16_t outBuffLen);
void PCM5122_PlaySineWaveSample(uint8_t Volume);

#endif /* BSP_COMPONENTS_PCM5122_PCM5122_H_ */
