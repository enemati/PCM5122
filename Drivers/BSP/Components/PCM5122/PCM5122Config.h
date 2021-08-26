/*
 * PCM5122Config.h
 *
 *  Created on: Aug 18, 2021
 *      Author: Eissa NEMATI
 */

#ifndef BSP_COMPONENTS_PCM5122_PCM5122CONFIG_H_
#define BSP_COMPONENTS_PCM5122_PCM5122CONFIG_H_

#include "GeneralHeader.h"

#define PCM5122_I2C_TIMEOUT_MAX    		0x1000 	// The value of the maximal timeout for BUS waiting loops
#define PCM5122_AUDIO_I2C_ADDRESS       0x9A
#define PCM5122_I2C_HANDLE				&hi2c1
#define PCM5122_I2S_HANDLE				&hi2s3

#endif /* BSP_COMPONENTS_PCM5122_PCM5122CONFIG_H_ */
