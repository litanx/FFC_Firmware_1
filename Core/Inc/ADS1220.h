/*
 * ADS1220.h
 *
 *  Created on: Dec 12, 2024
 *      Author: diego
 *
 * ADS1220 driver for STM32 with HAL.
 * https://github.com/Spirit532/ADS1220_STM32_HAL/tree/main
 */

#ifndef INC_ADS1220_H_
#define INC_ADS1220_H_

#include "main.h" // Your project's main should include HAL

//Commands
#define ADS1220_RESET 		0x06 // Good idea to reset on power-up
#define ADS1220_START 		0x08 // Both single-shot and continuous conversion must be started via 0x08
#define ADS1220_POWERDOWN 	0x03
#define ADS1220_RDATA 		0x10
#define ADS1220_WREG  		0x40
#define ADS1220_RREG  		0x20

//Registers
#define ADS1220_CONFIG_REG0_ADDRESS 0x00
#define ADS1220_CONFIG_REG1_ADDRESS 0x01
#define ADS1220_CONFIG_REG2_ADDRESS 0x02
#define ADS1220_CONFIG_REG3_ADDRESS 0x03

//Masks
#define ADS1220_REG_CONFIG1_DR_MASK       0xE0
#define ADS1220_REG_CONFIG1_MODE_MASK     0x18
#define ADS1220_REG_CONFIG0_PGA_GAIN_MASK 0x0E
#define ADS1220_REG_CONFIG0_MUX_MASK      0xF0

//Operating Mode
#define ADS1220_MODE_NORMAL 	0x00
#define ADS1220_MODE_DUTYCYCLE	0x08
#define ADS1220_MODE_TURBO		0x10

//Sample rate (Normal Mode)
#define ADS1220_NM_20SPS   0x00
#define ADS1220_NM_45SPS   0x20
#define ADS1220_NM_90SPS   0x40
#define ADS1220_NM_175SPS  0x60
#define ADS1220_NM_330SPS  0x80
#define ADS1220_NM_600SPS  0xA0
#define ADS1220_NM_1000SPS 0xC0

//Sample rate (Turbo Mode)
#define ADS1220_TM_40SPS   0x00
#define ADS1220_TM_90SPS   0x20
#define ADS1220_TM_180SPS  0x40
#define ADS1220_TM_350SPS  0x60
#define ADS1220_TM_660SPS  0x80
#define ADS1220_TM_1200SPS 0xA0
#define ADS1220_TM_2000SPS 0xC0

//Voltage Reference
#define ADS1220_VREF_INTERNAL	0x00		//00 : Internal 2.048-V reference selected (default)
#define ADS1220_VREF_EXT_REF_0 	0x40		//01 : External reference selected using dedicated REFP0 and REFN0 inputs
#define ADS1220_VREF_EXT_REF_1 	0x80 		//10 : External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs
#define ADS1220_VREF_AN_SUPPLY  0xC0		//11 : Analogue supply (AVDD – AVSS) used as reference

//PGA gain settings
#define ADS1220_PGA_GAIN_1   0x00
#define ADS1220_PGA_GAIN_2   0x02
#define ADS1220_PGA_GAIN_4   0x04
#define ADS1220_PGA_GAIN_8   0x06
#define ADS1220_PGA_GAIN_16  0x08
#define ADS1220_PGA_GAIN_32  0x0A
#define ADS1220_PGA_GAIN_64  0x0C
#define ADS1220_PGA_GAIN_128 0x0E

//Input mux
#define ADS1220_MUX_AIN0_AIN1 0x00
#define ADS1220_MUX_AIN0_AIN2 0x10
#define ADS1220_MUX_AIN0_AIN3 0x20
#define ADS1220_MUX_AIN1_AIN2 0x30
#define ADS1220_MUX_AIN1_AIN3 0x40
#define ADS1220_MUX_AIN2_AIN3 0x50
#define ADS1220_MUX_AIN1_AIN0 0x60
#define ADS1220_MUX_AIN3_AIN2 0x70
#define ADS1220_MUX_AIN0_AVSS 0x80
#define ADS1220_MUX_AIN1_AVSS 0x90
#define ADS1220_MUX_AIN2_AVSS 0xA0
#define ADS1220_MUX_AIN3_AVSS 0xB0

#define _BV(bit) (1<<(bit))

typedef struct
{
	uint8_t cfg_reg0; // = 0x00;   //AINP=AIN0, AINN=AIN1, gain=1, PGA is enabled
	uint8_t cfg_reg1; // = 0x04;   //20SPS, Mode=Normal, Conversion=Continuous, Temp mode disabled, Current source disabled
	uint8_t cfg_reg2; // = 0x10;   //Internal 2.048V VREF, 50/60Hz filter, transistors open, IDAC off
	uint8_t cfg_reg3; // = 0x00;   //IDAC1&2 are disabled, only DRDY signals conversion completion
} ADS1220_regs;


void ADS1220_writeRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value);
uint8_t ADS1220_readRegister(SPI_HandleTypeDef *hspi, uint8_t address);

void ADS1220_reset(SPI_HandleTypeDef *hspi);
uint8_t ADS1220_init(SPI_HandleTypeDef *hspi, ADS1220_regs *r);

void ADS1220_start_conversion(SPI_HandleTypeDef *hspi);
void ADS1220_read_data(SPI_HandleTypeDef *hspi);
void ADS1220_enable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_disable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_set_operating_mode(SPI_HandleTypeDef *hspi, int op_mode, ADS1220_regs *r);
void ADS1220_set_conv_mode_continuous(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_set_conv_mode_single_shot(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_set_voltage_ref(SPI_HandleTypeDef *hspi, int volt_ref, ADS1220_regs *r);
void ADS1220_enable_PSW(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_disable_PSW(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
void ADS1220_set_data_rate(SPI_HandleTypeDef *hspi, int datarate, ADS1220_regs *r);
void ADS1220_select_mux_config(SPI_HandleTypeDef *hspi, int channels_conf, ADS1220_regs *r);
void ADS1220_set_pga_gain(SPI_HandleTypeDef *hspi, int pgagain, ADS1220_regs *r);
uint8_t* ADS1220_get_config(SPI_HandleTypeDef *hspi, ADS1220_regs *r);
int32_t ADS1220_read_blocking(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout); // Timeout should be at least as long as sampletime+some clock cycles, obviously
int32_t ADS1220_read_singleshot(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout);
int32_t ADS1220_read_singleshot_channel(SPI_HandleTypeDef *hspi, uint8_t channel_num, ADS1220_regs *r, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout);
int32_t ADS1220_read_continuous(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout);

#endif /* INC_ADS1220_H_ */
