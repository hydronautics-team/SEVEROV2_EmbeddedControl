#include <math.h>
#include <MS5837.h>
//#include <string.h>
// For printf
//#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "i2c_er.h"

typedef enum {
	MS5837_I2C_NONE,
	MS5837_I2C_D1_TX_CONV_COM,
	MS5837_I2C_D1_CONVERTION,
	MS5837_I2C_D1_TX_ADC_COM,
	MS5837_I2C_D1_RX_ADC,
	MS5837_I2C_D2_TX_CONV_COM,
	MS5837_I2C_D2_CONVERTION,
	MS5837_I2C_D2_TX_ADC_COM,
	MS5837_I2C_D2_RX_ADC,
} MS5837_I2C_States;


I2C_HandleTypeDef * MS5837_hi2c;

#define	COMMAND_LENGTH 1
#define ADC_LENGTH 3
#define PROM_LENGTH 2
#define SEND_TIME 1500
#define RECEIVE_TIME 1500
#define DEVICE_ADDR  0b11101100
#define CONVERTION_TIMEOUT 20
#define TIMEOUT 200

//commands
uint8_t RES_DEVICE_COMM= 0x1E;
uint8_t CONVERT_D1_COMM = 0x40;//0x4A;
uint8_t CONVERT_D2_COMM = 0x52;//0x5A;
uint8_t ADC_READ_COMM = 0x00;

uint64_t C[7];
uint64_t D1, D2;
uint32_t D1_pres, D2_temp;
int32_t TEMP;
int32_t P;
int32_t presure;
int32_t initial_pressure = 0;

uint8_t adc_buff[3];
MS5837_I2C_States MS5837_I2C_State = MS5837_I2C_NONE;
uint32_t MS5837_conv_start_tick = CONVERTION_TIMEOUT;

void calculate();
void init_new_convertion();

int cmp(const void *a, const void *b) {
    return *(int*)a - *(int*)b;
}

bool MS5837_02BA_begin(I2C_HandleTypeDef * hi2c){
	return(MS5837_02BA_init(hi2c));
}

bool MS5837_02BA_init(I2C_HandleTypeDef * hi2c){
	MS5837_hi2c = hi2c;
	HAL_I2C_Init(hi2c);
	if(HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &RES_DEVICE_COMM, COMMAND_LENGTH, HAL_MAX_DELAY) != HAL_OK){
		return false;
	}
	//receiving C1 - C6
	uint8_t prom_addr = 0xA0;
	for (uint8_t i = 0; i < 7; i++) {
		uint8_t prom_buff[2];
		prom_addr += 2;
		HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &prom_addr, COMMAND_LENGTH, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(MS5837_hi2c, DEVICE_ADDR, prom_buff, PROM_LENGTH, HAL_MAX_DELAY);
		C[i] = (prom_buff[0] << 8) | (prom_buff[1]);
	}
	return true;
}

int32_t MS5837_02BA_get_actual_pressure(){
	uint32_t current_tick = HAL_GetTick();
	if(MS5837_I2C_NONE == MS5837_I2C_State){
		init_new_convertion();
	}
	if(MS5837_I2C_D1_CONVERTION == MS5837_I2C_State && current_tick - MS5837_conv_start_tick >= CONVERTION_TIMEOUT){
		MS5837_I2C_State = MS5837_I2C_D1_TX_ADC_COM;
		HAL_I2C_Master_Transmit_IT(MS5837_hi2c, DEVICE_ADDR, &ADC_READ_COMM, COMMAND_LENGTH);
	}
	if(MS5837_I2C_D2_CONVERTION == MS5837_I2C_State && current_tick - MS5837_conv_start_tick >= CONVERTION_TIMEOUT){
		MS5837_I2C_State = MS5837_I2C_D2_TX_ADC_COM;
		HAL_I2C_Master_Transmit_IT(MS5837_hi2c, DEVICE_ADDR, &ADC_READ_COMM, COMMAND_LENGTH);
	}
//	HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(MS5837_hi2c);
	return presure-initial_pressure;
}

void init_new_convertion(){
	MS5837_I2C_State = MS5837_I2C_D1_TX_CONV_COM;
	HAL_I2C_Master_Transmit_IT(MS5837_hi2c, DEVICE_ADDR, &CONVERT_D1_COMM, COMMAND_LENGTH);
//	MS5837_conv_start_tick = HAL_GetTick();
}

void MS5837_I2C_MasterRxCplt (I2C_HandleTypeDef * hi2c){
	if(hi2c == MS5837_hi2c){
		if(MS5837_I2C_D1_RX_ADC == MS5837_I2C_State){
			D1 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);
			MS5837_I2C_State = MS5837_I2C_D2_TX_CONV_COM;
			HAL_I2C_Master_Transmit_IT(MS5837_hi2c, DEVICE_ADDR, &CONVERT_D2_COMM, COMMAND_LENGTH);
		}
		if(MS5837_I2C_D2_RX_ADC == MS5837_I2C_State){
			D2 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);
			init_new_convertion();
			calculate();
		}
//		MS5837_conv_start_tick = HAL_GetTick();
	}
}

void MS5837_I2C_MasterTxCplt (I2C_HandleTypeDef * hi2c){
	if(hi2c == MS5837_hi2c){
		if(MS5837_I2C_D1_TX_CONV_COM == MS5837_I2C_State){
			MS5837_I2C_State = MS5837_I2C_D1_CONVERTION;
			MS5837_conv_start_tick = HAL_GetTick();
		}
		if(MS5837_I2C_D1_TX_ADC_COM == MS5837_I2C_State){
			MS5837_I2C_State = MS5837_I2C_D1_RX_ADC;
			HAL_I2C_Master_Receive_IT(MS5837_hi2c, DEVICE_ADDR, adc_buff, ADC_LENGTH);
		}
		if(MS5837_I2C_D2_TX_CONV_COM == MS5837_I2C_State){
			MS5837_I2C_State = MS5837_I2C_D2_CONVERTION;
			MS5837_conv_start_tick = HAL_GetTick();
		}
		if(MS5837_I2C_D2_TX_ADC_COM == MS5837_I2C_State){
			MS5837_I2C_State = MS5837_I2C_D2_RX_ADC;
			HAL_I2C_Master_Receive_IT(MS5837_hi2c, DEVICE_ADDR, adc_buff, ADC_LENGTH);
		}
	}
}

void MS5837_I2C_MasterError (I2C_HandleTypeDef * hi2c){
	__NOP();
}

int32_t MS5837_02BA_check_pressure(){
	//initializing D1 conversion
	HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &CONVERT_D1_COMM, COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_Delay(20);
	//reading D1 data
	HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &ADC_READ_COMM, COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5837_hi2c, DEVICE_ADDR, adc_buff, ADC_LENGTH, HAL_MAX_DELAY);
	D1 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);
	//initializing D2 conversion
	HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &CONVERT_D2_COMM, COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_Delay(20);
	//reading D2 data
	HAL_I2C_Master_Transmit(MS5837_hi2c, DEVICE_ADDR, &ADC_READ_COMM, COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5837_hi2c, DEVICE_ADDR, adc_buff, ADC_LENGTH, HAL_MAX_DELAY);
	D2 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);
	calculate();
	return presure-initial_pressure;
}

uint32_t MS5837_02BA_reset_pressure(){
	int32_t current_pressure[25];

	for(int i =0;i<25;i++)
	{
		current_pressure[i]=MS5837_02BA_check_pressure();
	}
	qsort(current_pressure, 25, sizeof(int32_t), cmp );
	init_new_convertion();
	return initial_pressure = current_pressure[24];
}

void calculate(){
	int64_t dT = 0;
	int64_t TEMP = 0;
	int64_t OFF = 0;
	int64_t SENS = 0;
	int64_t P = 0;

	int64_t SENSi = 0;
	int64_t OFFi = 0;
	int64_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	int64_t P2 = 0;
	int64_t TEMP2 = 0;

	//first order compensation
	dT = D2 -  C[4]*256;
	TEMP = 2000 + dT*C[4]/8388608;

//	OFF = C[1]*65536 + (C[3]*dT)/128;
//	SENS = C[0]*32768 + (C[2]*dT)/256;
//	P = ((D1 * SENS)/(2097152) - OFF)/8192;

	OFF = (C[1])*131072+(C[3]*dT)/64;

	SENS = C[0]*65536+(C[2]*dT)/128;

	P = (D1*SENS/(2097152)-OFF)/(32768);
	//second order compensation
	//	if (TEMP/100 < 20)
	//	{
	//		Ti = (3*dT*dT)/8589934592LL;
	//		OFFi = (3*(TEMP-2000)*(TEMP - 2000))/2;
	//		SENSi = (5*(TEMP - 2000)*(TEMP-2000))/8;
	//		if (TEMP/100 < -15)
	//		{
	//			OFFi = OFFi + 7*(TEMP + 1500l)*(TEMP + 1500l);
	//			SENSi = SENSi + 4*(TEMP + 1500l)*(TEMP + 1500l);
	//		}
	//	} else
	//	{
	Ti = (2*dT*dT)/1.37438953E11;
	OFFi = ((TEMP-2000)*(TEMP - 2000))/16;
	SENSi = 0;

	//	}

	OFF2 = OFF - OFFi;
	SENS2 = SENS - SENSi;

	TEMP2 = (TEMP - Ti) / 100; //C
	P2 = (((D1*SENS2)/2097152 - OFF2)/8192)/10; //mbar
	int32_t res[2];
	res[0] = P;
	res[1] = TEMP/100;
	//	char snum1[50];
	//	itoa(9999, snum1, 6);
	//	strcat(snum1,"\r\n");
	//	char snum1[50];
	//	itoa((int)P, snum1, 10);
	//	strcat(snum1,"\r\n");
	//	HAL_UART_Transmit(&huart2, (uint8_t*) snum1, strlen(snum1),6);



	presure = res[0]/10;
}

//void calculate(){
//	// Given C1-C6 and D1, D2, calculated TEMP and P
//	// Do conversion first and then second order temp compensation
//
//	int32_t dT = 0;
//	int64_t SENS = 0;
//	int64_t OFF = 0;
//	int32_t SENSi = 0;
//	int32_t OFFi = 0;
//	int32_t Ti = 0;
//	int64_t OFF2 = 0;
//	int64_t SENS2 = 0;
//
//	// Terms called
//	dT = D2_temp - (uint32_t)C[5]*256l;
//	SENS = (int64_t)C[1]*65536l+((int64_t)C[3]*dT)/128l;
//	OFF = (int64_t)C[2]*131072l+((int64_t)C[4]*dT)/64l;
//	P = (D1_pres*SENS/(2097152l)-OFF)/(32768l);
//
//	// Temp conversion
//	TEMP = 2000l+(int64_t)dT*C[6]/8388608LL;
//
//	//Second order compensation
//	if((TEMP/100)<20){         //Low temp
//		Ti = (11*(int64_t)dT*(int64_t)dT)/(34359738368LL);
//		OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
//		SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
//	}
//
//	//Calculate pressure and temp second order
//	OFF2 = OFF-OFFi;
//	SENS2 = SENS-SENSi;
//
//	TEMP = (TEMP-Ti);
//
//	P = (((D1_pres*SENS2)/2097152l-OFF2)/32768l);
//
//}
