/**
  ******************************************************************************
  * @file    DMA/FLASH_RAM/main.h 
  * @author  Gss Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include "gGPIO.h"
#include "gDelay.h"
#include "gUSART.h"
#include "gADC.h"
#include "gControlMotor.h"
#include "gTIM.h"
#include "gPID.h"
#include <math.h>
	
/*enum counter*/
enum {
	C_TIME = 0,
	MAIN = 1,
	SW_B = 3,
	SW_F = 4,
	ON_T1 = 5,
	PID_MD = 6
	
};
/*enum flag*/
enum {
	START_MAIN = 0,
	TX_EN = 1,
	TIME_START = 2,
	RX_UI = 3,
	ERRO1 = 4,
	ERRO2 = 6
};
/*enum ADC*/
enum{
	ADC_SPEED 	= 0,			
	ADC_ANGLE_L 	= 1,			
	ADC_ANGLE_R 	= 2				
};
/*enum dia chi cua cac slave*/
enum{
	ADDRESS_SLAVE_LASER_F = 258,		 	//slave co dia chi la 0x01
	ADDRESS_SLAVE_LASER_B = 257				//ta truyen 9bit bit dau bat len mot la truyen address
																		//-> dia chi can truyen la BIN: 0b1000000001 -> HEX: 0x101 -> DEC: 257
};

typedef enum{FALSE = 0, TRUE = 1} Bit;
typedef enum{LASER_FORNT = 0, LASER_BACK = 1} SLAVE;
/* main old -----------------------------------*/
/* define -------------------------------------*/
	#define SCAN_MAIN 		1
	#define NUM_BUFF_PID	5
	#define MDRIVER				0
	#define MSTEERING_L		1
	#define MSTEERING_R		2
	#define ROBOT_GS			3
	#define SPEED_MOTOR		3000
	#define PULSE_RATIO		1900					// la don vi xung/m
	 
	#define NUM_BUF_TX_USART	32
	/*define laser ---------------------------------------------------------------*/
	#define NUM_BUF_RX_LASER 	2
	/*define ADC -----------------------------------------------------------------*/
	#define NUM_BUF_ADC 			3
	/*define Motor ---------------------------------------------------------------*/
	#define GAP_PWM_MOTOR		500
	#define NUM_BUFF_MOTOR	3
	/*define SW begib & end ------------------------------------------------------*/
	#define PORT_SW 			GPIOD
	#define PIN_SW_END		GPIO_Pin_0
	#define PIN_SW_BEGIN	GPIO_Pin_1
/* variable -----------------------------------*/
	/* ADC bufffer */
	__IO uint16_t ADCValue[NUM_BUF_ADC];


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

