/**
  ******************************************************************************
  * @file    main.c 
  * @author  GSS Team
  * @version V1.0.0
  * @date    3-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Ma chuong trinh dieu khien robot G1 cho he thong Robot phun thuoc BVTV
  * <h2><center>&copy; COPYRIGHT 2012 Gremsy Co., Ltd</center></h2>
  ******************************************************************************  
  */ 
/* Private define ------------------------------------------------------------------*/
//define in main
#define test 0

#define SCAN_MAIN_RUN		1
#define SCAN_MAIN_TEST	1
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
#define LOST_SIGNAL					0
/*define ADC -----------------------------------------------------------------*/
#define NUM_BUF_ADC 			3
/*define Motor ---------------------------------------------------------------*/
#define GAP_PWM_MOTOR		500
#define NUM_BUFF_MOTOR	3
/*define SW begib & end ------------------------------------------------------*/
#define PORT_SW 			GPIOD
#define PIN_SW_END		GPIO_Pin_0
#define PIN_SW_BEGIN	GPIO_Pin_1

#define PIN_LASER_LEFT	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)
#define PIN_LASER_RIGHT	GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)
/* Private Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gPID.h"
#include <math.h>
/* Private typedef -------------------------------------------------------------------*/

typedef struct{
	int D;
	int V;
	int P;
	char checked;
} GARDEN;

typedef struct{ 
	char point;
	char flag;
	char length;
	char rxData[200];
}BUFF;
/* Private macro -----------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------*/
/* allway user */
Bit flag[10];
__IO uint16_t count[10];

/* garden parameter-----------*/
GARDEN garden;

/* motor parameter*/
int aset;
MOTOR Motor[3];
char tempID;
/* ADC bufffer */
__IO uint16_t ADCValue[NUM_BUF_ADC];
/* usart */
char txBufUsart[NUM_BUF_TX_USART];
BUFF rxUI;
char pointTX;

/* PID ---- */
tPID velocityPID;
tPID posPID;
tPID aPID;
tPID stL_PID;
tPID stR_PID;

double Kcoeff[5][3] ={{0.01,0,0},
											{0,0,0},
											{0,0,0},
											{20,0,0},
											{20,0,0}};
double ABCcoeff[5][3];
double historyControl[5][3];

int32_t countEncoder,countEncoderOld;
double vCurrent;
double pwmPID[3];
double velocitySet;

int speed,angleLeft,angleRight;
int OFFSET_ST;
int Ka, Ks;
//capture variable.
__IO uint32_t CaptureEncoder = 0;

/* Private function prototypes -----------------------------------------------*/
/* usart -----------------------------------------------*/
void delRxUI(void);
void txComputer(char c);
Bit CheckDataRxUI(void);
double ABS(double a);
/* support ---------------------------------------------*/
void initPID(void);
void configMotor(void);
void controlMD(void);
void controlSTL(void);
void controlSTR(void);

/*laser sensor ----------------------------------------*/
void controlRobot(void);
void mainTest(void);
void mainRun(void);
void mainStop(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
	gpioInitMode();
	usartInitMode();
	adcInitMode(); 
	TimInitMode();
	configMotor();
	initPID();
	delayMs(2000);
	putStringUsart(CMT_PRODUCT,USART2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);
	
  while (1){
		count[MAIN]++;
		if(test){
			mainTest();
		}
		else{
			mainRun();
		}
  }
}

/**
	@void: mainTest
	@para: none.
	@returnVar: none.
	@function: test cac ham process trong main.
*/
void mainTest(void){
	if(flag[0] == TRUE){
			if(CheckDataRxUI() == TRUE) 	GPIO_SetBits(GPIOD, GPIO_Pin_14);
			else 													GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	
			flag[0] = FALSE;
	}
	
	if(flag[START_MAIN] == TRUE){
	}
	else{
		Motor[0].pwm = 0;
		Motor[1].pwm = 0;
		Motor[2].pwm = 0;
		controlMotor(Motor[0]);
		controlMotor(Motor[1]);
		controlMotor(Motor[2]);			
	}
	delayMs(SCAN_MAIN_TEST);
}

/**
	@void: mainRun
	@para: none.
	@returnVar: none.
	@function: chua cac ham process trong main.
*/
void mainRun(void){
		/* kiem tra du lieu truyen ve*/
		if(flag[0] == TRUE){
  			if(CheckDataRxUI() == TRUE) 	GPIO_SetBits(GPIOD, GPIO_Pin_14);
  			else 													GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				flag[0] = FALSE;
		}
		
		if(flag[START_MAIN] == TRUE){
				angleLeft = readAdc(ADC_ANGLE_L);
				angleRight = readAdc(ADC_ANGLE_R);
				speed = readAdc(ADC_SPEED);
				/* chon goc pe lai cho robot */
				if(Motor[0].dir == CCW){
					OFFSET_ST = 100;
				}
				else{
					OFFSET_ST = -100;
				}
				
				controlMotor(Motor[MDRIVER]);
		}
		else{
				mainStop();
		}
		
		/* truyen du lieu len may tinh */
 		if(count[MAIN] == 300){
			count[MAIN] = 0;
			if(flag[TX_EN] == TRUE)	txComputer(1);
 		}
		
		delayMs(SCAN_MAIN_RUN);
}

void mainStop(void){
	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	Motor[0].pwm = 0;
	Motor[1].pwm = 0;
	Motor[2].pwm = 0;
	controlMotor(Motor[0]);
	controlMotor(Motor[1]);
	controlMotor(Motor[2]);
}

/* function support ------------------------------------*/
/**
	@void: initPID
	@para: none.
	@returnVar: none.
	@fuction: cai dat cac PID:
											- posPID( motor driver)
											- velocityPID( motor driver)
											- stR_PID( motor steering right)
											- stL_PID( motor steering left)
*/
void initPID(void){
	PIDInit(&velocityPID);
	velocityPID.ABCcoeff = &ABCcoeff[0][0];
	velocityPID.controlHistory = &historyControl[0][0];
	PIDCoeffCalc(&Kcoeff[0][0],&velocityPID);	
	
	PIDInit(&posPID);
	posPID.ABCcoeff = &ABCcoeff[1][0];
	posPID.controlHistory = &historyControl[1][0];
	PIDCoeffCalc(&Kcoeff[1][0],&posPID);
	
	PIDInit(&stL_PID);
	stL_PID.ABCcoeff = &ABCcoeff[3][0];
	stL_PID.controlHistory = &historyControl[3][0];
	PIDCoeffCalc(&Kcoeff[3][0],&stL_PID);
	
	PIDInit(&stR_PID);
	stR_PID.ABCcoeff = &ABCcoeff[4][0];
	stR_PID.controlHistory = &historyControl[4][0];
	PIDCoeffCalc(&Kcoeff[4][0],&stR_PID);
	
	stR_PID.controlReference = MID_STR;
	stL_PID.controlReference = MID_STL;
}

/**
	@void: configMotor
	@para: none.
	@returnVar: none.
	@fuction: - config pwms cho cac motor
					 - Set id hoat dong cho tung motor.	
*/
void configMotor(void){
	MotorConfig();
	Motor[MDRIVER].id = md;
	Motor[MSTEERING_L].id = msl;
	Motor[MSTEERING_R].id = msr;
}

/**
	@void: controlMD
	@para: none.
	@returnVar: none.
	@fuction: dieu khien motor Driver theo PID Conttrol.
						- velocity
						- position
*/
void controlMD(void){
	count[9]++;
	
	countEncoder += TIM_GetCounter(TIM8) - 30000;
	vCurrent = 200000.0/CaptureEncoder;
	
	/* control theo van toc --------------------------------------*/
	velocityPID.measureOutput =(double)vCurrent;	//nhap gia tri hien tai
	pwmPID[MDRIVER] += PID(&velocityPID);	
	if(pwmPID[MDRIVER] > 4095) pwmPID[MDRIVER] = 4095;
	else if(pwmPID[MDRIVER] < 0) pwmPID[MDRIVER] = 0;
	Motor[MDRIVER].pwm = (int)pwmPID[MDRIVER]; 
	controlMotor(Motor[MDRIVER]);
	
	if(count[9]%10 == 0){
			/* control theo vi tri --------------------------------------*/
			posPID.measureOutput = countEncoder;
			velocitySet = PID(&posPID);
		
			if(velocitySet < 0){
				velocitySet = -velocitySet;
				Motor[MDRIVER].dir = CCW;
			}
			else{
				Motor[MDRIVER].dir =CW;
			}

			if(velocitySet > garden.V) velocitySet = garden.V;
			velocityPID.controlReference = (double)velocitySet;
	}
}

/**
	@void: controlSTL
	@para: none.
	@returnVar: none.
	@fuction: dieu khien motor steering left theo PID Conttrol
*/
void controlSTL(void){
	stL_PID.measureOutput = angleLeft;

	pwmPID[1] = PID(&stL_PID);
	
	if(pwmPID[1] < 0 && stL_PID.measureOutput > MIN_STL){
		pwmPID[1] = -pwmPID[1];
		Motor[MSTEERING_L].dir = CCW;
	}
	else if(pwmPID[1] > 0 && stL_PID.measureOutput < MAX_STL){
		Motor[MSTEERING_L].dir = CW;
	}
	else{
		pwmPID[1] = 0;
	}
	
	if(pwmPID[1] > 4095) pwmPID[1] = 4095;
	Motor[MSTEERING_L].pwm = (int)pwmPID[1]; 
	controlMotor(Motor[MSTEERING_L]);	
}

/**
	@void: controlSTR
	@para: none.
	@returnVar: none.
	@fuction: dieu khien motor steering right theo PID Conttrol
*/
void controlSTR(void){
	stR_PID.measureOutput = angleRight;
	pwmPID[2] = PID(&stR_PID);
	
	if(pwmPID[2] < 0 && stL_PID.measureOutput > MIN_STR){
		pwmPID[2] = -pwmPID[2];
		Motor[MSTEERING_R].dir = CCW;
	}
	else if(pwmPID[2] > 0 && stL_PID.measureOutput < MAX_STR){
		Motor[MSTEERING_R].dir = CW;
	}
	else{
		pwmPID[2] = 0;
	}
	
	if(pwmPID[2] > 4095) pwmPID[2] = 4095;
	Motor[MSTEERING_R].pwm = (int)pwmPID[2]; 
	controlMotor(Motor[MSTEERING_R]);	
}


/**
	@void: controlRobot
	@para: int PIN_LASER_LEFT
				 int PIN_LASER_RIGHT
	@returnVar: none.
	@fuction: ham tim setpoint cho hai dong steering va control motor driver
*/
void controlRobot(void){
	
	Motor[MDRIVER].pwm = speed;
	
	if(PIN_LASER_LEFT == LOST_SIGNAL && PIN_LASER_RIGHT == LOST_SIGNAL){
		/* LOST SIGNAL -------------------------------------*/
		STM_EVAL_LEDOn(LED3);
		STM_EVAL_LEDOff(LED4);
		STM_EVAL_LEDOff(LED5);
		STM_EVAL_LEDOff(LED6);
		
		Motor[MDRIVER].pwm = 0; 
	}
	else if(PIN_LASER_LEFT != LOST_SIGNAL && PIN_LASER_RIGHT == LOST_SIGNAL){
		/* LEFT ---------------------------------------------*/
		STM_EVAL_LEDOn(LED4);
		STM_EVAL_LEDOff(LED3);
		STM_EVAL_LEDOff(LED5);
		STM_EVAL_LEDOff(LED6);
		
		stR_PID.controlReference = MID_STR - OFFSET_ST;	//lui
		stL_PID.controlReference = MID_STL - OFFSET_ST;
		
	}
	else if(PIN_LASER_LEFT == LOST_SIGNAL && PIN_LASER_RIGHT != LOST_SIGNAL){
		/* RIGHT ---------------------------------------------*/
		STM_EVAL_LEDOn(LED5);
		STM_EVAL_LEDOff(LED3);
		STM_EVAL_LEDOff(LED4);
		STM_EVAL_LEDOff(LED6);

		stR_PID.controlReference = MID_STR + OFFSET_ST; //lui
		stL_PID.controlReference = MID_STL + OFFSET_ST;
	}
	else if(PIN_LASER_LEFT != LOST_SIGNAL && PIN_LASER_RIGHT != LOST_SIGNAL){
		/* MID  ---------------------------------------------*/
		STM_EVAL_LEDOn(LED6);
		STM_EVAL_LEDOff(LED3);
		STM_EVAL_LEDOff(LED4);
		STM_EVAL_LEDOff(LED5);

		stR_PID.controlReference = MID_STR;
		stL_PID.controlReference = MID_STL;
	}
	
	controlSTL();
 	controlSTR();
}

/**
	@void: ABS
	@para: doble a
	@returnVar: double |a|
	@fuction: tra ve gia tri ko am cho so dua vao.
*/
double ABS(double a){
	if(a<0) return -a;
	else		return a;
}

/**
	@void: delRxUI
	@para: none
	@returnVar: none.
	@fuction: xoa cac phan tu trong mang rxUI.rxData[]
*/
void delRxUI(void){
	while(rxUI.point != 0){
		rxUI.rxData[rxUI.point--] = '\0';
	}
}

/**
	@void: CheckDataRxUI
	@para: none
	@returnVar: Bit(TRUE,FALSE)
	@fuction: - kiem tra du lieu truyen tu panel  co err hay ko? if TRUE them err
																														else if FALSE them successful
					 - nhap du lieu dc truyen vao cac mang da dc quy dinh
*/
Bit CheckDataRxUI(void){
		Bit temp = FALSE;
		char tempValue = 0;
		char i;
	
		if(rxUI.rxData[1] == 1){
			if(rxUI.rxData[2] == 'r'){
				TIM_Cmd(TIM4, ENABLE);
				flag[START_MAIN] = TRUE;
			}
			else if(rxUI.rxData[2] == 's'){
				TIM_Cmd(TIM4, DISABLE);
				flag[START_MAIN] = FALSE;
			}
			else if(rxUI.rxData[2] == 't'){
				flag[TX_EN] = TRUE;
				pointTX++;
				if(pointTX > 2)	pointTX = 0;
			}
		}
		else if(rxUI.rxData[1] == 2){
			for(i = 2; i < rxUI.length; i++){
				tempValue += rxUI.rxData[i];
			}
			
			if(tempValue == rxUI.rxData[rxUI.length]){
				garden.D = rxUI.rxData[2];garden.V = rxUI.rxData[3]*100; garden.P = rxUI.rxData[4];
				posPID.controlReference = (double)(garden.D*PULSE_RATIO);
				
				/* cac hang so pid cua Motor Driver*/
				Kcoeff[1][0] = (double)rxUI.rxData[5]/10.0;
				Kcoeff[1][1] = (double)rxUI.rxData[6]/1000.0;
				Kcoeff[1][2] = (double)rxUI.rxData[7]/10.0;
				
				/* thong so danh lai */
				Ks = rxUI.rxData[14];
				Ka = rxUI.rxData[15];

				PIDCoeffCalc(&Kcoeff[1][0],&posPID);
			}
			else temp = TRUE;
		}
		else if(rxUI.rxData[1] == 3){
			if( rxUI.rxData[4] == (char)(rxUI.rxData[2]+rxUI.rxData[3])){
				tempID = rxUI.rxData[2];
				Motor[tempID].dir = rxUI.rxData[3];
//				Motor[tempID].pwm = SPEED_MOTOR;
			}
			else{
				temp = TRUE;
			}
		}		
		return temp;
}

/**
	@void: txComputer
	@para: char c
	@returnVar: none.
	@fuction: truyen du lieu len may tinh theo cac mode:
						c == 0 -> dung viec truyen len may tinh
						c == 1 -> tryen thong so thay doi, ex: ADC, laser, tx: continus
						c == 2 -> truyen thong so set cung, ex: kp,ki,kd..., tx: onshot
*/
void txComputer(char c){
	if(c == 1){
 		sprintf(txBufUsart,"ADC_ANGLE_L: %d\r\n",speed);
 		putStringUsart(txBufUsart,USART2);		
	}
	else if(c == 2){
		sprintf(txBufUsart,"GARDEN-> D: %3.4f  \t V: %d \t P: %d\n",posPID.controlReference,garden.V,garden.P);
		putStringUsart(txBufUsart,USART2);
		sprintf(txBufUsart,"VPID-> Kp: %3.4f  \t Ki: %3.4f \t Kd: %3.4f\n",Kcoeff[0][0],Kcoeff[0][1],Kcoeff[0][2]);
		putStringUsart(txBufUsart,USART2);
		sprintf(txBufUsart,"PPID-> Kp: %3.4f  \t Ki: %3.4f \t Kd: %3.4f\n",Kcoeff[1][0],Kcoeff[1][1],Kcoeff[1][2]);
		putStringUsart(txBufUsart,USART2);
	}
}


/* function interrupt*/
/**
	@void: USART2_IRQHandler
	@para: none
	@rerurnVar: none.
	@fuction: chuong trinh duoc goi khi co du lieu duoc truyen toi
	usart2 	interface slave 	-> computer
														-> touch pad
					connect portA 		-> pin2-RX
														-> pin3-TX
*/
/**
	@void: USART2_IRQHandler
	@para: none
	@rerurn var: none.
	fuction: chuong trinh duoc goi khi co du lieu duoc truyen toi
*/
void USART2_IRQHandler(void)       
{
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
    {
			/* nhan du lieu tu may tinh */
      char data = USART_ReceiveData(USART2);

			if(data == 254){
				rxUI.flag = 1;				
				delRxUI();
				rxUI.point = 0;
			}
			else{
				if(rxUI.flag == 1){
						if(rxUI.point == 0){
							rxUI.length = data;
							rxUI.point = 1;
						}
						else if(rxUI.point < rxUI.length){
							rxUI.rxData[rxUI.point++] = data;
						}
						else if(rxUI.point == rxUI.length){
							rxUI.rxData[rxUI.length] = data;
							rxUI.flag = 0;
							flag[0] = TRUE;
						}
				}
			}
		}
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
	int capture;
/* PROCESS TIME -------------------------------------------------------------------------------------*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
	/* CONFIG TIME -----------------------------------------------------------------------------------*/
		/*CCR_Val[TIM4->4][channels]. quy dinh thoi gian quay lai.*/
		capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + readTimeCCR(TIME_4,1));
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		controlRobot();
		TIM_SetCounter(TIM8,30000);
	}
}

void TIM1_CC_IRQHandler(void)
{ 
  if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) 
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		CaptureEncoder = (uint32_t)TIM_GetCapture2(TIM1);
		TIM_SetCounter(TIM1,0);
	}
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
 	GPIO_SetBits(GPIOA, GPIO_Pin_7);
	sprintf(txBufUsart,"Wrong parameters value: file %s on line %d\r\n", file, line);
	putStringUsart(txBufUsart,USART2);
	
  while (1){
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
