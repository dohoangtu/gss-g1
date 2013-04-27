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
#include "main.h"

#define OFF_ADC		10
/* parivate valuable-----------------------------------------------------------------*/

/* Private fuction ------------------------------------------------------------------*/
void cotrolRobot1(void);
void cotrolRobot2(void);
void controlMD(void);
void controlSTL(void);
void controlSTR(void);

/* parivate variable ----------------------------------------------------------------*/
	uint32_t count;
	uint32_t cntReset;
	char flagToggle;
	int testTime1,testTime2;
	int setAngle,setAngleOld;
	Bit flagCompleGyro,flagCompleHome;
	uint16_t controlR_Max, controlL_Max,controlL_Min,controlR_Min;
	/*variable motor control -----------------------------------------*/
	MOTOR Motor[3];
	
	/* PID ---- */
	tPID velocityPID;
	tPID posPID;
	tPID aPID;
	tPID stL_PID;
	tPID stR_PID;

	double Kcoeff[5][3] ={{0.01,0,0},
												{0,0,0},
												{0,0,0},
												{1,0.0001,0},
												{1,0.0001,0}};
	double ABCcoeff[5][3];
	double historyControl[5][3];
	double pwmPID[3];
	
	char cb1,cb2,cq,cb3,cb4;
	uint32_t cntOut,cntOut1;
	uint32_t cntOK;
	char flagOut;
/*fuction int mode chip --------------------------------------------*/
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
	@void: mian
	@para: none.
	@retval: none.
	@brief: fuction main in project
*/
int main(){
	gpioInitMode();
	usartInitMode();
 	adcInitMode(); 
 	TimInitMode();
 	MotorConfig();
	initPID();
	delayMs(10);
	
	printf("Ready System----------------\n");
	STM_EVAL_LEDOn(LED4);
	/* dat gia tri ban dau */
	/* set id cho dong co */
 	Motor[MDRIVER].id = md;
 	Motor[MSTEERING_L].id = msr;
 	Motor[MSTEERING_R].id = msl;
	
	Motor[MDRIVER].dir = CW;
	Motor[MDRIVER].pwm = 2000;
	controlMotor(Motor[MDRIVER]);
	setAngle = 0;
	
	while(1){
		/* display ---------------------------------------*/
// 		count++;
// 		if(count >= 100000){
// 			count = 0;
// 			printf("statusRobot: %d\n",statusRobot);
// 		}
		
		/* read ADC -------------------------------------*/
 		ADCValue[ADC_SPEED] 	= (readAdc(ADC_SPEED));
 		ADCValue[ADC_ANGLE_R] = (readAdc(ADC_ANGLE_R));
 		ADCValue[ADC_ANGLE_L] = (readAdc(ADC_ANGLE_L));
		
		/* read MPU6050 ---------------------------------*/
		cotrolRobot2();
		/* control motor -----------------------*/
	}
}

void cotrolRobot1(void){
/* read sensor robot ------------------------------*/
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0) == 1){
		STM_EVAL_LEDOn(LED5);
		cb1 = 1;
	}
	else{
		cb1 = 0;
		STM_EVAL_LEDOff(LED5);
	}
	
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1){
		STM_EVAL_LEDOn(LED6);
		cb2 = 1;
	}
	else{
		cb2 = 0;
		STM_EVAL_LEDOff(LED6);
	}
	
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4) == 1){
		STM_EVAL_LEDOn(LED3);
		cq = 1;
	}
	else{
		cq = 0;
		STM_EVAL_LEDOff(LED3);
	}
/* process robot ------------------------------------*/
	if(cb1 == 0 && cb2 == 0){
		if(cq == 0){
// 			setAngle = -100; // lai sau
			setAngle = 100; // lai truoc
		}
		else{
// 			setAngle = 100; // lai sau
			setAngle = -100; //lai truoc
		}
// 		cntOut = 0;
	}
	else if(cb1 == 1 && cb2 == 0){
// 		if(setAngle == -100)		setAngle = 30;			//  lai sau
// 		else if(setAngle == 0)  setAngle = -50;
		setAngle = 50;					//lai truoc
	}
	else if(cb1 == 0 && cb2 == 1){
// 		if(setAngle == 100)		setAngle = -30;				// lai sau
// 		else if(setAngle == 0)  setAngle = 50;
		setAngle = -50;					// lai truoc
	}
	else{
// 		printf("cntOut: %d\n",cntOut);
// 		cntOut++;
		setAngle = 0;
	}
	
	setAngleOld = setAngle;
}

void cotrolRobot2(void){
/* read sensor robot ------------------------------*/
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0) == 1){
		cb1 = 1;
	}
	else{
		cb1 = 0;
	}
	
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1) == 1){
		cb2 = 1;
	}
	else{
		cb2 = 0;
	}
	
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1){
		cb3 = 1;
	}
	else{
		cb3 = 0;
	}
	
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4) == 1){
		cb4 = 1;
	}
	else{
		cb4 = 0;
		STM_EVAL_LEDOff(LED3);
	}
/* process robot ------------------------------------*/
	if(cb1 == 1){
		setAngle = 50;
	}
	else if(cb2 == 1 && cb1 == 0){
		setAngle = 100;					//lai truoc
	}
	else if(cb3 == 1 && cb4 == 0){
		setAngle = -100;					// lai truoc
	}
	else if(cb4 == 1){
		setAngle = -50;					// lai truoc
	}
	else if(cb4 == 1 && cb1 == 1){
		setAngle = 0;					// lai truoc
	}
	
	setAngleOld = setAngle;
}

/**
	@void: controlSTL
	@para: none.
	@returnVar: none.
	@fuction: dieu khien motor steering left theo PID Conttrol
*/
void controlSTL(void){
	stL_PID.measureOutput = ADCValue[ADC_ANGLE_L];

	pwmPID[1] = PID(&stL_PID);
	
	if(pwmPID[1] < -OFF_ADC && stL_PID.measureOutput > MIN_STL){
		pwmPID[1] = -pwmPID[1];
		Motor[MSTEERING_L].dir = CCW;
	}
	else if(pwmPID[1] > OFF_ADC && stL_PID.measureOutput < MAX_STL){
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
	stR_PID.measureOutput = ADCValue[ADC_ANGLE_R];
	pwmPID[2] = PID(&stR_PID);
	
	if(pwmPID[2] < -OFF_ADC && stR_PID.measureOutput > MIN_STR){
		pwmPID[2] = -pwmPID[2];
		Motor[MSTEERING_R].dir = CCW;
	}
	else if(pwmPID[2] > OFF_ADC && stR_PID.measureOutput < MAX_STR){
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
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f){
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, (uint8_t)ch);
	return ch;
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
		STM_EVAL_LEDToggle(LED4);
	/* CONFIG TIME -----------------------------------------------------------------------------------*/
		/*CCR_Val[TIM4->4][channels]. quy dinh thoi gian quay lai.*/
		capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + readTimeCCR(4,1));
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		testTime1++;
		if(testTime1 >= 10){
			testTime1 = 0;
			stL_PID.controlReference = MID_STL + setAngle;
			stR_PID.controlReference = MID_STR + setAngle;
		}
		
		controlSTR();
		controlSTL();
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
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);	
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
