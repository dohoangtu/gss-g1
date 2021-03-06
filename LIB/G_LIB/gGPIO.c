
#include "gGPIO.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define STATUS_LEDS 	GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15
#define WARNING_LEDS	GPIO_Pin_6 | GPIO_Pin_7
#define OUTPUT_RELAY	GPIO_Pin_6 | GPIO_Pin_3
#define INPUT_SENSOR					GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void gpioInitMode(void){
	/* OUTPUT ------------------------------------------------------------------*/
	gpioSetMode(STATUS_LEDS,GPIO_Mode_OUT, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOD);
	gpioSetMode(INPUT_SENSOR,GPIO_Mode_IN, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOD);
}

void gpioSetMode(uint32_t GPIO_PIN, GPIOMode_TypeDef GPIO_Mode,  GPIOOType_TypeDef GPIO_OType,
									GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd, GPIO_TypeDef* GPIOx){
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	uint32_t RCC_AHB1Periph = 0;
	if(GPIOx == GPIOA){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOA;
	}else if(GPIOx == GPIOB){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOB;
	}else if(GPIOx == GPIOC){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOC;
	}else if(GPIOx == GPIOD){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOD;
	}else if(GPIOx == GPIOE){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOE;
	}else if(GPIOx == GPIOF){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOF;
	}else if(GPIOx == GPIOG){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOG;
	}else if(GPIOx == GPIOH){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOH;
	}else if(GPIOx == GPIOI){
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOI;
	}
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph, ENABLE);
	
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
  GPIO_InitStructure.GPIO_OType = GPIO_OType;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
}

