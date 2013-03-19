/* Public include -----------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "gTIM.h"
#include "gGPIO.h"

/* Public typedef -----------------------------------------------------------*/
/* enum channels mmotor*/
enum{md , msr, msl};
/* typedef enum of dir motor*/
enum{CW, CCW};
/* typedef struct of MOTOR*/
typedef struct{
	char id;
	int pwm;
	char dir;
} MOTOR;

/* Public define ------------------------------------------------------------*/
#define OFFSET_MOTOR 10

#define OFFSET_STR	-40
#define MAX_STR 		MID_STR+300
#define MIN_STR 		MID_STR-300
#define MID_STR 		2953+OFFSET_STR
//#define OFFSET_ST 200;

#define OFFSET_STR	-40
#define MAX_STL 		MID_STL+300
#define MIN_STL 		MID_STL-300
#define MID_STL 		3024+OFFSET_STR

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void MotorConfig(void);
void controlMotor(MOTOR M);
