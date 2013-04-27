#include "gDelay.h"

void oneCycle(unsigned int nCount){
  while(nCount--);
}

void delayUs(int microseconds){
  while(microseconds--){
		oneCycle(48);
	}
}

void delayMs(int milliseconds){
	while(milliseconds--){
		
	}
}

void delayS(int seconds){
	while(seconds--){
		delayMs(1000);
	}
}
