#include "HF_Sonar.h"

#include <fcntl.h>
#include <stdio.h>

#include <AP_HAL.h>

#include <math.h>

extern const AP_HAL::HAL& hal;

HF_Sonar::HF_Sonar(void):{}

void HF_Sonar::init(){

	USART5->begin(9600);
	
	return;
}

int HF_Sonar::read(){

   iAvailable = USART5->available();
    if(iAvailable >= 3)
    {
        iHighbyte = USART5->read();
        if(iHighbyte == 100)
        {
            iHighbyte = USART5->read();
            iLowbyte = USART5->read();
            iDistance = (iHighbyte << 8) + iLowbyte;
        }
    }

    return iDistance;
}
