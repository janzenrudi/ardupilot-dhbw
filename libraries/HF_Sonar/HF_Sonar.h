/*
 * HF_Sonar.h
 *
 *  Created on: 24.03.2014
 *      Author: lmenz (jaru)
 */

#ifndef HF_Sonar_H
#define HF_Sonar_H

#define USART5 hal.uartC

#include <stdint.h>


class HF_Sonar
{
public:
    HF_Sonar();

    void init();
	int read();

private:
    int16_t iAvailable = 0;
	int16_t iHighbyte = 0;
	int16_t iLowbyte = 0;
	int iDistance = 0;
};

#endif // HF_Sonar_H
