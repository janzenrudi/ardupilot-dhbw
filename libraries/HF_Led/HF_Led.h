/*
 * HF_led.h
 *
 *  Created on: 24.03.2014
 *      Author: jaru
 */

#ifndef HF_LED_H
#define HF_LED_H

#define RELAY_PIN 115

#include <stdint.h>


class HF_Led
{
public:
    HF_Led();

    void led_flash(int curVolt);

private:
    bool led_on;
    int led_wait;
};

#endif // HF_LED_H
