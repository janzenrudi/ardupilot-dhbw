#include "HF_Led.h"

#include <fcntl.h>
#include <stdio.h>

#include <AP_HAL.h>
#include <AP_BattMonitor.h>     // Battery monitor library

#include <math.h>

extern const AP_HAL::HAL& hal;

HF_Led::HF_Led(void):led_on(true),led_wait(1){}

void HF_Led::led_flash(int curVolt){

    if (led_wait == 0){
        if (led_on==true){
            hal.gpio->pinMode(RELAY_PIN, GPIO_OUTPUT);
            hal.gpio->write(RELAY_PIN, 0);
            led_on=false;
        }
        else {
            hal.gpio->pinMode(RELAY_PIN, GPIO_OUTPUT);
            hal.gpio->write(RELAY_PIN, 1);
            led_on=true;
        }


        if (curVolt >= 1400){
            led_wait=55;
        }
        else if (curVolt >= 1300){
            led_wait=50;
        }
        else if (curVolt >= 1200){
            led_wait=45;
        }
        else if (curVolt >= 1100){
            led_wait=40;
        }
        else if (curVolt >= 1000){
            led_wait=10;
        }
        else {
            led_wait=1;
        }
    }
    led_wait--;

    return;
}
