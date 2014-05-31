/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define USART5 hal.uartC

#ifdef USERHOOK_INIT
void userhook_init()
{
<<<<<<< HEAD
    USART5->begin(9600);
	
	hfLed = new HF_Led();
    // put your initialisation code here
    // this will be called once at start-up
=======
	hfSonar = new HF_Sonar();
	hfSonar.init();
>>>>>>> 59f085c07ae21d555e26efc035ac794542d98f61
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    pMSd_g->write_hott((battery.voltage() * 100.0f));
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	g.sensor1 = hfSonar.read();

}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    led_flash(battery.voltage() * 100.0f);
}
#endif
