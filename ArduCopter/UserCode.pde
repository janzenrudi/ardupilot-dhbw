/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define USART5 hal.uartD

#ifdef USERHOOK_INIT
void userhook_init()
{
    USART5->begin(9600);
    // put your initialisation code here
    // this will be called once at start-up
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
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    iAvailable = USART5->available();
    if(iAvailable >= 3)
    {
        iHighbyte = USART5->read();
        if(iHighbyte == 100)
        {
            iHighbyte = USART5->read();
            iLowbyte = USART5->read();
            iDistance = (iHighbyte << 8) + iLowbyte;
            
            g.sensor1 = iDistance;
        }
    }
    // put your 10Hz code here
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
    // put your 1Hz code here
}
#endif
