/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//HoTT for PX4 only
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
	#include <nuttx/config.h>
	#include <unistd.h>
	#include <stdio.h>
	#include <poll.h>

	#include <uORB/uORB.h>
	#include <uORB/topics/battery_status.h>
	#include <uORB/topics/sensor_combined.h>

	//AP own ORB object...
	#include <uORB/topics/ap_data.h>
	ORB_DEFINE(ap_data, struct ap_data_s);

	//ORB handles
	orb_advert_t hBatteryTopic;
	orb_advert_t hSensorsTopic;
	orb_advert_t hApDataTopic;
#endif

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
	#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
		//setup ORB publishing
		struct battery_status_s bs;
		memset(&bs, 0, sizeof(bs));
		hBatteryTopic = orb_advertise(ORB_ID(battery_status), &bs);

		struct sensor_combined_s sen;
		memset(&sen,0,sizeof(sen));
		hSensorsTopic = orb_advertise(ORB_ID(sensor_combined), &sen);
		
		struct ap_data_s apData;
		memset(&apData, 0, sizeof(apData));
		hApDataTopic = orb_advertise(ORB_ID(ap_data), &apData);

		//setup ORB publishing
	#endif
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