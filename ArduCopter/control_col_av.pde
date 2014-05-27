/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_col_av.pde - init and run calls for stabilize flight mode
 */
 
#define DISTANCE_THRESHOLD 100
#define MAX_ANGLE 4500

// col_av_init - initialise collision avoidance controller
static bool col_av_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // col_av should never be made to fail
    return true;
}

// col_av_run - runs the main collision avoidance controller
// should be called at 100hz or more
static void col_av_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    
    int16_t pitch_angle = brakeFunction(g.sensor1)
    
    if(g.sensor1 < 100)
    {
        get_pilot_desired_lean_angles(g.rc_1.control_in, pitch_angle, target_roll, target_pitch);
    }
    else
    {
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    }

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
    }else{
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // body-frame rate controller is run directly from 100hz loop
    }

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}

static int16_t brakeFunction(const int16_t iDistance_p)
{
	int16_t iNormedDistance_l = iDistance_p / DISTANCE_THRESHOLD;
	int16_t iNormedAngle_l = 0;	
	int16_t iAngle_l = 0;
	if(iDistance_p < DISTANCE_THRESHOLD/2)
	{
		iNormedAngle_l = 1/(10*iNormedDistance_l);
	}
	else
	{
		iNormedAngle_l = -0.4*iNormedDistance_l + 0.4;
	}
	iAngle_l = iNormedAngle_l * MAX_ANGLE;
	
	return iAngle_l; 
}