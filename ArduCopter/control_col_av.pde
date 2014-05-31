/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_col_av.pde - init and run calls for stabilize flight mode
 */
 
#define DISTANCE_THRESHOLD 100
#define MAX_ANGLE 4500
#define THIRTY_DEGREE_IN_RADIAN 0.52359878

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
    
    int16_t pitch_angle = brakeFunction(g.sensor1);
    //Test2
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


static bool getAnglesSensors(int16_t &iRollAngle_p, int16_t &iPitchAngle_p)
{
    // Fuer jeden Sensor wird abhaengig von seiner Ausrichtung 
    // und dem gemessenen Abstand jeweils ein Winkel für Roll
    // und Pitch berechnet. Die Berechnung wird nur bei Unterschreitung
    // des Schwellwerts durchgeführt. Der Schwellwert gibt den minimalen
    // Abstand des Hexacopters von einem Objekt an.
    // Winkel = 0 -> Horizontal
    // Winkel > 0 -> Pitch: nach vorne neigen, Roll: nach rechts neigen
    // Winkel < 0 -> Pitch: nach hinten neigen, Roll: nach links neigen

    // Winkel-Variablen
    int16_t iRollDistance_l = 0;        
    int16_t iPitchDistance_l = 0;
    
    int16_t aiPitchAngles_l[6] = {0}; // Front, FrontLeft, BackLeft, Back, BackRight, FrontRight
    int16_t aiRollAngles_l[6] = {0}; // Front, FrontLeft, BackLeft, Back, BackRight, FrontRight
    
    
    // Sensor vorne
    if(g.sensor1 < DISTANCE_THRESHOLD)
    {
        aiPitchAngles_l[0] = brakeFunction(g.sensor1);    //Berechnung des Pitch-Winkels

        aiPitchAngles_l[0] *= -1;   // Ausweichen nach hinten
    }
	
    // Sensor vorne rechts
    if(g.sensor2 < DISTANCE_THRESHOLD)
    {
        iRollDistance_l = int16_t((double)cos(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor2); //Roll-Anteil
        iPitchDistance_l = int16_t((double)sin(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor2);// Pitch-Anteil
    
        aiRollAngles_l[1] = brakeFunction(iRollDistance_l);
        aiPitchAngles_l[1] = brakeFunction(iPitchDistance_l);
        
        aiRollAngles_l[1] *= -1;    // Ausweichen nach links
        aiPitchAngles_l[1] *= -1;   // Ausweichen nach hinten
        
    }
    
    // Sensor hinten rechts
    if(g.sensor3 < DISTANCE_THRESHOLD)
    {
        iRollDistance_l = int16_t((double)cos(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor3); 
        iPitchDistance_l = int16_t((double)sin(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor3);
    
        aiRollAngles_l[2] = brakeFunction(iRollDistance_l);
        aiPitchAngles_l[2] = brakeFunction(iPitchDistance_l);

        aiRollAngles_l[2] *= -1;    // Ausweichen nach links
    }

    // Sensor hinten
    if(g.sensor4 < DISTANCE_THRESHOLD)
    {
        aiPitchAngles_l[3] = brakeFunction(g.sensor4); 
    }

    // Sensor hinten links
    if(g.sensor5 < DISTANCE_THRESHOLD)
    {
        iRollDistance_l = int16_t((double)cos(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor5); 
        iPitchDistance_l = int16_t((double)sin(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor5);
    
        aiRollAngles_l[4] = brakeFunction(iRollDistance_l);
        aiPitchAngles_l[4] = brakeFunction(iPitchDistance_l);
    }

    // Sensor vorne links
    if(g.sensor6 < DISTANCE_THRESHOLD)
    {
        iRollDistance_l = int16_t((double)cos(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor6); 
        iPitchDistance_l = int16_t((double)sin(THIRTY_DEGREE_IN_RADIAN) * (double)g.sensor6);
    
        aiRollAngles_l[5] = brakeFunction(iRollDistance_l);
        aiPitchAngles_l[5] = brakeFunction(iPitchDistance_l);

        aiPitchAngles_l[5] *= -1;   //Ausweichen nach hinten
    }
    
    
    // Minimale und maximale Winkel berechnen
    int16_t iMaxRollAngle_l = 0;
    int16_t iMinRollAngle_l = 0;
    int16_t iMaxPitchAngle_l = 0;
    int16_t iMinPitchAngle_l = 0;
    
    for(int i=0 ; i<6 ; ++i)
    {
    	if(aiRollAngles_l[i] < iMinRollAngle_l)
    	{
    		iMinRollAngle_l = aiRollAngles_l[i];
    	}
    	else
    	{
    		iMaxRollAngle_l = aiRollAngles_l[i];
    	}
    	
    	if(aiPitchAngles_l[i] < iMinPitchAngle_l)
    	{
    		iMinPitchAngle_l = aiPitchAngles_l[i];
    	}
    	else
    	{
    		iMaxPitchAngle_l = aiPitchAngles_l[i];
    	}
    }
    
    // Wenn sowohl nach links (Roll negativ), als auch nach rechts (Roll positiv) 
    // ausgewichen werden soll, dann wird der Mittelwert gebildet.
    if(iMaxRollAngle_l != 0 && iMinRollAngle_l != 0)
    {
    	iRollAngle_p = iMaxRollAngle_l + iMinRollAngle_l;
    }
    else if(iMaxRollAngle_l != 0 && iMinRollAngle_l == 0)
    {
    	iRollAngle_p = iMaxRollAngle_l;
    }
    else
    {
    	iRollAngle_p = iMinRollAngle_l;
    }
    
    // Wenn sowohl nach hinten (Pitch negativ), als auch nach vorne (Pitch positiv) 
    // ausgewichen werden soll, dann wird der Mittelwert gebildet.
    if(iMaxPitchAngle_l != 0 && iMinPitchAngle_l != 0)
    {
    	iPitchAngle_p = iMaxPitchAngle_l + iMinRollAngle_l;
    }
    else if(iMaxPitchAngle_l != 0 && iMinPitchAngle_l == 0)
    {
    	iPitchAngle_p = iMaxPitchAngle_l;
    }
    else
    {
    	iPitchAngle_p = iMinPitchAngle_l;
    }
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
