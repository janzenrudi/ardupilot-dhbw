/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

int16_t iAvailable = 0;
int16_t iHighbyte = 0;
int16_t iLowbyte = 0;
int16_t iDistance = 0;

int16_t iReadByte = 0;

int8_t bStartByte = 255; 	// 1111 1111
int8_t bStopByte = 254;		// 1111 1110
int8_t bValidFlags = 0;

int16_t aiSensors[8] = {0};

MicroSD * mSD;

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


