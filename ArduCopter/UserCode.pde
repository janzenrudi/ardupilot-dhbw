/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define UART5 hal.uartC
#define NUMBER_OF_SENSORS 6
#define LENGTH_OF_OVERHEAD 3

#ifdef USERHOOK_INIT
void userhook_init()
{
    UART5->begin(9600);
    mSD = new MicroSD();
    hfLed = new HF_Led();
    
    g.sensor1 = 150;
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
    hfLed->led_flash((battery.voltage() * 100.0f));
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	// Vom ArduinoMicro werden Frames geschickt, die wie folgt aufgebaut sind:
	// 1.     Byte: Startbyte -> hier: 255
	// 2.     Byte: ValidByte -> Fuer jeden der 8 Sensoren ein Valid Bit
	// 3.-18. Byte: Daten	  -> Für jeden der 8 Sensoren 2 Bytes mit der Distanz in cm
	// 19.    Byte: Stopbyte  -> hier: 254
	
	iAvailable = UART5->available(); // Wie viele Bytes sind empfangen worden
	
	// Einlesen erfolgt nur, wenn ein kompletter Frame vorliegt
	while(iAvailable >= (NUMBER_OF_SENSORS * 2 + LENGTH_OF_OVERHEAD))
	{
		// Erstes Byte einlesen
		iReadByte = UART5->read();
		
		
		// Erstes eingelesenes Byte auf Startbyte ueberpruefen
		if(iReadByte == bStartByte)
		{
			//mSD->write_muell(iReadByte);
			
			// ValidByte einlesen
			bValidFlags = UART5->read(); 
			//mSD->write_muell(iReadByte);
			 
			// Abstandswerte der Sensoren einlesen 
			for(int8_t index=0 ; index < NUMBER_OF_SENSORS ; ++index)
			{
				iReadByte = UART5->read();
				aiSensors[index] = (iReadByte << 8);
				iReadByte = UART5->read();
				aiSensors[index] += iReadByte;
				//mSD->write_distance(aiSensors[index]);
			}
			
			// Letztes Byte einlesen
			iReadByte = UART5->read();
			
			
			// Letztes eingelesenes Byte auf Stopbyte ueberpruefen
			// Die eingelesenen Abstandswerte der Sensoren werden nur in den Parametern
			// gespeichert, wenn das letzte Byte dem Stopbyte entspricht.
			
			if(iReadByte == bStopByte)
			{
			//mSD->write_muell(iReadByte);
				// Der Abstandsert eines Sensors wird nur gespeichert, 
				// wenn das Valid-Bit = true ist.
				
				// Sensor 1
				if(!((bValidFlags & 0x80)>>7))
				{
					g.sensor1 = aiSensors[0];
				}
				else
				{
					g.sensor1 = -1; // Ungültiger Abstandswert
				}
				
				// Sensor 2
				if(!((bValidFlags & 0x40)>>6))
				{
					g.sensor2 = aiSensors[1];
				}
				else
				{
					g.sensor2 = -1; // Ungültiger Abstandswert
				}
	
				// Sensor 3
				if(!((bValidFlags & 0x20)>>5))
				{
					g.sensor3 = aiSensors[2];
				}
				else
				{
					g.sensor3 = -1; // Ungültiger Abstandswert
				}
				
				// Sensor 4
				if(!((bValidFlags & 0x10)>>4))
				{
					g.sensor4 = aiSensors[3];
				}
				else
				{
					g.sensor4 = -1; // Ungültiger Abstandswert
				}
				
				// Sensor 5
				if(!((bValidFlags & 0x08)>>3))
				{
					g.sensor5 = aiSensors[4];
				}
				else
				{
					g.sensor5 = -1; // Ungültiger Abstandswert
				}
				
				// Sensor 6
				if(!((bValidFlags & 0x04)>>2))
				{
					g.sensor6 = aiSensors[5];
				}
				else
				{
					g.sensor6 = -1; // Ungültiger Abstandswert
				}
				
				// Sensor 7
				if(!((bValidFlags & 0x02)>>1))
				{
					g.sensor7 = aiSensors[6];
				}
				else
				{
					g.sensor7 = -1; // Ungültiger Abstandswert
				}	
				
				// Sensor 8
				if(!(bValidFlags & 0x01))
				{
					g.sensor8 = aiSensors[7];
				}
				else
				{
					g.sensor8 = -1; // Ungültiger Abstandswert
				}
				
			} // if Stopbyte
		} // if Startbyte
		
		iAvailable = UART5->available(); // Ueberpruefung auf aktuellere Daten
	}
	
	/*
	iAvailable = UART5->available();
    if(iAvailable >= 3)
    {
        iHighbyte = UART5->read();
        if(iHighbyte == 100)
        {
            iHighbyte = UART5->read();
            iLowbyte = UART5->read();
            iDistance = (iHighbyte << 8) + iLowbyte;
            
            g.sensor1 = iDistance;
        }
    }*/
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
	if(g.sensor1 == 0)
	{
		g.sensor1 = 150;	
	}
	else
	{
		g.sensor1 = g.sensor1-1;
	}
    // put your 1Hz code here
}
#endif
