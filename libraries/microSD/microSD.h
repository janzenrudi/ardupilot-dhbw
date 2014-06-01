/*
 * microSD.h
 *
 *  Created on: 12.03.2014
 *      Author: lukasmenz
 */

#ifndef MICROSD_H_
#define MICROSD_H_

#include <stdint.h>

class MicroSD
{
public:
	MicroSD();
	~MicroSD();
	bool write_lowbyte(const uint8_t message_p);
	bool write_highbyte(const uint8_t message_p);
	bool write_muell(const uint8_t message_p);
	bool write_distance(const int16_t message_p);
private:
 	char *filepath;
 	int counter;
};



#endif /* MICROSD_H_ */
