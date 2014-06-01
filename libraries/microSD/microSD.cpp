#include "microSD.h"
#include <fcntl.h>
#include <stdio.h>

MicroSD::MicroSD():	filepath("/fs/microsd/Testlog.txt"),
					counter(0)
{}

MicroSD::~MicroSD(){}

bool MicroSD::write_lowbyte(const uint8_t message_p)
{
	FILE *_file = fopen(filepath, "a+");
	if(_file != NULL)
	{
		fprintf(_file, "%i. Lowbyte: %i\n", counter, message_p);
		++counter;
		fclose(_file);
		return true;
	}
	return false;
}

bool MicroSD::write_highbyte(const uint8_t message_p)
{
	FILE *_file = fopen(filepath, "a+");
	if(_file != NULL)
	{
		fprintf(_file, "%i. Highbyte: %i\n", counter, message_p);
		++counter;
		fclose(_file);
		return true;
	}
	return false;
}

bool MicroSD::write_muell(const uint8_t message_p)
{
	FILE *_file = fopen(filepath, "a+");
	if(_file != NULL)
	{
		fprintf(_file, "%i. Muell: %i\n", counter, message_p);
		++counter;
		fclose(_file);
		return true;
	}
	return false;
}

bool MicroSD::write_distance(const int16_t message_p)
{
	FILE * _file = fopen(filepath, "a+");
	if(_file != NULL)
	{
		fprintf(_file, "%i. Abstand: %i\n cm", counter, message_p);
		++counter;
		fclose(_file);
		return true;
	}
	return false;
}
