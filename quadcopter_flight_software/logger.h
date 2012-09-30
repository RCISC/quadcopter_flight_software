#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <Windows.h>

class QLogger
{
	HANDLE f;
public:
	QLogger();
	~QLogger();
	void log( const char * buffStr );
private:
	LARGE_INTEGER frequency;
};

#endif