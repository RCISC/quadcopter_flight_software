#ifndef GYRO_H
#define GYRO_H

DWORD WINAPI commReader( void* param );

struct GyroVals
{
	int timestamp;
	double x, y, z;
};

struct GyroThreadParam
{
	DWORD mainThreadID;
	//something like COM1 for <= 9
	//or "\\\\.\\COM24" for > 9. Note - it's \\.\, with backslashes escaped
	char portname[ 256 ];
};


#endif