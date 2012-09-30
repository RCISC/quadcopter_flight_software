#include "stdafx.h"
#include <stdio.h>
#include "gyro.h"
#include "common.h"
#include "serial.h"

DWORD WINAPI commReader( void* param )
{
	GyroThreadParam* pparam = ( GyroThreadParam * )param;

	HANDLE gyroPort = openPortS( pparam->portname, 9600 );

	if( INVALID_HANDLE_VALUE == gyroPort )
	{
		DWORD err = GetLastError();
		return 1;
	}

	char commBuff[512];
	while( true )
	{
		for( int i = 0; i < sizeof( commBuff ); )
		{
			DWORD n = 0;
			ReadFile( gyroPort, commBuff + i, 1, &n, NULL );
			if( n == 1 )
			{
				if( commBuff[i] == '\r' )
				{
					commBuff[i] = 0;
					double x_, y_, z_;
					int time_;
					if( 4 == sscanf_s( commBuff, "%d,%lf,%lf,%lf", &time_, &x_, &y_, &z_ ) )
					{
						//received new X, Y, Z.
						GyroVals* newVals = new GyroVals();
						newVals->timestamp = time_;
						newVals->x = x_;
						newVals->y = y_;
						newVals->z = z_;

						BOOL err = PostThreadMessage( pparam->mainThreadID, GYRO_MSG, NULL, (LPARAM)newVals );

					}
					i = 0;
				}
				else if( commBuff[i] < ' ' )
				{
				}
				else
				{
					i++;
				}
			}
		}
	}

	CloseHandle( gyroPort );

	return 0;
}