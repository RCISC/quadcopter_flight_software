#include "stdafx.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <Windows.h>


QLogger::QLogger()
{
	f = CreateFile( TEXT("log.txt"), GENERIC_WRITE, FILE_SHARE_READ, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if( INVALID_HANDLE_VALUE == f )
	{
		throw "error";
	}

	if( QueryPerformanceFrequency( &frequency ) == 0 || frequency.QuadPart == 0)
	{
		throw "error";
	}
	char logbuf[ 512 ];
	int bytes = _snprintf_s( logbuf, sizeof( logbuf ), _TRUNCATE, "New log. Counts per second: %lld \r\n\r\n", frequency.QuadPart );
	DWORD n;
	SetFilePointer( f, 0, NULL, FILE_END );
	WriteFile( f, logbuf, bytes, &n, NULL );
	
}

QLogger::~QLogger()
{
	if (f)
		CloseHandle(f);
}

void QLogger::log( const char * buffStr )
{

#define HIGHREZ_TIME
#ifdef HIGHREZ_TIME
	LARGE_INTEGER time;
	if( QueryPerformanceCounter( &time ) == 0)
	{
		//error
	}
#else
	SYSTEMTIME systime;
	GetSystemTime( &systime );
#endif

	char logbuf[ 512 ];
	int bytes = _snprintf_s( logbuf, sizeof( logbuf ), _TRUNCATE, "%lld %s\r\n", time.QuadPart / frequency.QuadPart, buffStr );
	DWORD n;
	WriteFile( f, logbuf, bytes, &n, NULL );
}