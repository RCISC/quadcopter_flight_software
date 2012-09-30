#include "stdafx.h"
#include <stdio.h>
#include "motor_control.h"
#include <stdint.h>
#include "serial.h"



/** Implements the Maestro's Get Position serial command.
* channel: Channel number from 0 to 23
* position: A pointer to the returned position value (for a servo channel, the units are quarter-milliseconds)
* Returns 1 on success, 0 on failure.
* For more information on this command, see the "Serial Servo Commands"
* section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroGetPosition(HANDLE port, unsigned char channel, unsigned short * position)
{
   unsigned char command[2];
   unsigned char response[2];
   BOOL success;
   DWORD bytesTransferred;

   // Compose the command.
   command[0] = 0x90;
   command[1] = channel;

   // Send the command to the device.
   success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to write Get Position command to serial port.  Error code 0x%x.", GetLastError());
      return 0;
   }
   if (sizeof(command) != bytesTransferred)
   {
      fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
      return 0;
   }

   // Read the response from the device.
   success = ReadFile(port, response, sizeof(response), &bytesTransferred, NULL);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to read Get Position response from serial port.  Error code 0x%x.", GetLastError());
      return 0;
   }
   if (sizeof(response) != bytesTransferred)
   {
      fprintf(stderr, "Error: Expected to read %d bytes but only read %d (timeout). "
         "Make sure the Maestro's serial mode is USB Dual Port or USB Chained.", sizeof(command), bytesTransferred);
      return 0;
   }

   // Convert the bytes received in to a position.
   *position = response[0] + 256*response[1];

   return 1;
}

/** Implements the Maestro's Set Target serial command.
* channel: Channel number from 0 to 23
* target: The target value (for a servo channel, the units are quarter-milliseconds)
* Returns 1 on success, 0 on failure.
* Fore more information on this command, see the "Serial Servo Commands"
* section of the Maestro User's Guide: http://www.pololu.com/docs/0J40 */
BOOL maestroSetTarget(HANDLE port, unsigned char channel, unsigned short target)
{
   unsigned char command[4];
   DWORD bytesTransferred;
   BOOL success;

   // Compose the command.
   command[0] = 0x84;
   command[1] = channel;
   command[2] = target & 0x7F;
   command[3] = (target >> 7) & 0x7F;

   // Send the command to the device.
   success = WriteFile(port, command, sizeof(command), &bytesTransferred, NULL);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to write Set Target command to serial port.  Error code 0x%x.", GetLastError());
      return 0;
   }
   if (sizeof(command) != bytesTransferred)
   {
      fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
      return 0;
   }

   return 1;
}

MotorControl::MotorControl()
{
	commPort = NULL;
	port = NULL;
}

bool MotorControl::openPort(  const char* portname )
{
	/*errno_t err = fopen_s( &commPort, portname, "wb" );
	if( ! commPort )
	{
		DWORD err = GetLastError();
	}*/
	int baudRate = 9600;
	port = openPortS( portname, baudRate );
	if (port == INVALID_HANDLE_VALUE){ return 1; }
	else { return 0; }
}

MotorControl::~MotorControl( )
{
	if( commPort )
		fclose( commPort );

	if( port )
		CloseHandle(port);
}

//value is 0 to 1
void MotorControl::setMotor( unsigned char motor, double value ) 
{
	const unsigned short minValue = 1000*4; //1000 microseconds; we're sending *4 because it goes in increments of .25
	const unsigned short maxValue = 2000*4; //2000 microseconds; widest we can do on 111 hz
	
	value *= ( maxValue - minValue );
	value += minValue;

	if( port )
	{
		maestroSetTarget( port, motor, (unsigned short)( value ) );
	}
}

void MotorControl::zeroMotors()
{
	for( int i = 0; i < 4; i++ )
	{
		setMotor( i, 0 );
	}
}