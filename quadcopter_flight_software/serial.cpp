#include "stdafx.h"
#include "serial.h"
#include <stdio.h>

//something like COM1 for <= 9
//or "\\\\.\\COM24" for > 9. Note - it's \\.\, with backslashes escaped

HANDLE openPortS(const char * portName, unsigned int baudRate)
{
   HANDLE port;
   DCB commState;
   BOOL success;
   COMMTIMEOUTS timeouts;

   /* Open the serial port. */
   port = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
   if (port == INVALID_HANDLE_VALUE)
   {
      switch(GetLastError())
      {
      case ERROR_ACCESS_DENIED:   
         fprintf(stderr, "Error: Access denied.  Try closing all other programs that are using the device.\n");
         break;
      case ERROR_FILE_NOT_FOUND:
         fprintf(stderr, "Error: Serial port not found.  "
            "Make sure that \"%s\" is the right port name.  "
            "Try closing all programs using the device and unplugging the "
            "device, or try rebooting.\n", portName);
         break;
      default:
         fprintf(stderr, "Error: Unable to open serial port.  Error code 0x%x.\n", GetLastError());
         break;
      }
      return INVALID_HANDLE_VALUE;
   }

   /* Set the timeouts. */
   success = GetCommTimeouts(port, &timeouts);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to get comm timeouts.  Error code 0x%x.\n", GetLastError());
      CloseHandle(port);
      return INVALID_HANDLE_VALUE;
   }
   timeouts.ReadIntervalTimeout = 1000;
   timeouts.ReadTotalTimeoutConstant = 1000;
   timeouts.ReadTotalTimeoutMultiplier = 0;
   timeouts.WriteTotalTimeoutConstant = 1000;
   timeouts.WriteTotalTimeoutMultiplier = 0;
   success = SetCommTimeouts(port, &timeouts);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to set comm timeouts.  Error code 0x%x.\n", GetLastError());
      CloseHandle(port);
      return INVALID_HANDLE_VALUE;
   }

   /* Set the baud rate. */
   success = GetCommState(port, &commState);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to get comm state.  Error code 0x%x.\n", GetLastError());
      CloseHandle(port);
      return INVALID_HANDLE_VALUE;
   }
   commState.BaudRate = baudRate;
   /*commState.ByteSize = 8;
   commState.Parity = NOPARITY;
   commState.StopBits = ONESTOPBIT;
   commState.BaudRate = CBR_9600;*/
   success = SetCommState(port, &commState);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to set comm state.  Error code 0x%x.\n", GetLastError());
      CloseHandle(port);
      return INVALID_HANDLE_VALUE;
   }

   /* Flush out any bytes received from the device earlier. */
   success = FlushFileBuffers(port);
   if (!success)
   {
      fprintf(stderr, "Error: Unable to flush port buffers.  Error code 0x%x.\n", GetLastError());
      CloseHandle(port);
      return INVALID_HANDLE_VALUE;
   }

   return port;
}