/*******************************************************************************
Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************************/
/*
This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
qgroundcontrol are printed by this program along with the heartbeats.


I compiled this program sucessfully on Ubuntu 10.04 with the following command

gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c

the rt library is needed for the clock_gettime on linux
*/
/* These headers are for QNX, but should all be standard on unix/linux */

#include "stdafx.h"
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/types.h>

#include <stdlib.h>
#include <fcntl.h>
#include <time.h>

#include <intrin.h>

#include "mavlink_udp.h"
#include "common.h"

/* This assumes you have the mavlink headers on your include path
or in the same folder as this source file */
#include "mavlink/include/common/mavlink.h"
#include <winsock2.h>
#include <WS2tcpip.h>
#define ssize_t int //from unistd.h

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

static uint64_t microsSinceEpoch()
{
	static LARGE_INTEGER pf = { 0 };
	if( ! pf.QuadPart )
		QueryPerformanceFrequency( &pf );
	LARGE_INTEGER pc;
	QueryPerformanceCounter( &pc );
	return ( pc.QuadPart * 1000 ) / pf.QuadPart;
}

static void postHeartbeat( SOCKET sock, const sockaddr_in& gcAddr, float position[] )
{
	mavlink_message_t msg;
	uint16_t len;
	int bytes_sent;
	uint8_t buf[BUFFER_LENGTH];

//#define OLD_MAV

	/*Send Heartbeat */
#ifdef OLD_MAV
	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_HELICOPTER, MAV_AUTOPILOT_GENERIC);
#else
	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
#endif
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, (char *)buf, len, 0, (sockaddr*)&gcAddr, sizeof(gcAddr));

	/* Send Status */
#ifdef OLD_MAV
	mavlink_msg_sys_status_pack(1, 200, &msg, MAV_MODE_GUIDED, MAV_NAV_HOLD, MAV_STATE_ACTIVE, 7500, 0, 0, 0);
#else
	mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
#endif
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, (char *)buf, len, 0, (sockaddr*)&gcAddr, sizeof (gcAddr));

	/* Send Local Position */
#ifdef OLD_MAV
	mavlink_msg_local_position_pack(1, 200, &msg, microsSinceEpoch(), 
	position[0], position[1], position[2],
	position[3], position[4], position[5]);
#else
	mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(), 
									position[0], position[1], position[2],
									position[3], position[4], position[5]);
#endif
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, (char *)buf, len, 0, (sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	/* Send attitude */
	//old and new, do difference.

	static float roll = 1.2;
	float pitch = 1.7;
	float yaw = 3.14;

#ifdef OLD_MAV
	mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), roll, pitch, yaw, 0.01, 0.02, 0.03);
#else
	mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), roll, pitch, yaw, 0.01, 0.02, 0.03);
#endif
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = sendto(sock, (char *)buf, len, 0, (sockaddr*)&gcAddr, sizeof(gcAddr));
	//Beep( 880, 100 );
}

DWORD WINAPI mav_thread( void * param )
{
	MavThreadParam * pparam = ( MavThreadParam* )param;
	const u_short MAV_PORT = 14550;
	float position[6] = {};
	SOCKET sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in gcAddr; 
	struct sockaddr_in locAddr;
//	struct sockaddr_in fromAddr;
	int recsize = 0;
	socklen_t fromlen = sizeof( gcAddr );
	int success = 0;


	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons( MAV_PORT + 1 );

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (SOCKET_ERROR == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
	{
		int err = WSAGetLastError();
		perror("error bind failed" );
		closesocket(sock);
		exit(EXIT_FAILURE);
	} 


	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr( pparam->target_ip );
	gcAddr.sin_port = htons( MAV_PORT );

	u_long opt = 1;
	ioctlsocket( sock, FIONBIO, &opt );

	mavlink_message_t msg;
	mavlink_status_t status;
	fd_set fdr;
	fdr.fd_array[0] = sock;
	timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	DWORD err = 0;
	DWORD tick = GetTickCount();
	for (;;)
	{
		if( GetTickCount() - tick > 1000 )
		{
			tick = GetTickCount();
			postHeartbeat( sock, gcAddr, position );
		}

		fdr.fd_count = 1;
		select( 0, &fdr, NULL, NULL, &tv );

		uint8_t buf[BUFFER_LENGTH];
		recsize = recvfrom(sock, (char *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if( recsize > 0 )
		{
			// Something received - print out all bytes and parse packet

			//printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (int i = 0; i < recsize; ++i)
			{
				printf("%02x ", buf[i]);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					//printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

					MavManualMsg* manualMsg = NULL;
					BOOL err;
					switch( msg.msgid )
					{
						case MAVLINK_MSG_ID_MANUAL_CONTROL:

							manualMsg = new MavManualMsg();
							manualMsg->roll = mavlink_msg_manual_control_get_roll( &msg )* -5.0;
							manualMsg->pitch = mavlink_msg_manual_control_get_pitch( &msg ) * - 5.0;

							//WARNING: Thrust and Yaw are flipped in QGroundControl for our joystick
							//We're unflipping them below!
							manualMsg->yaw = mavlink_msg_manual_control_get_thrust( &msg ) * -2.0;
							manualMsg->yaw += 1.0;
							manualMsg->thrust = mavlink_msg_manual_control_get_yaw( &msg ) * 2.0;

							err = PostThreadMessage( pparam->mainThreadID, MAV_MSG_CTRL, NULL, (LPARAM)manualMsg );
							if( !err )
							{
								//error
							}
							break;
						case MAVLINK_MSG_ID_COMMAND_LONG:
							//wait current position; continue flightplan; etc is 76

							err = PostThreadMessage( pparam->mainThreadID, MAV_MSG_ACTION, NULL, NULL );
							if( !err )
							{
								//error
							}

							break;
						case MAVLINK_MSG_ID_SET_MODE:
							//msgid 11 is disarm system or set different flight mode
							
							break;
						default:
							
							break;
					}


				}
			}
			printf("\n");
		}
	}
}
