#ifndef MAVLINK_UDP_H
#define MAVLINK_UDP_H

struct MavThreadParam
{
	DWORD mainThreadID;
	char target_ip[ 128 ];
};

struct MavManualMsg
{
	float roll; ///< roll
	float pitch; ///< pitch
	float yaw; ///< yaw
	float thrust; ///< thrust
};

DWORD WINAPI mav_thread( void * param );

#endif
