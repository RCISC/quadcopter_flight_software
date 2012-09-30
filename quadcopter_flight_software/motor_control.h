#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H


class MotorControl
{
public:
	MotorControl( );
	~MotorControl( );
	bool openPort( const char* portname );
	void zeroMotors();
	void setMotor( unsigned char motor, double value );

private:
		FILE* commPort;
		HANDLE port;
};

#endif