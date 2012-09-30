#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "logger.h"

struct PIDcoefficients
{
	double Kp, Ki, Kd;
};

class PidControl
{
public:
	PidControl( double P, double I, double D, QLogger* logger );
	void setPIDValue( double P , double I , double D );
	double pidMotorValue( double setPoint , double processValue, double curTime ); // the set point is what is set with a joystick , process value is actual value
	double Kp; // the coefficients for the pid control
	double Ki;
	double Kd;
private:
	double u_t_1;
	double t_last;
	double e_t_1;
	double e_t_2;

	double P_err;
	double I_err;
	double D_err;
	
	double error;
	double errorOld;

	QLogger* logger;
};

#endif