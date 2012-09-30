
#include "stdafx.h"
#include "pidControl.h"


PidControl::PidControl( double P, double I, double D, QLogger* loggerP )
{
	Kp = P;
	Ki = I;
	Kd = D;

	e_t_2 = 0;
	e_t_1 = 0;
	u_t_1 = 0;
	t_last = 0;

	P_err = 0;
	I_err = 0;
	D_err = 0;
	error = 0;
	errorOld = 0;

	if( loggerP )
	{
		logger = loggerP;
	}
	else
	{
		logger = NULL;
	}
}

double PidControl::pidMotorValue( double setPoint , double processValue, double curTime )	
{
	
	double delta_time = ( curTime - t_last ); //in milliseconds
	t_last = curTime;

	if( delta_time <= 0 )
	{
		delta_time = 1;
	}

	double e_t = setPoint - processValue;
	
	if( logger )
	{
		
		char logbuf[512];
		sprintf_s( logbuf, "PID loop: err: %lf, stpt %lf, val %lf", e_t, setPoint, processValue );
		logger->log( logbuf );
		
	}
	
	P_err = e_t;

	
	I_err += (errorOld*delta_time); //TODO: sub in error instead of errorOld?
	//windup protection
	if( 0 == Ki )
	{
		I_err = 0;
	}
	else if( I_err > ( 1 / Ki ) )
	{
		I_err = 1 / Ki;
	}
	else if( -I_err > ( 1 / Ki  ) )
	{
		I_err = -1 / Ki;
	}
	D_err = (e_t - errorOld)/delta_time;
	
	errorOld = e_t;

	return Kp*P_err + Ki*I_err + Kd*D_err;
	

	/*
	double toRet = u_t_1 + Kp*( e_t - e_t_1 ) + Ki*delta_time + Kd*(e_t - 2*e_t_1 + e_t_2) / delta_time;
	toRet = toRet > 1.0 ? 1.0 : toRet;
	toRet = toRet < -1.0 ? -1.0 : toRet;
	*/

	/*
	

	e_t_2 = e_t_1;
	e_t_1 = e_t;
	u_t_1 = toRet;
	t_last = curTime;

	return toRet;*/
}

void PidControl::setPIDValue( double P , double I , double D )
{
	Kp = P;
	Ki = I;
	Kd = D;
}