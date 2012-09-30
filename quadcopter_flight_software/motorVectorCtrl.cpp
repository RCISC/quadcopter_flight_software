#include "stdafx.h"
#include "motorVectorCtrl.h"
#include "math.h"

motorVectorCtrl::motorVectorCtrl()
{
	/*
	      [0]
	[1]			[3]
		  [2]
		  */

	liftC = 0;
	tiltXC = 0;
	tiltYC = 0;
	rotC = 0;

	liftVect[0] = 1;
	liftVect[1] = 1;
	liftVect[2] = 1;
	liftVect[3] = 1;

	tiltXvect[0] = 0;
	tiltXvect[1] = 1;
	tiltXvect[2] = 0;
	tiltXvect[3] = -1;
	
	tiltYvect[0] = 1;
	tiltYvect[1] = 0;
	tiltYvect[2] = -1;
	tiltYvect[3] = 0;

	rotvect[0] = 1;
	rotvect[1] = -1;
	rotvect[2] = 1;
	rotvect[3] = -1;

	stableNoInp = 0.0;
}

void motorVectorCtrl::setAllCoefficients(double lift, double rotate, double tiltX, double tiltY)
{
	liftC = lift;
	rotC = rotate;
	tiltXC = tiltX;
	tiltYC = tiltY;
}

double myMax( double a, double b )
{
	return (b<a)?a:b;
}

double myMin( double a, double b )
{
	return (b>a)?a:b;
}

/*multiplier by which to multiply lift in order to compensate for tilt*/
double motorVectorCtrl::adjustLiftPowForTilt( double x, double y )
{
	return 1 / ( cos( ( x * pi/ 180 ) ) * cos(y * pi/ 180) );
}

void motorVectorCtrl::computeVect()
{
	for( int i = 0; i < 4; i++ )
	{
		motorPower[i] = (liftC * liftVect[i] + rotC * rotvect[i] + tiltXC * tiltXvect[i] + tiltYC * tiltYvect[i]);
	}
	//magnitude
	double maxVal = myMax( myMax( motorPower[0], motorPower[1] ), 
					myMax( motorPower[2], motorPower[3] ) );
	
	double minVal = myMin( myMin( motorPower[0], motorPower[1] ), 
					myMin( motorPower[2], motorPower[3] ) );

	double scaleDown = 1;

	if( maxVal > ( 1 - stableNoInp ) )
	{
		scaleDown = maxVal / ( 1 - stableNoInp );
	}
	if( minVal < ( 0 - stableNoInp ) )
	{
		if( 0 == stableNoInp)
		{
			scaleDown = myMax( scaleDown, minVal );
		}
		else
		{
			scaleDown = myMax( scaleDown, minVal / ( 0 - stableNoInp ) );
		}
	}

	for( int i = 0; i < 4; i++ )
	{
		motorPower[i] /= scaleDown;
		motorPower[i] += stableNoInp;
		if( motorPower[i] < 0 )
			motorPower[i] = 0;
	}
}