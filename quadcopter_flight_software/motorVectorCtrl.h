#ifndef MOTORVECTORCTRL_H
#define MOTORVECTORCTRL_H

class motorVectorCtrl
{
public:
	motorVectorCtrl();
	void setAllCoefficients(double lift, double rotate, double tiltX, double tiltY);
	void computeVect();
	double adjustLiftPowForTilt( double x, double y );

	double motorPower[4];
	double stableNoInp;

private:
	double liftVect[4];
	double tiltXvect[4];
	double tiltYvect[4];
	double rotvect[4];

	double liftC;
	double tiltXC;
	double tiltYC;
	double rotC;
	
};

static const double pi = 3.1415926535;

#endif