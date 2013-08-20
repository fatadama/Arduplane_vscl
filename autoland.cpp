//autoland.cpp - function definitions for autoland class

#include "autoland.h"

/*
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[])
function for updating stored values using the discrete-time transfer functions

Takes the size of the transfer function and nuerator and denominator polynomials and updates the output value "numval"
	using the transfer function coefficients and the values of "denval." At the time of function call, "denval" has 
	already been updated with the most current value. "numval" HAS NOT.

inputs:
	num - length of the numerator in the transfer function
	den - length of the denominator vector
	numtf - polynomial coefficient array of the transfer function
	dentf - denominator coefficient array of the transfer function
	numval - previous values of the variable that is being updated.
	denval - previous values of the variable that is used to update "numval".
outputs:
	void
*/
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval[])
{
	float output = 0;
	for (int i = 0;i<num;i++)
	{
		output+=numtf[i]*denval[i];
	}
	for (int i = den-1;i>0;i--)
	{
		//update the values of numval at the same time.
		numval[i] = numval[i-1];
		output-=dentf[i]*numval[i];
	}
	output/=dentf[0];
	numval[0] = output;
}

//overloaded version of updateTransfer where the reference value is an error between two stored vectors. 
//tldr: denval1 - denval2 is used instead of denval.
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval1[],float denval2[])
{
	float output = 0;
	for (int i = 0;i<num;i++)
	{
		output+=numtf[i]*(denval1[i]-denval2[i]);
	}
	for (int i = den-1;i>0;i--)
	{
		//update the values of numval at the same time.
		numval[i] = numval[i-1];
		output-=dentf[i]*numval[i];
	}
	output/=dentf[0];
	numval[0] = output;
}

//update an array whose new value is either measured or computed elsewhere; i.e., array[1:end] = array[0:end-1], array[0]=newValue
void updateCommanded(int len,float newVal,float array[])
{
	for(int i = len-1;i>0;i--)
	{
		array[i] = array[i-1];
	}
	array[0] = newVal;
}

//constructor - not sure if required but does nothing
VSCL_autoland::VSCL_autoland()
{
	return;
}

//elevator_get(): returns the current commanded elevator deflection in centidegrees
int16_t VSCL_autoland::elevator_get()
{
	return elevator_out;
}

//throttle_get(): returns the current commanded throttle in centidegrees??
int16_t VSCL_autoland::throttle_get()
{
	return throttle_out;
}

//aileron_get(): returns the current commanded aileron in centidegrees
int16_t VSCL_autoland::aileron_get()
{
	return aileron_out;
}

void VSCL_autoland::theta_cmd(float thetaRefNow, float thetaNow)
{
//prefilter and controller coefficients
	static float Gl_num[] = {1,	-2.55881803866848,	2.29908251036304,	-0.723091390234076};
	static float Gl_den[] = {-2.56357963340450,	6.00479254086400,	-4.67553113001296,	1.23431588414154};
	static float Fl_num[] = {1,	-2.20947288324772,	1.63855114440923,	-0.398436035410764};
	static float Fl_den[] = {23.0884888853174,	-63.6611950931938,	58.7203678746963,	-18.1170194410692};
//stored values of previous inputs/outputs/intermediaries
	static float theta_ref[] = {0,0,0,0};
	static float theta_1[] = {0,0,0,0};
	static float theta[] = {0,0,0,0};
	static float deltae[] = {0,0,0,0};
//update theta and thetaRef
	updateCommanded(4,thetaRefNow,theta_ref);
	updateCommanded(4,thetaNow,theta);
//update theta_1
	updateTransfer(4,4,Fl_num,Fl_den,theta_1,theta_ref);
//update deltae_c (radians)
	updateTransfer(4,4,Gl_num,Gl_den,deltae,theta_1,theta);
//set the output value in centidegrees
	elevator_out = int16_t(deltae[0]*1.745);
}

void VSCL_autoland::elevator_update(float thetaNow)
{
//computes and sets elevator_out
	//call glideslope tracker
	theta_cmd(0.0,thetaNow);
}
