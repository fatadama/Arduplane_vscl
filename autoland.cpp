//autoland.cpp - function definitions for VSCL_autoland class

/*
	THROTTLE, ELEVATOR, AILERON, ROLL, AND PITCH ANGLES ARE CONSTRAINED
	CONSTRAINED aileron, elevator, pitch, and roll are stored in history vectors. This is somewhat different
	than what I did in simulation. This is a possible source of error if behavior is strange.
*/

#include "autoland.h"

//runway settings, as defines
//localizer global latitude in degrees*10^7
static const int32_t LOC_LAT = 306371735;
//localizer global longitude in degrees*10^7
static const int32_t LOC_LONG = -964850664;
//ETA_R is the runway direction. COS_ETA_R_CONST = cos(ETA_R)*1e2*1e-7*radius_of_earth*d2r()
static const float COS_ETA_R_CONST = -1.1132;//-0.78637;//-1.1132;
//SIN_ETA_R = sin(ETA_R)*1e-5*radius_of_earth*d2r()
static const float SIN_ETA_R_CONST = 0.0;//0.78637;//0.0;
//ETA_R: angle from the local north TO the direction of landing on the runway. Required to properly determine yaw angle relative to the runway.
static const float ETA_R = 3.14159265359;//2.3561945;
//flare altitude (cm):
static const uint8_t h_flare = 400;

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
	flare_alt = 0;
	elevator_out = 0;
	aileron_out = 0;
	throttle_out = 0;
	elevator_servo = 0;
	aileron_servo = 0;
	ref_yaw = -3159;//=-181 degrees, intended to signal the variable has not been initialized
	ref_pitch = -3159;
	ref_roll = -3159;
	ref_gamma = -3159;
	ref_lambda = -3159;
	_reset = 0;
	return;
}

//elevator_get(): returns the servo output to achieve current commanded elevator deflection in centidegrees
int32_t VSCL_autoland::elevator_get()
{
	//return elevator_out;
	return elevator_servo;
}

//throttle_get(): returns the current commanded throttle in centidegrees??
int32_t VSCL_autoland::throttle_get()
{
	return throttle_out;
}

//aileron_get(): returns the servo output to achieve current commanded aileron in centidegrees
	//CHECK THAT SERVO UNITS SHOULD BE CENTIDEGREES
int32_t VSCL_autoland::aileron_get()
{
	//return aileron_out;
	return aileron_servo;
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
	
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<4;ii++)
		{
			theta_ref[ii] = 0;
			theta_1[ii] = 0;
			theta[ii] = 0;
			deltae[ii] = 0;
		}
		return;
	}
//update theta and thetaRef
	updateCommanded(4,thetaRefNow,theta_ref);
	updateCommanded(4,thetaNow,theta);
//update theta_1
	updateTransfer(4,4,Fl_num,Fl_den,theta_1,theta_ref);
//update deltae_c (radians)
	updateTransfer(4,4,Gl_num,Gl_den,deltae,theta_1,theta);
	//constrain elevator to +/- .7853 radians = 45.00 deg
	deltae[0] = constrain(deltae[0],-.7853,.7853);
//set the output value in centidegrees
	elevator_out = int32_t(deltae[0]*5730);
//store theta_ref and elevator command:
	ref_pitch = int32_t(thetaRefNow*10000);//10^-4 radians
}

void VSCL_autoland::glideslope_cmd(float gammaRefNow, float gammaNow, float thetaNow, int32_t range)
{
//static arrays
	static float Gl_num[] = {5.51,-6.5,1.00};
	static float Gl_den[] = {1,-1,0};
	static float gamma[] = {0,0,0};
	static float gamma_ref[] = {0,0,0};
	static float theta_ref[] = {0,0,0};
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<3;ii++)
		{
			gamma[ii] = 0;
			gamma_ref[ii] = 0;
			theta_ref[ii] = 0;
		}
		return;
	}
//update gamma_ref and gamma
	//if (abs(range) > 5000){
		updateCommanded(3,gammaRefNow,gamma_ref);
		updateCommanded(3,gammaNow,gamma);
	//}
	/*else
	{
		updateCommanded(3,gammaRefNow*abs(range)*0.0002,gamma_ref);
		updateCommanded(3,gammaNow*abs(range)*0.0002,gamma);
	}*/
//update theta_ref:
	updateTransfer(3,3,Gl_num,Gl_den,theta_ref,gamma_ref,gamma);
//enforce state limits - if theta is greater than 20 deg, do not command it to increase
	if(fabs(thetaNow)>.3491)
	{
		theta_ref[0] = constrain(theta_ref[0],-.3491,.3491);
	}
//call theta_cmd to update elevator command
	theta_cmd(theta_ref[0],thetaNow);
}

/*
elevator_update(float thetaNow)

INPUTS:
	thetaNow - current pitch angle in radians, pitch up is positive
*/
void VSCL_autoland::elevator_update(int32_t x_lcl, int16_t alt_cm, float thetaNow)
{
	static int16_t alt_last_cm = 0;//static variable that stores the previous altitude
//handle reset
	if (_reset)
	{
		alt_last_cm = 0;
		return;
	}
//computes and sets elevator_out
	//for now, compute gammaNow from GPS coords passed to this function
	ref_gamma = (10000.0*alt_cm)/abs(x_lcl);
	if (alt_cm>h_flare)
	{
	flare_alt = 0;
//glideslope tracking branch
		//compute current glideslope angle:
		float gammaNow = float(alt_cm)/abs(x_lcl);
		ref_gamma = 10000.0*gammaNow;
		//call glideslope tracker with a constant 5 deg glideslope
		glideslope_cmd(.08727, 
			gammaNow,
			thetaNow-theta1,
			x_lcl);
	}
	else
	{
	flare_alt = 1;
//autoflare branch
		flare_cmd(.01*alt_cm,
			10.*float(alt_cm-alt_last_cm)/(float(millis()-last_update)),
			thetaNow-theta1);
	}
//store the altitude to estimate descent rate
	alt_last_cm = alt_cm;
//store the current time to estimate the time in between updates
	last_update = millis();
}

//set the aileron to meet the commanded bank angle
void VSCL_autoland::phi_cmd(float phiRefNow, float phiNow)
{
//static arrays
	static float G_num[] = {1,	-2.58165789764699,	2.17866605723825,	-0.596865099706433,	0};
	static float G_den[] = {-1.61359761025044,	2.60159907884129,	0.0342057294154141,	-1.43607324462026,	0.413865536847243};
	static float F_num[] = {1,0,0};
	static float F_den[] = {46.2943565401117, -68.8117701713310,	23.5174136312193};
	static float phi_ref[] = {0,0,0};
	static float phi_1[] = {0,0,0};
	
	static float phi_2[] = {0,0,0,0,0};
	//static float phi[] = {0,0,0,0,0};
	static float deltaa_c[] = {0,0,0,0,0};
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<5;ii++)
		{
			phi_2[ii] = 0;
			deltaa_c[ii] = 0;
		}
		for(int ii = 0;ii<3;ii++)
		{
			phi_1[ii] = 0;
			phi_ref[ii] = 0;
		}
		return;
	}
//update commanded values
	//updateCommanded(5,phiNow,phi);
	updateCommanded(3,phiRefNow,phi_ref);
//filtered state:
	updateTransfer(3,3,F_num,F_den,phi_1,phi_ref);
// error state:
	updateCommanded(5,phi_1[0]-phiNow,phi_2);
//commanded aileron:
	updateTransfer(5,5,G_num,G_den,deltaa_c,phi_2);
	//constrain aileron to +/- .7853 rad = 45.00 deg
	deltaa_c[0] = constrain(deltaa_c[0],-.7853,.7853);
//set target aileron in centidegrees
	aileron_out = int32_t(deltaa_c[0]*5730);
//store values in status vector
	ref_roll = int32_t(phiRefNow*10000.0);
}

void VSCL_autoland::psi_cmd(float psiRefNow, float psiNow, float phiNow, int16_t range)
{
//static arrays
	static float F_num2[] = {1,-1.65667676113393,0.666113042405827};
	static float F_den2[] = {15.2680726649204,-29.9262827863,14.6676464026516};
	static float G_num2[] = {1.0,-0.96364320647571};
	static float G_den2[] = {0.0382413880786626,-0.001912689195318};
	const static int Flen = 3;//assume num and den are the same length
	const static int Glen = 2;//same here
//
	static float psi_ref[] = {0,0,0};
	static float psi_1[] = {0,0,0};
//
    static float psi_2[] = {0,0};
	static float psi[] = {0,0};
	static float phi_ref[] = {0,0};
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<Flen;ii++)
		{
			psi_ref[ii] = 0;
			psi_1[ii] = 0;
		}
        for(int ii = 0;ii<Glen;ii++)
		{
			psi_2[ii] = 0;
			psi[ii] = 0;
			phi_ref[ii] = 0;
		}
		return;
	}
//update values:
	if((psiNow - ETA_R) < -3.14159){
		psiNow += 6.283185307;
	}
	else if((psiNow-ETA_R) > 3.14159)
	{
		psiNow -= 6.283185307;
	}
    updateCommanded(Glen,psiNow-ETA_R,psi);
//avoid singularity at X = 0 (local NED coordinate), recall range is in centimeters
	updateCommanded(Flen,psiRefNow,psi_ref);
//update filtered state
	updateTransfer(Flen,Flen,F_num2,F_den2,psi_1,psi_ref);
    updateCommanded(Glen,psi_1[0]-psi[0],psi_2);
//update commanded bank angle:
    updateTransfer(Glen,Glen,G_num2,G_den2,phi_ref,psi_2);
//enforce state limits - do not command bank angles above 20 degrees (.3491 rad) if current bank angle is greater than 20
	if(fabs(phiNow)>.3491)
	{
		phi_ref[0] = constrain(phi_ref[0],-.3491,.3491);
	}
//call aileron control function
	phi_cmd(phi_ref[0],phiNow);
//store reference heading in status vector
	ref_yaw = int32_t(10000.*psiRefNow);
}

void VSCL_autoland::localizer_cmd(float lambdaNow,float psiNow, float phiNow, int16_t range)
{
//inputs: lambda in radians, psi in radians, phi in radians, range in cm
//static arrays
	//static float G_num0[] = {1,-1.99340000000000,0.996900000000000};
	static float G_num0[] = {8.7,-16.2,7.5};
	//static float G_den0[] = {0.522600000000000,-0.997400000000000,	0.476600000000000};
	static float G_den0[] = {1.,-1.,0.};
	static float lambda[] = {0,0,0};
	static float psi_ref[] = {0,0,0};
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<3;ii++)
		{
			lambda[ii] = 0;
			psi_ref[ii] = 0;
		}
	}
//store current value of lambda
	if(abs(range) > 5000){
		updateCommanded(3,-1.0*lambdaNow,lambda);
	}
	else{
		updateCommanded(3,-.0002*abs(range)*lambdaNow,lambda);
	}
//update psi_ref:
	updateTransfer(3,3,G_num0,G_den0,psi_ref,lambda);
//call heading reference function
	psi_cmd(psi_ref[0],psiNow,phiNow,range);
}

void VSCL_autoland::aileron_update(int32_t x_lcl, int32_t y_lcl,float psiNow, float phiNow)
{
//compute localizer angle
	float lambdaNow = float(y_lcl)/abs(x_lcl);//radians
	ref_lambda = 10000.0*lambdaNow;
//call localizer function to compute aileron command:
	localizer_cmd(lambdaNow,psiNow,phiNow,x_lcl);
}

void VSCL_autoland::flare_cmd(float hNow, float hdotNow, float thetaNow)
{
//constant arrays
	/*static float Gf_num1[] = {2.60050000000000,	-5.10000000000000,	2.50000000000000},
		Gf_den1[] = {1,-1,0},
		Ff_num1[] = {1,	-2.44263292099462,	2.27575100243921,	-0.989813891276108,	0.182469577565696},
		Ff_den1[] = {4.71878410637755,	-8.93150040050544,	4.23849006186207},*/
        static float Gf_num1[] = {2.5250,-5.025,2.50},
        Gf_den1[] = {1.0,-1.0,0.0},
        Ff_num1[] = {.00726311442},
        Ff_den1[] = {1.0,-1.875842502,0.8831056166},
		TAU_INV = -0.4;
//static arrays
	static float theta_ref[] = {0,0,0},//Gdlen
		hdot_ref[] = {0},//Fnlen
		hdot_1[] = {0,0,0},//Fdlen
		hdot[] = {0,0,0};//Gnlen
	//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<3;ii++)
		{
			theta_ref[ii] = 0;
			hdot_1[ii] = 0;
			hdot[ii] = 0;
		}
		hdot_ref[0] = 0;
		return;
	}
//store reference vertical speed
	updateCommanded(1,hNow*TAU_INV,hdot_ref);
//store vertical speed
	updateCommanded(3,hdotNow,hdot);
//prefilter
	updateTransfer(1,3,Ff_num1,Ff_den1,hdot_1,hdot_ref);
//commanded pitch angle:
	updateTransfer(3,3,Gf_num1,Gf_den1,theta_ref,hdot_1,hdot);
//enforce state limits - if theta is greater than 10 deg, do not command it to increase
	if(fabs(thetaNow)>.1746)
	{
		theta_ref[0] = constrain(theta_ref[0],-.1746,.1746);
	}
//call elevator command function
	theta_cmd(theta_ref[0],thetaNow);
}

//update commanded throttle
void VSCL_autoland::airspeed_cmd(float uRefNow,float uNow)
{
	//static arrays
	static float Gu_num[] = {1,-1.71427727671312,0.736718178229491},
		Gu_den[] = {1.37968043263210,	-1.66949265764827,	0.290100334205593},
		Fu_num[] = {1,-1.83621739707991,	0.848629607268316},
		Fu_den[] = {9.94945495916093,	-19.3957837362669,	9.45874098729436};
	static float u[] = {0,0,0},
		uRef[] = {0,0,0},
		u1[] = {0,0,0},
		deltat_c[] = {0,0,0};
//handle reset
	if (_reset)
	{
		for(int ii = 0;ii<3;ii++)
		{
			u[ii] = 0;
			uRef[ii] = 0;
			u1[ii] = 0;
			deltat_c[ii] = 0;
		}
		return;
	}
	//update u_ref and u
	updateCommanded(3,uNow,u);
	updateCommanded(3,uRefNow,uRef);
	//update u_1:
	updateTransfer(3,3,Fu_num,Fu_den,u1,uRef);
	//update deltat_c:
	updateTransfer(3,3,Gu_num,Gu_den,deltat_c,u1,u);
//set throttle command
	throttle_out = int16_t(100*deltat_c[0]-DELTAT1);
//make sure throttle is in [0,100]
	throttle_out = constrain(throttle_out,0,100);
}

void VSCL_autoland::throttle_update(float uNow,int16_t alt_cm)
{
	if (alt_cm>h_flare)
	{
		airspeed_cmd(0.0,uNow-U1);
	}
	else
	{
//		airspeed_cmd(-2.0,uNow-U1);
                throttle_out = 0;
	}
}

int32_t VSCL_autoland::psi_get()
{
	return ref_yaw;
}

int32_t VSCL_autoland::theta_get()
{
	return ref_pitch;
}

int32_t VSCL_autoland::phi_get()
{
	return ref_roll;
}

int32_t VSCL_autoland::gamma_get()
{
	return ref_gamma;
}

int32_t VSCL_autoland::lambda_get()
{
	return ref_lambda;
}

void VSCL_autoland::reset()
{
	_reset = 1;
	//call functions to reset
	theta_cmd(0,0);
	glideslope_cmd(0,0,0,0);
	elevator_update(0,0,0.0);
	phi_cmd(0,0);
	psi_cmd(0,0,0,0);
	localizer_cmd(0,0,0,0);
	flare_cmd(0,0,0);
	airspeed_cmd(0,0);
	//return to normal operation
	_reset = 0;
}

void VSCL_autoland::update(int32_t lat_e7, int32_t lng_e7, int16_t alt_cm,float uNow, float thetaNow, float psiNow,float phiNow)
{
//compute X Y Z location in runway-centric NED style frame
//compute relative GPS coordinates
	lat_e7 -= LOC_LAT;
	lng_e7 -= LOC_LONG;
//compute relative X-Y coordinates
	int32_t x_lcl = lat_e7*COS_ETA_R_CONST + lng_e7*SIN_ETA_R_CONST;//cm
	int32_t y_lcl = lng_e7*COS_ETA_R_CONST - lat_e7*SIN_ETA_R_CONST;//cm
//update elevator including IMPORTANT altitude offset for testing:
	elevator_update(x_lcl,alt_cm - alt_testing_offset,thetaNow);
//update aileron:
	aileron_update(x_lcl,y_lcl,psiNow,phiNow);
//update throttle
	throttle_update(uNow,alt_cm - alt_testing_offset);
//compute the servo outputs
	servo_compute();
}

void VSCL_autoland::servo_compute()
{
	elevator_servo = -2.5322*elevator_out+741.65;//uses linear approximation tan(x)~~x. Check the sign of the deflection here
	aileron_servo = -3.8257*aileron_out-1776;//check the sign
}
