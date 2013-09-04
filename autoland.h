//autoland.h - class definition for control function implementation

#ifndef __USE_VSCL_AUTOLAND_H__
#define __USE_VSCL_AUTOLAND_H__

#include <AP_Common.h>

//runway settings are in autoland.cpp

//steady-state values of aircraft states:
//pitch angle
#define theta1 -.0197
//forward speed
#define U1 12.6431
//throttle steady-state (.6329)
#define DELTAT1 63

/*
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[])
function for updating stored values using the discrete-time transfer functions
*/
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval[]);

//overloaded version of updateTransfer where the reference value is an error between two stored vectors.
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval1[],float denval2[]);

//update an array whose new value is either measured or computed elsewhere; i.e., array[1:end] = array[0:end-1], array[0]=newValue
void updateCommanded(int len,float newVal,float array[]);

class VSCL_autoland{
	public:
	//constructor
		VSCL_autoland();
	//functions to compute output values
		void elevator_update(int32_t lat_e7, int32_t lng_e7, int16_t alt_cm, float thetaNow);
		void throttle_update(float uNow,int16_t alt_cm);
		void aileron_update(int32_t lat_e7, int32_t lng_e7,float psiNow, float phiNow);
	//functions to access commanded settings
		int16_t elevator_get();//returns values in centidegrees
		int16_t throttle_get();
		int16_t aileron_get();
		int16_t psi_get();//returns heading reference in 10^-4 radians
		int16_t theta_get();//returns pitch reference in 10^-4 radians
		int16_t phi_get();//returns roll reference in 10^-4 radians
		void reset();//function to reset the static arrays in the control computation functions
	private:
		//last time the update() function(s) got called - used to estimate descent rate to account for inaccuracy in update time
		int32_t last_update;
		//current commanded angle of each channel
		int16_t elevator_out;
		int16_t throttle_out;//throttle output is in percentage?
		int16_t aileron_out;
		//current commanded angles of each axis in 10^-4 radians
		int16_t ref_yaw;
		int16_t ref_pitch;
		int16_t ref_roll;
		// functions to compute control deflections
		void theta_cmd(float thetaRefNow, float thetaNow);
		void glideslope_cmd(float gammaRefNow, float gammaNow, float thetaNow);
		void phi_cmd(float phiRefNow, float phiNow);
		void psi_cmd(float psiRefNow, float psiNow, float phiNow, int16_t range);
		void localizer_cmd(float lambdaNow,float psiNow, float phiNow, int16_t range);
		void flare_cmd(float hNow, float hdotNow, float thetaNow);
		void airspeed_cmd(float uRefNow,float uNow);
		//flag to reset functions - for flight testing purposes only
		bool _reset;
};

#endif
