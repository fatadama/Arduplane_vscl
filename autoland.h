//autoland.h - class definition for control function implementation

#ifndef __USE_VSCL_AUTOLAND_H__
#define __USE_VSCL_AUTOLAND_H__

#include <AP_Common.h>

//runway settings, as defines
//localizer global latitude in degrees*10^7
#define LOC_LAT 306382350
//localizer global longitude in degrees*10^7
#define LOC_LONG -964855190
//ETA_R is the runway direction. COS_ETA_R_CONST = cos(ETA_R)*1e4*1e-5*radius_of_earth*d2r()
#define COS_ETA_R_CONST 7871
//SIN_ETA_R = sin(ETA_R)*1e4*1e-5*radius_of_earth*d2r()
#define SIN_ETA_R_CONST 7871

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
		//void throttle_update();
		//void aileron_update();
	//functions to access commanded settings
		int16_t elevator_get();
		int16_t throttle_get();
		int16_t aileron_get();
	private:
		//current PWM commanded settings of each channel
		int16_t elevator_out;
		int16_t throttle_out;//throttle output is in percentage?
		int16_t aileron_out;
		// functions to compute control deflections
		void theta_cmd(float thetaRefNow, float thetaNow);
		void glideslope_cmd(float gammaRefNow, float gammaNow, float thetaNow);
};

#endif
