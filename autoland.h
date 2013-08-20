//autoland.h - class definition for control function implementation

#ifndef __USE_VSCL_AUTOLAND_H__
#define __USE_VSCL_AUTOLAND_H__

#include <AP_Common.h>

/*
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[])
function for updating stored values using the discrete-time transfer functions
*/
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval[]);

//overloaded version of updateTransfer where the reference value is an error between two stored vectors.
void updateTransfer(int num, int den, float numtf[],float dentf[],float numval[],float denval1[],float denval2[]);

//update an array whose new value is either measured or computed elsewhere; i.e., array[1:end] = array[0:end-1], array[0]=newValue
void updateCommanded(int len,float newVal,float array[])
{
	for(int i = len-1;i>0;i--)
	{
		array[i] = array[i-1];
	}
	array[0] = newVal;
}
class VSCL_autoland{
	public:
	//constructor
		VSCL_autoland();
	//functions to compute output values
		void elevator_update(float thetaNow);
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
};

#endif
