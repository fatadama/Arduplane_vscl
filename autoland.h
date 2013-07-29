//controls.h
//header file for controls.cpp

#ifndef __CONTROLS_H_INCLUDED__
#define __CONTROLS_H_INCLUDED__

void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[]);
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval1[],double denval2[]);
void updateCommanded(int len,double newVal,double array[]);
double airspeed_cmd(double uRefNow, double uNow);
double glideslope_cmd(double gammaRefNow,double gammaNow,double thetaNow);
double theta_cmd(double thetaRefNow, double thetaNow);
double localizer_cmd(double lambdaNow,double psiNow, double phiNow, double range);
double psi_cmd(double psiRefNow,double psiNow,double phiNow, double range);
double phi_cmd(double phiRefNow,double phiNow);

#include<iostream>
using namespace std;

/*
double updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[])

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
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval[])
{
	double output = 0;
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
void updateTransfer(int num, int den, double numtf[],double dentf[],double numval[],double denval1[],double denval2[])
{
double output = 0;
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
void updateCommanded(int len,double newVal,double array[])
{
	for(int i = len-1;i>0;i--)
	{
		array[i] = array[i-1];
	}
	array[0] = newVal;
}

double airspeed_cmd(double uRefNow, double uNow)
{
	//return value
	double deltat = 0;
	//static arrays
	static double Gu_num[] = {1,-1.71427727671312,0.736718178229491};
	static double Gu_den[] = {1.37968043263210,	-1.66949265764827,	0.290100334205593};
	static double Fu_num[] = {1,-1.83621739707991,	0.848629607268316};
	static double Fu_den[] = {9.94945495916093,	-19.3957837362669,	9.45874098729436};
	static double u[] = {0,0,0};
	static double uRef[] = {0,0,0};
	static double u1[] = {0,0,0};
	static double deltat_c[] = {0,0,0};
	//update u_ref and u
	updateCommanded(3,uNow,u);
	updateCommanded(3,uRefNow,uRef);
	//update u_1:
	updateTransfer(3,3,Fu_num,Fu_den,u1,uRef);
	//update deltat_c:
	updateTransfer(3,3,Gu_num,Gu_den,deltat_c,u1,u);
	//return command:
	deltat = deltat_c[0];
	return deltat;
}

double glideslope_cmd(double gammaRefNow,double gammaNow,double thetaNow)
{
	//return value
	double deltae = 0;
	//static arrays
	static double Gl_num[] = {5.5100,-6.5,1.0000};
	static double Gl_den[] = {1,-1,0};
	static double gamma[] = {0,0,0};
	static double gamma_ref[] = {0,0,0};
	static double theta_ref[] = {0,0,0};
	//update gamma_ref and gamma
	updateCommanded(3,gammaRefNow,gamma_ref);
	updateCommanded(3,gammaNow,gamma);
	//update theta_ref:
	updateTransfer(3,3,Gl_num,Gl_den,theta_ref,gamma_ref,gamma);
	//call elevator command function with theta_ref(1):
	deltae = theta_cmd(theta_ref[0],thetaNow);
	return deltae;
}

double theta_cmd(double thetaRefNow, double thetaNow)
{
	double deltae_c = 0;
	//static arrays
	static double Gl_num[] = {1,	-2.55881803866848,	2.29908251036304,	-0.723091390234076};
	static double Gl_den[] = {-2.56357963340450,	6.00479254086400,	-4.67553113001296,	1.23431588414154};
	static double Fl_num[] = {1,	-2.20947288324772,	1.63855114440923,	-0.398436035410764};
	static double Fl_den[] = {23.0884888853174,	-63.6611950931938,	58.7203678746963,	-18.1170194410692};
	static double theta_ref[] = {0,0,0,0};
	static double theta_1[] = {0,0,0,0};
	static double theta[] = {0,0,0,0};
	static double deltae[] = {0,0,0,0};
	//update theta and thetaRef
	updateCommanded(4,thetaRefNow,theta_ref);
	updateCommanded(4,thetaNow,theta);
	//update theta_1
	updateTransfer(4,4,Fl_num,Fl_den,theta_1,theta_ref);
	//update deltae_c
	updateTransfer(4,4,Gl_num,Gl_den,deltae,theta_1,theta);
	deltae_c = deltae[0];

	return deltae_c;
}

double localizer_cmd(double lambdaNow,double psiNow, double phiNow, double range)
{
	double deltaa = 0;
	//static arrays
	static double G_num0[] = {1,	-1.99340000000000,	0.996900000000000};
	static double G_den0[] = {0.522600000000000,	-0.997400000000000,	0.476600000000000};
	static double lambda[] = {0,0,0};
	static double psi_ref[] = {0,0,0};
	//update lambda:
	updateCommanded(3,lambdaNow,lambda);
	//update psi_ref:
	updateTransfer(3,3,G_num0,G_den0,psi_ref,lambda);
	//call psi_cmd:
	deltaa = psi_cmd(psi_ref[0],psiNow,phiNow,range);
	return deltaa;
}

double psi_cmd(double psiRefNow,double psiNow,double phiNow, double range)
{
	double deltaa = 0;
	//static arrays
	static double F_num2[] = {1,	-2.90394468693377,	2.81265804708752,	-0.908421848795803};
	static double F_den2[] = {0.0878265897919200,	-0.174604400295509,	0.0911873082359091,	-0.00411798637437450};
	static double psi_ref[] = {0,0,0,0};
	static double psi_1[] = {0,0,0,0};
	static double psi = 0;
	static double phi_ref = 0;
	//update values:
	psi = psiNow;
//avoid singularity at x = 0
	if(fabs(range)>10)
	{
		updateCommanded(4,psiRefNow,psi_ref);
	}
	else
	{
		updateCommanded(4,psiRefNow,0);
	}
	//update filtered state
	updateTransfer(4,4,F_num2,F_den2,psi_1,psi_ref);
	//update commanded bank angle:
	phi_ref = psi_1[0]-psi;
	//enforce state limits:
	if(phi_ref>=0.3491 && phiNow>=0.3491)
	{
		phi_ref = 0.3491;
	}
	else if (phi_ref<=-0.3491 && phiNow <= -0.3491)
	{
		phi_ref = -0.3491;
	}
	//call phi_cmd:
	deltaa = phi_cmd(phi_ref,phiNow);
	return deltaa;
}

double phi_cmd(double phiRefNow,double phiNow)
{
	double deltaa = 0;
	//static arrays
	static double G_num[] = {1,	-2.58165789764699,	2.17866605723825,	-0.596865099706433,	0};
	static double G_den[] = {-1.61359761025044,	2.60159907884129,	0.0342057294154141,	-1.43607324462026,	0.413865536847243};
	static double F_num[] = {1,0,0};
	static double F_den[] = {46.2943565401117, -68.8117701713310,	23.5174136312193};
	static double phi_ref[] = {0,0,0};
	static double phi_1[] = {0,0,0,0,0};
	static double phi[] = {0,0,0,0,0};
	static double deltaa_c[] = {0,0,0,0,0};
	//update commanded:
	updateCommanded(5,phiNow,phi);
	updateCommanded(3,phiRefNow,phi_ref);
	//filtered state:
	//manually update phi_1[4] and phi_1[3], because updateTransfer only covers phi_1[0:2]
	phi_1[4] = phi_1[3];phi_1[3] = phi_1[2];
	updateTransfer(3,3,F_num,F_den,phi_1,phi_ref);
	//commanded aileron:
	updateTransfer(5,5,G_num,G_den,deltaa_c,phi_1,phi);
	deltaa = deltaa_c[0];
	return deltaa;
}

#endif