// ------------------------------------------------------------ //
// This file defines the functions of a class that implements the push recovery
// controller for the biped SoftLegs. 
// Where possible the function names and variable names are kept the
// same as the `push recovery controller` defined in MATLAB.
//
// Edoardo Sorelli: edoardo.sorelli@gmail.com
// ------------------------------------------------------------ //

#include "imu_management/EvaluateIMUClass.h"

/// -------------------- CONSTRUCTOR -------------------- ///

evaluate_IMU::evaluate_IMU(float node_rate)
{
	rate = node_rate;
	raw_acc.data  = 0;
	filt_acc.data = 0;
	raw_vel.data  = 0;
	filt_vel.data = 0;
	IMUdata.vel   = 0;
	IMUdata.pitch = 0;
	// sampling freqeuncy [kHz] = rate * 1000
	acc_filter = new Filter(FILT_T, N_TAPS, rate / 1000, FL, FU);  	   		// see filt.h for a better explaination 
	ROS_INFO("FILTER ERROR flag = %d <---", acc_filter->get_error_flag()); 	// see filt.h for the error value meaning
	if(acc_filter->get_error_flag() != 0 ) exit(1);    			   			// abort in an appropriate manner   

	array_acc = Eigen::VectorXd::Zero(20);
}

/// -------------------- CALLBACKS -------------------- ///

void evaluate_IMU::acc_Callback(const qb_interface::inertialSensorArray &acc)
{
	// m is an array but the dimension should be always 1
	// the acceleration is measured in [g], multiply for G to convert to [m/s]
	xdd = acc.m[0].x * G;
	ydd = acc.m[0].y * G;
	zdd = acc.m[0].z * G;  	// the gravity acceleration is evaluated too, it is negative

}
void evaluate_IMU::angles_Callback(const qb_interface::anglesArray &angles)
{
	// m is an array but the dimension should be always 1
	// the angles are measured in [degree], the conversion is done in the controller
	r = angles.m[0].r;		// roll
	p = angles.m[0].p;		// pitch
	y = angles.m[0].y;		// yaw
}

/// -------------------- EVALUATE IMU DATA -------------------- ///

void evaluate_IMU::evaluate_IMU_data(){

	// acc and vel filter debug
	raw_acc.data  = (xdd * cos(p)) + (zdd * sin(p));

	array_acc << array_acc.tail(array_acc.size() - 1), raw_acc.data;
    raw_acc.data = array_acc.mean();

	filt_acc.data = acc_filter->do_sample(raw_acc.data);

	raw_vel.data  += filt_acc.data  * (1 / rate);
	filt_vel.data += filt_acc.data * (1 / rate);

	// simplest implementation: v(k+1) = v(k) + a(k+1) * T;
	IMUdata.vel   = filt_vel.data;
	IMUdata.pitch = p;
}