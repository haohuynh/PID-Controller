/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {
	for (int i = 0; i < N; i++){
    	_pid_errors[i] = 0;  
    }
    
  	_delta_time = 1;
}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
    * Initialize PID coefficients (and errors, if needed)
    */
  
  _pid_coeffs[0] = Kpi;
  _pid_coeffs[1] = Kii;
  _pid_coeffs[2] = Kdi;
  _output_limits[0] = output_lim_mini;
  _output_limits[1] = output_lim_maxi;
}


void PID::UpdateError(double cte) {
   /**
    * Update PID errors based on cte.
    */
	_pid_errors[2] = (cte - _pid_errors[0]) / _delta_time;
    _pid_errors[1] += cte;
    _pid_errors[0] = cte;
}

double PID::TotalError() {
   /**
    * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
    */
    double control = 0.0;
  	
  	for (int i = 0; i < N; i++){
    	control -= _pid_coeffs[i] * _pid_errors[i];
    }
  	control = (control < _output_limits[0]) ? _output_limits[0] : control;
  	control = (control > _output_limits[1]) ? _output_limits[1] : control;
  
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
    * Update the delta time with new value
    */
	_delta_time = new_delta_time;
}