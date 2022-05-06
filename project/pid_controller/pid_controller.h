/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * Create the PID class
   **/

  	/*
    * The number of Errors/Coefficients
    */	
  	const static int N = 3;
  
    /*
    * Errors
    */
	double _pid_errors[N];
  
    /*
    * Coefficients
    */
	double _pid_coeffs[N];
  
    /*
    * Output limits
    */
  	double _output_limits[2];
  
    /*
    * Delta time
    */
	double _delta_time;
  
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


