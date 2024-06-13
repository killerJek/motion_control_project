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
   * TODO: Create the PID class
   **/
    double Kp{.0};
    double Ki{.0};
    double Kd{.0};
    double output_lim_max{.0};
    double output_lim_min{.0};
    double output{.0};
    double noise{.0};
    double error{.0};
    double prev_error{0.0};
    double dt{0.};
    double integral{0.};
    /*
    * Errors
    */

    /*
    * Coefficients
    */

    /*
    * Output limits
    */
  
    /*
    * Delta time
    */

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


