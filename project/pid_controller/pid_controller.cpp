/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->Kp = Kpi;
  this->Ki = Kii;
  this->Kd = Kdi;
  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;

}

void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  error = cte;
  derivative = (dt > 0) ? (cte - prev_error) / dt : 0.0;
  prev_error = error;

  integral+= error*dt;


}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double total_error = Kp*error+Kd*derivative+Ki*integral;
    return max(min(total_error, this->output_lim_max), this->output_lim_min);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
  return dt;
}