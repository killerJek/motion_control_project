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
  prev_error = error;
  //error = pow(cte, 2);
  error = cte;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    // float derivative = MAXFLOAT;
    float derivative = (error-prev_error);
    // if(dt > 0.0000001){
    //   derivative =(error-prev_error)/dt;

    // }
    integral+= error;
    double total_error = -Kp*error-Kd*derivative-Ki*integral;
    if(total_error> output_lim_max){
      return output_lim_max;
    }else if (total_error < output_lim_min){
      return output_lim_min;
    }else {
      return total_error;
    }
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
  return dt;
}