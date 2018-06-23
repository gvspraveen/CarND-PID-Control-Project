#include "PID.h"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  total_error = 0;
  best_error = numeric_limits<double>::max();
  num_steps = 0;
  next_twiddle_param_index = 0;
  twiddle_steps_interval = 200;
}

void PID::UpdateError(double cte) {
  if (num_steps == 0) {
      d_error = 0;
  } else {
      d_error = cte - p_error;
  }
  p_error = cte;
  i_error += cte;

  num_steps += 1;
}

double PID::TotalError() {
  return -1.0 * (
      (Kp * p_error) + (Ki * i_error) + (Kd * d_error)
  );
}

