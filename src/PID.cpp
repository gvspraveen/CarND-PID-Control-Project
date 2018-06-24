#include "PID.h"
#include <cmath>
#include <iostream>

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
  deltas = {0.05*Kp, 0.05*Kd, 0.1*Ki};

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  num_steps = 0;

  twiddle_initialized = false;
  total_error = 0;
  best_error = numeric_limits<double>::max();
  next_twiddle_param_index = 0;
  twiddle_param_added = false;
  twiddle_param_subtracted = false;
}

void PID::UpdateError(double cte) {
  if (num_steps == 0) {
      d_error = 0;
  } else {
      d_error = cte - p_error;
  }
  p_error = cte;
  i_error += cte;
  // If twiddle was run then reset num_steps and total_error
//  bool is_twiddle_run = false;
//  is_twiddle_run = runTwiddle(cte);
//  if (is_twiddle_run) {
//    num_steps = 0;
//    total_error = 0.0;
//  } else {
//    num_steps += 1;
//  }

    total_error += pow(cte, 2);
    num_steps += 1;
}

bool PID::runTwiddle(double cte) {
  // If this first time running twiddle, we dont have a best error yet. So just set best error as total error.
  if (!twiddle_initialized) {
    std::cout << "==================== Initializing twiddle =========================\n";
    twiddle_initialized = true;
    best_error = total_error;
    return true;
  }

  if (total_error < best_error) {
    std::cout << "==================== Found best error =========================\n";
    deltas[next_twiddle_param_index] *= 1.1;
    twiddle_param_added = false;
    twiddle_param_subtracted = false;
    next_twiddle_param_index = (next_twiddle_param_index + 1) % 3;
    best_error = total_error;
    return true;
  }

  double to_add = 0.0;
  int index_to_add = next_twiddle_param_index;

  if (!twiddle_param_added) {
    std::cout << "==================== Trying adding to param: " <<  next_twiddle_param_index << " =========================\n";

    to_add = deltas[next_twiddle_param_index];
    twiddle_param_added = true;
  } else if (!twiddle_param_subtracted) {
      std::cout << "==================== Trying subtracting from param: " <<  next_twiddle_param_index << " =========================\n";

      to_add = -2 * deltas[next_twiddle_param_index];
      twiddle_param_subtracted = true;
  } else {
      std::cout << "==================== Resetting param: " <<  next_twiddle_param_index << " =========================\n";

      to_add = deltas[next_twiddle_param_index];
      // Need to reset param and move onto next param
      deltas[next_twiddle_param_index] *= 0.9;
      twiddle_param_added = false;
      twiddle_param_subtracted = false;
      next_twiddle_param_index = (next_twiddle_param_index + 1) % 3;
  }

  if (index_to_add == 0) {
    Kp += to_add;
  }
  else if (index_to_add == 1) {
    Kd += to_add;
  }
  else {
    Ki += to_add;
  }

  return true;
}

double PID::TotalError() {
  return -1.0 * (
      (Kp * p_error) + (Ki * i_error) + (Kd * d_error)
  );
}

