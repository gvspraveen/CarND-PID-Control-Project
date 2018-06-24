#ifndef PID_H
#define PID_H

#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  vector<double> deltas;


  int num_steps;
  int next_twiddle_param_index;
  bool twiddle_param_added;
  bool twiddle_param_subtracted;
  double total_error;
  double best_error;
  bool twiddle_initialized;
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /**
   * Run twiddle periodically.
   * Returns true if twiddle was run. false if pre checks for running twiddle fail.
   */
  bool runTwiddle(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
