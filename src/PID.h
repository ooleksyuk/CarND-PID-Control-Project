#pragma once
#include <limits>

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

  /*
   * Twiddle params:
   */
  const double tol = std::numeric_limits<double>::min();
  double best_error = std::numeric_limits<double>::max();
  int idx = 0;
  int state = 0; // possible values 0, 1,2, 3
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

  /*
   * Twiddle cte param to get the optimal value.
   */
  void Twiddle(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};
