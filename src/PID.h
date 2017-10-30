#pragma once

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double previous_error;
  double total_square_error;
  double n;
  double steer;
  double errorSum;
  long counter;
  double minError;
  double maxError;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  *  Returns the average error.
  */
  double AverageError();

  /*
  * Returns the min error.
  */
  double MinError();

  /*
  * Returns the max error.
  */
  double MaxError();
};
