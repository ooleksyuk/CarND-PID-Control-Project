#include "PID.h"
#include <string>
using namespace std;

/*
* TODO: Complete the PID class.
*/
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;

  previous_error = 0.0;
  total_square_error = 0.0;
  n = 0.0;
  steer = 0.0;
  errorSum = 0.0;
  counter = 0;
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::min();

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = cte - p_error;
  previous_error = cte;

  errorSum += cte;
  counter++;

  if ( cte > maxError ) {
    maxError = cte;
  }
  if ( cte < minError ) {
    minError = cte;
  }
}

double PID::TotalError() {
  p_error * Kp + i_error * Ki + d_error * Kd;
}

double PID::AverageError() {
  return errorSum/counter;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}
