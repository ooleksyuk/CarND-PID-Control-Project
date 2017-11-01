#include "PID.h"
#include <cmath>
using namespace std;

/*
* Complete the PID class.
*/
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;

  p_error = 0.1;
  i_error = 0.001;
  d_error = 0.1;

  best_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  double p[] = { Kp, Kd, Ki };
  double dp[] = { p_error, d_error, i_error };

  if (TotalError() > tol) {
    switch (current_state) {
      case 0: {
        p[idx] += dp[idx];
        current_state = 1;
        break;
      }
      case 1: {
        if (fabs(cte) < fabs(best_error)) {
          best_error = cte;
          dp[idx] *= 1.1;
          current_state = 3;
        } else {
          p[idx] -= 2 * dp[idx];
          current_state = 2;
        }
        break;
      }
      case 2: {
        if (fabs(cte) < fabs(best_error)) {
          best_error = cte;
          dp[idx] *= 1.1;
        } else {
          p[idx] += dp[idx];
          dp[idx] *= 0.9;
        }
        current_state = 3;
        break;
      }
      case 3: {
        idx = (idx + 1) % 2;
        current_state = 0;
        break;
      }
    }
    p_error = dp[0];
    d_error = dp[1];
    i_error = dp[2];

    Kp = p[0];
    Kd = p[1];
    Ki = p[2];
  }
}

double PID::TotalError() {
  return fabs(p_error) + fabs(d_error) + fabs(d_error);
}
