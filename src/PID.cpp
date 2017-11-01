#include "PID.h"
#include <string>
#include <cmath>
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

  p_error = 0.1;
  i_error = 0.001;
  d_error = 0.1;

  last_idx = 0;
}

void PID::UpdateError(double cte) {
  double p[] = { Kp, Kd, Ki};
  std::vector<double> dp = { p_error, d_error, i_error };

  if (TotalError() > tol) {
    for (int i = 0; i < dp.size(), i++;) {
      best_error = cte;
      p[i] += dp[i];

      if (cte > best_error) {
        best_error = cte;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];

        if (cte < best_error) {
          best_error = cte;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
//  i_error += cte;
  p_error = dp[0];
  d_error = dp[1];
  i_error = dp[2];
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}
