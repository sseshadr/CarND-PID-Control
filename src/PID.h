#ifndef PID_H
#define PID_H

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
  double p[3];

  /*
  * Tuning variables
  */
  double dp[3];
  double tol;
  double currentError;
  double bestError;
  int idx;
  int N;

  /*
  * Flags
  */
  bool windup;
  int tuned;

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
  * Update errors to zero for twiddle.
  */
  void ReinitErrors();
};

#endif /* PID_H */
