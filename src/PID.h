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
  double dp[3]; //del coefficients
  double tol; //tolerance
  double currentError; //current iteration error
  double bestError; //best error recorded
  int idx; // sample index
  int N; // total samples to process for an iteration
  int tuneidx; // coefficient index

  /*
  * Flags
  */
  bool windup; //tracks integrator windup
  int tuned; //tracks tuned status 0 is uninitialized, 1 is initialized and tuning, 2 is tuned.
  int phase; //to help with switch case in twiddle

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
  * Reset values for twiddle iteration.
  */
  void Reset();

  /*
  * Perform parameter tuning for controller coefficients.
  */
  void Twiddle();

};

#endif /* PID_H */
