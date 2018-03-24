#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	// Initialize P, I and D gains with input parameters.
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;

	// Initialize P, I and D errors to 0
	p_error = 0;
	i_error = 0;
	d_error = 0;

	// Initialize flags to 0
	windup = 0;
	tuned = 0;

	// Initialize tuning variables
	dp[0] = 0.5;
	dp[1] = 0.5;
	dp[2] = 0.5;
	tol = 0.1;
	idx = 0;
	bestError = 0;
	currentError = 0;
	N = 100;
}

void PID::UpdateError(double cte) {

	// Update error terms for P, I and D.

	// For D, it is the difference between the current CTE and the previous CTE (same as p_error)
	d_error = cte - p_error;

	// For P, it is the same as the current CTE
	p_error = cte;

	// For I, it is the sum of previous I error and the current CTE (accumulate)
	// Look for windup due to saturation and clamp I error accordingly
	if (windup) {

		i_error = i_error;
	}
	else {

		i_error = i_error + cte;
	}

	// For currentError, use square of CTE as a metric
	currentError += (cte * cte);
	
}

double PID::TotalError() {

	// This is what I call total error. May be use it to get out of twiddle earlier. 
	// By its own, not really useful to compute controller output.

	return (p_error + i_error + d_error);
}

void PID::ReinitErrors()
{
	p_error = 0;
	i_error = 0;
	d_error = 0;
}