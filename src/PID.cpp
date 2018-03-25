#include "PID.h"
#include <iostream>

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
	phase = 0;

	// Initialize tuning variables
	dp[0] = 0.05;
	dp[1] = 0.05;
	dp[2] = 0.05;
	tol = 0.05;
	idx = 0;
	bestError = 10000;
	currentError = 0;
	N = 5000;
	tuneidx = 0;
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

	// For currentError, use MSE (Mean Squared Error) of CTE as a metric
	currentError += (cte * cte);
	// Get average
	if (idx == N) {
		currentError = currentError / N;
	}
	
}

double PID::TotalError() {

	// Let's just use this function to compute controller ouptut
	// Technically this is not an 'error' term

	double ctrl_out;

	ctrl_out = ((-p[0] * p_error) + (-p[1] * i_error) + (-p[2] * d_error));

	if (ctrl_out >= 1.0) {
		ctrl_out = 1.0;
		windup = 1.0;
	}
	else if (ctrl_out <= -1.0) {
		ctrl_out = -1.0;
		windup = 1.0;
	}
	return ctrl_out;
}

void PID::Reset()
{
	// Reset iteration variables and errors
	p_error = 0;
	i_error = 0;
	d_error = 0;
	idx = 0;
	currentError = 0;
}

void PID::Twiddle()
{
	// If del coeffs are less than tolerance, return without changing parameters
	if ((dp[0] + dp[1] + dp[2]) < tol) {
		tuned = 2;
		cout << "Twiddle completed..." << endl;
		return;
	}

	// Twiddle logic
	switch (phase)
	{
	case 0: {
		cout << "Twiddling up.." << endl;
		p[tuneidx] += dp[tuneidx];
		phase++;
	} break;

	case 1: {
		if (currentError < bestError) {
			bestError = currentError;
			dp[tuneidx] *= 1.1;
			phase = 0;
			tuneidx = (tuneidx + 1) % 3;
		}
		else {
			cout << "Twiddling down.." << endl;
			p[tuneidx] -= 2 * dp[tuneidx];
			phase++;
		}
	} break;

	case 2: {
		if (currentError < bestError) {
			bestError = currentError;
			dp[tuneidx] *= 1.1;
			phase = 0;
			tuneidx = (tuneidx + 1) % 3;
		}
		else {
			p[tuneidx] += 2 * dp[tuneidx];
			dp[tuneidx] *= 0.9;
			phase = 0;
			tuneidx = (tuneidx + 1) % 3;
		}
	} break;

	}

	return;
}