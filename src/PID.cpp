#include "PID.h"
#include <iostream>
#include <math.h>
#include <tuple>
using namespace std;

#define DEBUG 0
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double dp_p, double dp_i, double dp_d) {
	this ->Kp = Kp;
	this ->Kd = Kd;
	this ->Ki = Ki;
	this ->dp_p = dp_p;
	this ->dp_i = dp_i;
	this ->dp_d = dp_d;

	if (DEBUG)
		printf("\n (Kp, Ki, Kd): (%.06f, %.06f, %.06f)", Kp, Ki, Kd);

	if (DEBUG)
		printf("\n (dp_p, dp_i, dp_d): (%.06f, %.06f, %.06f)", dp_p, dp_i, dp_d);
}

void PID::UpdateError(double cte) {
	// Update the errors{
	d_error = 0;
	if (p_error != 9999.) d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	// increment update counter
	update_count++;
	// Create err value for twiddle
	if (update_count > 100)
		total_error += (cte * cte);
	// print for debugging
	if (DEBUG)
		printf("\n Current Values of errors are P_Error : %.02f I_Error: %.02f D_Error: %.02f", p_error, i_error, d_error);
}

double PID::output() {
	// Calculate output
	double output_val_ = -(Kp * p_error + Kd * d_error + Ki * i_error);
	// Limit between zero and one
	output_val_ = min(output_val_, 1.0);
	output_val_ = max(output_val_, -1.0);
	// print for debugging
	if (DEBUG)
		printf("\n For count %d, Output is %.03f", update_count, output_val_);
	return output_val_;
}

void PID::twiddle() {
	double p[] = { Kp, Ki, Kd };
	double dp[] = { dp_p, dp_i, dp_d };
	double current_error_ = total_error / (update_count - 100);
	print_vars = false;
	update_count = 0;
	total_error = 0.;
	i_error = 0;
	if (DEBUG) {
		cout << "\nIteration: " << iteration << " Current State: " << current_state << " Coefficient: " << coefficient_choice_;
		printf("\nCurrent Error: %.03f, Best Error: %.03f", current_error_, best_error);
	}
	if (current_state == 0) {
		best_error = current_error_;
		current_state = 1;
		return;
	}
	if (p[coefficient_choice_] <= 0.) {
		p[coefficient_choice_] = 0.;
		coefficient_choice_ = (coefficient_choice_ + 1) % 3;
	}
	switch (current_state) {
		case 1: {
			iteration++;
			p[coefficient_choice_] += dp[coefficient_choice_];
			if (current_error_ < best_error) {
				best_error = current_error_;
				dp[coefficient_choice_] *= 1.1;
				coefficient_choice_ = (coefficient_choice_ + 1) % 3;
				print_vars = true;
				if (DEBUG)
					printf("\n New Best error, iteration %d: %.06f", iteration, best_error);
			}
			else current_state = 2;
			if (DEBUG) {
				printf("\n Best Param Set (Kp, Ki, Kd): (%.06f, %.06f, %.06f)", Kp, Ki, Kd);
				printf("\n Sum_dp: %.03f", dp_p + dp_d + dp_i);
			}
			break;
		}
		case 2: {
			p[coefficient_choice_] -= 2 * dp[coefficient_choice_];
			if (p[coefficient_choice_] < 0.) p[coefficient_choice_] = 0.;
			if (current_error_ < best_error) {
				best_error = current_error_;
				dp[coefficient_choice_] *= 1.1;
				if (DEBUG)
					printf("\n New Best error, iteration %d: %.06f", iteration, best_error);
			}
			else {
				p[coefficient_choice_] += dp[coefficient_choice_];
				dp[coefficient_choice_] *= .9;
			}
			coefficient_choice_ = (coefficient_choice_ + 1) % 3;
			current_state = 1;
			print_vars = true;
			if (DEBUG) {
				printf("\n Best Param Set (Kp, Ki, Kd): (%.06f, %.06f, %.06f)", Kp, Ki, Kd);
				printf("\n Sum_dp: %.03f", dp_p + dp_d + dp_i);
			}
			break;
		}
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
	dp_p = dp[0];
	dp_i = dp[1];
	dp_d = dp[2];
}

