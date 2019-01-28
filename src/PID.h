#ifndef PID_H
#define PID_H

class PID {
public:

	// Coefficients and deltas
  double Kp, Ki, Kd;
	double dp_p, dp_i, dp_d;

	// Errors
	double p_error = 9999.;
	double i_error = 0.;
	double d_error = 0.;

	// other variables
	int update_count = 0;
	double total_error = 0.;
	int iteration = 0;
	int current_state = 0;
	int coefficient_choice_ = 0;
	double best_error;

	// twiddle variable initializations
	int count_threshold = 500;
	bool print_vars = false;

	// Constructor
	PID();

	//Destructor.
	virtual ~PID();

	// Initialize PID.
	void Init(double Kp, double Ki, double Kd, double dp_p, double dp_i, double dp_d);

	// Update the PID error variables given cross track error.
	void UpdateError(double cte);

	// Twiddle method to tune PID coefficients
	void twiddle();

	// Output Value
	double output();
};

#endif /* PID_H */