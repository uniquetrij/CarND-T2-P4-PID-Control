#ifndef PID_H
#define PID_H

class PID {
public:

    double sum_cte;
    double pre_cte;

    double steer;
    double throttle;

    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;

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
    //
    //    /*
    //     * Calculate the total PID error.
    //     */
    //    double TotalError();
};

double computeSteer(double tau_p, double tau_i, double tau_d, double cte, double sum_cte, double pre_cte);
double computeThrottle(double tau_p, double tau_i, double tau_d, double cte, double sum_cte, double pre_cte);

#endif /* PID_H */
