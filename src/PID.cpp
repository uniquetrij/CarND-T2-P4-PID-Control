#include <stddef.h>

#include "PID.h"
#include <math.h>
#include <cmath>
using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
    pre_cte = NULL;
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
}

void PID::UpdateError(double cte) {
    if (pre_cte == NULL) {
        pre_cte = cte;
    }
    sum_cte += cte;
    steer = computeSteer(Kp, Ki, Kd, cte, sum_cte, pre_cte);
    throttle = computeThrottle(0.1, 0.05, 2, cte, sum_cte, pre_cte);
    pre_cte = cte;
}
//
//double PID::TotalError() {
//}

double computeSteer(double tau_p, double tau_i, double tau_d, double cte, double sum_cte, double pre_cte) {
    return -tau_p * cte - tau_d * (cte - pre_cte) - tau_i * sum_cte;
}

double computeThrottle(double tau_p, double tau_i, double tau_d, double cte, double sum_cte, double pre_cte) {
    cte = abs(cte);
    return 0.3 -tau_p * cte - tau_d * (cte - abs(pre_cte));
}





