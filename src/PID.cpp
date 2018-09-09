#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    acc_err = 0.0;
    /* best_err = 435.3;  counter = 2000; */
    //counter = 0;
    //bFlag = 0;
    //bFirstStep = 0;
}

void PID::UpdateError(double cte) {
    /*
     * Update the PID error variables given cross track error.
     */
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    /*
     * Calculate the total PID error.
     */
    double err = 0.0;
    err = - Kp * p_error - Kd * d_error - Ki * i_error;
    
    if(err < -1)
    {
        err = 1;
    }
    if(err > 1)
    {
        err = 1;
    }
    return err;
}

