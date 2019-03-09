#include "PID.h"
#include <cmath>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;

   p_error = 0;
   i_error = 0;
   d_error = 0;

   dpp = 0.01;
   dpi = 0.0001;
   dpd = 0.1;

   current_id = 0;
   current_sequence = 0;

   best_err_initialized = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return  -Kp * p_error - Ki * i_error - Kd * d_error;
}

double * PID::getKbyId(const int& id) {
    switch (id) {
        case 0:
            return &Kp;
        case 1:
            return &Ki;
        case 2:
            return &Kd;
        default:
            throw ("Id not available");
    }
}

double * PID::getDpbyId(const int& id) {
    switch (id) {
        case 0:
            return &dpp;
        case 1:
            return &dpi;
        case 2:
            return &dpd;
        default:
            throw ("Id not available");
    }
}

void PID::twiddle(double err) {
    std::cout << Kp << "," << Ki << "," << Kd << std::endl;
    std::cout << dpp << "," << dpi << "," << dpd << std::endl;
    /*if(std::abs(dpp)+std::abs(dpi)+std::abs(dpd) < 0.1)
        return;*/

    double* K = getKbyId(current_id);
    double* dp = getDpbyId(current_id);

    switch (current_sequence) {
        case 0:
            (*K) += (*dp);
            current_sequence++;
            break;
        case 1:
            if (err < best_err) {
                best_err = err;
                (*dp) *= 1.1;

                // reset
                current_id = (current_id >= 2) ? 0 : (current_id+1);
                current_sequence = 0;
            }

            else {
                (*K) -= 2 * (*dp);
                current_sequence++;
            }
            break;
        case 2:
            if (err < best_err) {
                best_err = err;
                (*dp) *= 1.1;
            }
            else {
                (*K) += (*dp);
                (*dp) *= 0.9;
            }

            // reset
            current_id = (current_id >= 2) ? 0 : (current_id+1);
            current_sequence = 0;
            break;
        default:
            throw ("Sequence must be reset");
    }
}