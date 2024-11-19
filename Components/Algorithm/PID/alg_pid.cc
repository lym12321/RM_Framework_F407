//
// Created by fish on 2024/9/8.
//

#include "alg_pid.h"

#define Limit(input, max) { input = input > max ? max : (input < -max ? -max : input); }

using namespace Algorithm;

void PID::clear() {
    err[0] = err[1] = err[2] = p_out = i_out = d_out = out = 0;
}

double PID::update(double current, double target) {
    err[2] = err[1], err[1] = err[0], err[0] = target - current;
    if(type_ == POSITION) {
        p_out =  Kp_ * err[0];
        i_out += Ki_ * err[0];
        d_out =  Kd_ * (err[0] - err[1]);
        Limit(i_out, iout_limit_);
        out = p_out + i_out + d_out;
        Limit(out, out_limit_);
    }
    if(type_ == DELTA) {
        p_out = Kp_ * (err[0] - err[1]);
        i_out = Ki_ * err[0];
        d_out = Kd_ * (err[0] - 2.0 * err[1] + err[2]);
        out += p_out + i_out + d_out;
        Limit(out, out_limit_);
    }
    return out;
}