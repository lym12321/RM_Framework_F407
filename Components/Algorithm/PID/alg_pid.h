//
// Created by fish on 2024/9/8.
//

#pragma once

/*
 * Algorithm - PID
 * Usage:
 *    auto pid = Algorithm::PID(type, Kp, Ki, Kd, out_limit, iout_limit);
 *    auto out = pid.update(current, target);
 */

namespace Algorithm {
    class PID {
      public:
        typedef enum {
            POSITION,
            DELTA
        } pid_type_e;
        typedef struct {
            double Kp, Ki, Kd, out_limit, iout_limit;
        } pid_param_t;
        PID() : Kp_(0), Ki_(0), Kd_(0), out_limit_(0), iout_limit_(0), type_(POSITION) {}
        explicit PID(pid_type_e e) : Kp_(0), Ki_(0), Kd_(0), out_limit_(0), iout_limit_(0), type_(e) {}
        PID(pid_type_e e, double Kp, double Ki, double Kd, double out_limit, double iout_limit) :
            Kp_(Kp), Ki_(Ki), Kd_(Kd), out_limit_(out_limit), iout_limit_(iout_limit), type_(e) {}
        void clear();
        double update(double current, double target);
        void set_para(double Kp, double Ki, double Kd) { Kp_ = Kp; Ki_ = Ki; Kd_ = Kd; this->clear(); }
        void set_para(double Kp, double Ki, double Kd, double out_limit, double iout_limit) {
            Kp_ = Kp, Ki_ = Ki, Kd_ = Kd, out_limit_ = out_limit, iout_limit_ = iout_limit; this->clear();
        }
        void set_para(const pid_param_t &param) {
            Kp_ = param.Kp, Ki_ = param.Ki, Kd_ = param.Kd, out_limit_ = param.out_limit, iout_limit_ = param.iout_limit; this->clear();
        }
      private:
        pid_type_e type_;
        double Kp_, Ki_, Kd_, out_limit_, iout_limit_;
        double p_out = 0, i_out = 0, d_out = 0, out = 0, err[3] = { 0 };
    };
}