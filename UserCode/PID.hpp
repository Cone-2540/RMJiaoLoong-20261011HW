#pragma once

#include <math.h>

class PID {
  public:
    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k) {
        setParameters(kp, ki, kd, i_max, out_max, d_filter_k);
        reset();
    }

    void setParameters(float kp, float ki, float kd, float i_max,
                       float out_max, float d_filter_k) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        i_max_ = fabsf(i_max);
        out_max_ = fabsf(out_max);
        d_filter_k_ = clamp(d_filter_k, 0.0f, 0.99f);
    }

    void reset() {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        previous_derivative_ = 0.0f;
        first_calc_ = true;
    }

    float calc(float ref, float fdb, float feedforward = 0.0f) {
        float error = ref - fdb;
        if (!isfinite(error) || !isfinite(feedforward)) {
            reset();
            return 0.0f;
        }
        float derivative = first_calc_ ? 0.0f : error - previous_error_;
        derivative = d_filter_k_ * previous_derivative_ +
                     (1.0f - d_filter_k_) * derivative;
        float next_integral = clamp(integral_ + error, -i_max_, i_max_);
        float output = kp_ * error + ki_ * next_integral + kd_ * derivative + feedforward;
        if (!((output > out_max_ && ki_ * error > 0.0f) ||
              (output < -out_max_ && ki_ * error < 0.0f))) {
            integral_ = next_integral;
        }
        previous_error_ = error;
        previous_derivative_ = derivative;
        first_calc_ = false;
        output = kp_ * error + ki_ * integral_ + kd_ * derivative + feedforward;
        return clamp(output, -out_max_, out_max_) - feedforward;
    }

  private:
    static float clamp(float value, float minimum, float maximum) {
        return value < minimum ? minimum : (value > maximum ? maximum : value);
    }

    float kp_, ki_, kd_;
    float i_max_, out_max_, d_filter_k_;
    float integral_;
    float previous_error_;
    float previous_derivative_;
    bool first_calc_;
};
