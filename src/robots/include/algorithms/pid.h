#pragma once
#include <algorithm>

class PIDController
{
public:
    PIDController(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd),
          error_sum_(0.0f), last_error_(0.0f),
          max_error_sum_(1.0f)
    {}

    // Compute the PID output given the current error.
    float compute(float error)
    {
        error_sum_ += error;
        // Anti-windup: Clamp the accumulated error.
        error_sum_ = std::min(std::max(error_sum_, -max_error_sum_), max_error_sum_);
        float d = error - last_error_;
        float output = kp_ * error + ki_ * error_sum_ + kd_ * d;
        last_error_ = error;
        return output;
    }

    // Reset internal PID state.
    void reset()
    {
        error_sum_ = 0.0f;
        last_error_ = 0.0f;
    }

    // Set the maximum accumulated error (for anti-windup purposes).
    void setMaxErrorSum(float max_error_sum)
    {
        max_error_sum_ = max_error_sum;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float error_sum_;
    float last_error_;
    float max_error_sum_;
};
