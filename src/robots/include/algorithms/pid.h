#pragma once
#include <algorithm>

/**
 * @brief A Proportional-Integral-Derivative (PID) controller.
 */
class PIDController
{
public:
    /**
     * @brief Constructs a new PIDController object.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    PIDController(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd),
          error_sum_(0.0f), last_error_(0.0f),
          max_error_sum_(1.0f) // Default max integral sum
    {
    }

    /**
     * @brief Updates the PID parameters and resets the controller's internal state.
     * @param kp New proportional gain.
     * @param ki New integral gain.
     * @param kd New derivative gain.
     */
    void updateParameters(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        reset();
    }

    /**
     * @brief Computes the PID output based on the current error.
     * @param error The current error value (setpoint - process_variable).
     * @return float The control output.
     */
    float compute(float error)
    {
        // Accumulate the error for the integral term
        error_sum_ += error;
        // Clamp the integral sum to prevent integral windup
        error_sum_ = std::min(std::max(error_sum_, -max_error_sum_), max_error_sum_);
        // Calculate the derivative term (change in error)
        float d = error - last_error_;
        // Calculate the PID output
        float output = kp_ * error + ki_ * error_sum_ + kd_ * d;
        // Store the current error for the next derivative calculation
        last_error_ = error;
        return output;
    }

    /**
     * @brief Resets the internal state of the PID controller (error sum and last error).
     */
    void reset()
    {
        error_sum_ = 0.0f;
        last_error_ = 0.0f;
    }

    /**
     * @brief Sets the maximum allowed value for the integral error sum.
     * This helps to prevent integral windup.
     * @param max_error_sum The maximum absolute value for the error sum.
     */
    void setMaxErrorSum(float max_error_sum)
    {
        max_error_sum_ = max_error_sum;
    }

private:
    float kp_;            ///< Proportional gain.
    float ki_;            ///< Integral gain.
    float kd_;            ///< Derivative gain.
    float error_sum_;     ///< Sum of errors for the integral term.
    float last_error_;    ///< Previous error value for the derivative term.
    float max_error_sum_; ///< Maximum allowed value for the error sum (to prevent windup).
};