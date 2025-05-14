#pragma once

#include <stdint.h>

/**
 * @brief Enum representing the discrete pose of the line relative to the robot.
 */
enum class DiscreteLinePose
{
    LineOnLeft,  ///< Line is detected on the left side.
    LineOnRight, ///< Line is detected on the right side.
    LineNone,    ///< No line is detected.
    LineBoth,    ///< Line is detected on both sides (e.g., at a crossing).
};

/**
 * @brief Class for estimating the line pose based on sensor readings.
 */
class LineEstimator
{
public:
    /**
     * @brief Estimates the continuous line pose.
     * A positive value indicates the line is to the right, a negative value to the left.
     * The magnitude indicates the distance from the center.
     * @param left_val Normalized reading from the left line sensor.
     * @param right_val Normalized reading from the right line sensor.
     * @return float Continuous line pose estimation.
     */
    static float estimate_continuous_line_pose(float left_val, float right_val)
    {
        // The difference between right and left sensor values gives a continuous measure.
        // If right_val > left_val, the result is positive (line to the right).
        // If left_val > right_val, the result is negative (line to the left).
        // If left_val == right_val, the result is zero (centered or no line).
        return (right_val - left_val);
    }

    /**
     * @brief Estimates the discrete line pose based on normalized sensor readings.
     * @param l_norm Normalized reading from the left line sensor.
     * @param r_norm Normalized reading from the right line sensor.
     * @return DiscreteLinePose The estimated discrete pose of the line.
     */
    static DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm)
    {
        // Threshold to determine if a sensor detects a line.
        static constexpr float kThreshold = 0.5f;

        bool left_detected = l_norm > kThreshold;
        bool right_detected = r_norm > kThreshold;

        if (left_detected && right_detected)
        {
            return DiscreteLinePose::LineBoth;
        }
        else if (left_detected)
        {
            return DiscreteLinePose::LineOnLeft;
        }
        else if (right_detected)
        {
            return DiscreteLinePose::LineOnRight;
        }
        else
        {
            return DiscreteLinePose::LineNone;
        }
    }
};
