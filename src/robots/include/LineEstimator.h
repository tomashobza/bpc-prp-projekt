#pragma once

#include <stdint.h>

enum class DiscreteLinePose
{
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineEstimator
{
public:
    static float estimate_continuous_line_pose(float left_val, float right_val)
    {
        return (left_val - right_val);
    }

    static DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm)
    {
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
