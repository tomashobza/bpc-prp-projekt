#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

namespace algorithms
{

    class PlanarImuIntegrator
    {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.01538f) {}

        // Integrate gyro_z over time to estimate orientation
        void update(float gyro_z, double dt)
        {
            // Subtract the calibrated offset and integrate
            theta_ += (gyro_z - gyro_offset_) * dt;

            // Normalize theta to [-π, π]
            theta_ = std::fmod(theta_, 2.0f * M_PI);
            if (theta_ > M_PI)
            {
                theta_ -= 2.0f * M_PI;
            }
            else if (theta_ < -M_PI)
            {
                theta_ += 2.0f * M_PI;
            }
        }

        // Compute average offset from static samples for calibration
        void setCalibration(std::vector<float> gyro)
        {
            if (gyro.empty())
                return;

            gyro_offset_ = std::accumulate(gyro.begin(), gyro.end(), 0.0f) /
                           static_cast<float>(gyro.size());
        }

        // Return current yaw angle in radians
        [[nodiscard]] float getYaw() const
        {
            return theta_;
        }

        // Reset the integrator state
        void reset()
        {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

        float get_offset() {
            return gyro_offset_;
        }

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}
