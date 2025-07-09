#ifndef AUTONOMY_UTILS_LIB_MADGWICK_AHRS_H_
#define AUTONOMY_UTILS_LIB_MADGWICK_AHRS_H_

#include <array>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/quaternion.hpp>

namespace ros2
{
    /// @brief Madgwick AHRS (Attitude and Heading Reference System) algorithm implementation
    class MadgwickAHRS
    {
    public:
        /**
         * @brief Constructor for MadgwickAHRS
         * @param samplePeriod Sample period in seconds
         * @param beta Tuning parameter for algorithm (default 0.1)
         */
        MadgwickAHRS(float samplePeriod, float beta = 0.1f);

        /**
         * @brief Destructor for MadgwickAHRS
         */
        ~MadgwickAHRS();

        /**
         * @brief Set sample period dynamically (in seconds)
         * @param samplePeriod New sample period in seconds
         */
        void setSamplePeriod(float samplePeriod);

        /**
         * @brief Set beta dynamically
         * @param beta New beta value
         */
        void setBeta(float beta);

        /**
         * @brief Set gyroscope bias (rad/s)
         * @param bx Gyroscope bias around x-axis
         * @param by Gyroscope bias around y-axis
         * @param bz Gyroscope bias around z-axis
         */
        void setGyroBias(float bx, float by, float bz);

        /**
         * @brief Optionally estimate gyro bias online (call when stationary)
         * @param gx Gyroscope x-axis reading (rad/s)
         * @param gy Gyroscope y-axis reading (rad/s)
         * @param gz Gyroscope z-axis reading (rad/s)
         * @param alpha Low-pass filter coefficient (default 0.01)
         */
        void calibrateGyroBias(float gx, float gy, float gz, float alpha = 0.01f);

        /**
         * @brief Set accelerometer outlier threshold (m/s^2)
         * @param threshold Threshold value for accelerometer outliers
         * @param min_norm Minimum norm for accelerometer to consider valid (default 0.5 * 9.81)
         */
        void setAccelOutlierThreshold(float threshold);

        /**
         * @brief Set minimum norm for accelerometer to consider valid
         * @param min_norm Minimum norm value (default 0.5 * 9.81)
         */
        void setMinAccelNorm(float min_norm);

        /**
         * @brief Set accelerometer offset (m/s^2)
         * @param ax Accelerometer offset x
         * @param ay Accelerometer offset y
         * @param az Accelerometer offset z
         */
        void setAccelOffset(float ax, float ay, float az);

        /**
         * @brief Update orientation with gyroscope (rad/s) and accelerometer (m/s^2)
         * @param gx Gyroscope x-axis reading (rad/s)
         * @param gy Gyroscope y-axis reading (rad/s)
         * @param gz Gyroscope z-axis reading (rad/s)
         * @param ax Accelerometer x-axis reading (m/s^2)
         * @param ay Accelerometer y-axis reading (m/s^2)
         * @param az Accelerometer z-axis reading (m/s^2)
         */
        void update(float gx, float gy, float gz, float ax, float ay, float az);

        /**
         * @brief Get current orientation as geometry_msgs::msg::Quaternion
         * @return Quaternion message (x, y, z, w)
         */
        geometry_msgs::msg::Quaternion getQuaternion() const;

        /**
         * @brief Get current orientation as geometry_msgs::msg::Quaternion with yaw set to zero
         * @return Quaternion message (x, y, z, w) with yaw = 0
         */
        geometry_msgs::msg::Quaternion getQuaternionNoYaw() const;

        /**
         * @brief Get roll, pitch, yaw (in radians)
         * @return Roll, pitch, yaw as an array {roll, pitch, yaw}
         */
        std::array<float, 3> getRPY() const;

        /**
         * @brief Get current gyro bias
         * @return Gyro bias as an array {bx, by, bz}
         */
        std::array<float, 3> getGyroBias() const;

    private:
        float samplePeriod_;
        
        float beta_;
        
        // Internal quaternion storage: (w, x, y, z)
        geometry_msgs::msg::Quaternion q_; 
        
        std::array<float, 3> gyro_bias_;
        
        float accel_outlier_threshold_;

        float min_accel_norm_;

        std::array<float, 3> accel_offset_ = {0.0f, 0.0f, 0.0f};
    };
}

#endif // AUTONOMY_UTILS_LIB_MADGWICK_AHRS_H_
