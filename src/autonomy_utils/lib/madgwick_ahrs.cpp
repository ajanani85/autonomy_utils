#include <autonomy_utils/lib/madgwick_ahrs.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace ros2
{
    MadgwickAHRS::MadgwickAHRS(float samplePeriod, float beta)
        : samplePeriod_(samplePeriod), beta_(beta)
    {
        // Initialize quaternion to identity
        q_.w = 1.0f;
        q_.x = 0.0f;
        q_.y = 0.0f;
        q_.z = 0.0f;
        gyro_bias_ = {0.0f, 0.0f, 0.0f};
        accel_outlier_threshold_ = 2.0f * 9.81f; // 2g default
        min_accel_norm_ = 0.5f * 9.81f;          // 0.5g default
        accel_offset_ = {0.0f, 0.0f, 0.0f};
    }

    MadgwickAHRS::~MadgwickAHRS()
    {
    }

    // Set sample period dynamically (in seconds)
    void MadgwickAHRS::setSamplePeriod(float samplePeriod)
    {
        samplePeriod_ = samplePeriod;
    }

    // Set beta dynamically
    void MadgwickAHRS::setBeta(float beta)
    {
        beta_ = beta;
    }

    // Set gyroscope bias (rad/s)
    void MadgwickAHRS::setGyroBias(float bx, float by, float bz)
    {
        gyro_bias_ = {bx, by, bz};
    }

    // Optionally estimate gyro bias online (call when stationary)
    void MadgwickAHRS::calibrateGyroBias(float gx, float gy, float gz, float alpha)
    {
        gyro_bias_[0] = (1 - alpha) * gyro_bias_[0] + alpha * gx;
        gyro_bias_[1] = (1 - alpha) * gyro_bias_[1] + alpha * gy;
        gyro_bias_[2] = (1 - alpha) * gyro_bias_[2] + alpha * gz;
    }

    // Set accelerometer outlier threshold (m/s^2)
    void MadgwickAHRS::setAccelOutlierThreshold(float threshold)
    {
        accel_outlier_threshold_ = threshold;
    }
    void MadgwickAHRS::setMinAccelNorm(float min_norm)
    {
        min_accel_norm_ = min_norm;
    }

    // Set accelerometer offset (m/s^2)
    void MadgwickAHRS::setAccelOffset(float ax, float ay, float az)
    {
        accel_offset_[0] = ax;
        accel_offset_[1] = ay;
        accel_offset_[2] = az;
    }

    // Update with gyroscope (rad/s) and accelerometer (m/s^2)
    void MadgwickAHRS::update(float gx, float gy, float gz, float ax, float ay, float az)
    {
        // Remove gyro bias
        gx -= gyro_bias_[0];
        gy -= gyro_bias_[1];
        gz -= gyro_bias_[2];

        // Remove accelerometer offset
        // ax -= accel_offset_[0];
        // ay -= accel_offset_[1];
        // az -= accel_offset_[2];

        float q1 = q_.w, q2 = q_.x, q3 = q_.y, q4 = q_.z;
        float norm;
        float s1, s2, s3, s4;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q4 = 2.0f * q4;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _4q3 = 4.0f * q3;
        float _8q2 = 8.0f * q2;
        float _8q3 = 8.0f * q3;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        float q4q4 = q4 * q4;

        // Outlier rejection for accelerometer
        norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (norm < min_accel_norm_ || norm > accel_outlier_threshold_)
            return; // reject outlier or free-fall
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Gradient descent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
        norm = std::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        if (norm == 0.0f)
            return;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
        s4 /= norm;

        // Rate of change of quaternion
        float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
        float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
        float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
        float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * samplePeriod_;
        q2 += qDot2 * samplePeriod_;
        q3 += qDot3 * samplePeriod_;
        q4 += qDot4 * samplePeriod_;
        norm = std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        q_.w = q1 / norm;
        q_.x = q2 / norm;
        q_.y = q3 / norm;
        q_.z = q4 / norm;
    }

    // Get current orientation as geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion MadgwickAHRS::getQuaternion() const
    {
        return q_;
    }

    // Get roll, pitch, yaw (in radians)
    std::array<float, 3> MadgwickAHRS::getRPY() const
    {
        float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;
        float roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        float pitch = std::asin(2.0f * (q0 * q2 - q3 * q1));
        float yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
        return {roll, pitch, yaw};
    }

    // Get current gyro bias
    std::array<float, 3> MadgwickAHRS::getGyroBias() const
    {
        return gyro_bias_;
    }

    // Get orientation quaternion with yaw set to zero
    geometry_msgs::msg::Quaternion MadgwickAHRS::getQuaternionNoYaw() const
    {
        // Remove yaw from the quaternion without converting to RPY
        // q = [w, x, y, z] (w, x, y, z)
        // Extract yaw from current quaternion
        float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;
        float yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
        float half_neg_yaw = -yaw / 2.0f;
        float cy = std::cos(half_neg_yaw);
        float sy = std::sin(half_neg_yaw);
        // Quaternion for negative yaw rotation (roll=0, pitch=0, yaw=-yaw)
        geometry_msgs::msg::Quaternion q_neg_yaw;
        q_neg_yaw.w = cy;
        q_neg_yaw.x = 0.0f;
        q_neg_yaw.y = 0.0f;
        q_neg_yaw.z = sy;
        // Quaternion multiplication: q_no_yaw = q * q_neg_yaw
        geometry_msgs::msg::Quaternion q_no_yaw;
        q_no_yaw.w = q0 * q_neg_yaw.w - q1 * q_neg_yaw.x - q2 * q_neg_yaw.y - q3 * q_neg_yaw.z;
        q_no_yaw.x = q0 * q_neg_yaw.x + q1 * q_neg_yaw.w + q2 * q_neg_yaw.z - q3 * q_neg_yaw.y;
        q_no_yaw.y = q0 * q_neg_yaw.y - q1 * q_neg_yaw.z + q2 * q_neg_yaw.w + q3 * q_neg_yaw.x;
        q_no_yaw.z = q0 * q_neg_yaw.z + q1 * q_neg_yaw.y - q2 * q_neg_yaw.x + q3 * q_neg_yaw.w;
        // Normalize result
        float norm = std::sqrt(q_no_yaw.w * q_no_yaw.w + q_no_yaw.x * q_no_yaw.x + q_no_yaw.y * q_no_yaw.y + q_no_yaw.z * q_no_yaw.z);
        q_no_yaw.w /= norm;
        q_no_yaw.x /= norm;
        q_no_yaw.y /= norm;
        q_no_yaw.z /= norm;
        return q_no_yaw;
    }
} // namespace ros2