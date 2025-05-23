#include <autonomy_utils/lib/ros2_core.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <autonomy_utils/lib/rotate.h>

std::shared_ptr<ros2::LifeCycle> ros2::Node::node = nullptr;

class ImuOrientationConverterClass
{
public:
    ImuOrientationConverterClass()
    {
        // Constructor implementation

        if(ros2::Node::node == nullptr)
        {
            throw std::runtime_error("Node is not initialized");
            return;
        }

        ros2::Node::node->register_on_configure_callback(std::bind(&ImuOrientationConverterClass::onConfigure, this));
        
        ros2::Node::node->register_on_activate_callback(std::bind(&ImuOrientationConverterClass::onActivate, this));
    }

    ~ImuOrientationConverterClass()
    {
        // Destructor implementation
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr orientation_pub_;

    void getParams()
    {
        // Get parameters from the parameter server
    }

    void onActivate()
    {

    }

    void onConfigure()
    {
        getParams();
        // Create a subscription to the IMU topic
        imu_sub_ = ros2::Node::node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ImuOrientationConverterClass::imuCB, this, std::placeholders::_1));

        // Create a publisher for the orientation topic
        orientation_pub_ = ros2::Node::node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/imu/orientation", 10);
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(orientation_pub_->get_subscription_count() == 0)
        {
            return;
        }   

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        ros2::Rotate::getRPY(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, roll, pitch, yaw);
        geometry_msgs::msg::Vector3Stamped orientation_msg;
        orientation_msg.header.stamp = msg->header.stamp;
        orientation_msg.vector.x = ros2::Rotate::toDegree(roll);
        orientation_msg.vector.y = ros2::Rotate::toDegree(pitch);
        orientation_msg.vector.z = ros2::Rotate::toDegree(yaw);
        orientation_pub_->publish(orientation_msg);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    ros2::Node::node = std::make_shared<ros2::LifeCycle>("imu_orientation_converter_node");

    std::shared_ptr<ImuOrientationConverterClass> node = std::make_shared<ImuOrientationConverterClass>();

    rclcpp::spin(ros2::Node::node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}