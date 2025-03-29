#include <autonomy_utils/lib/keyboard_class.h>
#include <autonomy_utils/lib/ros2_core.h>

std::shared_ptr<ros2::LifeCycle> ros2::Node::node = nullptr; 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    ros2::Node::node = std::make_shared<ros2::LifeCycle>("keyboard_node");

    std::shared_ptr<ros2::KeyboardClass> keyboard_class = std::make_shared<ros2::KeyboardClass>();

    rclcpp::spin(ros2::Node::node->get_node_base_interface());
}