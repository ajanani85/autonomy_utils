#include <autonomy_utils/lib/keyboard_class.h>
#include <termios.h>
#include <unistd.h>

namespace ros2
{
KeyboardClass::KeyboardClass()
{
    if(!ros2::Node::node)
    {
        throw std::runtime_error("ros2::Node::node is nullptr!"); 
    }
    // ros2::Node::node->register_on_configure_callback(std::bind(&KeyboardClass::configure, this));
    // ros2::Node::node->register_on_activate_callback(std::bind(&KeyboardClass::activate, this));
    configure();
    activate();
}


KeyboardClass::~KeyboardClass()
{
    running_ = false;
    if (key_thread_.joinable()) {
        key_thread_.join();
    }
}

void KeyboardClass::configure()
{
    ROS_INFO("Configuring ...");

    keyboard_pub_ = ros2::Node::node->create_publisher<std_msgs::msg::String>("/key_pressed",1);
    key_thread_ = std::thread(&KeyboardClass::keyLoop, this);
}


void KeyboardClass::activate()
{
    ROS_INFO("Activating ...");
}


void KeyboardClass::keyLoop()
{
    while (running_) 
    {
    
        ROS_INFO("HELOO");
        char key = getKey();
        std_msgs::msg::String msg;
        msg.data = key;
        keyboard_pub_->publish(msg);
        usleep(10000); // Small delay to reduce CPU usage
    }

    
}

char KeyboardClass::getKey() 
{
    struct termios oldt, newt;
    char ch;
    int nread;

    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;  // Read at least 1 character at a time
    newt.c_cc[VTIME] = 0; // No timeout, so it waits for input

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    nread = read(STDIN_FILENO, &ch, 1);

    // Restore old terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return (nread > 0) ? ch : '\0'; // Return character if read, otherwise return null

}

}