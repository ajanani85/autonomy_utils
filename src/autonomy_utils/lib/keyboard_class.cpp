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

    configure();
}


KeyboardClass::~KeyboardClass()
{

}

void KeyboardClass::configure()
{
    keyboard_pub_ = ros2::Node::node->create_publisher<std_msgs::msg::String>("/key_pressed",1);

    check_key_timer_ = ros2::Node::node->create_wall_timer(ros2::toDuration(0.05), std::bind(&KeyboardClass::timerCB, this));

}


void KeyboardClass::timerCB()
{

    char key = getKey();
    if (key != '\0') 
    {
        std_msgs::msg::String msg;
        msg.data = getKey();
        keyboard_pub_->publish(msg);
    }

    
}

char KeyboardClass::getKey() 
{
    static struct termios oldt, newt;
    char ch;
    int nread;

    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 1;  // Read at least 1 character at a time
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