#include <iostream>
#include <termios.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"


char get_keyboard() {
    // Read a single character.
    char c = 0;
    std::cin >> c;
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_keyboard");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("keyboard", 10);
    ros::Rate loop_rate(100);
    std::cout << "start read keyboard" << std::endl;

    // Retrieve the properties of the standard input stream.
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Set unbuffered mode.
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        char c = get_keyboard();
        msg.data = c;

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    // Restore the properties of the standard input stream.
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return 0;
}
