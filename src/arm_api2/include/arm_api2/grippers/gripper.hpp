#ifndef GRIPPER_H
#define GRIPPER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>

class Gripper {
public:
    // Constructor
    Gripper(){};
    virtual ~Gripper(){};

    // Method to open the gripper
    virtual void open() = 0;

    // Method to close the gripper
    virtual void close() = 0;

private:
    bool isOpen;
    // TODO: Add ROS2 action client or topic publisher which is going to be used
};

#endif // GRIPPER_H