#ifndef SRC_ACTUATOR_NODE_H
#define SRC_ACTUATOR_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mur_common/actuation_msg.h>

#define CONTROL_TOPIC "/mur/control/actuation"
#define STEER_TOPIC "/steer_drive_controller/steering_position_controller/command"
#define DRIVE_TOPIC "/steer_drive_controller/wheel_drive_controller/command"

class Actuator 
{
public:
    Actuator(ros::NodeHandle, std::string&, bool&);
    void spin();

private:
    bool equal_drive;

    static constexpr float MAX_OVERALL_TORQUE = 100;
    static constexpr float MAX_WHEEL_TORQUE = MAX_OVERALL_TORQUE / 4;

    std::string veh_name;
    ros::NodeHandle nh;

    ros::Publisher pub_steer;
    ros::Publisher pub_drive;

    ros::Subscriber sub_ctrl;

    std_msgs::Float64MultiArray drive_msg;
    std_msgs::Float64MultiArray steer_msg;

    int launchPublishers();
    int launchSubscribers();

    void pushEqualDrive(const float&);
    void pushSteer(const float&); 

    void ctrlCallback(const mur_common::actuation_msg&);
    void printMessage(const std::vector<double>&);
};


#endif // SRC_ACTUATOR_NODE_H
