#ifndef SRC_ACTUATOR_NODE_H
#define SRC_ACTUATOR_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mur_common/actuation_msg.h>

#define CONTROL_TOPIC "/mur/control/actuation"
#define STEER_TOPIC "/steer_drive_controller/steering_position_controller/command"
#define DRIVE_LF_TOPIC "/steer_drive_controller/front_left_drive_controller/command"
#define DRIVE_LR_TOPIC "/steer_drive_controller/rear_left_drive_controller/command"
#define DRIVE_RF_TOPIC "/steer_drive_controller/front_right_drive_controller/command"
#define DRIVE_RR_TOPIC "/steer_drive_controller/rear_right_drive_controller/command"

class Actuator 
{
public:
    Actuator(ros::NodeHandle&, std::string&, bool&);
private:
    bool equal_drive;
    const float MAX_TORQUE = 400.0;
    std::string veh_name;
    ros::NodeHandle nh;

    ros::Publisher pub_drive_lf;
    ros::Publisher pub_drive_lr;
    ros::Publisher pub_drive_rf;
    ros::Publisher pub_drive_rr;
    ros::Publisher pub_steer;
    ros::Subscriber sub_ctrl;

    int launchPublishers();
    int launchSubscribers();

    void pushEqualDrive(const float&);
    void pushSteer(const float&); 

    void ctrlCallback(const mur_common::actuation_msg &msg);
};


#endif // SRC_ACTUATOR_NODE_H
