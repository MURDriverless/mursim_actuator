#include "actuator_node.h"
#include <iostream>

Actuator::Actuator(ros::NodeHandle n, std::string &veh_name, bool &equal_drive)
    : equal_drive(equal_drive), nh(n), veh_name(veh_name)
{

    if (ros::ok())
     {
        launchSubscribers(); 
        launchPublishers(); 
     }

    ROS_INFO_STREAM("Publishers and subscribers launched.");
}

int Actuator::launchPublishers()
{
    pub_drive_lf = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_LF_TOPIC, 1);
    pub_drive_lr = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_LR_TOPIC, 1);
    pub_drive_rf = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_RF_TOPIC, 1);
    pub_drive_rr = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_RR_TOPIC, 1);
    pub_steer = nh.advertise<std_msgs::Float64MultiArray>("/" + veh_name + STEER_TOPIC, 1);
}

int Actuator::launchSubscribers()
{
    sub_ctrl = nh.subscribe(CONTROL_TOPIC, 1, &Actuator::ctrlCallback, this);
}

void Actuator::pushEqualDrive(const float &n_acc)
{
    float max_wheel_tq = MAX_TORQUE / 4;

    std_msgs::Float64 acc_msg;
    acc_msg.data = max_wheel_tq * n_acc;

    pub_drive_lf.publish(acc_msg);
    pub_drive_lr.publish(acc_msg);
    pub_drive_rf.publish(acc_msg);
    pub_drive_rr.publish(acc_msg);
}

void Actuator::pushSteer(const float &steer)
{
    std::vector<float> steer_cmd {steer, steer};
    
    std_msgs::Float64MultiArray steer_msg;
    steer_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    steer_msg.layout.dim[0].size = steer_cmd.size();
    steer_msg.layout.dim[0].stride = 1;

    steer_msg.data.clear();
    steer_msg.data.insert(steer_msg.data.end(), steer_cmd.begin(), steer_cmd.end());
    std::cout << steer_msg << std::endl;

    pub_steer.publish(steer_msg);
}

void Actuator::spin()
{
    ros::spinOnce();
}

void Actuator::ctrlCallback(const mur_common::actuation_msg &msg)
{
    if (equal_drive)
    {
	   pushEqualDrive(msg.acceleration_threshold);
    }

    pushSteer(msg.steering);
}

std::string model;
bool eq_drive;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "/mur/control/actuation");
    ros::NodeHandle n;

    n.getParam("vehicle_model", model);
    n.getParam("equal_drive", eq_drive);

    Actuator actuator(n, model, eq_drive);
	
    while (ros::ok())
    {
        actuator.spin();
    }
}
