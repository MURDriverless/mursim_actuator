#include "actuator_node.h"
#include <iostream>

Actuator::Actuator(ros::NodeHandle n, std::string &veh_name, bool &equal_drive)
    : equal_drive(equal_drive), nh(n), veh_name(veh_name)
{
    // Setup message layout for publishing
    drive_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    drive_msg.layout.dim[0].size = 4;
    drive_msg.layout.dim[0].stride = 1;

    steer_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    steer_msg.layout.dim[0].size = 2;
    steer_msg.layout.dim[0].stride = 1;

    if (ros::ok())
     {
        launchSubscribers(); 
        launchPublishers(); 
     }

    ROS_INFO_STREAM("Publishers and subscribers launched.");
}

int Actuator::launchPublishers()
{
    pub_steer = nh.advertise<std_msgs::Float64MultiArray>("/" + veh_name + STEER_TOPIC, 1);
    pub_drive = nh.advertise<std_msgs::Float64MultiArray>("/" + veh_name + DRIVE_TOPIC, 1);
}

int Actuator::launchSubscribers()
{
    sub_ctrl = nh.subscribe(CONTROL_TOPIC, 1, &Actuator::ctrlCallback, this);
}

void Actuator::pushEqualDrive(const float &n_acc)
{
    float acc_msg = n_acc * MAX_WHEEL_TORQUE;

    drive_msg.data.assign({acc_msg, acc_msg, acc_msg, acc_msg});
    pub_drive.publish(drive_msg);
}

void Actuator::pushSteer(const float &steer)
{
    steer_msg.data.assign({steer, steer});
    pub_steer.publish(steer_msg);
}

void Actuator::printMessage(const std::vector<double>& cmd)
{
    for (auto &c: cmd)
        std::cout << c << ' ';
}

void Actuator::spin()
{
    ros::spinOnce();
}

void Actuator::ctrlCallback(const mur_common::actuation_msg &msg)
{
    pushEqualDrive(msg.acceleration_threshold);
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
    ros::Rate freq(20);
	
    while (ros::ok())
    {
        actuator.spin();
        freq.sleep();
    }
}
