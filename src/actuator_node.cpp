#include "actuator_node.h"

Actuator::Actuator(ros::NodeHandle &n, std::string &veh_name, bool &equal_drive)
    : equal_drive(equal_drive), nh(n), veh_name(veh_name)
{
    launchSubscribers(); 
    launchPublishers(); 
}

int Actuator::launchPublishers()
{
    pub_drive_lf = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_LF_TOPIC, 1);
    pub_drive_lr = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_LR_TOPIC, 1);
    pub_drive_rf = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_RF_TOPIC, 1);
    pub_drive_rr = nh.advertise<std_msgs::Float64>("/" + veh_name + DRIVE_RR_TOPIC, 1);
    pub_steer = nh.advertise<std_msgs::Float64>("/" + veh_name + STEER_TOPIC, 1);
}

int Actuator::launchSubscribers()
{
    sub_ctrl = nh.subscribe(CONTROL_TOPIC, 1, &Actuator::ctrlCallback, this);
}

void Actuator::pushEqualDrive(const float &n_acc)
{
    float max_wheel_tq = MAX_TORQUE / 4;
    pub_drive_lf.publish(n_acc * max_wheel_tq);
    pub_drive_lr.publish(n_acc * max_wheel_tq);
    pub_drive_rf.publish(n_acc * max_wheel_tq);
    pub_drive_rr.publish(n_acc * max_wheel_tq);
}

void Actuator::pushSteer(const float &steer)
{
    pub_steer.publish(steer);
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

    if (ros::ok())
    {
	Actuator actuator(n, model, eq_drive);
    }

    while (ros::ok())
    {
	ros::spin();		
    }
}
