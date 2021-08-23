#include <polar_control/polar_hw_interface.h>

namespace polar_control_ns
{
PolarHWInterface::PolarHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  joint_subscriber = nh.subscribe("/polar_serial/read", 1, &PolarHWInterface::jointSubscriberCallback, this);
  joint_publisher = nh.advertise<polar_control::PolarCommand>("/polar_serial/write", 1);

  ROS_INFO("PolarHWInterface constructed.");
}

void PolarHWInterface::jointSubscriberCallback(const polar_control::PolarCommand::ConstPtr &msg){
  printf("\njointSubscriberCallback\n");

  for(int i = 0; i < num_joints_; i++){
    joint_position_[i] = msg->position[i];
    joint_velocity_[i] = msg->velocity[i];
    printf("[%s] %f\n", joint_names_[i].c_str(), joint_position_[i]);
  }
}

void PolarHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("PolarHWInterface Ready.");
}

void PolarHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
  printf("read\n");
  ros::spinOnce();
}

void PolarHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  // enforceLimits(elapsed_time);
  printf("write\n");
  // static struct PolarHWInterface::polar_command cmd; from header
  static polar_control::PolarCommand msg;
  for(int i = 0; i < num_joints_; i++){
    msg.position[i] = joint_position_command_[i];
    msg.velocity[i] = joint_velocity_command_[i];
    printf("[%s] %f %f\n", joint_names_[i].c_str(), joint_position_command_[i], joint_velocity_command_[i]);
  }

  joint_publisher.publish(msg);

}

void PolarHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
  printf("enforceLimits\n");
}

}  // namespace polar_control_ns
