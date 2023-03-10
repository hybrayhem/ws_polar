#include <polar_control/polar_hw_interface.h>

namespace polar_control_ns
{
PolarHWInterface::PolarHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  joint2_offset = 0.3813; // -
  joint3_offset = -0.3951; // +
  joint5_offset = 0;//-0.3333; // -

  state_pub = nh.advertise<polar_control::PolarCommand>("/polar_control/write", 1);
  ROS_INFO("PolarHWInterface constructed.");
}

long PolarHWInterface::map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PolarHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();
  for(int i = 0; i < 3; i++){
    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        continue;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
        break;
    }
    else
    {
        ROS_INFO_STREAM("Serial Port not active");
    }
  }

  char str[98];
  uint8_t junk = '5';
  ser.write((uint8_t *)&junk, 1);

  ROS_INFO("PolarHWInterface initialized.");
}

void PolarHWInterface::read(ros::Duration& elapsed_time)
{
  // printf("\nread\n");
  static struct polar_joints state;
  //ser.read((uint8_t*) &state, sizeof(struct polar_joints))
  if (ser.read((uint8_t*) &state, sizeof(struct polar_joints) ) == sizeof(struct polar_joints)){
  joint_position_[0] = state.polar_joint1;
  joint_position_[1] = state.polar_joint2 - joint2_offset;
  joint_position_[2] = state.polar_joint3 - joint3_offset;
  joint_position_[3] = state.polar_joint4;
  joint_position_[4] = state.polar_joint5 - joint5_offset;
  joint_position_[5] = state.polar_joint6;
  joint_position_[6] = -map(state.polar_hand_joint1, 34, 70, 0, 1.0472);
  // joint_position_[6] = -60;
  }
  
}

void PolarHWInterface::write(ros::Duration& elapsed_time)
{
  static struct polar_joints cmd;
  // printf("\nwrite\n");

  // Safety
  enforceLimits(elapsed_time);

  // Publish info for debug
  polar_control::PolarCommand polar_command;
  for(int i = 0; i < num_joints_; i++){
    // printf("[%s] %f\n", joint_names_[i].c_str(), joint_position_command_[i]);
    polar_command.position[i] = joint_position_command_[i];
    polar_command.joint_names[i] = joint_names_[i];
  }
  state_pub.publish(polar_command);

  // Serial write
  cmd.polar_joint1 = joint_position_command_[0];
  cmd.polar_joint2 = joint_position_command_[1] + joint2_offset;
  cmd.polar_joint3 = joint_position_command_[2] + joint3_offset;
  cmd.polar_joint4 = joint_position_command_[3];
  cmd.polar_joint5 = joint_position_command_[4] + joint5_offset;
  cmd.polar_joint6 = joint_position_command_[5];
  cmd.polar_hand_joint1 = map(-joint_position_command_[6], 0, 1.0472, 34, 70);
  // cmd.polar_hand_joint1 = 70;
  ser.write((const uint8_t*) &cmd, sizeof(struct polar_joints));

}

void PolarHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  // printf("enforceLimits\n");
}

}  // namespace polar_control_ns
