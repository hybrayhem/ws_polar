#ifndef POLAR_HW_INTERFACE_H
#define POLAR_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <polar_control/PolarCommand.h>
#include <serial/serial.h>

// #define DEG_TO_RAD 0.01745329251
// #define RAD_TO_DEG 57.2957795131

namespace polar_control_ns
{

struct polar_joints
{
    float polar_joint1;
    float polar_joint2;
    float polar_joint3;
    float polar_joint4;
    float polar_joint5;
    float polar_joint6;
    float polar_hand_joint1;
};

/** \brief Hardware interface for a robot */
class PolarHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  PolarHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
  serial::Serial ser;
  ros::Publisher state_pub;
  
  // offsets for changing home position and adapt embedded side
  float joint2_offset;
  float joint3_offset;
  float joint5_offset;
};  // class

}  // namespace ros_control_boilerplate

#endif
