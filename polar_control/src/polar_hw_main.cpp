#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <polar_control/polar_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polar_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::shared_ptr<polar_control_ns::PolarHWInterface> polar_hw_interface(
      new polar_control_ns::PolarHWInterface(nh));
  polar_hw_interface->init();

  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, polar_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved

  return 0;
}
