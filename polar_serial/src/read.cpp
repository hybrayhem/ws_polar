#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

struct polar_command
{
    float polar_joint1;
    float polar_joint2;
    float polar_joint3;
    float polar_joint4;
    float polar_joint5;
    float polar_joint6;
    float polar_hand_joint1;
};

serial::Serial ser;
struct polar_command data;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polar_serial_read");
    ros::NodeHandle n;

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(25);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_INFO_STREAM("Serial Port not active");
    }

    ros::Rate loop_rate(100);
    uint8_t buffer;
    while (ros::ok()){
        ser.read((uint8_t*) &data, (sizeof(struct polar_command)));
        // printf("\n %u \n", buffer);
        // memcpy(&data, (const uint8_t*) &buffer, sizeof(polar_command));

        printf("Read:\n%f \n%f \n%f \n%f \n%f \n%f \n%f\n",
                 data.polar_joint1,
                 data.polar_joint2,
                 data.polar_joint3,
                 data.polar_joint4,
                 data.polar_joint5,
                 data.polar_joint6,
                 data.polar_hand_joint1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
