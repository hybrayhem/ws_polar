#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

struct test
{
    float base_motor;
    float elbow_motor1;
    float elbow_motor2;
    float wrist_motor1;
    float wrist_motor2;
    float gripper_motor;
    float hand_motor;
};

serial::Serial ser;
struct test data, old_data;
// float maxd = 0.0;

void write_callback(const sensor_msgs::JointState &msg)
{
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    //ser.write(msg->data);
    printf("\n[%d] Position:\n", msg.header.seq);
    int i;
    for (i=0; i<6; i++){
        printf("%s: %.20f\n", msg.name[i].c_str(), msg.position[i]);
        // printf("%s: %.20f --> %.20f\n", msg.name[i].c_str(), msg.position[i], roundf(msg.position[i] * 1000000000) / 1000000000);
    }
    // struct test data;

    data.base_motor=msg.position[0];
    data.elbow_motor1 = msg.position[1];
    data.elbow_motor2 = msg.position[2];
    data.wrist_motor1 = msg.position[3];
    data.wrist_motor2 = msg.position[4];
    data.gripper_motor = msg.position[5];
    
    // float threshold = 0.000000002;
    // float diff[6] = {
    //     data.base_motor - old_data.base_motor,
    //     data.elbow_motor1 - old_data.elbow_motor1,
    //     data.elbow_motor2 - old_data.elbow_motor2,
    //     data.wrist_motor1 - old_data.wrist_motor1,
    //     data.wrist_motor2 - old_data.wrist_motor2,
    //     data.gripper_motor - old_data.gripper_motor,
    // };
    // old_data = data;

    // // printf("changes:\n");
    // int flag = 0;
    // for (i = 0; i < 6; i++) {
    //     // printf("%.20f\n", diff[i]);
    //     if(fabs(diff[i]) > threshold) flag = 1;
    //     // if(fabs(diff[i]) > maxd) maxd = fabs(diff[i]);
    // }
    // // printf("max difference = %f\n", maxd);

    // if(flag){
        printf("Serial write.\n");
        ser.write((const uint8_t*) &data, sizeof(struct test)); 
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_joint_driver_node");
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("joint_states", 1000, write_callback);
    // ros::Publisher read_pub = nh.advertise<sensor_msgs::JointState>("read", 1000);

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

    ros::Rate loop_rate(5);

    char str[98];
    uint8_t junk = '5';
    ser.write((uint8_t *)&junk, 1);
/*    while (ros::ok)
    {

        scanf("%f", &data.base_motor);
        scanf("%f", &data.elbow_motor1);
        scanf("%f", &data.elbow_motor2);
        scanf("%f", &data.wrist_motor1);
        scanf("%f", &data.wrist_motor2);
        scanf("%f", &data.gripper_motor);
        uint8_t *temp = (uint8_t *)&data;

        for (int i = 0; i < 24; i++)
        {
            sprintf(&str[i * 4], "%03d ", (int)temp[i]);
        }
        str[96] = '\n';
        str[97] = '\0';
        printf("%s", str);

        ros::spinOnce();

        size_t a = ser.write(temp, 24);
        std::cout << a << std::endl;

        
        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        
        loop_rate.sleep();
    }*/
    ros::spin();
}
