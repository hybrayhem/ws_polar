// #include <ros/ros.h>
// #include <serial/serial.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// // #include <sensor_msgs/JointState.h>
// #include <math.h>
// #include <polar_control/PolarCommand.h>

// struct polar_command
// {
//     float polar_joint1;
//     float polar_joint2;
//     float polar_joint3;
//     float polar_joint4;
//     float polar_joint5;
//     float polar_joint6;
//     float polar_hand_joint1;
// };

// serial::Serial ser;
// struct polar_command data, old_data;

// void write_callback(const polar_control::PolarCommand &msg)
// {
//     printf("\nVelocity - Position:\n");
//     int i;
//     for (i=0; i<7; i++){
//         printf("%f  %f\n", msg.position[i], msg.velocity[i]);
//     }

//     /*data.polar_joint1 = msg.position[0];
//     data.polar_joint2  =  msg.position[1];
//     data.polar_joint3  =  msg.position[2];
//     data.polar_joint4  =  msg.position[3];
//     data.polar_joint5  =  msg.position[4];
//     data.polar_joint6  =  msg.position[5];
//     data.polar_hand_joint1  =  msg.position[6];*/

//     data.polar_joint1  =  roundf(msg.position[0] * 100000) / 100000;
//     data.polar_joint2  =  roundf(msg.position[1] * 100000) / 100000;
//     data.polar_joint3  =  roundf(msg.position[2] * 100000) / 100000;
//     data.polar_joint4  =  roundf(msg.position[3] * 100000) / 100000;
//     data.polar_joint5  =  roundf(msg.position[4] * 100000) / 100000;
//     data.polar_joint6  =  roundf(msg.position[5] * 100000) / 100000;
//     data.polar_hand_joint1  =  roundf(msg.position[6] * 100000) / 100000;
    
//     /*float threshold = 0.000000002;
//     float diff[7] = {
//         data.polar_joint1 - old_data.polar_joint1,
//         data.polar_joint2 - old_data.polar_joint2,
//         data.polar_joint3 - old_data.polar_joint3,
//         data.polar_joint4 - old_data.polar_joint4,
//         data.polar_joint5 - old_data.polar_joint5,
//         data.polar_joint6 - old_data.polar_joint6,
//         data.polar_hand_joint1 - old_data.polar_hand_joint1,
//     };
//     old_data = data;

//     int flag = 0;
//     for (i = 0; i < 7; i++) {
//         if(fabs(diff[i]) > threshold) flag = 1;
//     }*/

//     // if(flag){
//         printf("Serial write.\n");
//         ser.write((const uint8_t*) &data, sizeof(struct polar_command)); 
//     // }
// }

// int _main(int argc, char **argv)
// {
//     ros::init(argc, argv, "polar_serial_write");
//     ros::NodeHandle nh;
//     ros::Subscriber write_sub = nh.subscribe("/polar_serial/write", 1000, write_callback);

//     try
//     {
//         ser.setPort("/dev/ttyACM0");
//         ser.setBaudrate(115200);
//         serial::Timeout to = serial::Timeout::simpleTimeout(10);
//         ser.setTimeout(to);
//         ser.open();
//     }
//     catch (serial::IOException &e)
//     {
//         ROS_ERROR_STREAM("Unable to open port ");
//         return -1;
//     }

//     if (ser.isOpen())
//     {
//         ROS_INFO_STREAM("Serial Port initialized");
//     }
//     else
//     {
//         ROS_INFO_STREAM("Serial Port not active");
//     }

//     ros::Rate loop_rate(5);

//     char str[98];
//     uint8_t junk = '5';
//     ser.write((uint8_t *)&junk, 1);

//     ros::spin();
// }
