#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

using namespace std;

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyTHS1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()) ROS_INFO_STREAM("Serial Port initialized");
    else return -1;

    ros::Rate loop_rate(1);
    
    // while(ros::ok()){

    //     ros::spinOnce();

    //     if(ser.available()){
    //         ROS_INFO_STREAM("Reading from serial port");
    //         std_msgs::String result;
    //         result.data = ser.read(ser.available());
    //         ROS_INFO_STREAM("Read: " << result.data);
    //         read_pub.publish(result);
    //     }
    //     loop_rate.sleep();

    // }

    std_msgs::String data_send;

    int temp_1 = 0, temp_2 = 0, temp_3 = 0;

    while(ros::ok())
    {
        data_send.data = "{\"theta1\":\"" + to_string(temp_1) +
             "\",\"theta2\":\"" + to_string(temp_2) +
             "\",\"theta3\":\"" + to_string(temp_3) + "\"}\r\n";

        ser.write(data_send.data);

        temp_1 += 1;
        temp_2 += 2;
        temp_3 += 3;

        if(temp_1>100) temp_1 = 0;
        if(temp_2>100) temp_2 = 0;
        if(temp_3>100) temp_3 = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }
}
