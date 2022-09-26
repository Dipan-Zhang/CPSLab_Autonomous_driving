#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


std::string driving_mode ("manual");
ros::Publisher pub;


void callback_keyboard(const ackermann_msgs::AckermannDriveStamped & msg) {
    pub.publish(msg);
    }

void callback_js(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (driving_mode == "js") {
      pub.publish(msg);
    }
    }

void callback_controller(const ackermann_msgs::AckermannDriveStamped & msg) {
    if (driving_mode == "ftg"){
       pub.publish(msg);
    }
    }

void callback_mode(const std_msgs::String & msg) {
        driving_mode = msg.data;
    }

void callback_aeb(const ackermann_msgs::AckermannDriveStamped & msg) {
    pub.publish(msg);
    driving_mode = "manual";
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mux");
    ros::NodeHandle n;

    // Publisher
    //pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 1);  // on car
    pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1); // for simulation
    // Subscriber
    ros::Subscriber sub_key = n.subscribe("/drive_keyboard", 1, callback_keyboard);
    ros::Subscriber sub_ctrl = n.subscribe("/drive_controller", 1, callback_controller);
    ros::Subscriber sub_mode = n.subscribe("/mode", 1, callback_mode);
    //ros::Subscriber sub_js = n.subscribe("/vesc/js", 1, callback_js); // on car
    ros::Subscriber sub_js = n.subscribe("/js", 1, callback_js); // for simulation

    ros::spin();

    return 0;
}
