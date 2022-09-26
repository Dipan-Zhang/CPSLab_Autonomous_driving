#define _USE_MATH_DEFINES

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Car Parameters
float speed_limit = 1;
float angle_limit = 0.34;
float range_front[2] = {-M_PI/2, M_PI/2};   // indices (-pi/2, pi/2)
float range_left[2] = {3*M_PI/4, M_PI/4}; // indices (675, 944)
float range_right[2] = {-3*M_PI/4, -M_PI/4};  // indices (135, 404)

// Controller Parameters
float dist2wall = 1.5; // [m]
float throttle = 0.5;
float kp = 5;//0.6 * 10;
float kd = 5;//0.075*10*1;
float ki = 0.05;//1.2 * 10/1;
float pred_horizon = 5;
float speed_tolerance = 0.1;  // for deciding when the car is standing still

// Global Variables
float current_speed;
float min_dist_rt[2] = {0,0};
float turning_angle;
float error[2] = {0,0};
float error_int;
float p_error;
float d_error;
float i_error;
float pred_dist;

// Variables for DataPlottinig
ros::Publisher p_error_pub;
ros::Publisher i_error_pub;
ros::Publisher d_error_pub;
ros::Publisher turning_angle_pub;
ros::Publisher error_pub;
ros::Publisher dist2wall_pub;
ros::Publisher pred_dist_pub;


// Clamp values higher or lower than a limit
float clamp(float x, float lowerlimit, float upperlimit) {
  if (x < lowerlimit)
    x = lowerlimit;
  if (x > upperlimit)
    x = upperlimit;
  return x;
}

// Get index for given angle
int ang2ind(float angle, float increment, float angle_min){
  int index;
  return index = (angle-angle_min)/increment;
}

void callback_speed(const nav_msgs::Odometry::ConstPtr& msg) {
  current_speed = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg_scan) {
  // Get values of distances
  std::vector<float> ranges(msg_scan->ranges.cbegin(),msg_scan->ranges.cend());

  // Min and increment angle of scan data
  float min_angle = msg_scan->angle_min;
  float inc = msg_scan->angle_increment;

  // Transform angle definitions to indices
  int indices_rt[] = {ang2ind(range_right[0],inc,min_angle), ang2ind(range_right[1],inc,min_angle)};

  // Find minimum distance (left and right)
  min_dist_rt[1] = *min_element(ranges.begin()+indices_rt[0],ranges.begin()+indices_rt[1]);

  // Find predicted position with linear interpolatioin of previous position
  float delta_AB = min_dist_rt[1] - min_dist_rt[0];
  pred_dist = min_dist_rt[1] + pred_horizon*delta_AB;


  /* --- Steering Controller --- */

  // Compute error from desired distance to wall
  error[1] = dist2wall - pred_dist;
  if (current_speed > speed_tolerance) {
    error_int += error[1];
  } // only count error, when car is moving

  // PID_Controller action
  p_error = kp*error[1];
  d_error = kd*(error[1]-error[0])/25;
  i_error = ki*error_int;
  turning_angle = p_error + d_error + i_error;

  // Enforce boundaries for turning angle
  turning_angle = clamp(turning_angle,-1,1);

  // Store data for next time step
  error[0] = error[1];
  min_dist_rt[0] = min_dist_rt[1];
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pid_controller");
  ros::NodeHandle n;

  // Publisher and Subscriber
  ros::Subscriber sub = n.subscribe("/scan",1,callback);
  ros::Subscriber speed_sub = n.subscribe("/odom",1,callback_speed);
  ros::Publisher ctrl_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_controller", 1);

  ros::Rate loop_rate(25);

  while(ros::ok()) {

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = throttle * speed_limit;
    drive_msg.steering_angle = turning_angle * angle_limit;

    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;

    // publish AckermannDriveStamped message to drive topic
    ctrl_pub.publish(drive_st_msg);

    /* Visualization Publishers */
    p_error_pub = n.advertise<std_msgs::Float32>("/p_error",1);
    i_error_pub = n.advertise<std_msgs::Float32>("/i_error",1);
    d_error_pub = n.advertise<std_msgs::Float32>("/d_error",1);
    turning_angle_pub = n.advertise<std_msgs::Float32>("/turning_angle",1);
    error_pub = n.advertise<std_msgs::Float32>("/error",1);
    dist2wall_pub = n.advertise<std_msgs::Float32>("/dist2wall",1);
    pred_dist_pub = n.advertise<std_msgs::Float32>("/pred_dist2wall",1);

    /* ----------------------------------------------- */
    std_msgs::Float32 p_e_msg;
    p_e_msg.data = p_error;
    p_error_pub.publish(p_e_msg);

    std_msgs::Float32 i_e_msg;
    i_e_msg.data = i_error;
    i_error_pub.publish(i_e_msg);

    std_msgs::Float32 d_e_msg;
    d_e_msg.data = d_error;
    d_error_pub.publish(d_e_msg);

    std_msgs::Float32 error_msg;
    error_msg.data = error[1];
    error_pub.publish(error_msg);

    std_msgs::Float32 dist2wall_msg;
    dist2wall_msg.data = min_dist_rt[1];
    dist2wall_pub.publish(dist2wall_msg);

    std_msgs::Float32 pred_dist2wall_msg;
    pred_dist2wall_msg.data = pred_dist;
    pred_dist_pub.publish(pred_dist2wall_msg);

    std_msgs::Float32 turning_angle_msg;
    turning_angle_msg.data = turning_angle;
    turning_angle_pub.publish(turning_angle_msg);
    /* ----------------------------------------------- */

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
