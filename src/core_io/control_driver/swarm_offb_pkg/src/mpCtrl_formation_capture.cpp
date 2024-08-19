/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <tf/tf.h>

#define ROS_RATE 20.0

char mode_sel;
void mode_sel_cb(const std_msgs::Char::ConstPtr &msg)
{
    mode_sel = msg->data;
}

mavros_msgs::State cur_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}
geometry_msgs::PoseStamped cur_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cur_pos = *msg;

    cur_pos.pose.position.x = cur_pos.pose.position.x;
    cur_pos.pose.position.y = cur_pos.pose.position.y;
}

geometry_msgs::TwistStamped cur_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    cur_vel = *msg;
}

nav_msgs::Odometry leader_cur_state;
void leader_state_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    leader_cur_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_node");
    ros::NodeHandle nh("~");

    std::string ns = nh.getNamespace();

    std::string s_uav_id;
    nh.param<std::string>("uav_id", s_uav_id, "999");
    int i_uav_id = std::stoi(s_uav_id);

    int num_of_uav;
    nh.param("num_of_uav", num_of_uav, 0);

    double height;
    nh.param("height", height, 3.0);

    double bias_x;
    nh.param("bias_x", bias_x, 0.0);

    double bias_y;
    nh.param("bias_y", bias_y, 0.0);

    double radius;
    nh.param("radius", radius, 1.0);

    float omega = M_PI / 16;
    float dt = 1 / ROS_RATE;
    int k = 0;
    bool takeoff_flag = false;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav" + s_uav_id + "/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav" + s_uav_id + "/mavros/local_position/velocity_local", 10, vel_cb);
    ros::Subscriber mode_sub = nh.subscribe("/mode_select", 1000, mode_sel_cb);
    ros::Subscriber leader_pose_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1000, leader_state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav" + s_uav_id + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav" + s_uav_id + "/mavros/set_mode");
    // the setpoint publishing rate MUST be faster than 2Hz

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/setpoint_position/local", 10);
    ros::Publisher local_accel_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav" + s_uav_id + "/mavros/setpoint_raw/local", 10);
    ros::Publisher err_pub = nh.advertise<std_msgs::Float64MultiArray>("/uav" + s_uav_id + "/errors", 10);

    ros::Rate rate(ROS_RATE);

    // wait for FCU connection
    while (ros::ok() && !cur_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped hover_pose;
    hover_pose.pose.position.x = 0;
    hover_pose.pose.position.y = 0;
    hover_pose.pose.position.z = height;

    geometry_msgs::PoseStamped exp_pos;
    exp_pos.pose.position.x = 0;
    exp_pos.pose.position.y = 0;
    exp_pos.pose.position.z = height;

    geometry_msgs::TwistStamped exp_vel;
    exp_vel.twist.linear.x = 0;
    exp_vel.twist.linear.y = 0;
    exp_vel.twist.linear.z = 0;

    geometry_msgs::Vector3Stamped exp_acc;
    exp_acc.vector.x = 0;
    exp_acc.vector.y = 0;
    exp_acc.vector.z = 0;

    std_msgs::Float64MultiArray err_msgs;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(hover_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    mavros_msgs::PositionTarget mpCtrl;
    mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE;
    mpCtrl.coordinate_frame = mpCtrl.FRAME_LOCAL_NED;
    mpCtrl.yaw = 0;

    while (ros::ok())
    {
        std::cout << "current state is " << cur_state.mode << std::endl;
        if (cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        else if (!cur_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        else
        {
            if (!takeoff_flag)
            {
                if (cur_pos.pose.position.z > height + 0.1 || cur_pos.pose.position.z < height - 0.1)
                {
                    hover_pose.pose.position.x = 0;
                    hover_pose.pose.position.y = 0;
                    hover_pose.pose.position.z = height;
                    local_pos_pub.publish(hover_pose);
                }
                else
                {
                    takeoff_flag = true;
                    ROS_INFO("Take off done");
                    hover_pose.pose.position.x = cur_pos.pose.position.x;
                    hover_pose.pose.position.y = cur_pos.pose.position.y;
                    hover_pose.pose.position.z = height;
                }
            }
            else
            {
                std::cout << mode_sel << std::endl;
                double phi = omega * k * dt + M_PI / 2 + 2 * M_PI * i_uav_id / num_of_uav;
                switch (mode_sel)
                {
                default:
                    ROS_INFO("Current Mode: Default.");
                    exp_pos.pose.position.x = hover_pose.pose.position.x;
                    exp_pos.pose.position.y = hover_pose.pose.position.y;
                    exp_pos.pose.position.z = hover_pose.pose.position.z;
                    exp_vel.twist.linear.x = 0;
                    exp_vel.twist.linear.y = 0;
                    exp_vel.twist.linear.z = 0;
                    exp_acc.vector.x = 0;
                    exp_acc.vector.y = 0;
                    exp_acc.vector.z = 0;
                    break;
                case '1':
                    ROS_INFO("Current Mode: Tracking.");
                    exp_pos.pose.position.x = leader_cur_state.pose.pose.position.x + radius * cos(phi) - bias_x;
                    exp_pos.pose.position.y = leader_cur_state.pose.pose.position.y + radius * sin(phi) - bias_y;
                    exp_pos.pose.position.z = leader_cur_state.pose.pose.position.z + height;
                    exp_vel.twist.linear.x = leader_cur_state.twist.twist.linear.x;
                    exp_vel.twist.linear.y = leader_cur_state.twist.twist.linear.y;
                    exp_vel.twist.linear.z = 0;
                    exp_acc.vector.x = 0;
                    exp_acc.vector.y = 0;
                    exp_acc.vector.z = 0;
                    break;
                case '2':
                    ROS_INFO("Current Mode: Capture.");
                    exp_pos.pose.position.x = leader_cur_state.pose.pose.position.x + radius * cos(phi) - bias_x;
                    exp_pos.pose.position.y = leader_cur_state.pose.pose.position.y + radius * sin(phi) - bias_y;
                    exp_pos.pose.position.z = leader_cur_state.pose.pose.position.z + height;
                    exp_vel.twist.linear.x = leader_cur_state.twist.twist.linear.x - omega * radius * sin(phi);
                    exp_vel.twist.linear.y = leader_cur_state.twist.twist.linear.y + omega * radius * cos(phi);
                    exp_vel.twist.linear.z = 0;
                    exp_acc.vector.x = 0;
                    exp_acc.vector.y = 0;
                    exp_acc.vector.z = 0;
                    k++;
                    hover_pose.pose.position = cur_pos.pose.position;
                    hover_pose.pose.position.z = height;
                    break;
                }
                mpCtrl.position = exp_pos.pose.position;
                mpCtrl.velocity = exp_vel.twist.linear;
                mpCtrl.acceleration_or_force = exp_acc.vector;
                local_accel_pub.publish(mpCtrl);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}