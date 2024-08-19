/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include "swarm_offb_pkg/regular_motion.hpp"
#include <vechicle_controller/qr_controller.h>

#define ROS_RATE 20.0
std::string s_uav_id = "001";
int i_uav_id = std::stoi(s_uav_id);

int num_of_uav = 2;
float height = 1;

mavros_msgs::State cur_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

geometry_msgs::PoseStamped cur_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cur_pos = *msg;
}

geometry_msgs::TwistStamped cur_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    cur_vel = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_regular_motion");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav" + s_uav_id + "/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav" + s_uav_id + "/mavros/local_position/velocity_local", 10, vel_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav" + s_uav_id + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav" + s_uav_id + "/mavros/set_mode");

    ros::Publisher local_acc_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav" + s_uav_id + "/mavros/setpoint_raw/local", 10);
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    // wait for FCU connection
    while (ros::ok() && !cur_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = height;

    geometry_msgs::PoseStamped exp_pos;
    exp_pos.pose.position.z = height;
    geometry_msgs::TwistStamped exp_vel;
    exp_vel.twist.linear.z = 0;
    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::PositionTarget mpCtrl;
    // mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE + mpCtrl.IGNORE_PX + mpCtrl.IGNORE_PY + mpCtrl.IGNORE_PZ + mpCtrl.IGNORE_VX + mpCtrl.IGNORE_VY + mpCtrl.IGNORE_VZ;
    mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE + mpCtrl.IGNORE_AFX + mpCtrl.IGNORE_AFY + mpCtrl.IGNORE_AFZ;
    mpCtrl.coordinate_frame = mpCtrl.FRAME_LOCAL_NED;
    mpCtrl.yaw = 0;

    bool takeoff_flag = false;
    bool init_flag = false;
    bool offb_flag = false;
    int count;
    int k = 0;
    double half_side_length = 1.5;
    double const_vel = 0.1;
    Moving_along_line m_square(half_side_length, 1 / ROS_RATE, const_vel, height);
    m_square.init_path();
    auto pos_list = m_square.get_pos_list();
    auto vel_list = m_square.get_vel_list();

    for (auto col : pos_list)
    {
        std::cout << col.pose.position.x << ' ' << col.pose.position.y << std::endl;
    }
    for (auto col : vel_list)
    {
        std::cout << col.twist.linear.x << ' ' << col.twist.linear.y << std::endl;
    }
    uav_controller u_ctrl(1 / ROS_RATE);
    geometry_msgs::Vector3Stamped ctrl_acc;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !offb_flag)
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                offb_flag = true;
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
                if (abs(cur_pos.pose.position.z - height) > 0.1)
                {
                    pose.pose.position.x = 0;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = height;
                    local_pos_pub.publish(pose);
                }
                else
                {
                    takeoff_flag = true;
                    ROS_INFO("Take off done");
                }
            }
            if (!init_flag && takeoff_flag)
            {
                ROS_INFO("Current Mode: Trying To Initilization.");
                count++;
                if (abs(cur_pos.pose.position.x - pos_list[0].pose.position.x) > 0.1 ||
                    abs(cur_pos.pose.position.y - pos_list[0].pose.position.y) > 0.1)
                {
                    pose.pose.position.x = pos_list[0].pose.position.x;
                    pose.pose.position.y = pos_list[0].pose.position.y;
                    pose.pose.position.z = height;
                    local_pos_pub.publish(pose);
                    count = 0;
                }
                else
                {
                    if (count < 10)
                    {
                        pose.pose.position.x = pos_list[0].pose.position.x;
                        pose.pose.position.y = pos_list[0].pose.position.y;
                        pose.pose.position.z = height;
                        local_pos_pub.publish(pose);
                    }
                    else
                    {
                        init_flag = true;
                        count = 0;
                        ROS_INFO("Move To done.");
                    }
                }
            }
            if (init_flag)
            {
                // std::cout << "current mode is: " << mode_sel << std::endl;
                // ROS_INFO("Current Mode: Moving Along Circle.");
                count = k % pos_list.size();
                exp_pos = pos_list[count];
                exp_vel = vel_list[count];
                // u_ctrl.setState(cur_pos, cur_vel, exp_pos, exp_vel);
                // auto ctrl_acc = u_ctrl.getCtrl_pid();

                mpCtrl.position.x = exp_pos.pose.position.x;
                mpCtrl.position.y = exp_pos.pose.position.y;
                mpCtrl.position.z = height;
                mpCtrl.velocity.x = exp_vel.twist.linear.x;
                mpCtrl.velocity.y = exp_vel.twist.linear.y;
                mpCtrl.velocity.z = 0;
                local_acc_pub.publish(mpCtrl);
                k++;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
