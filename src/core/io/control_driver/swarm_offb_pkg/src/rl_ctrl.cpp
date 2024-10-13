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
#include <std_msgs/Float32MultiArray.h>
#include "swarm_offb_pkg/regular_motion.hpp"
#include <vechicle_controller/qr_controller.h>
#include <random>

#define ROS_RATE 50.0
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

    ros::Publisher err_pub = nh.advertise<std_msgs::Float32MultiArray>("/states_save", 10);

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
    mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE + mpCtrl.IGNORE_PX + mpCtrl.IGNORE_PY + mpCtrl.IGNORE_AFX + mpCtrl.IGNORE_AFY + mpCtrl.IGNORE_AFZ;
    mpCtrl.coordinate_frame = mpCtrl.FRAME_LOCAL_NED;
    mpCtrl.yaw = 0;

    std_msgs::Float32MultiArray err_msgs;

    bool takeoff_flag = false;
    bool init_flag = false;
    bool offb_flag = false;
    int count = 0;
    int k = 0;
    double half_side_length = 1.5;
    double const_vel = 0.1;
    double omega = M_PI / 16;
    double radius = 1;
    double dt = 1 / ROS_RATE;
    uav_controller u_ctrl(1 / ROS_RATE);
    geometry_msgs::Vector3Stamped ctrl_acc;

    ros::Time last_request = ros::Time::now();
    // ------------ RL ctrl ------------------------------------------------------------
    double t = 0, tstart = 0;
    double x00 = 0.0, y00 = 1.0;

    double random_num;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    // double KK[2][4] = {{-0.98526, -0.9803, 9.9501, 0}, {0.9803, -9.8526, 0, 9.9501}};
    double KK[2][4] = {{0, 0, 2, 0}, {0, 0, 0, 2}};
    std::cout << "RL param init done ." << std::endl;
    // ------------ RL ctrl ------------------------------------------------------------
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
                if (abs(cur_pos.pose.position.x) > 0.1 ||
                    abs(cur_pos.pose.position.y) > 0.1)
                {
                    pose.pose.position.x = 0;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = height;
                    local_pos_pub.publish(pose);
                    count = 0;
                }
                else
                {
                    if (count < 10)
                    {
                        pose.pose.position.x = 0;
                        pose.pose.position.y = 0;
                        pose.pose.position.z = height;
                        local_pos_pub.publish(pose);
                    }
                    else
                    {
                        init_flag = true;
                        count = 0;
                        ROS_INFO("Move To done.");
                        tstart = ros::Time::now().toSec();
                    }
                    std::cout << count << std::endl;
                }
            }
            if (init_flag)
            {
                // std::cout << "current mode is: " << mode_sel << std::endl;
                // ROS_INFO("Current Mode: Moving Along Circle.");
                // u_ctrl.setState(cur_pos, cur_vel, exp_pos, exp_vel);
                // auto ctrl_acc = u_ctrl.getCtrl_pid();

                double t = ros::Time::now().toSec() - tstart;
                x00 = radius * sin(t);
                y00 = radius * cos(t);
                random_num = dis(gen);
                mpCtrl.velocity.x = -(KK[0][0] * x00 + KK[0][1] * y00 + KK[0][2] * cur_pos.pose.position.x +
                                      KK[0][3] * cur_pos.pose.position.y) +
                                    sin(random_num * t);
                random_num = dis(gen);
                mpCtrl.velocity.y = -(KK[1][0] * x00 + KK[1][1] * y00 + KK[1][2] * cur_pos.pose.position.x +
                                      KK[1][3] * cur_pos.pose.position.y) +
                                    cos(random_num * t);

                mpCtrl.position.z = height;
                mpCtrl.velocity.z = 0;
                local_acc_pub.publish(mpCtrl);
                k++;
                std::cout << k << std::endl;

                err_msgs.data = {
                    static_cast<float>(x00), static_cast<float>(y00),
                    static_cast<float>(cur_pos.pose.position.x), static_cast<float>(cur_pos.pose.position.y),
                    static_cast<float>(mpCtrl.velocity.x), static_cast<float>(mpCtrl.velocity.y),
                    static_cast<float>(t)};
                err_pub.publish(err_msgs);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
