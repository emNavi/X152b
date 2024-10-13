/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <vechicle_controller/qr_controller.h>
#include "swarm_offb_pkg/regular_motion.hpp"

#define ROS_RATE 50.0

uav_controller u_ctrl(1 / ROS_RATE);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
geometry_msgs::PoseStamped cur_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cur_pos = *msg;
    // cur_pos.pose.position.x = cur_pos.pose.position.x;
    // cur_pos.pose.position.y = cur_pos.pose.position.y + 3;
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

char mode_sel;
std_msgs::Float32MultiArray new_params;
void param_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    mode_sel = static_cast<int>(msg->data[1]) + '0';
    std::cout << "head data is: " << static_cast<int>(msg->data[0]) << std::endl;
    if (static_cast<int>(msg->data[0]) == 2)
    {
        std::cout << "changing flight mode." << std::endl;
    }
    else if (static_cast<int>(msg->data[0]) == 11)
    {
        // reset PID parameters
        ROS_INFO("Reset PID parameters.");
        float P_param[3];
        float I_param[3];
        float D_param[3];
        for (int i = 2; i < 11; i++)
        {
            if (i <= 4)
            {
                P_param[i - 2] = msg->data[i];
                ROS_INFO_STREAM("P parameters "
                                << "P[" << i - 2 << "] is: " << P_param[i - 2]);
            }
            else if (i <= 7)
            {
                I_param[(i - 2) % 3] = msg->data[i];
                ROS_INFO_STREAM("I parameters "
                                << "I[" << (i - 2) % 3 << "] is: " << I_param[(i - 2) % 3]);
            }
            else
            {
                D_param[(i - 2) % 3] = msg->data[i];
                ROS_INFO_STREAM("D parameters "
                                << "D[" << (i - 2) % 3 << "] is: " << D_param[(i - 2) % 3]);
            }
        }
        u_ctrl.reset_params_PID(P_param, I_param, D_param);
    }
    else if (static_cast<int>(msg->data[0]) == 8)
    {
        // reset FG parameters
        ROS_INFO("Reset Robust parameters.");
        float F_param[3];
        float G_param[3];
        for (int i = 2; i < 8; i++)
        {
            if (i <= 4)
            {
                F_param[i - 2] = msg->data[i];
                ROS_INFO_STREAM("F parameters "
                                << "F[" << i - 2 << "] is: " << F_param[i - 2]);
            }
            else
            {
                G_param[(i - 2) % 3] = msg->data[i];
                ROS_INFO_STREAM("G parameters "
                                << "G[" << (i - 2) % 3 << "] is: " << G_param[(i - 2) % 3]);
            }
        }
        u_ctrl.reset_params_robust(F_param, G_param);
    }
    else
    {
        ROS_INFO("Invalid head value, Please check values with: \n '1': PID parameters, input numbers should be 11. \n '2': Robust parameters, input numbers should be 8. \n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_capture_node");
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
    int count = 0;
    bool takeoff_flag = false;
    double phi;
    ros::Subscriber leader_pose_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1000, leader_state_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav" + s_uav_id + "/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav" + s_uav_id + "/mavros/local_position/velocity_local", 10, vel_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav" + s_uav_id + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav" + s_uav_id + "/mavros/set_mode");
    // the setpoint publishing rate MUST be faster than 2Hz

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/setpoint_position/local", 10);
    ros::Publisher local_accel_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav" + s_uav_id + "/mavros/setpoint_raw/local", 10);

    ros::Subscriber mode_sub = nh.subscribe("/array_socket", 1000, param_cb);

    ros::Publisher err_pub = nh.advertise<std_msgs::Float32MultiArray>("/uav" + s_uav_id + "/errors", 10);

    ros::Rate rate(ROS_RATE);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped hover_pose;
    hover_pose.pose.position.x = 0;
    hover_pose.pose.position.y = 0;
    hover_pose.pose.position.z = height;

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
    /* ------------------init params --------------------------------------------------*/
    bool init_flag = false;

    // geometry_msgs::PoseStamped hover_pose;
    // hover_pose.pose.position.x = 0;
    // hover_pose.pose.position.y = 0;
    // hover_pose.pose.position.z = height;

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

    geometry_msgs::PoseStamped temp_pos;
    temp_pos.pose.position.z = height;

    Capture m_cap(radius, omega, height);
    mavros_msgs::PositionTarget mpCtrl;
    // acc ctrl
    mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE + mpCtrl.IGNORE_PX + mpCtrl.IGNORE_PY + mpCtrl.IGNORE_PZ + mpCtrl.IGNORE_VX + mpCtrl.IGNORE_VY + mpCtrl.IGNORE_VZ;
    mpCtrl.coordinate_frame = mpCtrl.FRAME_LOCAL_NED;
    std_msgs::Float32MultiArray err_msgs;
    /* ------------------init params done --------------------------------------------*/
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if (!current_state.armed &&
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
            m_cap.set_leader_state(leader_cur_state);
            phi = omega * k * dt + M_PI / 2 + 2 * M_PI * i_uav_id / num_of_uav;
            if (!takeoff_flag)
            {
                if (abs(cur_pos.pose.position.z - height) > 0.1)
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
                }
            }
            if (!init_flag && takeoff_flag)
            {
                ROS_INFO("Current Mode: Trying To Initilization.");
                count++;
                m_cap.set_phi(phi);
                if (abs(cur_pos.pose.position.x - m_cap.get_exp_pos_with_leader().pose.position.x + bias_x) > 0.1 ||
                    abs(cur_pos.pose.position.y - m_cap.get_exp_pos_with_leader().pose.position.y + bias_y) > 0.1)
                {
                    hover_pose.pose.position.x = m_cap.get_exp_pos_with_leader().pose.position.x - bias_x;
                    hover_pose.pose.position.y = m_cap.get_exp_pos_with_leader().pose.position.y - bias_y;
                    hover_pose.pose.position.z = height;
                    local_pos_pub.publish(hover_pose);
                    count = 0;
                }
                else
                {
                    if (count < 10)
                    {
                        local_pos_pub.publish(hover_pose);
                    }
                    else
                    {
                        init_flag = true;
                        ROS_INFO("Move To done.");
                        hover_pose.pose.position.x = cur_pos.pose.position.x;
                        hover_pose.pose.position.y = cur_pos.pose.position.y;
                        hover_pose.pose.position.z = height;
                    }
                    std::cout << count << std::endl;
                }
            }
            if (init_flag)
            {
                // std::cout << "current mode is: " << mode_sel << std::endl;
                switch (mode_sel)
                {
                default:
                    ROS_INFO("Current Mode: Default.");
                    u_ctrl.setState(cur_pos, cur_vel, hover_pose, m_cap.get_zero_vel());
                    break;
                case '1':
                    ROS_INFO("Current Mode: Tracking.");
                    m_cap.set_phi(phi);
                    temp_pos.pose.position.x = m_cap.get_exp_pos_with_leader().pose.position.x - bias_x;
                    temp_pos.pose.position.y = m_cap.get_exp_pos_with_leader().pose.position.y - bias_y;
                    u_ctrl.setState(cur_pos, cur_vel, temp_pos, m_cap.get_trac_exp_vel_with_leader());
                    break;
                case '2':
                    ROS_INFO("Current Mode: Capture.");
                    m_cap.set_phi(phi);
                    temp_pos.pose.position.x = m_cap.get_exp_pos_with_leader().pose.position.x - bias_x;
                    temp_pos.pose.position.y = m_cap.get_exp_pos_with_leader().pose.position.y - bias_y;
                    u_ctrl.setState(cur_pos, cur_vel, temp_pos, m_cap.get_cap_exp_vel_with_leader());
                    hover_pose = cur_pos;
                    k++;
                    break;
                }
                geometry_msgs::Vector3Stamped ctrl_accel = u_ctrl.getCtrl_robust();
                mpCtrl.acceleration_or_force.x = ctrl_accel.vector.x + m_cap.get_exp_acc().vector.x; // 前馈
                mpCtrl.acceleration_or_force.y = ctrl_accel.vector.y + m_cap.get_exp_acc().vector.y;
                mpCtrl.acceleration_or_force.z = ctrl_accel.vector.z;
                local_accel_pub.publish(mpCtrl);
            }
            err_msgs.data = {static_cast<float>(cur_pos.pose.position.x - m_cap.get_exp_pos_with_leader().pose.position.x + bias_x),
                             static_cast<float>(cur_pos.pose.position.y - m_cap.get_exp_pos_with_leader().pose.position.y + bias_y),
                             static_cast<float>(cur_pos.pose.position.z - m_cap.get_exp_pos_with_leader().pose.position.z),
                             static_cast<float>(cur_vel.twist.linear.x - m_cap.get_cap_exp_vel_with_leader().twist.linear.x),
                             static_cast<float>(cur_vel.twist.linear.y - m_cap.get_cap_exp_vel_with_leader().twist.linear.y),
                             static_cast<float>(cur_vel.twist.linear.z - m_cap.get_cap_exp_vel_with_leader().twist.linear.z)};
            err_pub.publish(err_msgs);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
