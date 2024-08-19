/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <tf/tf.h>

#define ROS_RATE 50.0
std::string s_uav_id = "000";
int i_uav_id = std::stoi(s_uav_id);

int num_of_uav = 2;
float bias_x = 3;
float bias_y = 0;

float P_param[3] = {-2, -2, -2};
float I_param[3] = {-0.2, -0.2, -0.2};
float D_param[3] = {-0.8, -0.8, -0.8};

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

geometry_msgs::Vector3Stamped ctrl_accel;
geometry_msgs::Vector3Stamped pidController(geometry_msgs::PoseStamped expected_pose,
                                            geometry_msgs::TwistStamped expected_velocity,
                                            geometry_msgs::Vector3Stamped expected_accel)
{
    float err_x = cur_pos.pose.position.x - expected_pose.pose.position.x;
    float err_y = cur_pos.pose.position.y - expected_pose.pose.position.y;
    float err_z = cur_pos.pose.position.z - expected_pose.pose.position.z;
    float err_vx = cur_vel.twist.linear.x - expected_velocity.twist.linear.x;
    float err_vy = cur_vel.twist.linear.y - expected_velocity.twist.linear.y;
    float err_vz = cur_vel.twist.linear.z - expected_velocity.twist.linear.z;

    float unc_x = P_param[0] * err_x + D_param[0] * err_vx + expected_accel.vector.x;
    float unc_y = P_param[1] * err_y + D_param[1] * err_vy + expected_accel.vector.y;
    float unc_z = P_param[2] * err_z + D_param[2] * err_vz; // + i_z * integration_z;

    ctrl_accel.vector.x = unc_x;
    ctrl_accel.vector.y = unc_y;
    ctrl_accel.vector.z = unc_z;
    return ctrl_accel;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav" + s_uav_id + "capture_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav" + s_uav_id + "/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav" + s_uav_id + "/mavros/local_position/velocity_local", 10, vel_cb);
    ros::Subscriber mode_sub = nh.subscribe("mode_select", 1000, mode_sel_cb);

    ros::Subscriber leader_pose_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1000, leader_state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav" + s_uav_id + "/mavros/setpoint_position/local", 10);
    ros::Publisher local_accel_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav" + s_uav_id + "/mavros/setpoint_raw/local", 10);
    ros::Publisher err_pub = nh.advertise<std_msgs::Float64MultiArray>("/uav" + s_uav_id + "/errors", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav" + s_uav_id + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav" + s_uav_id + "/mavros/set_mode");
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    // wait for FCU connection
    while (ros::ok() && !cur_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    float omega = M_PI / 32;
    float k = 0;
    float height = 2;
    float radius = 3;
    bool takeoff_flag = false;

    float dt = 1 / ROS_RATE;

    float phi = 2 * M_PI / num_of_uav;

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

    // leader_cur_state.pose.pose.position.x = 0;
    // leader_cur_state.pose.pose.position.y = 0;
    // leader_cur_state.pose.pose.position.z = 0;
    // leader_cur_state.twist.twist.linear.x = 0;
    // leader_cur_state.twist.twist.linear.y = 0;
    // leader_cur_state.twist.twist.linear.z = 0;
    mavros_msgs::PositionTarget mpCtrl;
    mpCtrl.type_mask = mpCtrl.IGNORE_YAW_RATE + mpCtrl.IGNORE_PX + mpCtrl.IGNORE_PY + mpCtrl.IGNORE_PZ + mpCtrl.IGNORE_VX + mpCtrl.IGNORE_VY + mpCtrl.IGNORE_VZ;
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
                    ctrl_accel = pidController(exp_pos, exp_vel, exp_acc);
                    break;
                case '1':
                    ROS_INFO("Current Mode: Tracking.");
                    exp_pos.pose.position.x = leader_cur_state.pose.pose.position.x + radius * cos(omega * k * dt + phi * i_uav_id);
                    exp_pos.pose.position.y = leader_cur_state.pose.pose.position.y + radius * sin(omega * k * dt + phi * i_uav_id);
                    exp_pos.pose.position.z = leader_cur_state.pose.pose.position.z + height;
                    exp_vel.twist.linear.x = leader_cur_state.twist.twist.linear.x;
                    exp_vel.twist.linear.y = leader_cur_state.twist.twist.linear.y;
                    exp_vel.twist.linear.z = 0;
                    exp_acc.vector.x = 0;
                    exp_acc.vector.y = 0;
                    exp_acc.vector.z = 0;
                    ctrl_accel = pidController(exp_pos, exp_vel, exp_acc);
                    break;
                case '2':
                    ROS_INFO("Current Mode: Capture.");
                    exp_pos.pose.position.x = leader_cur_state.pose.pose.position.x + radius * cos(omega * k * dt + phi * i_uav_id);
                    exp_pos.pose.position.y = leader_cur_state.pose.pose.position.y + radius * sin(omega * k * dt + phi * i_uav_id);
                    exp_pos.pose.position.z = leader_cur_state.pose.pose.position.z + height;
                    exp_vel.twist.linear.x = leader_cur_state.twist.twist.linear.x - omega * radius * sin(omega * k * dt + phi * i_uav_id);
                    exp_vel.twist.linear.y = leader_cur_state.twist.twist.linear.y + omega * radius * cos(omega * k * dt + phi * i_uav_id);
                    exp_vel.twist.linear.z = 0;
                    exp_acc.vector.x = 0;
                    exp_acc.vector.y = 0;
                    exp_acc.vector.z = 0;
                    k++;
                    ctrl_accel = pidController(exp_pos, exp_vel, exp_acc);

                    hover_pose.pose.position = cur_pos.pose.position;
                    hover_pose.pose.position.z = height;
                    break;
                }
                mpCtrl.acceleration_or_force.x = ctrl_accel.vector.x;
                mpCtrl.acceleration_or_force.y = ctrl_accel.vector.y;
                mpCtrl.acceleration_or_force.z = ctrl_accel.vector.z;

                local_accel_pub.publish(mpCtrl);

                err_msgs.data = {cur_pos.pose.position.x - exp_pos.pose.position.x,
                                 cur_pos.pose.position.y - exp_pos.pose.position.y,
                                 cur_pos.pose.position.z - exp_pos.pose.position.z,
                                 cur_vel.twist.linear.x - exp_vel.twist.linear.x,
                                 cur_vel.twist.linear.y - exp_vel.twist.linear.y,
                                 cur_vel.twist.linear.z - exp_vel.twist.linear.z};
                err_pub.publish(err_msgs);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}