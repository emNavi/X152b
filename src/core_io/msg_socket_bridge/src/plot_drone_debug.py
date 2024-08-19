#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray,Int16
import rospy
import sys
import yaml
import time
import socket
import json
import math
from mavros_msgs.msg import AttitudeTarget,PositionTarget
from nav_msgs.msg import Odometry

atti_target_msg = AttitudeTarget()
odom_msg = Odometry()
pos_target_msg = PositionTarget()

first_run_atti_target = True
first_run_local_odom = True
first_run_pos_target = True


def atti_target_cb(msg):
    global atti_target_msg,first_run_atti_target
    atti_target_msg = msg
    first_run_atti_target = False
    
def local_odom_cb(msg):
    global odom_msg,first_run_local_odom
    odom_msg = msg
    first_run_local_odom = False

def pos_target_cb(msg):
    global pos_target_msg,first_run_pos_target
    pos_target_msg = msg
    first_run_pos_target = False

def get_rpy(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    dcm20 = 2 * (x * z - w * y)
    pitch = math.asin(-dcm20)

    dcm10 = 2 * (x * y + w * z)
    dcm00 = w*w + x*x - y*y - z*z
    yaw = math.atan(dcm10/ dcm00)

    dcm21 = 2 * (w * x + y * z)
    dcm22 = w*w - x*x - y*y + z*z
    roll = math.atan(dcm21/dcm22)

    return [roll,pitch,yaw]



if __name__ == '__main__':
    rospy.init_node('plot_drone_debug', anonymous=True)

    rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, atti_target_cb)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, local_odom_cb)
    rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, pos_target_cb)
    address = '10.42.0.1'
    port = 9870
    sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
    rate = rospy.Rate(200) # hz
    while not rospy.is_shutdown():
        if first_run_atti_target or first_run_local_odom or first_run_atti_target:
            continue
        data = {
            "atti_target_msg": {
                "timestamp": atti_target_msg.header.stamp.to_sec(),
                "body_rate": {
                    "x": atti_target_msg.body_rate.x,
                    "y": atti_target_msg.body_rate.y,
                    "z": atti_target_msg.body_rate.z,
                },
                "angle": {
                    "roll": get_rpy(atti_target_msg.orientation)[0],
                    "pitch": get_rpy(atti_target_msg.orientation)[1],
                    "yaw": get_rpy(atti_target_msg.orientation)[2],
                },
                "thrust": atti_target_msg.thrust
            },
            "odom_msg": {
                "timestamp": odom_msg.header.stamp.to_sec(),
                "pos":{
                    "x": odom_msg.pose.pose.position.x,
                    "y": odom_msg.pose.pose.position.y,
                    "z": odom_msg.pose.pose.position.z,
                },
                "angular": {
                    "roll": get_rpy(odom_msg.pose.pose.orientation)[0],
                    "pitch": get_rpy(odom_msg.pose.pose.orientation)[1],
                    "yaw": get_rpy(odom_msg.pose.pose.orientation)[2],
                },
                "linear_vel":{
                    "x": odom_msg.twist.twist.linear.x,
                    "y": odom_msg.twist.twist.linear.y,
                    "z": odom_msg.twist.twist.linear.z,
                },
                "angular_vel": {
                    "x": odom_msg.twist.twist.angular.x,
                    "y": odom_msg.twist.twist.angular.y,
                    "z": odom_msg.twist.twist.angular.z,
                },
            },
            "pos_target_msg": {
                "timestamp": pos_target_msg.header.stamp.to_sec(),
                "pos": {
                    "x": pos_target_msg.position.x,
                    "y": pos_target_msg.position.y,
                    "z": pos_target_msg.position.z,
                },
                "vel": {
                    "x": pos_target_msg.velocity.x,
                    "y": pos_target_msg.velocity.y,
                    "z": pos_target_msg.velocity.z,
                },
                "acc": {
                    "x": pos_target_msg.acceleration_or_force.x,
                    "y": pos_target_msg.acceleration_or_force.y,
                    "z": pos_target_msg.acceleration_or_force.z,
                },
                "yaw": pos_target_msg.yaw
            },
            "delta_x": pos_target_msg.position.x - odom_msg.pose.pose.position.x,
            "delta_y": pos_target_msg.position.y - odom_msg.pose.pose.position.y,
            "delta_z": pos_target_msg.position.z - odom_msg.pose.pose.position.z,
            "delta_ang_x": get_rpy(atti_target_msg.orientation)[0] - get_rpy(odom_msg.pose.pose.orientation)[0],
            "delta_ang_y": get_rpy(atti_target_msg.orientation)[1] - get_rpy(odom_msg.pose.pose.orientation)[1],
            "delta_ang_z": get_rpy(atti_target_msg.orientation)[2] - get_rpy(odom_msg.pose.pose.orientation)[2],   

        }
        # print(data)
        sock.sendto( json.dumps(data).encode(), (address, port) )
        rate.sleep()
    rospy.spin()  # 保持ROS节点运行
