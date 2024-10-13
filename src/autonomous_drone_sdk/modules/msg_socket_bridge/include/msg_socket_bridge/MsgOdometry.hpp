#ifndef __MSG_ODOMETRY_
#define __MSG_ODOMETRY_

#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"
#include <nav_msgs/Odometry.h>

class MsgOdometry:public BaseMsg<nav_msgs::OdometryPtr,nav_msgs::Odometry,BUF_LEN_SHORT>
{
private:
    /* data */
public:
    MsgOdometry(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}
    
    int serialize(const nav_msgs::OdometryPtr &msg)
    {
        char *ptr = send_buf_;

        // child_frame_id
        ptr += serialize_one_string(msg->child_frame_id,ptr);
        // header
        ptr += serialize_one_string(msg->header.frame_id,ptr);
        ptr += serialize_one_member<uint32_t>(msg->header.seq,ptr);
        ptr += serialize_one_member<double>(msg->header.stamp.toSec(),ptr);


        ptr += serialize_one_member<double>(msg->pose.pose.position.x,ptr);
        ptr += serialize_one_member<double>(msg->pose.pose.position.y,ptr);
        ptr += serialize_one_member<double>(msg->pose.pose.position.z,ptr);

        ptr += serialize_one_member<double>(msg->pose.pose.orientation.w,ptr);
        ptr += serialize_one_member<double>(msg->pose.pose.orientation.x,ptr);
        ptr += serialize_one_member<double>(msg->pose.pose.orientation.y,ptr);
        ptr += serialize_one_member<double>(msg->pose.pose.orientation.z,ptr);

        for (size_t j = 0; j < 36; j++)
        {
            ptr += serialize_one_member<double>( msg->pose.covariance[j],ptr);
        }


        ptr += serialize_one_member<double>(msg->twist.twist.linear.x,ptr);
        ptr += serialize_one_member<double>(msg->twist.twist.linear.y,ptr);
        ptr += serialize_one_member<double>(msg->twist.twist.linear.z,ptr);

        ptr += serialize_one_member<double>(msg->twist.twist.angular.x,ptr);
        ptr += serialize_one_member<double>(msg->twist.twist.angular.y,ptr);
        ptr += serialize_one_member<double>(msg->twist.twist.angular.z,ptr);

        for (size_t j = 0; j < 36; j++)
        {
            ptr += serialize_one_member<double>(msg->twist.covariance[j],ptr);
        }

        return ptr - send_buf_; //length
    }
    int deserialize(nav_msgs::OdometryPtr &msg)
    {
        char *ptr = recv_buf_;

        size_t len;

        // child_frame_id
        ptr += deserialize_one_string(&msg->child_frame_id,ptr);
        // header
        ptr += deserialize_one_string(&msg->header.frame_id,ptr);
        ptr += deserialize_one_member<uint32_t>(&msg->header.seq,ptr);

        double sec;
        ptr += deserialize_one_member<double>(&sec,ptr);
        msg->header.stamp.fromSec(sec);

        ptr += deserialize_one_member<double>(&msg->pose.pose.position.x,ptr);
        ptr += deserialize_one_member<double>(&msg->pose.pose.position.y,ptr);
        ptr += deserialize_one_member<double>(&msg->pose.pose.position.z,ptr);

        ptr += deserialize_one_member<double>(&msg->pose.pose.orientation.w,ptr);
        ptr += deserialize_one_member<double>(&msg->pose.pose.orientation.x,ptr);
        ptr += deserialize_one_member<double>(&msg->pose.pose.orientation.y,ptr);
        ptr += deserialize_one_member<double>(&msg->pose.pose.orientation.z,ptr);

        for (size_t j = 0; j < 36; j++)
        {
            ptr += deserialize_one_member<double>(&msg->pose.covariance[j],ptr);
        }

        ptr += deserialize_one_member<double>(&msg->twist.twist.linear.x,ptr);
        ptr += deserialize_one_member<double>(&msg->twist.twist.linear.y,ptr);
        ptr += deserialize_one_member<double>(&msg->twist.twist.linear.z,ptr);

        ptr += deserialize_one_member<double>(&msg->twist.twist.angular.x,ptr);
        ptr += deserialize_one_member<double>(&msg->twist.twist.angular.y,ptr);
        ptr += deserialize_one_member<double>(&msg->twist.twist.angular.z,ptr);


        for (size_t j = 0; j < 36; j++)
        {
            ptr += deserialize_one_member<double>(&msg->twist.covariance[j],ptr);
        }

        return ptr - recv_buf_;
    }


};
#endif

// recv_buf_= send_buf_;
// std::copy(send_buf_, send_buf_ +16, recv_buf_); // 使用 std::copy 复制数组内容
// std_msgs::Float32MultiArrayPtr dmsg;
// dmsg.reset(new std_msgs::Float32MultiArray);
// deserialize(dmsg);