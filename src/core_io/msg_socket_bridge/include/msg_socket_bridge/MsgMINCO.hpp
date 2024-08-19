#ifndef __MSG_MINCO_
#define __MSG_MINCO_

#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <msg_socket_bridge/MINCOTraj.h>

class MsgMINCO:public BaseMsg<msg_socket_bridge::MINCOTrajPtr,msg_socket_bridge::MINCOTraj,BUF_LEN_SHORT>
{
private:
    /* data */
public:

    MsgMINCO(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}
    
    int serialize(const msg_socket_bridge::MINCOTrajPtr &msg)
    {
        char *ptr = send_buf_;
        
        ptr += serialize_one_member<int16_t>(msg->drone_id,ptr);
        ptr += serialize_one_member<int32_t>(msg->traj_id,ptr);

        ptr += serialize_one_member<double>(msg->start_time.toSec(),ptr);
        ptr += serialize_one_member<float>(msg->des_clearance,ptr);

        ptr += serialize_one_member<uint8_t>(msg->order,ptr);

        ptr += serialize_one_member<float>(msg->start_p[0],ptr);
        ptr += serialize_one_member<float>(msg->start_p[1],ptr);
        ptr += serialize_one_member<float>(msg->start_p[2],ptr);

        ptr += serialize_one_member<float>(msg->start_v[0],ptr);
        ptr += serialize_one_member<float>(msg->start_v[1],ptr);
        ptr += serialize_one_member<float>(msg->start_v[2],ptr);

        ptr += serialize_one_member<float>(msg->start_a[0],ptr);
        ptr += serialize_one_member<float>(msg->start_a[1],ptr);
        ptr += serialize_one_member<float>(msg->start_a[2],ptr);

        ptr += serialize_one_member<float>(msg->end_p[0],ptr);
        ptr += serialize_one_member<float>(msg->end_p[1],ptr);
        ptr += serialize_one_member<float>(msg->end_p[2],ptr);

        ptr += serialize_one_member<float>(msg->end_v[0],ptr);
        ptr += serialize_one_member<float>(msg->end_v[1],ptr);
        ptr += serialize_one_member<float>(msg->end_v[2],ptr);

        ptr += serialize_one_member<float>(msg->end_a[0],ptr);
        ptr += serialize_one_member<float>(msg->end_a[1],ptr);
        ptr += serialize_one_member<float>(msg->end_a[2],ptr);

        ptr += serialize_one_member<size_t>(msg->inner_x.size(),ptr);
        for (size_t j = 0; j < msg->inner_x.size(); j++)
        {
            ptr += serialize_one_member<float>(msg->inner_x[j],ptr);
        }

        ptr += serialize_one_member<size_t>(msg->inner_y.size(),ptr);
        for (size_t j = 0; j < msg->inner_y.size(); j++)
        {
            ptr += serialize_one_member<float>(msg->inner_y[j],ptr);
        }

        ptr += serialize_one_member<size_t>(msg->inner_z.size(),ptr);
        for (size_t j = 0; j < msg->inner_z.size(); j++)
        {
            ptr += serialize_one_member<float>(msg->inner_z[j],ptr);
        }

        ptr += serialize_one_member<size_t>(msg->duration.size(),ptr);
        for (size_t j = 0; j < msg->duration.size(); j++)
        {
            ptr += serialize_one_member<float>(msg->duration[j],ptr);
        }
        return ptr - send_buf_; //length
    }
    int deserialize(msg_socket_bridge::MINCOTrajPtr &msg)
    {
        char *ptr = recv_buf_;
 
        ptr += deserialize_one_member<int16_t>(&msg->drone_id,ptr);
        ptr += deserialize_one_member<int32_t>(&msg->traj_id,ptr);
        double sec;
        ptr += deserialize_one_member<double>(&sec,ptr);
        msg->start_time.fromSec(sec);

        ptr += deserialize_one_member<float>(&msg->des_clearance,ptr);
        ptr += deserialize_one_member<uint8_t>(&msg->order,ptr);

        ptr += deserialize_one_member<float>(&msg->start_p[0],ptr);
        ptr += deserialize_one_member<float>(&msg->start_p[1],ptr);
        ptr += deserialize_one_member<float>(&msg->start_p[2],ptr);

        ptr += deserialize_one_member<float>(&msg->start_v[0],ptr);
        ptr += deserialize_one_member<float>(&msg->start_v[1],ptr);
        ptr += deserialize_one_member<float>(&msg->start_v[2],ptr);

        ptr += deserialize_one_member<float>(&msg->start_a[0],ptr);
        ptr += deserialize_one_member<float>(&msg->start_a[1],ptr);
        ptr += deserialize_one_member<float>(&msg->start_a[2],ptr);

        ptr += deserialize_one_member<float>(&msg->end_p[0],ptr);
        ptr += deserialize_one_member<float>(&msg->end_p[1],ptr);
        ptr += deserialize_one_member<float>(&msg->end_p[2],ptr);

        ptr += deserialize_one_member<float>(&msg->end_v[0],ptr);
        ptr += deserialize_one_member<float>(&msg->end_v[1],ptr);
        ptr += deserialize_one_member<float>(&msg->end_v[2],ptr);

        ptr += deserialize_one_member<float>(&msg->end_a[0],ptr);
        ptr += deserialize_one_member<float>(&msg->end_a[1],ptr);
        ptr += deserialize_one_member<float>(&msg->end_a[2],ptr);

        size_t inner_size_x;
        ptr += deserialize_one_member<size_t>(&inner_size_x,ptr);
        msg->inner_x.resize(inner_size_x);
        for (size_t j = 0; j < msg->inner_x.size(); j++)
        {
            ptr += deserialize_one_member<float>(&msg->inner_x[j],ptr);
        }

        size_t inner_size_y;
        ptr += deserialize_one_member<size_t>(&inner_size_y,ptr);
        msg->inner_y.resize(inner_size_y);
        for (size_t j = 0; j < msg->inner_y.size(); j++)
        {
            ptr += deserialize_one_member<float>(&msg->inner_y[j],ptr);
        }

        size_t inner_size_z;
        ptr += deserialize_one_member<size_t>(&inner_size_z,ptr);
        msg->inner_z.resize(inner_size_z);
        for (size_t j = 0; j < msg->inner_z.size(); j++)
        {
            ptr += deserialize_one_member<float>(&msg->inner_z[j],ptr);
        }

        size_t size_duration;
        ptr += deserialize_one_member<size_t>(&size_duration,ptr);
        msg->duration.resize(size_duration);
        for (size_t j = 0; j < msg->duration.size(); j++)
        {
            ptr += deserialize_one_member<float>(&msg->duration[j],ptr);
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