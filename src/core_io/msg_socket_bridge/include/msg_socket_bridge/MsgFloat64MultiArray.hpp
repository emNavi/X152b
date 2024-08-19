#ifndef __FLOAT_64_MULTI_ARRAY_MSG_
#define __FLOAT_64_MULTI_ARRAY_MSG_

#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"
#include <std_msgs/Float64MultiArray.h>

class MsgFloat64MultiArray:public BaseMsg<std_msgs::Float64MultiArrayPtr,std_msgs::Float64MultiArray,BUF_LEN_SHORT>
{
private:
    /* data */
public:

    MsgFloat64MultiArray(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}
    
    int serialize(const std_msgs::Float64MultiArrayPtr &msg)
    {
        char *ptr = send_buf_;

        size_t len = msg->data.size();
        ptr += serialize_one_member<size_t>(len,ptr);
        for(int i=0;i<len;i++)
        {
            ptr += serialize_one_member<double>(msg->data[i],ptr);
        }

        return ptr - send_buf_; //length
    }
    int deserialize(std_msgs::Float64MultiArrayPtr &msg)
    {
        char *ptr = recv_buf_;
        
        size_t len;
        ptr += deserialize_one_member<size_t>(&len,ptr);
        msg->data.resize(len);
        for(int i=0;i<len;i++)
        {
            ptr += deserialize_one_member<double>(&msg->data[i],ptr);
        }
        return ptr - recv_buf_;
    }


};
#endif