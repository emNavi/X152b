#ifndef __MSG_MAVROS_POSITION_TARGET_
#define __MSG_MAVROS_POSITION_TARGET_

#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"
#include <msg_socket_bridge/PositionTarget.h>

class MsgMavrosPositionTarget:public BaseMsg<msg_socket_bridge::PositionTargetPtr,msg_socket_bridge::PositionTarget,BUF_LEN_SHORT>
{
private:
    /* data */
public:

    MsgMavrosPositionTarget(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}
    
    int serialize(const msg_socket_bridge::PositionTargetPtr &msg)
    {
        char *ptr = send_buf_;
        
        // header
        ptr += serialize_one_string(msg->header.frame_id,ptr);
        ptr += serialize_one_member<uint32_t>(msg->header.seq,ptr);
        ptr += serialize_one_member<double>(msg->header.stamp.toSec(),ptr);

        // coordinate_frame
        ptr += serialize_one_member<uint8_t>(msg->coordinate_frame,ptr);
        // type_mask
        ptr += serialize_one_member<uint16_t>(msg->type_mask,ptr);
        // position geometry_msgs/Point
        ptr += serialize_one_member<double>(msg->position.x,ptr);
        ptr += serialize_one_member<double>(msg->position.y,ptr);
        ptr += serialize_one_member<double>(msg->position.z,ptr);
        // velocity geometry_msgs/Vector3
        ptr += serialize_one_member<double>(msg->velocity.x,ptr);
        ptr += serialize_one_member<double>(msg->velocity.y,ptr);
        ptr += serialize_one_member<double>(msg->velocity.z,ptr);
        // acceleration_or_force geometry_msgs/Vector3
        ptr += serialize_one_member<double>(msg->acceleration_or_force.x,ptr);
        ptr += serialize_one_member<double>(msg->acceleration_or_force.y,ptr);
        ptr += serialize_one_member<double>(msg->acceleration_or_force.z,ptr);
        
        ptr += serialize_one_member<float>(msg->yaw,ptr);
        ptr += serialize_one_member<float>(msg->yaw_rate,ptr);
        
        return ptr - send_buf_; //length
    }
    int deserialize(msg_socket_bridge::PositionTargetPtr &msg)
    {
        char *ptr = recv_buf_;
 
        
        // header
        ptr += deserialize_one_string(&msg->header.frame_id,ptr);
        ptr += deserialize_one_member<uint32_t>(&msg->header.seq,ptr);

        double sec;
        ptr += deserialize_one_member<double>(&sec,ptr);
        msg->header.stamp.fromSec(sec);


        // coordinate_frame
        ptr += deserialize_one_member<uint8_t>(&msg->coordinate_frame,ptr);
        // type_mask
        ptr += deserialize_one_member<uint16_t>(&msg->type_mask,ptr);
        // position geometry_msgs/Point
        ptr += deserialize_one_member<double>(&msg->position.x,ptr);
        ptr += deserialize_one_member<double>(&msg->position.y,ptr);
        ptr += deserialize_one_member<double>(&msg->position.z,ptr);
        // velocity geometry_msgs/Vector3
        ptr += deserialize_one_member<double>(&msg->velocity.x,ptr);
        ptr += deserialize_one_member<double>(&msg->velocity.y,ptr);
        ptr += deserialize_one_member<double>(&msg->velocity.z,ptr);
        // acceleration_or_force geometry_msgs/Vector3
        ptr += deserialize_one_member<double>(&msg->acceleration_or_force.x,ptr);
        ptr += deserialize_one_member<double>(&msg->acceleration_or_force.y,ptr);
        ptr += deserialize_one_member<double>(&msg->acceleration_or_force.z,ptr);
        
        ptr += deserialize_one_member<float>(&msg->yaw,ptr);
        ptr += deserialize_one_member<float>(&msg->yaw_rate,ptr);




        return ptr - recv_buf_;
    }


};
#endif


// recv_buf_= send_buf_;
// std::copy(send_buf_, send_buf_ +16, recv_buf_); // 使用 std::copy 复制数组内容
// std_msgs::Float32MultiArrayPtr dmsg;
// dmsg.reset(new std_msgs::Float32MultiArray);
// deserialize(dmsg);