#ifndef __MSG_BSPLINE_
#define __MSG_BSPLINE_

#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <msg_socket_bridge/Bspline.h>

class MsgBspline:public BaseMsg<msg_socket_bridge::BsplinePtr,msg_socket_bridge::Bspline,BUF_LEN_SHORT>
{
private:
    /* data */
public:

    MsgBspline(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}
    
    int serialize(const msg_socket_bridge::BsplinePtr &msg)
    {
        char *ptr = send_buf_;
        
        ptr += serialize_one_member<int32_t>(msg->drone_id,ptr);
        ptr += serialize_one_member<int32_t>(msg->order,ptr);

        ptr += serialize_one_member<double>(msg->start_time.toSec(),ptr);
        ptr += serialize_one_member<int64_t>(msg->traj_id,ptr);

        ptr += serialize_one_member<double>(msg->yaw_dt,ptr);

        ptr += serialize_one_member<size_t>(msg->knots.size(),ptr);
        for (size_t j = 0; j < msg->knots.size(); j++)
        {
            ptr += serialize_one_member<double>(msg->knots[j],ptr);
        }

        ptr += serialize_one_member<size_t>(msg->pos_pts.size(),ptr);

        for (size_t j = 0; j < msg->pos_pts.size(); j++)
        {
            ptr += serialize_one_member<double>(msg->pos_pts[j].x,ptr);
            ptr += serialize_one_member<double>(msg->pos_pts[j].y,ptr);
            ptr += serialize_one_member<double>(msg->pos_pts[j].z,ptr);
        }

        ptr += serialize_one_member<size_t>(msg->yaw_pts.size(),ptr);

        for (size_t j = 0; j < msg->yaw_pts.size(); j++)
        {
            ptr += serialize_one_member<double>(msg->yaw_pts[j],ptr);
        }
        return ptr - send_buf_; //length
    }
    int deserialize(msg_socket_bridge::BsplinePtr &msg)
    {
        char *ptr = recv_buf_;
 
        ptr += deserialize_one_member<int32_t>(&msg->drone_id,ptr);
        ptr += deserialize_one_member<int32_t>(&msg->order,ptr);
        double sec;
        ptr += deserialize_one_member<double>(&sec,ptr);
        msg->start_time.fromSec(sec);

        ptr += deserialize_one_member<int64_t>(&msg->traj_id,ptr);
        ptr += deserialize_one_member<double>(&msg->yaw_dt,ptr);


        size_t knots_num;
        ptr += deserialize_one_member<size_t>(&knots_num,ptr);
        msg->knots.resize(knots_num);

        for (size_t j = 0; j < msg->knots.size(); j++)
        {
            ptr += deserialize_one_member<double>(&msg->knots[j],ptr);
        }
        size_t pos_pts_num;
        ptr += deserialize_one_member<size_t>(&pos_pts_num,ptr);
        msg->pos_pts.resize(pos_pts_num);

        for (size_t j = 0; j < msg->pos_pts.size(); j++)
        {
            ptr += deserialize_one_member<double>(&msg->pos_pts[j].x,ptr);
            ptr += deserialize_one_member<double>(&msg->pos_pts[j].y,ptr);
            ptr += deserialize_one_member<double>(&msg->pos_pts[j].z,ptr);

        }

        size_t yaw_pts_num;
        ptr += deserialize_one_member<size_t>(&yaw_pts_num,ptr);
        msg->yaw_pts.resize(yaw_pts_num);

        for (size_t j = 0; j < msg->yaw_pts.size(); j++)
        {
            ptr += deserialize_one_member<double>(&msg->yaw_pts[j],ptr);
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