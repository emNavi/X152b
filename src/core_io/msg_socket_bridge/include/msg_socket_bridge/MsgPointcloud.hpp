#ifndef __MSG_POINTCLOUD_HPP_
#define __MSG_POINTCLOUD_HPP_
// TODO


#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/TopicEnum.hpp"
#include "msg_socket_bridge/BaseMsg.hpp"


#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #define MSG_HEADER_SERIALIZE_PROCESS_CODE(header)do { \
//         ptr += serialize_one_member<uint32_t>(header.seq,ptr);\
//         ptr += serialize_one_member<double>(header.stamp.toSec(),ptr);\
//         ptr += serialize_one_member<size_t>(header.frame_id.length(),ptr);\
//         ptr += serialize_one_member<void *>((void *)header.frame_id.c_str(),header.frame_id.length(),ptr);\
// } while(0)

class MsgPointcloud:public BaseMsg<sensor_msgs::PointCloud2Ptr,sensor_msgs::PointCloud2,BUF_LEN_LONG>
{
private:
    /* data */
public:
    MsgPointcloud(ros::NodeHandle &nh,TOPIC_ENUM msg_type,int send_fd,struct sockaddr_in send_addr,std::string send_topic,std::string recv_topic)
    : BaseMsg(nh,msg_type,send_fd,send_addr,send_topic,recv_topic) {}

    int serialize(const sensor_msgs::PointCloud2Ptr &msg)
    {
        char *ptr = send_buf_;

        // header
        // MSG_HEADER_SERIALIZE_PROCESS_CODE(msg->header);
        ptr += serialize_one_member<uint32_t>(msg->header.seq,ptr);
        ptr += serialize_one_member<double>(msg->header.stamp.toSec(),ptr);

        ptr += serialize_one_member<size_t>(msg->header.frame_id.length(),ptr);
        ptr += serialize_one_member<void *>((void *)msg->header.frame_id.c_str(),msg->header.frame_id.length(),ptr);

        // height
        ptr += serialize_one_member<uint32_t>(msg->height,ptr);

        // width 
        ptr += serialize_one_member<uint32_t>(msg->width,ptr);

        // fields
        for (size_t j = 0; j < msg->fields.size(); j++)
        {
            ptr += serialize_one_member<size_t>(msg->fields.at(j).name.length(),ptr);
            ptr += serialize_one_member<void *>((void *)msg->header.frame_id.c_str(),msg->fields.at(j).name.length(),ptr);    
            ptr += serialize_one_member<uint32_t>( msg->fields.at(j).offset,ptr);
            ptr += serialize_one_member<uint8_t>( msg->fields.at(j).datatype,ptr);
            ptr += serialize_one_member<uint32_t>( msg->fields.at(j).count,ptr);
        }
        // is_bigendian
        ptr += serialize_one_member<uint8_t>(msg->is_bigendian,ptr);
        // point_step
        ptr += serialize_one_member<uint32_t>( msg->point_step,ptr);
        // row_step
        ptr += serialize_one_member<uint32_t>( msg->row_step,ptr);
        // data
        for (size_t j = 0; j < msg->data.size(); j++)
        {
            ptr += serialize_one_member<uint8_t>(msg->data.at(j),ptr);
        }
        // is_dense
        ptr += serialize_one_member<uint8_t>(msg->is_dense,ptr);

        return ptr - send_buf_; //length
    }
    int deserialize(sensor_msgs::PointCloud2Ptr &msg)
    {
        char *ptr = recv_buf_;

        // header
        std::string frame_id;
        ptr += deserialize_one_string(&frame_id,ptr);
        msg->header.frame_id= frame_id;
        ptr += deserialize_one_member<uint32_t>(&msg->header.seq,ptr);
        double sec;
        ptr += deserialize_one_member<double>(&sec,ptr);
        msg->header.stamp.fromSec(sec);

        // height && width
        ptr += deserialize_one_member<uint32_t>(&msg->height,ptr);
        ptr += deserialize_one_member<uint32_t>(&msg->width,ptr);

        // fields
        size_t len;
        ptr += deserialize_one_member<size_t>(&len,ptr);
        for (size_t j = 0; j < len; j++)
        {
            ptr += deserialize_one_string(&msg->fields.at(j).name,ptr);
            ptr += deserialize_one_member<uint32_t>(&msg->fields.at(j).offset,ptr);
            ptr += deserialize_one_member<uint8_t>(&msg->fields.at(j).datatype,ptr);
            ptr += deserialize_one_member<uint32_t>(&msg->fields.at(j).count,ptr);
        }
        // is_bigendian
        ptr += deserialize_one_member<uint8_t>(&msg->is_bigendian,ptr);
        // point_step
        ptr += deserialize_one_member<uint32_t>(&msg->point_step,ptr);
        // row_step
        ptr += deserialize_one_member<uint32_t>(&msg->row_step,ptr);
        // data
        for (size_t j = 0; j < msg->data.size(); j++)
        {
            ptr += deserialize_one_member<uint8_t>(&msg->data.at(j),ptr);
        }
        // is_dense
        ptr += deserialize_one_member<uint8_t>(&msg->is_dense,ptr);


        return ptr - recv_buf_;
    }


};

#endif