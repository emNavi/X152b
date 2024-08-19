#ifndef __BASE_MSG_HPP_
#define __BASE_MSG_HPP_
#include <string.h>
#include <ros/ros.h>
#include "msg_socket_bridge/TopicEnum.hpp"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

char socket_buf_[61000];

#pragma pack(1)
struct MsgHeader
{
    std::string topic_name;
    TOPIC_ENUM topic_id;
    int robot_id;
    uint32_t length;
    int package_id;
    int8_t segment_id;
    int8_t segment_num;

};
#pragma pack()

template <typename T_ptr, typename T, size_t L>
class BaseMsg
{
private:
    // int communication_mode_;
    std::string send_topic_;
    std::string recv_topic_;
    // bool is_master_;
    TOPIC_ENUM msg_type_;
    ros::NodeHandle nh_;

    ros::Time t_last_;
    T_ptr msg_ptr_;

protected:
    struct sockaddr_in send_addr_;
    uint16_t max_freq = 0;

    // std::random_device rd;                           // 用于生成随机种子
    // std::mt19937 mt(rd());                           // 使用 Mersenne Twister 生成器
    // std::uniform_int_distribution<int> dist(1, 100); // 生成范围在1到100之间的随机整数

public:
    int send_fd_;
    char recv_buf_[L], send_buf_[L];
    T_ptr recv_msg_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    BaseMsg(ros::NodeHandle &nh, TOPIC_ENUM msg_type, int fd, sockaddr_in send_addr, std::string send_topic, std::string recv_topic);
    ~BaseMsg();
    virtual void msg_cb(const T_ptr &msg)
    {
        // ROS_INFO("msg_cb");
        if (!check_freq())
        {
            return;
        }
        size_t data_len = serialize(msg);

        MsgHeader header;
        header.length = data_len; // 不包含header
        header.topic_name = send_topic_;
        header.package_id = 65535;
        header.topic_id = msg_type_;
        header.segment_id = 255;
        if (data_len < BUF_LEN_FULL)
        {
            memcpy(socket_buf_, (char *)&header, sizeof(MsgHeader));
            memcpy(socket_buf_ + sizeof(MsgHeader), send_buf_, data_len);
            // ROS_INFO_STREAM("socket_buf_len "<< data_len);
            if (sendto(send_fd_, socket_buf_, sizeof(MsgHeader) + data_len, 0, (struct sockaddr *)&send_addr_, sizeof(send_addr_)) <= 0)
            {
                ROS_ERROR("UDP SEND ERROR (short)!!!");
            }
        }
        else
        {
            for (int i = 0; i <= data_len / BUF_LEN_FULL; i++)
            {
                int send_len = (i == data_len / BUF_LEN_FULL) ? data_len - (i * BUF_LEN_FULL) : BUF_LEN_FULL;
                memcpy(socket_buf_, (char *)&header, sizeof(MsgHeader));
                memcpy(socket_buf_ + sizeof(MsgHeader), send_buf_+i*BUF_LEN_FULL, send_len);

                if (sendto(send_fd_, socket_buf_, send_len, 0, (struct sockaddr *)&send_addr_, sizeof(send_addr_)) <= 0)
                {
                    ROS_ERROR("UDP SEND ERROR (long)!!!");
                }
            }
        }
    };

    virtual int recv_process(size_t valread)
    {
        if (get_recv_topic() == "")
        {
            return 1;
        }
        // ROS_INFO_STREAM("start deserialize"<<valread);

        if (valread == deserialize(recv_msg_))
        {
            // ROS_INFO_STREAM("valread"<<valread);
            pub_.publish(*recv_msg_);
            return 0;
        }
        else
        {
            ROS_ERROR("Received message length not equal MSG_LEN!!!");
            return -1;
        }
    }

    virtual int serialize(const T_ptr &msg) = 0;
    virtual int deserialize(T_ptr &msg) = 0;

    template <typename U>
    size_t serialize_one_member(U data, char *ptr)
    {
        // TODO: 为什么不能只用形参的指针
        *((U *)ptr) = data;
        return sizeof(U);
    }

    template <typename U>
    size_t serialize_one_member(U data, size_t len, char *ptr)
    {
        // TODO: 为什么不能只用形参的指针
        //  *((U *)ptr) = data;msg->child_frame_id.c_str()
        memcpy((void *)ptr, (void *)data, len * sizeof(char));

        return len * sizeof(char);
    }

    template <typename U>
    size_t deserialize_one_member(U *data, char *ptr)
    {
        // TODO: 为什么不能只用形参的指针
        *data = *((U *)ptr);
        return sizeof(U);
    }
    size_t serialize_one_string(std::string data, char *ptr)
    {
        size_t len = data.size();
        ptr += serialize_one_member<size_t>(len, ptr);
        ptr += serialize_one_member<char *>((char *)data.c_str(), len, ptr);
        return sizeof(size_t) + len * sizeof(char);
    }
    size_t deserialize_one_string(std::string *data, char *ptr)
    {
        size_t len;
        ptr += deserialize_one_member<size_t>(&len, ptr);
        data->assign((const char *)ptr, len);
        return sizeof(size_t) + len * sizeof(char);
    }

    void set_max_freq(uint16_t max_f)
    {
        max_freq = max_f;
    }

    bool check_freq()
    {
        ros::Time t_now = ros::Time::now();
        if (max_freq == 0)
        {
            return true;
        }
        if ((t_now - t_last_).toSec() * max_freq < 1.0)
        {
            return false;
        }
        t_last_ = t_now;
        return true;
    }
    TOPIC_ENUM get_msg_type()
    {
        return msg_type_;
    }
    std::string get_recv_topic()
    {
        return recv_topic_;
    }
};

template <typename T_ptr, typename T, size_t L>
BaseMsg<T_ptr, T, L>::BaseMsg(ros::NodeHandle &nh, TOPIC_ENUM msg_type, int send_fd, sockaddr_in send_addr, std::string send_topic, std::string recv_topic)
{
    msg_ptr_.reset(new T);
    recv_msg_.reset(new T);
    send_fd_ = send_fd;
    nh_ = nh;
    msg_type_ = msg_type;
    send_topic_ = send_topic;
    recv_topic_ = recv_topic;
    send_addr_ = send_addr;
    if (send_topic_ != "")
    {
        // sub_ = nh.subscribe("gridmap", 100, drone_gridmap_cb, ros::TransportHints().tcpNoDelay());
        sub_ = nh_.subscribe(send_topic, 100, &BaseMsg::msg_cb, this, ros::TransportHints().tcpNoDelay());
    }

    if (recv_topic != "")
    {
        pub_ = nh_.advertise<T>(recv_topic, 100);
    }
}
template <typename T_ptr, typename T, size_t L>
BaseMsg<T_ptr, T, L>::~BaseMsg()
{
}

#endif