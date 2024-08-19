
# 数据结构

- MsgHeader
- MsgBody

```c++
struct MsgHeader {
    std::string topic_name;
    int robot_id;
    uint32_t length;
    int package_id;
    uint8_t segment_id;
};
```


Msgbody 目前提供

1. MsgFloat32MultiArray (测试完成)
2. MsgBspline (测试完成)
3. MsgOdometry (测试完成)
4. MsgPointcloud (序列化与反序列化没有测试)
5. MsgMINCO ()


# 快速链接新的Topic
## 如果是已经提供的消息
在 bridge_node中，添加

1. 设定一个 massage 标识 <new_msg>
```c++
enum TOPIC_ENUM
{
  OTHER_ODOM = 10,
  OTHER_BSPLINE,
  TAKEOFF,
  LAND,
  POINT_CMD,
  POINTCLOUD,
  <new_msg>
} massage_type_;
```

1. 在 bridge_node 中新建 一个**handle**
2. 实例化这个 handle
   ```c++
   	broadcast_bspline_msg_handle = new MsgBspline(nh, TOPIC_ENUM::OTHER_BSPLINE, udp_send_short_fd_, addr_udp_short_send_, "/broadcast_bspline2", "/broadcast_bspline");

   ```
   > 计划设计三种通讯模式,两种UDP方式，一种TCP方式\
   > 1. UDP,A topic in B topic out, 适合广播\
   > 2. UDP，A topic in A topic out ,适合单向传输，master-> robots\
   > 3. TCP，A topic in A topic out ,适合单向传输,保证到达，master-> robots
3. 添加接收


# 使用
## 限制传输频率


## 局部变量不能传递地址




# Tools
## Ping
**消息格式**

1. 发送端 send： Req_drone_id,Req_drone_request_time
2. 接收端 recv： Req_drone_id,Req_drone_request_time
3. 发送端 send： Req_drone_id,Req_drone_request_time

接受:  Recv_drone_id,Recv_drone_request_time,Req_drone_id,Req_drone_request_time

**连续发送**
100次连续发送，