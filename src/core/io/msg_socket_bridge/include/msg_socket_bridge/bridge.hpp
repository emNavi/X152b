#ifndef __BRIDGE_HPP_
#define __BRIDGE_HPP_

#include "msg_socket_bridge/TopicEnum.hpp"
// 8080 8060 already in used by gitlab, TCP is useless,give a random port value
#define PORT 8099
#define UDP_MAP_PORT 58099
#define UDP_PORT 8081

#define UDP_PORT_SHORT 58051
#define UDP_PORT_MID 58099
#define UDP_PORT_LONG 58050


// UDP broadcast MAX 65535-20(header)-8(udp_header)
#define BUF_LEN_SHORT 4096 // 2KB
#define BUF_LEN_FULL 60000  // max single frame,about 60KB
#define BUF_LEN_LONG 3000000 // about 3MB

struct sockaddr_in addr_udp_short_send_;
struct sockaddr_in addr_udp_mid_send_;

struct sockaddr_in addr_udp_long_send_;

int udp_recv_short_fd_, udp_recv_mid_fd_, udp_recv_long_fd_;
int udp_send_short_fd_, udp_send_mid_fd_, udp_send_long_fd_;

// char send_buf_short_[BUF_LEN_SHORT],send_buf_mid_[BUF_LEN_FULL],send_buf_long_[BUF_FULL_LEN];
char recv_buf_short_[BUF_LEN_SHORT],recv_buf_full_[BUF_LEN_FULL],recv_buf_long_[BUF_LEN_LONG];
#endif