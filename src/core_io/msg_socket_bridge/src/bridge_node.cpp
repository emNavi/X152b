#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <vector>
#include <memory>

#include "msg_socket_bridge/udp_socket.hpp"
// #include "msg_socket_bridge/BaseMsg.hpp"
#include "msg_socket_bridge/bridge.hpp"
#include "msg_socket_bridge/Float32MultiArray.hpp"
#include "msg_socket_bridge/MsgFloat64MultiArray.hpp"

#include "msg_socket_bridge/MsgOdometry.hpp"
#include "msg_socket_bridge/MsgOdometry2.hpp"

#include "msg_socket_bridge/MsgBspline.hpp"
#include "msg_socket_bridge/MsgPointcloud.hpp"
#include "msg_socket_bridge/MsgMINCO.hpp"

#include "msg_socket_bridge/MsgMavrosPositionTarget.hpp"
using namespace std;

string udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
bool is_master;
bool pub_cloud;

uint8_t mission_num = 1;

Float32MultiArray *takeoff_msg_handle;
Float32MultiArray *land_msg_handle;
Float32MultiArray *point_cmd_msg_handle;
Float32MultiArray *ego_trigger_msg_handle;

MsgOdometry2 *other_odom_msg_handle;
// MsgOdometry *my_odom_msg_handle;

MsgBspline *broadcast_bspline_msg_handle;
MsgMINCO *broadcast_minco_msg_handle;
MsgPointcloud *pointcloud_msg_handle;

MsgFloat64MultiArray *ping_msg_handle;

Float32MultiArray *topology_1_msg_handle;
Float32MultiArray *topology_2_msg_handle;

MsgMavrosPositionTarget *leader_ctrl_msg_handle;

int32_t current_pkg_id = 0;			// 下一个pkg_id 到来，就丢弃
int16_t current_drone_id = -1;		// 有来自新飞机的消息，就丢弃
int32_t current_recv_piece_num = 0; // 统计已接收的段数
void udp_full_recv_process()
{
	int valread;
	struct sockaddr_in addr_client;
	socklen_t addr_len;

	// Connect for recv
	if (udp_bind_to_port(UDP_PORT_LONG, udp_recv_long_fd_) < 0)
	{
		ROS_ERROR("[msg_socket_bridge]Socket receive creation error!");
		exit(EXIT_FAILURE);
	}

	while (true)
	{
		if ((valread = recvfrom(udp_recv_long_fd_, recv_buf_full_, BUF_LEN_FULL, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
		{
			perror("recvfrom error:");
			exit(EXIT_FAILURE);
		}
		// parse_header
		MsgHeader *msg_header = (MsgHeader *)recv_buf_full_;

		switch (msg_header->topic_id)
		{
		case POINTCLOUD:
			if (msg_header->length < BUF_LEN_FULL)
			{
				memcpy(pointcloud_msg_handle->recv_buf_, recv_buf_full_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
				pointcloud_msg_handle->recv_process(valread - sizeof(MsgHeader));
			}
			else
			{
				if (msg_header->robot_id != current_drone_id)
				{
					ROS_WARN_STREAM("msg drop out !!!,drone_id!=current_drone_id");
					current_drone_id = msg_header->robot_id;
					current_recv_piece_num = 0;
					current_pkg_id = msg_header->package_id;
				}

				// New msg come in ,drop out old one
				if (msg_header->package_id > current_pkg_id)
				{
					current_recv_piece_num = 0;
					current_pkg_id = msg_header->package_id;
				}
				if (msg_header->package_id == current_pkg_id)
				{
					memcpy(pointcloud_msg_handle->recv_buf_ + BUF_LEN_FULL * msg_header->segment_id, recv_buf_full_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
					if (current_recv_piece_num == msg_header->segment_num)
					{
						pointcloud_msg_handle->recv_process(valread - sizeof(MsgHeader));
						current_recv_piece_num = 0;
					}
					current_recv_piece_num++;
				}
				else
				{
					ROS_WARN_STREAM("MSG segment drop out !!!, new pkg_id " << msg_header->package_id << "<current_pkg_id " << current_pkg_id);
				}
			}
			break;
		default:
			break;
		}
	}
}
void udp_short_recv_process()
{
	int valread;
	struct sockaddr_in addr_client;
	socklen_t addr_len;

	// Connect for recv
	if (udp_bind_to_port(UDP_PORT_SHORT, udp_recv_short_fd_) < 0)
	{
		ROS_ERROR("[msg_socket_bridge]Socket receive creation error!");
		exit(EXIT_FAILURE);
	}

	while (true)
	{
		if ((valread = recvfrom(udp_recv_short_fd_, recv_buf_short_, BUF_LEN_SHORT, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
		{
			perror("recvfrom error:");
			exit(EXIT_FAILURE);
		}
		// parse_header
		MsgHeader *msg_header = (MsgHeader *)recv_buf_short_;

		switch (msg_header->topic_id)
		{
		case TAKEOFF:
			/* code */
			memcpy(takeoff_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			takeoff_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case LAND:
			memcpy(land_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			land_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case POINT_CMD:
			memcpy(point_cmd_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			point_cmd_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case OTHER_ODOM:
			memcpy(other_odom_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			other_odom_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case OTHER_BSPLINE:
			// ROS_INFO_STREAM("OTHER_BSPLINE" << valread);
			memcpy(broadcast_bspline_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			broadcast_bspline_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case OTHER_MINCO:
			memcpy(broadcast_minco_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			broadcast_minco_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case PING_DELAY:
			memcpy(ping_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			ping_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case EGO_TRIGGER:
			memcpy(ego_trigger_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			ego_trigger_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case TOPOLOGY_1:
			memcpy(topology_1_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			topology_1_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case TOPOLOGY_2:
			memcpy(topology_2_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			topology_2_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		case LEADER_CTRL:
			memcpy(leader_ctrl_msg_handle->recv_buf_, recv_buf_short_ + sizeof(MsgHeader), valread - sizeof(MsgHeader));
			leader_ctrl_msg_handle->recv_process(valread - sizeof(MsgHeader));
			break;
		default:
			break;
		}
	}
}
// UDP Receive msg end

int main(int argc, char **argv)
{
	ros::init(argc, argv, "msg_socket_bridge");
	ROS_INFO("START Msg Bridge");

	ros::NodeHandle nh("~");
	int drone_id;
	nh.param("broadcast_ip", udp_ip_, string("10.42.0.255"));
	nh.param("is_master", is_master, false);
	nh.param("drone_id", drone_id, -1);

	udp_send_short_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT_SHORT, addr_udp_short_send_);
	udp_send_long_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT_LONG, addr_udp_long_send_);

	// SINGLE MODE
	if (is_master)
	{
		ROS_INFO("MASTER_MODE");
		takeoff_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TAKEOFF, udp_send_short_fd_, addr_udp_short_send_, "/swarm_takeoff", "");
		land_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::LAND, udp_send_short_fd_, addr_udp_short_send_, "/swarm_land", "");
		point_cmd_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::POINT_CMD, udp_send_short_fd_, addr_udp_short_send_, "/swarm_command", "");
		ego_trigger_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::EGO_TRIGGER, udp_send_short_fd_, addr_udp_short_send_, "/swarm/ego_trigger", "");

		pointcloud_msg_handle = new MsgPointcloud(nh, TOPIC_ENUM::POINTCLOUD, udp_send_long_fd_, addr_udp_long_send_, "/pointcloud", "");
	}
	else
	{
		ROS_INFO("ROBOT_MODE");
		takeoff_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TAKEOFF, udp_send_short_fd_, addr_udp_short_send_, "", "/swarm_takeoff");
		land_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::LAND, udp_send_short_fd_, addr_udp_short_send_, "", "/swarm_land");
		point_cmd_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::POINT_CMD, udp_send_short_fd_, addr_udp_short_send_, "", "/swarm_command");
		ego_trigger_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::EGO_TRIGGER, udp_send_short_fd_, addr_udp_short_send_, "", "/swarm/ego_trigger");
		pointcloud_msg_handle = new MsgPointcloud(nh, TOPIC_ENUM::POINTCLOUD, udp_send_long_fd_, addr_udp_long_send_, "", "/pointcloud");
	}
	// DULE MODE
	other_odom_msg_handle = new MsgOdometry2(nh, TOPIC_ENUM::OTHER_ODOM, udp_send_short_fd_, addr_udp_short_send_, "my_odom", "/others_odom");
	other_odom_msg_handle->set_max_freq(30);
	other_odom_msg_handle->set_drone_id(drone_id);
	broadcast_minco_msg_handle = new MsgMINCO(nh, TOPIC_ENUM::OTHER_MINCO, udp_send_short_fd_, addr_udp_short_send_, "/broadcast_traj_from_planner", "/broadcast_traj_to_planner");
	broadcast_bspline_msg_handle = new MsgBspline(nh, TOPIC_ENUM::OTHER_BSPLINE, udp_send_short_fd_, addr_udp_short_send_, "/broadcast_bspline", "/broadcast_bspline2");
	ping_msg_handle = new MsgFloat64MultiArray(nh, TOPIC_ENUM::PING_DELAY, udp_send_short_fd_, addr_udp_short_send_, "/ping/send", "/ping/recv");


	if(drone_id == 2)
	{
		topology_1_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_1, udp_send_short_fd_, addr_udp_short_send_, "/mc001/theta", "");
		topology_2_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_2, udp_send_short_fd_, addr_udp_short_send_, "/trash1", "/trash2");
		leader_ctrl_msg_handle = new MsgMavrosPositionTarget(nh, TOPIC_ENUM::LEADER_CTRL, udp_send_short_fd_, addr_udp_short_send_, "", "/leader_control");
	}
	else if(drone_id == 3)
	{
		topology_1_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_1, udp_send_short_fd_, addr_udp_short_send_, "", "/mc001/theta");
		topology_2_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_2, udp_send_short_fd_, addr_udp_short_send_, "/mc002/theta", "");
		leader_ctrl_msg_handle = new MsgMavrosPositionTarget(nh, TOPIC_ENUM::LEADER_CTRL, udp_send_short_fd_, addr_udp_short_send_, "", "/leader_control");

	}
	else if(drone_id == 4)
	{
		topology_1_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_1, udp_send_short_fd_, addr_udp_short_send_, "", "/mc001/theta");
		topology_2_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_2, udp_send_short_fd_, addr_udp_short_send_, "", "/mc002/theta");
		leader_ctrl_msg_handle = new MsgMavrosPositionTarget(nh, TOPIC_ENUM::LEADER_CTRL, udp_send_short_fd_, addr_udp_short_send_, "", "/leader_control");

	}
	else if(drone_id == 1)
	{
		leader_ctrl_msg_handle = new MsgMavrosPositionTarget(nh, TOPIC_ENUM::LEADER_CTRL, udp_send_short_fd_, addr_udp_short_send_, "/leader_control", "");
		topology_1_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_1, udp_send_short_fd_, addr_udp_short_send_, "/trash1", "/trash2");
		topology_2_msg_handle = new Float32MultiArray(nh, TOPIC_ENUM::TOPOLOGY_2, udp_send_short_fd_, addr_udp_short_send_, "/trash1", "/trash2");

	
	}


	boost::thread udp_short_recv_thd(udp_short_recv_process);
	udp_short_recv_thd.detach();
	ros::Duration(0.1).sleep();

	boost::thread udp_full_recv_thd(udp_full_recv_process);
	udp_full_recv_thd.detach();
	ros::Duration(0.1).sleep();

	ros::spin();
	return 0;
}
