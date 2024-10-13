#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdio>
#include <cerrno>
#include <cstdlib>

class UdpBroadcastRecv
{
private:
    int _udp_recv_fd;
	int udp_bind_to_port(const int port, int &server_fd);

public:
    UdpBroadcastRecv(/* args */);
    ~UdpBroadcastRecv();

};

UdpBroadcastRecv::UdpBroadcastRecv(/* args */)
{
}
int UdpBroadcastRecv::udp_bind_to_port(const int port, int &server_fd)
{
	struct sockaddr_in address;
	int opt = 1;

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
				   &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(port);

	// Forcefully attaching socket to the port
	if (bind(server_fd, (struct sockaddr *)&address,
			 sizeof(address)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	return server_fd;
}
void commuinication_init(uint32_t max_msg_len)
{
	int valread;
	struct sockaddr_in addr_client;
	socklen_t addr_len;

	// Connect for recv
	if (udp_bind_to_port(UDP_PORT_SHORT, _udp_recv_fd) < 0)
	{
		ROS_ERROR("[msg_socket_bridge]Socket receive creation error!");
		exit(EXIT_FAILURE);
	}
}

void start()
{

}

void stop()
{

}

void recv_process()
{

}

UdpBroadcastRecv::~UdpBroadcastRecv()
{
}

void udp_short_recv_process()
{


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
		default:
			break;
		}
	}
}