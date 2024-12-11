
#ifndef __UDP_SOCKET_HPP_
#define __UDP_SOCKET_HPP_
#include <ros/ros.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
// udp broadcast
int init_broadcast(const char *ip, const int port, sockaddr_in &addr_udp_send)
{
	int fd;

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
	{
		ROS_ERROR("[bridge_node]Socket sender creation error!");
		exit(EXIT_FAILURE);
	}

	int so_broadcast = 1;
	if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
	{
		std::cout << "Error in setting Broadcast option";
		exit(EXIT_FAILURE);
	}

	addr_udp_send.sin_family = AF_INET;
	addr_udp_send.sin_port = htons(port);

	if (inet_pton(AF_INET, ip, &addr_udp_send.sin_addr) <= 0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
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
#endif