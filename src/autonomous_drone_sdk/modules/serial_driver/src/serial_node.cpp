#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// 生成c++串口初始化和读取代码
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

using namespace std;
int serial_fd = -1;
int open_serial(string port, int baud_rate)
{
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY); // 打开串口
    if (serial_fd == -1)
    {
        cout << "串口打开失败！" << endl;
        return -1;
    }
    else
    {
        cout << "串口打开成功！" << endl;
    }
    struct termios options; // 串口配置结构体
    tcgetattr(serial_fd, &options);
    options.c_cflag |= CLOCAL; // 忽略调制解调器状态行
    options.c_cflag |= CREAD;  // 启用接收器
    options.c_cflag &= ~CSIZE; // 字符长度掩码。取值为CS5, CS6, CS7或CS8
    options.c_cflag |= CS8;    // 8位数据位
    options.c_cflag &= ~PARENB; // 校验位
    options.c_cflag &= ~CSTOPB; // 停止位
    options.c_iflag |= IGNPAR;  // 忽略帧错误和奇偶校验错
    options.c_oflag = 0;        // 输出模式
    options.c_lflag = 0;        // 不激活终端模式
    options.c_cc[VTIME] = 0; // 读取一个字符等待1*(1/10)s
    options.c_cc[VMIN] = 1;  // 读取字符的最少个数为1
    cfsetospeed(&options, B115200); // 设置波特率为115200
    cfsetispeed(&options, B115200);
    tcflush(serial_fd, TCIFLUSH); // 清空输入缓存区
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) // TCSANOW：不等数据传输完毕就立即改变属性
    {
        cout << "串口设置失败！" << endl;
        return -1;
    }
    else
    {
        cout << "串口设置成功！" << endl;
    }
    return 0;
}

// int read_serial(char *buffer, int buffer length)
// {
//     int bytes_read = 0;
//     int bytes_count = 0;
//     while (bytes_count < buffer_length)
//     {
//         bytes_read = read(serial_fd, buffer + bytes_count, buffer_length - bytes_count);
//         if (bytes_read > 0)
//         {
//             bytes_count += bytes_read;
//         }
//         else
//         {
//             cout << "串口读取失败！" << endl;
//             return -1;
//         }
//     }
//     return 0;
// }


int main(int argc, char **argv)
{
    unsigned char buffer[16];
    short ax[6] = {0};
    short gx[6] = {0};
    short sAngle[6] = {0};
    short sMag[6] = {0};
    sensor_msgs::Imu imu_msg; // 声明一个名为imu_msg的imu消息
    sensor_msgs::MagneticField mag_msg; // 声明一个名为imu_msg的imu消息

    ros::init(argc, argv, "imu_pub"); // 初始化ROS节点
    ros::init(argc, argv, "mag_pub"); // 初始化ROS节点
    ros::NodeHandle nh;               // 创建节点句柄
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000); // 创建一个Publisher，发布名为/imu的topic，消息类型为sensor_msgs::Imu，队列长度1000
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("MagneticField", 1000); // 创建一个Publisher，发布名为/imu的topic，消息类型为sensor_msgs::Imu，队列长度1000
    // ros::Rate loop_rate(10); // 设置循环的频率为10Hz
    open_serial("/dev/ttyUSB0", 115200);

    imu_msg.header.frame_id = "base_link";   // 填充imu_msg的头部坐标系为base_link
    imu_msg.orientation.x = 0.0;             // 填充imu_msg的四元数
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    while (ros::ok())
    {
        read(serial_fd, buffer, 1);
        if (buffer[0] == 0x55)
        {
            read(serial_fd, buffer + 1, 1);
            if (buffer[1] == 0x51)
            {
                read(serial_fd, ax, 9);
                // cout << "ax: " << ax[0] / 32768.0 * 16 << endl;
                // cout << "ay: " << ax[1] / 32768.0 * 16 << endl;
                // cout << "az: " << ax[2] / 32768.0 * 16 << endl;
                imu_msg.linear_acceleration.x = ax[0] / 32768.0 * 16.0 * 9.80665; // 填充imu_msg的线性加速度
                imu_msg.linear_acceleration.y = ax[1] / 32768.0 * 16.0 * 9.80665;
                imu_msg.linear_acceleration.z = ax[2] / 32768.0 * 16.0 * 9.80665;
                        imu_pub.publish(imu_msg); // 发布imu消息
            }
            else if (buffer[1] == 0x52)
            {
                read(serial_fd, gx, 9);
                // cout << "gx: " << gx[0] / 32768.0 * 2000 << endl;
                // cout << "gy: " << gx[1] / 32768.0 * 2000 << endl;
                // cout << "gz: " << gx[2] / 32768.0 * 2000 << endl;
                
                imu_msg.angular_velocity.x = gx[0] / 32768.0 * 2000 * 3.1415926 / 180.0; // 填充imu_msg的角速度
                imu_msg.angular_velocity.y = gx[1] / 32768.0 * 2000 * 3.1415926 / 180.0;
                imu_msg.angular_velocity.z = gx[2] / 32768.0 * 2000 * 3.1415926 / 180.0;
                           imu_pub.publish(imu_msg); // 发布imu消息
                
            }
	    else if (buffer[1] == 0x53)
            {
                read(serial_fd, sAngle, 9);
		imu_msg.header.stamp = ros::Time::now(); // 填充imu_msg的头部时间戳
                imu_msg.orientation.x = sAngle[0] / 32768.0 * 180.0; // 填充imu_msg的角度
                imu_msg.orientation.y = sAngle[1] / 32768.0 * 180.0;
                imu_msg.orientation.z = sAngle[2] / 32768.0 * 180.0;
		imu_pub.publish(imu_msg); // 发布imu消息
            }
	  
        }
        // ros::spinOnce();          // 处理ROS的信息，比如订阅消息,并调用回调函数
        // loop_rate.sleep();        // 按照循环频率延时
    }
    close(serial_fd);
    return 0;
}
