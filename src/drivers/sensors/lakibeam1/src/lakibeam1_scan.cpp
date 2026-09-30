#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include "scan_assembler.hpp"
#include <array>
#include <cerrno>
#include <stdexcept>
#include "../include/remote.h"

#define DEG2RAD(x) ((x)*M_PI / 180.f)
using namespace std;
class lakibeam1_scan : public rclcpp::Node
{
public:
	lakibeam1_scan():Node("laser_scan_publisher")
	{
		declare_parameters();
		get_parameters();
		scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(
			output_topic, rclcpp::SensorDataQoS());
		info();
		if (configure_sensor)
		{
			scan_config();
		}
        assembler = std::make_unique<lakibeam::ScanAssembler>(
            std::stod(scanfreq), [this](const lakibeam::Scan &scan) { publish_scan(scan); });
        if (create_socket() != 0) throw std::runtime_error("Cannot bind LiDAR UDP socket");
        timer = create_wall_timer(std::chrono::milliseconds(1), [this] { receive_packets(); });
	}
    ~lakibeam1_scan() override { if (sockfd >= 0) close(sockfd); }
protected:
	void get_parameters()
	{
		get_parameter<string>("frame_id",frame_id);
		get_parameter<std::string>("port",port);
		get_parameter<string>("hostip",hostip);
		get_parameter<string>("sensorip",sensorip);
		get_parameter<string>("output_topic",output_topic);
		get_parameter<string>("scanfreq",scanfreq);
		get_parameter<string>("filter",filter);
		get_parameter<string>("laser_enable",laser_enable);
		get_parameter<string>("scan_range_start",scan_range_start);
		get_parameter<string>("scan_range_stop",scan_range_stop);
		get_parameter<bool>("inverted",inverted);
		get_parameter<bool>("configure_sensor",configure_sensor);
		get_parameter<int>("angle_offset",angle_offset);
	};

	void declare_parameters()
	{
		declare_parameter<string>("frame_id","laser_link");
		declare_parameter<string>("port","2368");
		declare_parameter<string>("hostip","0.0.0.0");
		declare_parameter<string>("sensorip","192.168.198.2");
		declare_parameter<string>("output_topic","scan");
		declare_parameter<string>("scanfreq","30");
		declare_parameter<string>("filter","3");
		declare_parameter<string>("laser_enable","true");
		declare_parameter<string>("scan_range_start","45");
		declare_parameter<string>("scan_range_stop","315");
		declare_parameter<bool>("inverted",false);
		declare_parameter<bool>("configure_sensor",false);
		declare_parameter<int>("angle_offset",0);
	};
	void info()
	{
		RCLCPP_INFO(get_logger(),"frame_id:%s", frame_id.c_str());
		RCLCPP_INFO(get_logger(),"output_topic:%s", output_topic.c_str());
		RCLCPP_INFO(get_logger(),"inverted:%s", (inverted ? "True" : "False"));
		RCLCPP_INFO(get_logger(),"hostip:%s", hostip.c_str());
		RCLCPP_INFO(get_logger(),"sensorip:%s", sensorip.c_str());
		RCLCPP_INFO(get_logger(),"port:%s", port.c_str());
		RCLCPP_INFO(get_logger(),"scanfreq:%s", scanfreq.c_str());
		RCLCPP_INFO(get_logger(),"filter:%s", filter.c_str());
		RCLCPP_INFO(get_logger(),"laser_enable:%s", laser_enable.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_start:%s", scan_range_start.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_stop:%s", scan_range_stop.c_str());
		RCLCPP_INFO(get_logger(),"configure_sensor:%s", configure_sensor ? "True" : "False");

	};
	void scan_config()
	{
		RCLCPP_INFO(get_logger(),"scan_config");
		sensor_config(sensorip, "/api/v1/sensor/scanfreq", scanfreq);
		sensor_config(sensorip, "/api/v1/sensor/filter/level", filter);
		sensor_config(sensorip, "/api/v1/sensor/laser_enable", laser_enable);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/start", scan_range_start);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/stop", scan_range_stop);		
		RCLCPP_INFO(get_logger(),"scan_config1");

	};
	int create_socket()
    {
		RCLCPP_INFO(get_logger(),"create_socket");
		// rclcpp::sleep_for(std::chrono::milliseconds(2000));
		// get_telemetry_data(sensorip);
        sockfd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if(sockfd == -1)
        {
            RCLCPP_INFO(get_logger(),"Failed to create socket");
            return -1;
        }

        memset(&ser_addr, 0, sizeof(ser_addr));
        ser_addr.sin_family = AF_INET;
        ser_addr.sin_addr.s_addr = inet_addr(hostip.c_str());
        ser_addr.sin_port = htons(atoi(port.c_str()));

        if(bind(sockfd, (struct sockaddr*)&ser_addr, sizeof(ser_addr)) < 0)
        {
            RCLCPP_INFO(get_logger(),"Socket bind error!");
            close(sockfd);
            sockfd = -1;
            return -1;
        }
        return 0;
    };
    void receive_packets()
    {
        std::array<uint8_t, 1207> bytes{}; // extra byte detects oversized UDP datagrams
        for (int packet = 0; packet < 64; ++packet)
        {
            socklen_t length = sizeof(clent_addr);
            const auto count = recvfrom(sockfd, bytes.data(), bytes.size(), 0,
                reinterpret_cast<sockaddr *>(&clent_addr), &length);
            if (count < 0)
            {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "LiDAR receive failed: %s", std::strerror(errno));
                break;
            }
            if (!assembler->packet(bytes.data(), static_cast<size_t>(count), now().nanoseconds()))
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                    "Rejected malformed, duplicate or out-of-order LiDAR datagram");
        }
    }

    void publish_scan(const lakibeam::Scan &frame)
    {
        sensor_msgs::msg::LaserScan scan;
        const size_t count = frame.ranges.size();
        const double increment = 2. * M_PI / count;
        scan.header.stamp = rclcpp::Time(frame.start_ns, get_clock()->get_clock_type());
        scan.header.frame_id = frame_id;
        // Keep acquisition order for increasing point times. Inverted scans
        // use decreasing angles instead of reversing time-ordered samples.
        scan.angle_min = inverted ? DEG2RAD(180 + angle_offset) - increment :
                                    DEG2RAD(-180 + angle_offset);
        scan.angle_increment = inverted ? -increment : increment;
        scan.angle_max = scan.angle_min + scan.angle_increment * (count - 1);
        scan.scan_time = frame.duration;
        scan.time_increment = frame.duration / count;
        scan.range_min = 0.08;
        scan.range_max = 100.0;
        scan.ranges = frame.ranges;
        scan.intensities = frame.intensities;
        scan_pub->publish(scan);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Publishing %s, data points: %zu", output_topic.c_str(), count);
    }

private:
    string hostip, sensorip, port, frame_id, output_topic,scanfreq,filter,laser_enable,scan_range_start,scan_range_stop;
    int angle_offset;
    bool inverted, configure_sensor;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    struct sockaddr_in ser_addr, clent_addr;
    int sockfd = -1;
    std::unique_ptr<lakibeam::ScanAssembler> assembler;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = make_shared<lakibeam1_scan>();
	rclcpp::spin(node);
	rclcpp::shutdown();	

	return 0;
}
