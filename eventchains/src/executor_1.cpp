#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

extern "C" {
	#include <time.h>
}

using std::placeholders::_1;
using Integer=std_msgs::msg::Int64;
using namespace std::chrono;

static bool isPrime (unsigned int value)
{
	unsigned int i, root;
	if (value == 1)       return false;
	if (value == 2)       return true;
	if ((value % 2) == 0) return false;
	root = (int)(1.0 + sqrt(value));
	for (i = 3; (i < root) && (value % i != 0); i += 2);
	return (i < root ? false : true);
}


class Radar : public rclcpp::Node
{
private:
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_on_radar_scan_radar_topic;
		rclcpp::TimerBase::SharedPtr timer_on_radar_scan;

		void on_radar_scan ()
		{
			Integer msg;
			std::cout << "Start: on_radar_scan" << std::endl;
			uint64_t timestamp_end = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			msg.data = static_cast<int64_t>(timestamp_end);
			pub_on_radar_scan_radar_topic->publish(msg);
			std::cout << "End: on_radar_scan ()" << std::endl;
		}

public:
	Radar(): Node("Radar", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {
		auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();

		pub_on_radar_scan_radar_topic = this->create_publisher<std_msgs::msg::Int64>("radar_topic", qos);
		timer_on_radar_scan = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&Radar::on_radar_scan, this));
	}

};

class Lidar : public rclcpp::Node
{
private:
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_on_lidar_scan_lidar_topic;
		rclcpp::TimerBase::SharedPtr timer_on_lidar_scan;

		void on_lidar_scan ()
		{
			Integer msg;
			std::cout << "Start: on_lidar_scan" << std::endl;
			int sum = 0;
			for (int i = 0; i < 10000000; ++i) {
				sum += isPrime(i);
			}
			msg.data = sum;
			pub_on_lidar_scan_lidar_topic->publish(msg);
			std::cout << "End: on_lidar_scan (" << sum << ")" << std::endl;
		}

public:
	Lidar(): Node("Lidar", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {
		auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
		pub_on_lidar_scan_lidar_topic = this->create_publisher<std_msgs::msg::Int64>("lidar_topic", qos);
		timer_on_lidar_scan = this->create_wall_timer(std::chrono::milliseconds(8000), std::bind(&Lidar::on_lidar_scan, this));
	}

};

class Control : public rclcpp::Node
{
private:
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_obstacle_detect_radar_topic;
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_local_planning_lidar_topic;
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_on_brake_brake_topic;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_obstacle_detect_brake_topic;

		void obstacle_detect (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;
			Integer msg;

			// Spin a bit to simulate work
			int sum = 0;
			std::cout << "Start: obstacle_detect" << std::endl;
			for (int i = 0; i < 10000000; ++i) {
				sum += isPrime(i);
			}

			// Dispatch message to brake
			msg.data = msg_recv->data;
			pub_obstacle_detect_brake_topic->publish(msg);
			std::cout << "End: obstacle_detect (" << sum << ")" << std::endl;
		}

		void local_planning (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;

			// Spin a bit to simulate work
			int sum = 0;
			std::cout << "Start: local_planning" << std::endl;
			for (int i = 0; i < 20000000; ++i) {
				sum += isPrime(i);
			}
			std::cout << "End: local_planning (" << sum << ")" << std::endl;
		}

		void on_brake (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;
			uint64_t timestamp_end   = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			uint64_t timestamp_start = static_cast<uint64_t>(msg_recv->data);
			uint64_t duration        = timestamp_end - timestamp_start;
			std::cout << "on_brake(): duration = " << duration << "ms" << std::endl;
		}

public:
	Control(): Node("Control", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {

		auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();

		sub_obstacle_detect_radar_topic = this->create_subscription<std_msgs::msg::Int64>("radar_topic", 30,
				std::bind(&Control::obstacle_detect, this, _1));
		sub_local_planning_lidar_topic = this->create_subscription<std_msgs::msg::Int64>("lidar_topic", 30,
				std::bind(&Control::local_planning, this, _1));
		sub_on_brake_brake_topic = this->create_subscription<std_msgs::msg::Int64>("brake_topic", 30,
				std::bind(&Control::on_brake, this, _1));
		pub_obstacle_detect_brake_topic = this->create_publisher<std_msgs::msg::Int64>("brake_topic", qos);
	}

};


int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node_handle_Radar = std::make_shared<Radar>();
	auto node_handle_Lidar = std::make_shared<Lidar>();
	auto node_handle_Control = std::make_shared<Control>();

	/* Select the executor type here */
	//rclcpp::executors::MultiThreadedExecutor executor_1(rclcpp::ExecutorOptions(), 2, false, std::chrono::nanoseconds(-1));
	rclcpp::executors::SingleThreadedExecutor executor_1;

	executor_1.add_node(node_handle_Radar);
	executor_1.add_node(node_handle_Lidar);
	executor_1.add_node(node_handle_Control);
	executor_1.spin();
	rclcpp::shutdown();
}
