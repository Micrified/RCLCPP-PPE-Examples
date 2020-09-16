#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "eventchains/on_callback.hpp"

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
		sub_obstacle_detect_radar_topic = this->create_priority_subscription<std_msgs::msg::Int64>("radar_topic", 10,
				std::bind(&Control::obstacle_detect, this, _1), 4);
		sub_local_planning_lidar_topic = this->create_priority_subscription<std_msgs::msg::Int64>("lidar_topic", 10,
				std::bind(&Control::local_planning, this, _1), 1);
		sub_on_brake_brake_topic = this->create_priority_subscription<std_msgs::msg::Int64>("brake_topic", 10,
				std::bind(&Control::on_brake, this, _1), 5);
		pub_obstacle_detect_brake_topic = this->create_publisher<std_msgs::msg::Int64>("brake_topic", 10);
	}

};


int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node_handle_Control = std::make_shared<Control>();

	rclcpp::executors::SingleThreadedExecutor executor_2;
	executor_2.add_node(node_handle_Control);
	executor_2.spin();
	rclcpp::shutdown();
}
