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

static rclcpp::QoS get_qos ()
{
	rmw_qos_profile_t profile = {
		RMW_QOS_POLICY_HISTORY_KEEP_ALL,
		1024,
		RMW_QOS_POLICY_RELIABILITY_RELIABLE,
		RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
		{0, 0},
		{120, 0},
		RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
		{10, 0},
		false
	};
	return rclcpp::QoS(rclcpp::KeepAll(), profile);
}

class Radar : public rclcpp::Node
{
private:
		int publish_count;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_on_radar_scan_radar_topic;
		rclcpp::TimerBase::SharedPtr timer_on_radar_scan;

		void on_radar_scan ()
		{
			Integer msg;
			std::cout << "Brake [" << ++publish_count << "]: Triggered!" << std::endl;

			uint64_t timestamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			msg.data = static_cast<int64_t>(timestamp);
			pub_on_radar_scan_radar_topic->publish(msg);
		}

public:
	Radar(): Node("Radar", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {
		auto qos = get_qos();

		pub_on_radar_scan_radar_topic = this->create_publisher<std_msgs::msg::Int64>("radar_topic", qos);
		timer_on_radar_scan = this->create_wall_timer(std::chrono::milliseconds(6000), std::bind(&Radar::on_radar_scan, this),
			nullptr, 2);
	}

};

class Lidar : public rclcpp::Node
{
private:
		int publish_count;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_on_lidar_scan_lidar_topic;
		rclcpp::TimerBase::SharedPtr timer_on_lidar_scan;

		void on_lidar_scan ()
		{
			Integer msg;
			std::cout << "Planning [" << ++publish_count << "]: Triggered!" << std::endl;

			uint64_t timestamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			msg.data = static_cast<int64_t>(timestamp);
			pub_on_lidar_scan_lidar_topic->publish(msg);
		}

public:
	Lidar(): Node("Lidar", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {
		auto qos = get_qos();
		pub_on_lidar_scan_lidar_topic = this->create_publisher<std_msgs::msg::Int64>("lidar_topic", qos);
		timer_on_lidar_scan = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&Lidar::on_lidar_scan, this),
			nullptr, 1);
	}

};

class Control : public rclcpp::Node
{
private:
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_obstacle_detect_radar_topic;
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_local_planning_lidar_topic;
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_global_planning;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_obstacle_detect_brake_topic;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_global_planning_topic;

		void obstacle_detect (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;
			Integer msg;

			// // Spin a bit to simulate work
			// int sum = 0;
			// //std::cout << "Start: obstacle_detect" << std::endl;
			// for (int i = 0; i < 5000000; ++i) {
			// 	sum += isPrime(i);
			// }

			// Dispatch message to brake
			msg.data = msg_recv->data;
			pub_obstacle_detect_brake_topic->publish(msg);
			//std::cout << "End: obstacle_detect (" << sum << ")" << std::endl;
		}

		void local_planning (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;
			Integer msg;

			// Spin a bit to simulate work
			volatile int sum = 0;
			for (int i = 0; i < 20000000; ++i) {
				sum += isPrime(i);
			}

			msg.data = msg_recv->data;
			pub_global_planning_topic->publish(msg);
		}

		void global_planning (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			(void)msg_recv;
			static int global_planning_recv_count;
			global_planning_recv_count++;

			// Spin a bit to simulate work
			volatile int sum = 0;
			for (int i = 0; i < 2000000; ++i) {
				sum += isPrime(i);
			}

			uint64_t timestamp_end   = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			uint64_t timestamp_start = static_cast<uint64_t>(msg_recv->data);
			uint64_t duration        = timestamp_end - timestamp_start;
			std::cout << "Planning [" << global_planning_recv_count << "] end-to-end latency = " << duration << "ms" << std::endl; 
		}

public:
	Control(): Node("Control", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {

		auto qos = get_qos();
		pub_global_planning_topic = this->create_publisher<std_msgs::msg::Int64>("global_planning", qos);
		sub_global_planning = this->create_priority_subscription<std_msgs::msg::Int64>("global_planning", 30,
				std::bind(&Control::global_planning, this, _1), 1);
		sub_obstacle_detect_radar_topic = this->create_priority_subscription<std_msgs::msg::Int64>("radar_topic", 30,
				std::bind(&Control::obstacle_detect, this, _1), 2);
		sub_local_planning_lidar_topic = this->create_priority_subscription<std_msgs::msg::Int64>("lidar_topic", 30,
				std::bind(&Control::local_planning, this, _1), 1);
		pub_obstacle_detect_brake_topic = this->create_publisher<std_msgs::msg::Int64>("brake_topic", qos);
	}

};

class Actuators : public rclcpp::Node
{
private:
		rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_on_brake_brake_topic;

		void on_brake (const std_msgs::msg::Int64::SharedPtr msg_recv) const
		{
			static int on_brake_recv_count; 
			on_brake_recv_count++;
			(void)msg_recv;

			uint64_t timestamp_end   = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			uint64_t timestamp_start = static_cast<uint64_t>(msg_recv->data);
			uint64_t duration        = timestamp_end - timestamp_start;
			std::cout << "Brake [" << on_brake_recv_count << "] end-to-end latency = " << duration << "ms" << std::endl; 
		}

public:
	Actuators(): Node("Actuators", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {

		auto qos = get_qos();

		sub_on_brake_brake_topic = this->create_priority_subscription<std_msgs::msg::Int64>("brake_topic", 30,
				std::bind(&Actuators::on_brake, this, _1), 2);
	}

};


#define USE_PPE

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node_handle_Radar = std::make_shared<Radar>();
	auto node_handle_Lidar = std::make_shared<Lidar>();
	auto node_handle_Control = std::make_shared<Control>();
	auto node_handle_Actuators = std::make_shared<Actuators>();

#ifdef USE_PPE
	rclcpp::executors::PreemptivePriorityExecutor executor_1;
#else
	rclcpp::executors::MultiThreadedExecutor executor_1(rclcpp::ExecutorOptions(), 2, false, std::chrono::nanoseconds(-1));
#endif
  
	executor_1.add_node(node_handle_Radar);
	executor_1.add_node(node_handle_Lidar);
	executor_1.add_node(node_handle_Control);
	executor_1.add_node(node_handle_Actuators);
	executor_1.spin();
	rclcpp::shutdown();
}
