#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

extern "C" {
	#include <time.h>
}


// Some placeholders
using std::placeholders::_1;
using Integer=std_msgs::msg::Int64;

// Computation function (for busy work)
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


class A : public rclcpp::Node
{
private:
		char const *data_name = "A";
		int64_t data_executor_id = 1;
		rclcpp::Publisher<Integer>::SharedPtr pub_topic_engine;
		rclcpp::Subscription<Integer>::SharedPtr sub_topic_engine;

		rclcpp::TimerBase::SharedPtr timer_topic_brake;
		rclcpp::TimerBase::SharedPtr timer_topic_engine;


		void on_brake_timer ()
		{
			std::cout << "<braking sounds - schrrrr!>" << std::endl << std::flush;
		}

		void on_engine_timer ()
		{
			Integer message;
			message.data = 30000000;
			pub_topic_engine->publish(message);
		}

		void on_engine_callback (const Integer::SharedPtr message) const {
			int n_primes = 0;
			std::cout << "on_engine_callback(" << message->data << ")" << std::endl << std::flush;
			for (off_t i = 0; i < message->data; ++i) {
				n_primes += isPrime(i);
			}
			std::cout << "on_engine_callback: " << n_primes 
				<< " primes in range [0," << message->data << ")" << std::endl << std::flush;
		}

public:

	// Note: Need to disable parameter event publisher to avoid flooding executor!
	A(): Node("A", rclcpp::NodeOptions().start_parameter_event_publisher(false)) {


#define MAKE_BRAKE_PREEMPT

#ifdef MAKE_BRAKE_PREEMPT
		int priority_brake_timer     = 3;
		int priority_engine_timer    = 2;
		int priority_engine_callback = 1;
#else
		int priority_brake_timer     = 1;
		int priority_engine_timer    = 3;
		int priority_engine_callback = 2;
#endif
		// The engine topic
		pub_topic_engine = this->create_publisher<Integer>("topic_engine", 10);

		// The engine topic callback
		sub_topic_engine = this->create_priority_subscription<Integer>("topic_engine",
			10, std::bind(&A::on_engine_callback, this, _1), priority_engine_callback);

		// Create timer for brake
		timer_topic_brake = this->create_wall_timer(std::chrono::milliseconds(1000),
			std::bind(&A::on_brake_timer, this), nullptr, priority_brake_timer);

		// Create timer for engine topic
		timer_topic_engine = this->create_wall_timer(std::chrono::milliseconds(10000),
			std::bind(&A::on_engine_timer, this), nullptr, priority_engine_timer);
	}

};



int main (int argc, char *argv[])
{

	// Init ROS2
	rclcpp::init(argc, argv);

	// Init the node
	auto node = std::make_shared<A>();

	// Init the executor
	rclcpp::executors::PreemptivePriorityExecutor executor_1(rclcpp::ExecutorOptions(), {50,99}, 
		rclcpp::executors::P_FP);

	// Add node
	executor_1.add_node(node);

	// Spin
	executor_1.spin();

	// Shutdown ROS
	rclcpp::shutdown();
}
