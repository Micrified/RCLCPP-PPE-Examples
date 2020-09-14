#if !defined(ON_CALLBACK_H)
#define ON_CALLBACK_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
std_msgs::msg::Int64 *on_callback (std_msgs::msg::Int64::SharedPtr msg_recv, int64_t executor_id, const char *node_name, const char *callback_name, int64_t callback_priority, int64_t callback_wcet_ns, bool is_timer_triggered);
#endif
