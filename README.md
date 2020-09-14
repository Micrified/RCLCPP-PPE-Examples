# RCLCPP-PPE Examples

## Automatic

This package has a simple executor with a few elements

1. A timer that runs a brake callback (just prints a line on `stdout`). It runs every second
2. A timer that publishes to a topic called `"topic_engine"`. It runs every ten seconds
3. A callback for the `"topic_engine"` topic, which computes primes for about 8 seconds (full CPU use). 

By commenting in and out the `#define MAKE_BRAKE_PREEMPT` preprocessor directive, you can observe how the PPE preempts (or doesn't) the longer, CPU intensive callback with the short, quick brake timer. 
