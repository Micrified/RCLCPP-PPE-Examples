# RCLCPP-PPE Examples

## Automatic

This package has a simple executor with a few elements

1. A timer that runs a brake callback (just prints a line on `stdout`). It runs every second
2. A timer that publishes to a topic called `"topic_engine"`. It runs every ten seconds
3. A callback for the `"topic_engine"` topic, which computes primes for about 8 seconds (full CPU use). 

By commenting in and out the `#define MAKE_BRAKE_PREEMPT` preprocessor directive, you can observe how the PPE preempts (or doesn't) the longer, CPU intensive callback with the short, quick brake timer.

## EventChains

This package has two distinct chains of functionality embedded in one executor: 

1. Chain 1: `Radar::on_radar_scan()` -[`"radar_topic"`]-> `Control::obstacle_detect()` -[`"brake_topic"`]-> `Actuators::on_brake()`
2. Chain 2: `Lidar::on_lidar_scan()` -[`"lidar_topic"`]-> `Control::local_planning()` -[`"global_planning"`]-> `Control::global_planning()`

All elements of chain 1 are assigned priority level `2`, and elements of chain 2 priority level `1`. When running the PPE, you should see that chain 1 completes almost instantly when fired (rate: 1/6Hz). Meanwhile, chain 2 takes longer and suffers from preemptions. 

It should be noted that priorities perform best when consistent across a chain. This is because you don't want higher priority parts of the chain (say, the timer and some intermediate processing callback) from constantly running and starving the end of the chain. By giving them the same priority chronological ordering is better respected, although higher priority chains will of course preempt them. 
