// Import ROS library
#include <ros/ros.h>
// Import Float64 header file from std_msgs package
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
  // Initialize the arm_mover node
  ros::init(argc, argv, "arm_mover");

  // Create a handle to the arm_mover node in order to communicate with the ROS master
  ros::NodeHandle n;

  // Create a publisher that can publish a std_msgs::Float64 message on the
  // /simple_arm/joint_1_position_controller/command topic
  ros::Publisher joint1_pub =
      n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  // Create a publisher that can publish a std_msgs::Float64 message on the
  // /simple_arm/joint_2_position_controller/command topic
  ros::Publisher joint2_pub =
      n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

  // Set loop frequency of 10Hz
  /*
   * Choosing too high value may result in unnecessary CPU usage while choosing a value
   * too low could resul in high system latency.
  */
  ros::Rate loop_rate(10);

  int start_time, elapsed;

  // Get ROS start time
  while (not start_time)
  {
    /*
     *We set start_time to the current time. In a moment we will use this to determine how
     *much time has elapsed. When using ROS with simulated time (as we are doing here),
     *ros-Time-now will initially return 0, until the first message has been received on
     *the /clock topic. This is why start_time is set and polled continuously until a
     *nonzero value is returned.
    */
    start_time = ros::Time::now().toSec();
  }

  while (ros::ok())
  {
    // Get ROS elapsed time
    elapsed = ros::Time::now().toSec() - start_time;

    // Set the arm joint angles
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
    joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

    // Publish the arm joint angles
    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Sleep for the time remaining until 10 Hz is reached
    /*
     * When the node receives the signal to shut down either from the RIS master or via
     * signal in a console window, the loop will exit.
    */
    loop_rate.sleep();
  }
  return 0;
}
