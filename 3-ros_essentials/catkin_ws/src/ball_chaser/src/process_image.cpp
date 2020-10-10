#include <ros/ros.h>
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#include <vector>
#include <algorithm>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified
// direction
void drive_robot(float lin_x, float ang_z)
{
  // Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget dtt_srv;
  dtt_srv.request.linear_x = lin_x;
  dtt_srv.request.angular_z = ang_z;

  ROS_DEBUG_STREAM("Commanding the robot linear: " << lin_x << ", angular: " << ang_z);

  if (!client.call(dtt_srv))
  {
    ROS_ERROR_STREAM("Failed to call service command_robot");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  unsigned int ball_pos;
  int left = 0;
  int right = 0;
  int middle = 0;

  // Loop through each pixel in the image and check if there's a bright white one
  // Iterate by threes due to RGB concatenation
  for (unsigned int i = 0; i < img.height * img.step; i += 3)
  {
    // [R, G, B]
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 3] == white_pixel)
    {
      ROS_DEBUG_STREAM("The ball is in the image");
      ball_pos = i % img.step;
      // Then, identify if this pixel falls in the left, mid, or right side of the image
      if (ball_pos < img.step / 3)
      {
        ROS_DEBUG_STREAM("Ball pixel on the left hand side");
        left += 1;
      }
      else if (ball_pos > img.step * 2 / 3)
      {
        ROS_DEBUG_STREAM("Ball pixel on the right hand side");
        right += 1;
      }
      else
      {
        ROS_DEBUG_STREAM("Ball pixel in the middle");
        middle += 1;
      }
    }
  }
  // Depending on the white ball position, call the drive_bot function and pass
  // velocities
  // to it
  if ((left | right | middle) != 0)
  {
    if (left > right && left > middle)
    {
      ROS_DEBUG_STREAM("Most part of the ball is on the left hand side");
//      drive_robot(0.01f / left, 1.0f * left);
      drive_robot(0.2f, 0.4f);
    }
    else if (right > left && right > middle)
    {
      ROS_DEBUG_STREAM("Most part of the ball is on the right hand side");
//      drive_robot(0.01f / right, -1.0f * right);
      drive_robot(0.2f, -0.4f);
    }
    else
    {
      ROS_DEBUG_STREAM("Most part of the ball is in the middle");
      drive_robot(0.2f, 0.0f);
    }
  }
  // Request a stop when there's no white ball seen by the camera
  else
  {
    ROS_DEBUG_STREAM("Stop robot, it doesn't see the white ball");
    drive_robot(0.0f, 0.0f);
  }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the
  // process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
