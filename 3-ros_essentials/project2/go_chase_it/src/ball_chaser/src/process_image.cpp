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

  ROS_INFO_STREAM("Commanding the robot linear: " << lin_x << ", angular: " << ang_z);

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
  // {left, right, middle}
  std::vector<int> ball_pixels = { 0, 0, 0 };

  // Loop through each pixel in the image and check if there's a bright white one
  for (unsigned int i = 0; i < img.height * img.step; i++)
  {
    if (img.data[i] == white_pixel)
    {
      ROS_INFO_STREAM("The ball is in the image");
      ball_pos = i % img.step;
      // Then, identify if this pixel falls in the left, mid, or right side of the image
      if (ball_pos < img.step / 3)
      {
        ROS_INFO_STREAM("Ball pixel on the left hand side");
        ball_pixels[0] += 1;
      }
      else if (ball_pos > img.step / 3)
      {
        ROS_INFO_STREAM("Ball pixel on the right hand side");
        ball_pixels[1] += 1;
      }
      else
      {
        ROS_INFO_STREAM("Ball pixel in the middle");
        ball_pixels[2] += 1;
      }
      break;
    }
  }
  // Depending on the white ball position, call the drive_bot function and pass
  // velocities
  // to it
  if ((ball_pixels[0] | ball_pixels[1] | ball_pixels[2]) != 0)
  {
    int location =
        std::max_element(ball_pixels.begin(), ball_pixels.end()) - ball_pixels.begin();
    float closeness_indicator = *std::max_element(ball_pixels.begin(), ball_pixels.end());
    if (location == 0)
    {
      ROS_INFO_STREAM("Most part of the ball is on the left hand side");
      drive_robot(0.1f / closeness_indicator, 0.1f * closeness_indicator);
    }
    else if (location == 1)
    {
      ROS_INFO_STREAM("Most part of the ball is on the right hand side");
      drive_robot(0.1f / closeness_indicator, -0.1f * closeness_indicator);
    }
    else
    {
      ROS_INFO_STREAM("Most part of the ball is in the middle");
      drive_robot(0.1f / closeness_indicator, 0.0f);
    }
  }
  // Request a stop when there's no white ball seen by the camera
  else
  {
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
