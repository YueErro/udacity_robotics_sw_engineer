#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalStatusArray.h>

const double PICKUP_X = -0.35;
const double PICKUP_Y = 0.5;
const double DROPOFF_X = PICKUP_Y;
const double DROPOFF_Y = -0.15;

bool pickup = false;
bool dropoff = false;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void moveBaseGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg)
{
  if (!status_msg->status_list.empty())
  {
    ROS_DEBUG_STREAM(status_msg->status_list[0].status);
    if (status_msg->status_list[0].status == actionlib_msgs::GoalStatus::SUCCEEDED)
    {
      if (pickup && !dropoff)
      {
        ROS_INFO_STREAM("Picking up the cube");
        ros::Duration(5.0).sleep();

        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        dropoff = true;
        ROS_INFO_STREAM("Picked up the cube");
      }
      else if (!pickup && dropoff)
      {
        ROS_INFO_STREAM("Dropping off the cube");
        ros::Duration(5.0).sleep();

        marker.pose.position.x = DROPOFF_X;
        marker.pose.position.y = DROPOFF_Y;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);

        dropoff = false;
        ROS_INFO_STREAM("Dropped off the cube");
      }
    }
    else if (dropoff && status_msg->status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE)
      pickup = false;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(30);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber move_base_goal_status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 1, moveBaseGoalStatusCallback);

  // Wait 1 second to create the publishers and subscribers
  ros::Duration(1.0).sleep();

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "virtual_cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  // marker.pose.position.x = PICKUP_X + EPSILON/2;
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.125x0.125x0.125 here means 0.125m on a side
  marker.scale.x = 0.125;
  marker.scale.y = 0.125;
  marker.scale.z = 0.125;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  marker_pub.publish(marker);

  pickup = true;
  ros::Duration(5.0).sleep();

  ros::spin();
  return 0;
}
