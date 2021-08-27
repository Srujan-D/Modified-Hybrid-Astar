#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the rear_right_wheel_holder_link frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped rear_right;
  rear_right.header.frame_id = "rear_right_wheel_holder_link";

  //we'll just use the most recent transform available for our simple example
  rear_right.header.stamp = ros::Time();

  //just an arbitrary point in space
  rear_right.point.x = -0.4;
  rear_right.point.y = -0.28;
  rear_right.point.z = -0.0125; //-0.0125

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("cargo_cart_bed_link", rear_right, base_point);

    ROS_INFO("rear_right_wheel_holder_link: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        rear_right.point.x, rear_right.point.y, rear_right.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"rear_right_wheel_holder_link\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener_right");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}