#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher_right");
    ros::NodeHandle nh;
    ros::Rate r(30);

    tf::TransformBroadcaster br;

    while(nh.ok()){
        br.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.4,-0.28,-0.0125)),
                ros::Time::now(),"cargo_cart_bed_link", "rear_right_wheel_holder_link"));

        r.sleep();
    }
}