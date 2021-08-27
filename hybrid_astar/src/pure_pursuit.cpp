#include "../include/hybrid_astar/pure_pursuit.h"

using namespace std;

// Current pose of the robot
float x_r;
float y_r;
float yaw_r;

// Current pose of the trailer
float x_t;
float y_t;
float yaw_t;

float alpha;
float phi;

// PID Constants
const float Kp = 0.3; // Proportional constant

// Pure Pursuit Constants
const float ld = 0.3; // look-ahead distance
const float kf = 0.1; // look forward gain
const float max_vel = 0.2; // maximum velocity
const float max_ang_vel = 0.3; // maximum angular
// Backward motion
const float k = 1.0;

// Non-Linear Controller
std::vector<float> curvatures;


/*	
	Callback function for "global_path" to implement the pure pursuit controller
	
	Subscribes: global_path
	Publishers: target_point, cmd_vel
*/
void pure_pursuit() {

	tf::TransformListener listener_robot;
	tf::StampedTransform transform_robot;
	listener_robot.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));

	tf::TransformListener listener_trailer;
	tf::StampedTransform transform_trailer;
	listener_trailer.waitForTransform("map", "cargo_cart_link", ros::Time::now(), ros::Duration(2.0));

	// Writing tracking data to rosbag
	// hybrid_astar::Tracking test;
	// rosbag::Bag bag("/home/tanujthakkar/ROS/catkin_ws/src/hybrid_astar/rosbags/tracking.bag", rosbag::bagmode::Write);
	// // bag.open("../rosbags/tracking.bag", rosbag::bagmode::Write);

	// test.reference_path.header.stamp = ros::Time::now();
	// test.reference_path.header.frame_id = "/map";
	// test.reference_path.poses.clear();

	// test.tracked_path.header.stamp = ros::Time::now();
	// test.tracked_path.header.frame_id = "/map";
	// test.tracked_path.poses.clear();

	// listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
	// geometry_msgs::PoseStamped pose_stamped;
	// float x = 0;
	// float y = 0;
	// float yaw;
	// float yawt;
	// path.header.stamp = ros::Time::now();
	// path.header.frame_id = "/map";
	// path.poses.clear();

	// trailer_path.header.stamp = ros::Time::now();
	// trailer_path.header.frame_id = "/map";
	// trailer_path.poses.clear();

	// yaw = tf::getYaw(transform_robot.getRotation());
	// yawt = tf::getYaw(transform_robot.getRotation());
	// x = transform_robot.getOrigin().x() + 0.1;// - DELTAR * cos(yaw);
	// y = transform_robot.getOrigin().y() + 0.1;// - DELTAR * sin(yaw);
	
	// // std::vector<float> path_x;
	// // std::vector<float> path_y;
	// // std::vector<float> track_x;
	// // std::vector<float> track_y;
	// float err_lat = 0;
	// float cross_track = 0;

	// float t = 0;

	// for(int i = 0; i < 400; ++i) {
	// 	pose_stamped.header.stamp = ros::Time::now();
	// 	pose_stamped.header.frame_id = "map";
	// 	// x = x + 0.1 * cos(yaw);
	// 	// y = y + 0.1 * sin(yaw);
	// 	// yaw = pi_2_pi(yaw + 0.1 / WHEELBASE * tan(to_rad(10)));
	// 	// yawt = pi_2_pi(yawt - 0.1 / RTR * sin(yaw - yawt));

	// 	x = 6 * sin(t);
	// 	y = 6 * sin(t) * cos(t);
	// 	t = t + 0.0157;
		
	// 	pose_stamped.pose.position.x = x;
	// 	pose_stamped.pose.position.y = y;
	// 	tf::Quaternion quat = tf::createQuaternionFromYaw(t);
	// 	pose_stamped.pose.orientation.x = quat.x();
	// 	pose_stamped.pose.orientation.y = quat.y();
	// 	pose_stamped.pose.orientation.z = quat.z();
	// 	pose_stamped.pose.orientation.w = quat.w();
	// 	path.poses.push_back(pose_stamped);

	// 	test.reference_path.poses.push_back(pose_stamped);
	// 	// pose_stamped.pose.position.x = x;// - ((DELTAR + 0.4) - DELTAT) * cos(yawt);
	// 	// pose_stamped.pose.position.y = y;// - ((DELTAR + 0.4) - DELTAT) * sin(yawt);
	// 	// quat = tf::createQuaternionFromYaw(yawt);
	// 	// pose_stamped.pose.orientation.x = quat.x();
	// 	// pose_stamped.pose.orientation.y = quat.y();
	// 	// pose_stamped.pose.orientation.z = quat.z();
	// 	// pose_stamped.pose.orientation.w = quat.w();
	// 	// path.poses.push_back(pose_stamped);
	// 	// trailer_path.poses.push_back(pose_stamped);

	// 	dirs.push_back(-1);
	// }

	// hybrid_path_pub.publish(path);
	// hybrid_trailer_path_pub.publish(trailer_path);

	int index = 0;
	int n = path.poses.size();
	printf("path length: %d\n", n);

	float lf;
	float tx; // target x
	float ty; // target y
	float alpha;
	float alpha_;
	float delta;

	path = tractor_path;
	// trailer_path = path;

	// cin.get();
	while(true) {
		
		listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
		listener_trailer.lookupTransform("map", "cargo_cart_link", ros::Time(0), transform_trailer);

		yaw_r = tf::getYaw(transform_robot.getRotation());
		x_r = transform_robot.getOrigin().x();// - DELTAR * cos(yaw_r);
		y_r = transform_robot.getOrigin().y();// - DELTAR * sin(yaw_r);
		// ROS_INFO("x_r: %f y_r: %f yaw_r: %f", x_r, y_r, yaw_r);

		yaw_t = tf::getYaw(transform_trailer.getRotation());
		// x_t = x_r;
		// y_t = y_r;
		x_t = transform_robot.getOrigin().x() - ((DELTAR + 0.4) - DELTAT) * cos(yaw_t);
		y_t = transform_robot.getOrigin().y() - ((DELTAR + 0.4) - DELTAT) * sin(yaw_t);
		// ROS_INFO("x_t: %f y_t: %f yaw_t: %f", x_t, y_t, yaw_t);

		lf = kf * max_vel + ld;

		// cin.get();
		// ROS_INFO("DIR: %d", dirs[index]);
		// Forward Motion Pure Pursuit Control Law
		if(dirs[index] == 1) {
			printf("Tractor path index: %d\n", index);

			// if(abs(abs(x)-abs(x_r)) > 0.1 || abs(abs(y)-abs(y_r)) > 0.1) {
			// 	x = x_r;
			// 	y = y_r;
			// 	pose_stamped.pose.position.x = x_r;
			// 	pose_stamped.pose.position.y = y_r;
			// 	tf::Quaternion quat = tf::createQuaternionFromYaw(yaw_r);
			// 	pose_stamped.pose.orientation.x = quat.x();
			// 	pose_stamped.pose.orientation.y = quat.y();
			// 	pose_stamped.pose.orientation.z = quat.z();
			// 	pose_stamped.pose.orientation.w = quat.w();
			// 	test.tracked_path.poses.push_back(pose_stamped);
			// }

			if(abs(n - index) < 2) {
				if(hypot(x_r - path.poses[n-1].pose.position.x, y_r - path.poses[n-1].pose.position.y) < 0.4) {
					printf("Tractor path index: %d\n", index);
					ROS_INFO("GOAL REACHED!");
					break;
				}
			}

			float m = hypot(x_r - path.poses[0].pose.position.x, y_r - path.poses[0].pose.position.y);
			for (int i = index; i < index + 4; ++i) {
				// printf("m: %f point: %f\n", m, hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y));
				if(hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y) < m) {
					m = hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y);
					index = i;
				}

				if(hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y) > 0.1) {
					index = i;
					break;
				}
			}

			tx = path.poses[index].pose.position.x;
			ty = path.poses[index].pose.position.y;
			geometry_msgs::PointStamped target_point;
			target_point.header.stamp = ros::Time::now();
			target_point.header.frame_id = "/map";
			target_point.point.x = tx;
			target_point.point.y = ty;
			target_point_pub.publish(target_point);

			alpha = atan2(ty - y_r, tx - x_r) - yaw_r;
			delta = atan2(2.0 * WHEELBASE * sin(alpha), lf);

			// err_lat = (-1 * (x_r - path.poses[index].pose.position.x) * sin(tf::getYaw(path.poses[index].pose.orientation)) + (y_r - path.poses[index].pose.position.y) * cos(tf::getYaw(path.poses[index].pose.orientation)));
			// test.err_lat.push_back((-1 * (x_r - path.poses[index].pose.position.x) * sin(tf::getYaw(path.poses[index].pose.orientation)) + (y_r - path.poses[index].pose.position.y) * cos(tf::getYaw(path.poses[index].pose.orientation))));

			// if(cross_track != (lf * sin(alpha))) {
			// 	cross_track = lf * sin(alpha);
			// 	test.cross_track.push_back(lf * sin(alpha));
			// }

			// printf("err_lat: %f cross_track: %f", err_lat, cross_track);

			geometry_msgs::Twist cmd;
			cmd.linear.x = 0.2;
			cmd.angular.z = delta;
			// printf("alpha: %f alpha_: %f delta: %f linear: %f\n", alpha, alpha_, delta, cmd.linear.x);
			cmd_pub.publish(cmd);
		}

		// Reverse Motion Pure Pursuit Control Law
		if(dirs[index] == -1) {
			printf("Trailer path index: %d\n", index);

			// if(abs(abs(x)-abs(x_t)) > 0.1 || abs(abs(y)-abs(y_t)) > 0.1) {
			// 	x = x_t;
			// 	y = y_t;
			// 	pose_stamped.pose.position.x = x_t;
			// 	pose_stamped.pose.position.y = y_t;
			// 	tf::Quaternion quat = tf::createQuaternionFromYaw(yaw_t);
			// 	pose_stamped.pose.orientation.x = quat.x();
			// 	pose_stamped.pose.orientation.y = quat.y();
			// 	pose_stamped.pose.orientation.z = quat.z();
			// 	pose_stamped.pose.orientation.w = quat.w();
			// 	test.tracked_path.poses.push_back(pose_stamped);
			// }

			if(abs(n - index) < 2) {
				if(hypot(x_t - trailer_path.poses[n-1].pose.position.x, y_t - trailer_path.poses[n-1].pose.position.y) < 0.3) {
					ROS_INFO("GOAL REACHED!");
					break;
				}
			}

			float m = hypot(x_t - trailer_path.poses[0].pose.position.x, y_t - trailer_path.poses[0].pose.position.y);
			for (int i = index; i < index + 4; ++i) {
				if(hypot(x_t - trailer_path.poses[i].pose.position.x, y_t - trailer_path.poses[i].pose.position.y) < m) {
					m = hypot(x_t - trailer_path.poses[i].pose.position.x, y_t - trailer_path.poses[i].pose.position.y);
					index = i;
				}
				// printf("m: %f point: %f\n", m, hypot(x_t - path.poses[i].pose.position.x, y_t - path.poses[i].pose.position.y));
			}

			for (int i = index; i < index + 4; ++i) {
				if(hypot(x_t - trailer_path.poses[i].pose.position.x, y_t - trailer_path.poses[i].pose.position.y) > 0.1) {
					index = i;
					break;
				}
			}

			tx = trailer_path.poses[index].pose.position.x;
			ty = trailer_path.poses[index].pose.position.y;
			geometry_msgs::PointStamped target_point;
			target_point.header.stamp = ros::Time::now();
			target_point.header.frame_id = "/map";
			target_point.point.x = tx;
			target_point.point.y = ty;
			target_point_pub.publish(target_point);
			// // alpha=atan2((yd(i)-y2),(xd(i)-x2))-beta2
			// // delta_des=atan(d*(K*sign(v1)*(phi+atan(2*d*sin(alpha)/lookahead_dist))-sin(phi)/e));

			phi = pi_2_pi(yaw_r - yaw_t) * -1.0;
			alpha = pi_2_pi(atan2(ty - y_t, tx - x_t) - yaw_t);

			// test.err_lat.push_back((-1 * (x_t - trailer_path.poses[index].pose.position.x) * sin(tf::getYaw(trailer_path.poses[index].pose.orientation)) + (y_t - trailer_path.poses[index].pose.position.y) * cos(tf::getYaw(trailer_path.poses[index].pose.orientation))));
			
			// if(cross_track != (lf * sin(alpha))) {
			// 	cross_track = lf * sin(alpha);
			// 	test.cross_track.push_back(lf * sin(alpha));
			// }

			delta = -0.1 * (k * -1.0 * (phi + atan(2 * RTR * sin(alpha)/lf)) - sin(phi)/RTR);

			// printf(" err_lat: %f cross_track: %f\n", err_lat, cross_track);

			geometry_msgs::Twist cmd;
			cmd.linear.x = -0.1;
			// cmd.linear.x = min((float)-0.2, -abs(err_lat));
			cmd.angular.z = delta;
			cmd_pub.publish(cmd);
			// printf("phi: %f alpha: %f delta: %f linear: %f\n", phi, alpha, delta, cmd.linear.x);
		}
	}

	// cout << "err_lat: " << *max_element(err_lat.begin(), err_lat.end()) << endl;
	// cout << "cross_track: " << *max_element(cross_track.begin(), cross_track.end()) << endl;
	// for (int j = 0; j < track_x.size(); ++j) {
	// 	printf("%f ,", track_x[j]);
	// }

	// bag.write("Tracking", ros::Time::now(), test);
	// bag.close();
}