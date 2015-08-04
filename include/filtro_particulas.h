#ifndef FILTRO_PARTICULAS_H
#define FILTRO_PARTICULAS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"

#include "map_server/image_loader.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <random_numbers/random_numbers.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

using namespace std;

class Filtro_Particulas
{
	public:
		Filtro_Particulas(ros::NodeHandle n, double res);
		virtual ~Filtro_Particulas();

	public:
		void readLandmarks();
		void createParticles();

		void fakeLaser();
		void findObstacle();
		void moveParticles();
		void weightParticles();
		void resample();

		void gaussian(double mu, double sigma, double x);
		void set_noise(double move_noise, double turn_noise);

		void odomCallback (const nav_msgs::OdometryConstPtr& msg);
		void laserCallback (const sensor_msgs::LaserScanConstPtr& scan);
		//void min_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& min_xy);
		//void max_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& max_xy);
		void occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy);
		void free_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& free_coordxy);

		void spin();

	private:
		ros::NodeHandle n_;
		double res_;

		nav_msgs::Odometry odometry_;

		ros::Subscriber odom_sub_;
		ros::Subscriber scan_sub_;
		//ros::Subscriber min_xy_sub_;
		//ros::Subscriber max_xy_sub_;
		ros::Subscriber occ_coordxy_sub_;
		ros::Subscriber free_coordxy_sub_;

		ros::Publisher particle_cloud_;

		geometry_msgs::Pose2D single_pose_;
		geometry_msgs::Pose2D particle_pose_[1000];

		int landmarks_[4000][2];
		int landmarks_xy_[4000];
		int free_xy_[40000];
		int l_;
		int f_;
		int num_free_;
		int min_x_;
		int min_y_;
		int max_x_;
		int max_y_;
		int once_;
		int num_part_;
		geometry_msgs::Pose2D delta_pose_;
		geometry_msgs::Pose2D pose_anterior_;

		geometry_msgs::Pose2D pose;

		//int delta_y_;
		//int delta_theta_;

		float laser_data_[3];
		float pose_x_;
		float pose_y_;
		float pose_theta_;

		double gaussian_;
		double move_noise_;
		double turn_noise_;

};

#endif
