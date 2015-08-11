#ifndef FILTRO_PARTICULAS_H
#define FILTRO_PARTICULAS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "map_server/image_loader.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <random>
#include <cmath>
#include <cstdlib>

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
		double findObstacle(double x, double y);
		void moveParticles();
		void weightParticles();
		void resample();
		double measurementProb(int particleMP, int laserMP);

		double gaussian(double mu, double sigma, double x);
		double gaussian(double mu, double sigma);
		//void set_noise(double move_noise, double turn_noise);

		void odomCallback (const nav_msgs::OdometryConstPtr& msg);
		void laserCallback (const sensor_msgs::LaserScanConstPtr& scan);
		//void min_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& min_xy);
		//void max_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& max_xy);
		void occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy);
		void free_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& free_coordxy);

		void pubInicialPose();
		void cloud();

		void spin();




	private:
		ros::NodeHandle n_;
		double res_;

		int num_part_;
		int qtdd_laser_;
		double passo_base;
		int range_max_fakelaser; //[m]
		double error_particles_;

		nav_msgs::Odometry odometry_;

		ros::Subscriber odom_sub_;
		ros::Subscriber scan_sub_;
		//ros::Subscriber min_xy_sub_;
		//ros::Subscriber max_xy_sub_;
		ros::Subscriber occ_coordxy_sub_;
		ros::Subscriber free_coordxy_sub_;

		ros::Publisher particle_cloud_pub_;
		ros::Publisher initial_pose_pub_;

		geometry_msgs::Pose2D single_pose_;
		geometry_msgs::Pose2D particle_pose_[10000];
		geometry_msgs::Pose2D particle_resample_[10000];
		geometry_msgs::PoseWithCovarianceStamped initial_pose_;
		geometry_msgs::PoseStamped initial_pose2_;
		//geometry_msgs::Quaternion odom_quat_;

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
		int zerar;
		bool obstacle_finded_;
		int obstacle_;
		int achou;
		int loop;
		int cont;
		double total;
		double probt;
		double passo;
		double sum;

		int rand_xy;
		double pose_x;
		double pose_y;

		double x;
		double y;
		int xi;
		int yi;
		int i;
		int num_laser;


		geometry_msgs::Pose2D delta_pose_;
		geometry_msgs::Pose2D pose_anterior_;

		geometry_msgs::Pose2D pose;

		//int delta_y_;
		//int delta_theta_;

		double laser_data_[1000];
		geometry_msgs::Pose2D fake_laser_pose_[1000];
		double pose_x_;
		double pose_y_;
		double pose_theta_;
		double fake_laser_data_[10000][3];
		double gaussian_;
		double move_noise_;
		double turn_noise_;
		double laser_noise_;
		int size_occ_coordxy_;
		double weight_part_laser_[10000][3];
		double weight_part_[10000];
		double pool_[10000];
		double weight_ord_[10000];

		bool free_ok_;
		bool occ_ok_;
		bool odom_ok_;
		bool laser_ok_;
		int create_particle_ok_;



};

#endif
