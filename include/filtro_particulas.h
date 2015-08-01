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

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;

class Filtro_Particulas
{
	public:
		Filtro_Particulas(ros::NodeHandle n);
		virtual ~Filtro_Particulas();

	public:
		//funções para criar partículas
		void readLandmarks();
		void createParticles();
		void fakeLaser();
		void findObstacle();

		//funções para atualizar as partículas
		void odomCallback (const nav_msgs::OdometryConstPtr& msg);

		void coordxCallback (const std_msgs::Int32MultiArray::ConstPtr& coordx);
		void coordyCallback (const std_msgs::Int32MultiArray::ConstPtr& coordy);
		void coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& coordxy);

		void spin();

	private:
		ros::NodeHandle n_;
		nav_msgs::Odometry odometry_;

		ros::Subscriber odom_sub_;
		ros::Subscriber scan_sub_;
		ros::Subscriber coordx_sub_;
		ros::Subscriber coordy_sub_;
		ros::Subscriber coordxy_sub_;

		ros::Publisher particle_cloud_;

		geometry_msgs::Pose2D single_pose_;
		geometry_msgs::Pose2D particle_pose_[1000];

		int landmarks_[4000][2];
		int landmarks_xy_[4000];
		int l_;
		int min_x_;
		int min_y_;
		int max_x_;
		int max_y_;
		int once_;
		int num_part_;

};

#endif
