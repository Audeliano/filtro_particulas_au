#include "filtro_particulas.h"

Filtro_Particulas::Filtro_Particulas(ros::NodeHandle n, double res)
{
	n_ = n;
	res_ = res;

//	min_xy_sub_ = n.subscribe("min_xy", 4000, &Filtro_Particulas::coordxCallback, this);
//	max_xy_sub_ = n.subscribe("max_xy", 4000, &Filtro_Particulas::coordyCallback, this);
	occ_coordxy_sub_ = n.subscribe("occ_coordxy", 4000, &Filtro_Particulas::occ_coordxyCallback, this);
	free_coordxy_sub_ = n.subscribe("free_coordxy", 4000, &Filtro_Particulas::free_coordxyCallback, this);
	scan_sub_ = n.subscribe("scan", 10, &Filtro_Particulas::laserCallback, this);
	odom_sub_ = n.subscribe("odom", 10, &Filtro_Particulas::odomCallback, this);

	once_ = 0;
	l_ = 0;
	f_ = 0;
	min_x_ = 10000;
	min_y_ = 10000;
	max_x_ = -10000;
	max_y_ = -10000;

	single_pose_.x = 0;
	single_pose_.y = 0;
	single_pose_.theta = 0;
	num_part_ = 30;
	num_free_ = 0;
	pose_x_ = 0;
	pose_y_ = 0;
	pose_theta_ = 0;
	gaussian_ = 0;
	move_noise_ = 0;
	turn_noise_ = 0;

	delta_pose_.x = 0;
	delta_pose_.y = 0;
	delta_pose_.theta = 0;
	pose_anterior_.x = 0;
	pose_anterior_.y = 0;
	pose_anterior_.theta = 0;

	pose.x = 0;
	pose.y = 0;
	pose.theta = 0;


}

Filtro_Particulas::~Filtro_Particulas()
{
	//min_xy_sub_.shutdown();
	//max_xy_sub_.shutdown();
	occ_coordxy_sub_.shutdown();
	free_coordxy_sub_.shutdown();
	scan_sub_.shutdown();
	odom_sub_.shutdown();
}
/*
void Filtro_Particulas::min_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& min_xy)
{
	//Carregar os valores de x dos landmarks.
	std::vector<int>::const_iterator cx = min_xy->data.begin();
	std::vector<int>::const_iterator cxm = min_xy->data.begin()+1;

	min_x_ = *cx;
	max_x_ = *cxm;

	return;
}

void Filtro_Particulas::max_xyCallback (const std_msgs::Int32MultiArray::ConstPtr& max_xy)
{
	//Carregar os valores de y dos landmarks.
	std::vector<int>::const_iterator cy = coordy->data.begin();
	std::vector<int>::const_iterator cym = coordy->data.begin()+1;

	min_y_ = *cy;
	max_y_ = *cym;

	l_ = 0;

	for(std::vector<int>::const_iterator it = coordy->data.begin()+2 ; it != coordy->data.end(); ++it){

		landmarks_[l_][1] = *it;
		l_++;
	}
	return;
}
*/

void Filtro_Particulas::occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy)
{
	//Carregar os valores de xy dos landmarks.

	l_ = 0;

	for(std::vector<int>::const_iterator it = occ_coordxy->data.begin() ; it != occ_coordxy->data.end(); ++it){

		landmarks_xy_[l_] = *it;
		l_++;
	}
	return;
}

void Filtro_Particulas::free_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& free_coordxy)
{
	f_ = 0;

	for(std::vector<int>::const_iterator it = free_coordxy->data.begin() ; it != free_coordxy->data.end(); ++it){

		free_xy_[f_] = *it;
		f_++;
	}
	num_free_ = f_;

	return;

}

void Filtro_Particulas::laserCallback (const sensor_msgs::LaserScanConstPtr& scan)
{
	// 1,57 / 0,006 ~ 260

	laser_data_[0] = scan -> ranges[0];		//+90º
	laser_data_[1] = scan -> ranges[255];	//0º
	laser_data_[2] = scan -> ranges[510];	//-90º

	//cout << laser_data_[0] << " | " << laser_data_[1] << " | " << laser_data_[2] <<endl;
}

void Filtro_Particulas::odomCallback (const nav_msgs::OdometryConstPtr& msg)
{
	pose_x_ = msg->pose.pose.position.x;
	pose_y_ = msg->pose.pose.position.y;
	pose_theta_ = tf::getYaw(msg->pose.pose.orientation); //em radianos
	//pose_theta_ = tf::getYaw(msg->pose.pose.orientation) * 180 / M_PI; //em graus

	//cout<<"x: "<<pose_x_<<" | y: "<<pose_y_<<" | theta: "<<pose_theta_<<endl;

}

void Filtro_Particulas::set_noise(double move_noise, double turn_noise)
{
	move_noise_ = move_noise;
	turn_noise_ = turn_noise;
}

void Filtro_Particulas::createParticles()
{
	//Criando partículas randômicas.
	if(once_ == 3)
	{

		//mudando a semente do random
		srand(time(NULL));

		int rand_xy = 0;
		double pose_x = 0;
		double pose_y = 0;

		for (int i = 0; i < num_part_; i++)
		{
			rand_xy = rand() % num_free_; //random de 0 a num_free_

			pose_x = (free_xy_[rand_xy])/10000; //separa x de y
			pose_y = (free_xy_[rand_xy])%10000;

			single_pose_.x = pose_x * res_; //1 pixel -> 0.05m
			single_pose_.y = pose_y * res_;
			single_pose_.theta = (rand() % 360 + 0) - 180; //em graus //
			single_pose_.theta = single_pose_.theta * M_PI / 180; //em radianos

			particle_pose_[i] = single_pose_;

			cout<<"x: "<<single_pose_.x<<" ; y: "<<single_pose_.y<<" ; theta: "<<single_pose_.theta<<endl;
			//cout<<"particle_pose["<<i<<"]:\n"<<particle_pose_[i]<<endl;
		}
	}
}

void Filtro_Particulas::readLandmarks()
{
	if(once_ == 3)
	{
		//Teste para verificar se a matriz foi carregada corretamente.
		for(int f=0;f<l_;f++){
			//cout << "[[" << landmarks_[f][0] << ", " << landmarks_[f][1] << "]] ";
			cout << " (" << landmarks_xy_[f] << ")" << endl;

		}
		cout<<"\nl_: "<<l_<<endl;
		//cout << "max: " << max_x_ << "; " << max_y_ << endl;
		//cout << "min: " << min_x_ << "; " << min_y_ << endl;
		cout<<endl;
		cout<<endl;
		cout<<endl;
		cout<<endl;
	}
}

void Filtro_Particulas::gaussian(double mu, double sigma, double x)
{
	gaussian_ = exp(- (pow((mu - x), 2) / pow(sigma, 2) / 2)) / sqrt(2 * M_PI * pow(sigma, 2));
}

void Filtro_Particulas::fakeLaser()
{
	//laser_data_[0] +90º
	//laser_data_[1] 0º
	//laser_data_[2] -90º



	for (int i = 0; i < num_part_; i++)
	{
		//pose.theta de cada fake_laser
		fake_laser_pose_[0].theta = (M_PI / 2) + particle_pose_[i].theta;
		if(fake_laser_pose_[0].theta > M_PI)
			fake_laser_pose_[0].theta -= 2 * M_PI;
		if(fake_laser_pose_[0].theta <= - M_PI)
			fake_laser_pose_[0].theta += 2 * M_PI;

		fake_laser_pose_[1].theta = 0 + particle_pose_[i].theta;

		fake_laser_pose_[2].theta = - (M_PI / 2) + particle_pose_[i].theta;
		if(fake_laser_pose_[2].theta > M_PI)
			fake_laser_pose_[2].theta -= 2 * M_PI;
		if(fake_laser_pose_[2].theta <= - M_PI)
			fake_laser_pose_[2].theta += 2 * M_PI;

		fake_laser_pose_[0].x = fake_laser_pose_[1].x = fake_laser_pose_[2].x = particle_pose_[i].x;
		fake_laser_pose_[0].y = fake_laser_pose_[1].y = fake_laser_pose_[2].y = particle_pose_[i].y;






	}
}

void Filtro_Particulas::moveParticles()
{
	int pose_x = 2;
	pose_x_ += pose_x;

	delta_pose_.x = pose_x_ - pose_anterior_.x; //+ random.gauss(0,0);
	delta_pose_.y = pose_y_ - pose_anterior_.y; //+ random.gauss(0,0);
	delta_pose_.theta = pose_theta_ - pose_anterior_.theta; //+ random.gauss(0,0);
	//cout<<"delta_pose.x: "<<delta_pose_.x<<" ; delta_pose_.y: "<<delta_pose_.y<<" ; delta_pose_.theta: "<<delta_pose_.theta<<endl;

	pose_anterior_.x = pose_x_;
	pose_anterior_.y = pose_y_;
	pose_anterior_.theta = pose_theta_;
	//cout<<"pose_anterior_.x: "<<pose_anterior_.x<<" ; pose_anterior_.y: "<<pose_anterior_.y<<" ; pose_anterior_.theta: "<<pose_anterior_.theta<<endl;

	int p = 0;
	for(p = 0; p < num_part_; p++)
	{
		particle_pose_[p].x += delta_pose_.x;
		particle_pose_[p].y += delta_pose_.y;
		particle_pose_[p].theta += delta_pose_.theta;
		//cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p]<<endl;
	}

	cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p-1]<<endl;
}

void Filtro_Particulas::spin()
{
	ros::Rate loopRate(5);

	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();
		//readLandmarks();
		createParticles();
		moveParticles();

		if(once_ < 5){once_++;}
	}

}
