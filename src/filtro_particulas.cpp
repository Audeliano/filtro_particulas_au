#include "filtro_particulas.h"

Filtro_Particulas::Filtro_Particulas(ros::NodeHandle n, double res)
{
	n_ = n;
	res_ = res;

	occ_coordxy_sub_ = n.subscribe("occ_coordxy", 4000, &Filtro_Particulas::occ_coordxyCallback, this);
	free_coordxy_sub_ = n.subscribe("free_coordxy", 4000, &Filtro_Particulas::free_coordxyCallback, this);
	scan_sub_ = n.subscribe("scan", 10, &Filtro_Particulas::laserCallback, this);
	odom_sub_ = n.subscribe("odom", 10, &Filtro_Particulas::odomCallback, this);

	initial_pose_pub_ = n.advertise<geometry_msgs::Pose2D>("initialpose2", 1, true);
	particle_cloud_pub_ = n.advertise<geometry_msgs::PoseArray>("particlecloudAU", 2, true);

//--------------------------------------------------------------------------------//

	num_part_ = 300;
	qtdd_laser_ = 101;

	passo_base = 0.015;
	range_max_fakelaser = 5; //[m]
	laser_noise_ = qtdd_laser_ / 2;

	laser_data_noise_ = 0.05;
	move_noise_ = 0.05;
	turn_noise_ = 0.1; //5.73°

	error_particles_ = 0.15;

//--------------------------------------------------------------------------------//


	num_laser = 0;
	ang_max_ = 0;
	zerar = 0;
	l_ = 0;
	f_ = 0;
	min_x_ = 10000;
	min_y_ = 10000;
	max_x_ = -10000;
	max_y_ = -10000;

	single_pose_.x = 0;
	single_pose_.y = 0;
	single_pose_.theta = 0;
	num_free_ = 0;
	pose_x_ = 0;
	pose_y_ = 0;
	pose_theta_ = 0;
	gaussian_ = 0;



	delta_pose_.x = 0;
	delta_pose_.y = 0;
	delta_pose_.theta = 0;
	pose_anterior_.x = 0;
	pose_anterior_.y = 0;
	pose_anterior_.theta = 0;

	pose.x = 0;
	pose.y = 0;
	pose.theta = 0;

	obstacle_finded_ = false;

	size_occ_coordxy_ = 0;
	obstacle_ = 0;
	achou = 0;
	loop = 0;
	cont = 0;
	total = 0;
	probt = 0;
	passo = 0;
	sum = 0;

	rand_xy = 0;
	pose_x = 0;
	pose_y = 0;

	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;

	occ_ok_ = false;
	odom_ok_ = false;
	laser_ok_ = false;
	free_ok_ = false;
	create_particle_ok_ = 1;

}

Filtro_Particulas::~Filtro_Particulas()
{
	occ_coordxy_sub_.shutdown();
	free_coordxy_sub_.shutdown();
	scan_sub_.shutdown();
	odom_sub_.shutdown();
	initial_pose_pub_.shutdown();
	particle_cloud_pub_.shutdown();
}

void Filtro_Particulas::occ_coordxyCallback (const std_msgs::Int32MultiArray::ConstPtr& occ_coordxy)
{
	//Carregar os valores de xy dos landmarks.

	l_ = 0;

	for(std::vector<int>::const_iterator it = occ_coordxy->data.begin() ; it != occ_coordxy->data.end(); ++it){

		landmarks_xy_[l_] = *it;
		l_++;
	}
	occ_ok_ = true;
	//cout<<"sizeof: "<<l_<<endl;

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
	free_ok_ = true;

	return;
}

void Filtro_Particulas::laserCallback (const sensor_msgs::LaserScanConstPtr& scan)
{
	// 1,57 / 0,006 ~ 260
	ang_max_ = scan -> angle_max;
	int it = 510 / (qtdd_laser_ - 1);

	for (int laser_num = 0 ; laser_num < qtdd_laser_ ; laser_num++)
	{
		laser_data_[laser_num] = scan -> ranges[laser_num * it] + (scan -> ranges[laser_num * it] * gaussian(0.0, laser_data_noise_));
		//cout<<"laser_data_["<<laser_num<<"]: "<<laser_data_[laser_num]<<endl<<endl;
	}


//	laser_data_[0] = scan -> ranges[0]; //+ gaussian(0.0, laser_noise_);	//+90º
//	laser_data_[1] = scan -> ranges[1];// + gaussian(0.0, laser_noise_);	//0º
//	laser_data_[2] = scan -> ranges[2];// + gaussian(0.0, laser_noise_);	//-90º

//	laser_data_[3] = scan -> ranges[127] + gaussian(0.0, laser_noise_);	//45º
//	laser_data_[4] = scan -> ranges[382] + gaussian(0.0, laser_noise_);	//-45º

	laser_ok_ = true;
	//cout << laser_data_[0] << " | " << laser_data_[1] << " | " << laser_data_[2] <<endl;
}

void Filtro_Particulas::odomCallback (const nav_msgs::OdometryConstPtr& msg)
{
	pose_x_ = msg->pose.pose.position.x;// + gaussian(0.0, move_noise_);
	pose_y_ = msg->pose.pose.position.y;// + gaussian(0.0, move_noise_);
	pose_theta_ = tf::getYaw(msg->pose.pose.orientation);// + gaussian(0.0, move_noise_); //em radianos

//	cout<<"theta: "<<pose_theta_<<" ; quat: "<<quat<<endl;

	//cout<<"x: "<<pose_x_<<" | y: "<<pose_y_<<" | theta: "<<pose_theta_<<endl;
	odom_ok_ = true;
}

void Filtro_Particulas::createParticles()
{
	//Criando partículas randômicas.

	//mudando a semente do random
	if(create_particle_ok_ == 1){
		cout<<"Criei as partículas!##########@@@@@@@@@@!!!!!!!!!!!"<<endl;
		srand(time(NULL));

		rand_xy = 0;
		pose_x = 0;
		pose_y = 0;


		for (int i = 0; i < num_part_; i++)
		{
			rand_xy = rand() % num_free_; //random de 0 a num_free_

			pose_x = (free_xy_[rand_xy])/10000; //separa x de y
			pose_y = (free_xy_[rand_xy])%10000;

			single_pose_.x = pose_x * res_; //1 pixel -> 0.05m
			single_pose_.y = pose_y * res_;
			single_pose_.theta = (rand() % 360 + 0) - 180; //em graus
			single_pose_.theta = single_pose_.theta * M_PI / 180; //em radianos

			particle_pose_[i] = single_pose_;

			//cout<<"x: "<<single_pose_.x<<" ; y: "<<single_pose_.y<<" ; theta: "<<single_pose_.theta<<endl;
			//cout<<"particle_pose["<<i<<"]:\n"<<particle_pose_[i]<<endl;
		}
	}
	create_particle_ok_ = 0;
}

double Filtro_Particulas::gaussian(double mu, double sigma, double x)
{
	gaussian_ = (exp(- (pow((mu - x), 2) / pow(sigma, 2) / 2.0))) / sqrt(2.0 * M_PI * pow(sigma, 2));

	//cout<<"Gaussian (mu,sigma,x): mu: "<<mu<<" ; laser_Data: "<<x<<" ; gaussian3: "<<gaussian_<<endl;
	//usleep(25000);
	//cout<<mu<<endl;

	return gaussian_;

}

double Filtro_Particulas::gaussian(double mu, double sigma)
{
	std::random_device rd;
	std::mt19937 gen(rd());

	std::normal_distribution<double> d(mu,sigma);

	double number = d(gen);
	//cout<<"Gaussian(mu,sigma): sigma: "<<sigma<<" ; gaussian2: "<<number<<endl;
	return number;
}

void Filtro_Particulas::fakeLaser()
{
	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;
	num_laser = 0;
	achou = 0;
	total = 0;

	if(cont >= 30)
	{
		create_particle_ok_ = 1;
		cont = 0;
	}

	for (int i = 0; i < num_part_; i++)
	{
		//pose.theta de cada fake_laser
/*		fake_laser_pose_[0].theta = (M_PI / 2.0) + particle_pose_[i].theta;
		if(fake_laser_pose_[0].theta > M_PI)
			fake_laser_pose_[0].theta -= 2.0 * M_PI;
		if(fake_laser_pose_[0].theta <= - M_PI)
			fake_laser_pose_[0].theta += 2.0 * M_PI;

		fake_laser_pose_[1].theta = 0 + particle_pose_[i].theta;

		fake_laser_pose_[2].theta = - (M_PI / 2.0) + particle_pose_[i].theta;
		if(fake_laser_pose_[2].theta > M_PI)
			fake_laser_pose_[2].theta -= 2.0 * M_PI;
		if(fake_laser_pose_[2].theta <= - M_PI)
			fake_laser_pose_[2].theta += 2.0 * M_PI;

		//pose.x e pose.y para cada fake_laser
		fake_laser_pose_[0].x = fake_laser_pose_[1].x = fake_laser_pose_[2].x = particle_pose_[i].x;
		fake_laser_pose_[0].y = fake_laser_pose_[1].y = fake_laser_pose_[2].y = particle_pose_[i].y;
*/
		//cout<<"num_particle: "<<i<<" | fake_laser_pose_[0]: \n"<< fake_laser_pose_[0]<<endl;
		//cout<<"num_particle: "<<i<<" | fake_laser_pose_[1]: \n"<< fake_laser_pose_[1]<<endl;
		//cout<<"num_particle: "<<i<<" | fake_laser_pose_[2]: \n"<< fake_laser_pose_[2]<<endl;

		probt = 1.0;

		double it = M_PI / (qtdd_laser_ - 1);

		for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
		{
			fake_laser_pose_[num_laser].theta = ((ang_max_) - (num_laser * it)) + particle_pose_[i].theta;
			if(fake_laser_pose_[num_laser].theta > M_PI)
				fake_laser_pose_[num_laser].theta -= 2.0 * M_PI;
			if(fake_laser_pose_[num_laser].theta <= - M_PI)
				fake_laser_pose_[num_laser].theta += 2.0 * M_PI;

			fake_laser_pose_[num_laser].x = particle_pose_[i].x;
			fake_laser_pose_[num_laser].y = particle_pose_[i].y;
		}

		for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
		{
			passo = 0;
			int iteracao = range_max_fakelaser / passo_base;

			for(int p = 1; p <= iteracao; p++)
			{
				//varredura do fake_laser
				passo = passo_base * p; //+random.gauss
				//passo += gaussian(0.0, passo_base);
				//cout<<"passo: "<<passo<<endl;
				x = fake_laser_pose_[num_laser].x + (cos(fake_laser_pose_[num_laser].theta) * passo);
				y = fake_laser_pose_[num_laser].y + (sin(fake_laser_pose_[num_laser].theta) * passo);
				//if(x >= 0 && y >= 0)
				{
					//cout<<"Nao arredondado--- "<<"x: "<<x<<"; y: "<<y<<endl;
					//arredondando os valores de x e y
					xi = x / res_;
					yi = y / res_;
					//cont++;
					//cout<<"Arredondado--- "<<"xi: "<<xi<<"; yi: "<<yi<<" cont: "<<cont<<endl;

					findObstacle(xi, yi);
					if (obstacle_finded_ == true){
						fake_laser_data_[i][num_laser] = obstacle_;
						//weight_part_laser_[i][num_laser] = passo;// += gaussian(0.0, laser_noise_);

						//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;
						p = iteracao;

					}else fake_laser_data_[i][num_laser] = 0;
				}
			}
			weight_part_laser_[i][num_laser] = passo;
			//weight_part_laser_[i][num_laser] += gaussian(0.0, laser_noise_);

			//cout<<"Part["<<i<<"]["<<num_laser<<"] = "<<passo<<" | laser_data["<<num_laser<<"] = "<<laser_data_[num_laser]<<endl;
			//usleep(100000);

			measurementProb(i,num_laser);

			//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;

			//usleep(100000);
			//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;
			//cout<<"fake_laser_data_["<<i<<"]"<<"["<<num_laser<<"]: "<<fake_laser_data_[i][num_laser]<<endl;
		}
		//cout<<"\n"<<endl;
		weight_part_[i] = probt;
		total += weight_part_[i];
		//cout<<"weight_part_"<<i<<" | probt: "<<probt<<endl;
		//usleep(25000);

	}
	//cout<<"ACHOU OBST: "<<achou<<endl;
	//cout<<"Fake_laser()"<<endl;
}

double Filtro_Particulas::findObstacle(double x, double y)
{
	int esq, meio, dir;
	esq = 0;
	dir = l_ - 1;
	//cout<<"x: "<<x<<" ; y: "<<y<<endl;

	obstacle_ = (10000 * x) + y;
	//cout<<"obstacle: "<<obstacle_<<endl;
	while (esq <= dir){
		//cout<<"esq: "<<esq<<" ; dir: "<<dir<<endl;
		//cout<<"lesq: "<<landmarks_xy_[esq]<<" ; ldir: "<<landmarks_xy_[dir]<<endl;
		//usleep(250000);
		meio = (esq + dir) / 2;
		if ( landmarks_xy_[meio] == obstacle_)
		{
			obstacle_finded_ = true;
			achou++;
			return obstacle_;
		}
		if ( landmarks_xy_[meio] < obstacle_) esq = meio + 1;
		else dir = meio - 1;
	}
	//cout<<"findObstacle"<<endl;
	obstacle_finded_ = false;
	return -1;
}

double Filtro_Particulas::measurementProb(int particleMP, int laserMP)
{

	probt *= gaussian(weight_part_laser_[particleMP][laserMP], laser_noise_, laser_data_[laserMP]);
	//usleep(250000);
	//cout<<"weight["<<p<<"]["<<l<<"]: "<<weight_part_laser_[p][l]<<" | "<<laser_noise_<<" | "<<laser_data_[l]<<" | prob: "<<prob<<endl;
	//cout<<"Particula: "<<p<<" | Num_laser: "<<l<<" ; Data: "<<laser_data_[l]<<" | Peso: "<<weight_part_laser_[p][l]<<endl;

	return probt;
}

void Filtro_Particulas::resample()
{
	double soma = 0;
	double max_w = -1;
	for(int n = 0 ; n < num_part_ ; n++)
	{
		//Normaliza os pesos
		weight_part_[n] = weight_part_[n] / total;
		//cout<<"weight_part_ "<<n<<" : "<<weight_part_[n]<<endl;
		//usleep(25000);

		if(weight_part_[n] > max_w){
			max_w = weight_part_[n];
			//cout<<"max_w: "<<max_w<<endl;
			//cout<<"weight_part_ "<<i<<" : "<<weight_part_[i]<<" Particle_pose-> x: "<<particle_pose_[i].x<<" y: "<<particle_pose_[i].y<<" theta: "<<particle_pose_[i].theta<<endl;
		}
		//soma += weight_part_[n];
		//usleep(200000);
		//cout<<"Normaliz: "<<n<<" ; prob: "<<weight_part_[n]<<" ; Soma: "<<soma<<endl;
	}

	int index_b = 0;
	int indexi = 0;
	srand(time(NULL));
	index_b = rand() % 101;
	indexi = index_b * num_part_ / 100;
	//cout<<"index_b: "<<index_b<<endl;
	//cout<<"indexi: "<<indexi<<endl;
	int beta_b = 0;
	double beta = 0;

	for (int p = 0; p < num_part_; p++)
	{
		srand(time(NULL));
		beta_b = rand() % 101;
		//cout<<"beta_b: "<<beta_b<<endl;
		beta += (beta_b) * 2.0 * max_w / 100.0;
		//cout<<"beta: "<<beta<<endl;
		while(beta > weight_part_[indexi])
		{
			beta -= weight_part_[indexi];
			//cout<<"beta-: "<<beta<<endl;
			indexi = (indexi + 1) % num_part_;
			//cout<<"index+: "<<indexi<<endl;
		}
		particle_resample_[p] = particle_pose_[indexi];
		//particle_resample_[p].x = particle_pose_[indexi].x;
		//particle_resample_[p].y = particle_pose_[indexi].y;
		//particle_resample_[p].theta = particle_pose_[indexi].theta;
		//usleep(2000000);
		//cout<<"partic: "<<p<<" indexi: "<<indexi<<" Particle_pose-> x: "<<particle_pose_[indexi].x<<" y: "<<particle_pose_[indexi].y<<" theta: "<<particle_pose_[indexi].theta<<endl;
	}
	for(int n = 0 ; n < num_part_ ; n++){
		particle_pose_[n] = particle_resample_[n];
		//particle_pose_[n].x = particle_resample_[n].x;
		//particle_pose_[n].y = particle_resample_[n].y;
		//particle_pose_[n].theta = particle_resample_[n].theta;
		//usleep(3000000);
		//cout<<"ResampleParticle: "<<n<<" | x: "<<particle_pose_[n].x<<" ; y: "<<particle_pose_[n].y<<" ; theta: "<<particle_pose_[n].theta<<endl;
		//cout<<"ResampleParticle: "<<n-1<<" | x: "<<particle_pose_[n-1].x<<" ; y: "<<particle_pose_[n-1].y<<" ; theta: "<<particle_pose_[n-1].theta<<endl;
	}
	//cout<<"resample"<<endl;

}

void Filtro_Particulas::moveParticles()
{
	delta_pose_.x = pose_x_ - pose_anterior_.x;
	delta_pose_.y = pose_y_ - pose_anterior_.y;
	delta_pose_.theta = pose_theta_ - pose_anterior_.theta;
	//cout<<"delta_pose.x: "<<delta_pose_.x<<" ; delta_pose_.y: "<<delta_pose_.y<<" ; delta_pose_.theta: "<<delta_pose_.theta<<endl;

	pose_anterior_.x = pose_x_;
	pose_anterior_.y = pose_y_;
	pose_anterior_.theta = pose_theta_;
	//cout<<"pose_anterior_.x: "<<pose_anterior_.x<<" ; pose_anterior_.y: "<<pose_anterior_.y<<" ; pose_anterior_.theta: "<<pose_anterior_.theta<<endl;

	int p = 0;
	if(delta_pose_.x != 0 || delta_pose_.y != 0 || delta_pose_.theta != 0){
		for(p = 0; p < num_part_; p++)
		{
			particle_pose_[p].x += delta_pose_.x + (delta_pose_.x * gaussian(0.0, move_noise_));
			particle_pose_[p].y += delta_pose_.y + (delta_pose_.y * gaussian(0.0, move_noise_));
			particle_pose_[p].theta += delta_pose_.theta; //+ gaussian(0.0, turn_noise_);
			if(particle_pose_[p].theta > M_PI)
				particle_pose_[p].theta -= 2.0 * M_PI;
			if(particle_pose_[p].theta <= - M_PI)
				particle_pose_[p].theta += 2.0 * M_PI;

			//cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p]<<endl;
			//usleep(150000);
			//cout<<"MoveParticle: "<<p<<" | x: "<<particle_pose_[p].x<<" ; y: "<<particle_pose_[p].y<<" ; theta: "<<particle_pose_[p].theta<<endl;

		}
		//cout<<"Deltas_pose: "<<delta_pose_.x<<" | "<<delta_pose_.y<<" | "<<delta_pose_.theta<<endl;
		//cout<<"Gaussians(move e turn): "<<gaussian(0.0, move_noise_)<<" | "<<gaussian(0.0, turn_noise_)<<endl;
		//usleep(250000);

		//cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p-1]<<endl;
		//cout<<"moveParticulas"<<endl;
	}
}

void Filtro_Particulas::pubInicialPose()
{
	double xmedia = 0;
	double ymedia = 0;
	double thetamedia = 0;
	sum = 0.0;

	for (int i = 0; i < num_part_ ; i++)
	{
		xmedia += particle_pose_[i].x;
		ymedia += particle_pose_[i].y;
		thetamedia += particle_pose_[i].theta;
		if(thetamedia > M_PI)
			thetamedia -= 2.0 * M_PI;
		if(thetamedia <= - M_PI)
			thetamedia += 2.0 * M_PI;
	}

	xmedia = xmedia / (double)num_part_;
	ymedia = ymedia / (double)num_part_;
	thetamedia = thetamedia / (double)num_part_;

	for (int i = 0; i < num_part_ ; i++)
	{
		double dx = particle_pose_[i].x - xmedia;
		//cout<<"partx - xmedia: "<<particle_pose_[i].x<<" - "<<xmedia<<endl;
		double dy = particle_pose_[i].y - ymedia;
		//cout<<"party - ymedia: "<<particle_pose_[i].y<<" - "<<ymedia<<endl;
		double err = sqrt( (dx * dx) + (dy * dy) );
		//cout<<"erro: "<<err<<endl;
		sum += err;
		//cout<<"sum: "<<sum<<endl;
	}
	sum = sum / num_part_;
	//cout<<"sum: "<<sum<<endl;
	//usleep(250000);

	//odom_quat_= tf::createQuaternionMsgFromYaw(thetamedia);

	//initial_pose_.pose.pose.position.x = xmedia;
	//initial_pose_.pose.pose.position.y = ymedia;
	//initial_pose_.pose.pose.orientation = odom_quat_;

	//initial_pose_pub_.publish(initial_pose_);
	if(sum < error_particles_)
	{
		initial_pose2_.x = xmedia;
		initial_pose2_.y = ymedia;
		initial_pose2_.theta = thetamedia;

		initial_pose_pub_.publish(initial_pose2_);

		zerar++;
		if(zerar > 10)
		{
			cont = 0;
		}

	}else{
		cont++;
		zerar = 0;
	}
	cout<<"cont: "<<cont<<" | sum: "<<sum<<endl;
}

void Filtro_Particulas::cloud()
{
	geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
	cloud_msg.poses.resize(num_part_);
	for(int i = 0;i<num_part_;i++)
	{
		tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(particle_pose_[i].theta),
				tf::Vector3(particle_pose_[i].x, particle_pose_[i].y, 0)),cloud_msg.poses[i]);
	}
	particle_cloud_pub_.publish(cloud_msg);
}

void Filtro_Particulas::spin()
{
	ros::Rate loopRate(10);
	pose_anterior_.x = pose_x_;
	pose_anterior_.y = pose_y_;
	pose_anterior_.theta = pose_theta_;

	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();

		cout<<free_ok_<<occ_ok_<<odom_ok_<<laser_ok_<<endl;
		if (free_ok_ == true && occ_ok_ == true && odom_ok_ == true && laser_ok_ == true){
			createParticles();
			if(create_particle_ok_ == 0){
				fakeLaser();
				//cout<<"fakeLaser()"<<endl;
				cloud();
				moveParticles();
				//cout<<"moveParticles()"<<endl;
				resample();
				//cout<<"resample()"<<endl;
				pubInicialPose();
				//cout<<"pubinitialPose()"<<endl;
			}
		}
	}
}
