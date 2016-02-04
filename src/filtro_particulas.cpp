#include "filtro_particulas.h"

Filtro_Particulas::Filtro_Particulas(ros::NodeHandle n)
{
	n_ = n;

	occ_coordxy_sub_ = n.subscribe("occ_coordxy", 4000, &Filtro_Particulas::occ_coordxyCallback, this);
	free_coordxy_sub_ = n.subscribe("free_coordxy", 4000, &Filtro_Particulas::free_coordxyCallback, this);
	scan_sub_ = n.subscribe("scan", 10, &Filtro_Particulas::laserCallback, this);
	odom_sub_ = n.subscribe("odom", 10, &Filtro_Particulas::odomCallback, this);
	map_meta_data_sub_ = n.subscribe("map_metadata", 10, &Filtro_Particulas::mapCallback, this);

	initial_pose_pub_ = n.advertise<geometry_msgs::Pose2D>("filterparticlepose", 1, true);
	particle_cloud_pub_ = n.advertise<geometry_msgs::PoseArray>("particlecloudAU", 2, true);

//--------------------------------------------------------------------------------//
	freq_ = 20.0;

	num_part_ = 350;
	qtdd_laser_ = 10;
	res_ = 0.0;//resolution_.resolution;
	passo_base = 0.0;//5*res_; //0.05;//0.025;
	//cout<<"map resolution: "<<res_<<endl;
	range_max_fakelaser = 5.6; //[m]
	laser_noise_ = qtdd_laser_;

	laser_data_noise_ = 0.05;
	move_noise_ = 0.07;//0.03;
	turn_noise_ = 0.1;//0.03; //0.1 rad = 5.73°

	error_particles_ = 0.14; //0.45 ~ dist de 0.3m da particula (na media) ; 0.28 ~ 0.2m; 0.14 ~ 0.1m

//--------------------------------------------------------------------------------//

	reduz_gauss_ = 1.0;
	arctan_ = 0.0;
	hipot_ = 0.0;
	num_laser = 0;
	ang_min_ = 0;
	convergiu_ = 0;
	l_ = 0;
	f_ = 0;
	min_x_ = 10000;
	min_y_ = 10000;
	max_x_ = -10000;
	max_y_ = -10000;
	num_energy_ = 0;

	single_pose_.x = 0;
	single_pose_.y = 0;
	single_pose_.theta = 0;
	num_free_ = 0;
	pose_x_ = 0;
	pose_y_ = 0;
	pose_theta_ = 0;
	gaussian_ = 0;

	twist_x_ = 0.0;

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
	index_max_w_ = 0;

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
	zerar_deltas_ = false;
	create_particle_ok_ = 1;
	grids_ok_ = false;

}

Filtro_Particulas::~Filtro_Particulas()
{
	occ_coordxy_sub_.shutdown();
	free_coordxy_sub_.shutdown();
	scan_sub_.shutdown();
	odom_sub_.shutdown();
	map_meta_data_sub_.shutdown();
	initial_pose_pub_.shutdown();
	particle_cloud_pub_.shutdown();
}

void Filtro_Particulas::mapCallback(const nav_msgs::MapMetaDataConstPtr& msg)
{
	map_meta_data_ = msg->resolution;
	res_ = map_meta_data_;
	passo_base = 5*res_;
	cout<<"map resolution: "<<res_<<endl;
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
	//cout<<"num_free_coordxy: "<<num_free_<<endl;
	free_ok_ = true;

	return;
}

void Filtro_Particulas::laserCallback (const sensor_msgs::LaserScanConstPtr& scan)
{
	// 1,57 / 0,006 ~ 260
	// 3,14 / 0,01 = 360
	ang_min_ = scan -> angle_min;
	//int it = (scan->ranges.size() - 1) / (qtdd_laser_ - 1); //259 / (qtdd_laser_ - 1)
	int it = (scan->ranges.size()) / (qtdd_laser_); // 360 / (qtdd_laser_)

	cout<<"scan->ranges.size(): "<<scan->ranges.size()<<endl;

	for (int laser_num = 0 ; laser_num < qtdd_laser_ ; laser_num++)
	{
		if(scan -> ranges[laser_num * it] >= scan->range_min && scan -> ranges[laser_num * it] <= scan->range_max)
		{
			laser_data_[laser_num] = scan -> ranges[laser_num * it];
			//cout<<"LaserData:  "<<laser_num<<" = "<<laser_data_[laser_num]<<endl;
		}
		else
		{
			laser_data_[laser_num] = -1;
			//cout<<"DEU NAN OU INF!!!!  "<<laser_num<<" = "<<laser_data_[laser_num]<<endl;
		}

/*		if( isnan(scan -> ranges[laser_num * it]))
		{
			cout<<laser_data_[laser_num]<<endl;
			if(laser_data_[laser_num] < 5.6){
				laser_data_[laser_num] = 10.0;}//scan->range_min;}

			cout<<"Part ["<<laser_num<<"] isnan!!!! Laser_data: "<<laser_data_[laser_num]<<endl<<endl;
		}
		else laser_data_[laser_num] = scan -> ranges[laser_num * it];

		if(isinf(scan -> ranges[laser_num * it]))
		{
			cout<<laser_data_[laser_num]<<endl;
			laser_data_[laser_num] = scan->range_max;
			cout<<"Part ["<<laser_num<<"] isinf!!!! Laser_data: "<<laser_data_[laser_num]<<endl<<endl;
		}
		else laser_data_[laser_num] = scan -> ranges[laser_num * it]; //+ (scan -> ranges[laser_num * it] * gaussian(0.0, laser_data_noise_));
		//cout<<"laser_data_["<<laser_num<<"]: "<<laser_data_[laser_num]<<endl<<endl;
*/
	}
	//cout<<endl;

	laser_ok_ = true;
	//cout << laser_data_[0] << " | " << laser_data_[1] << " | " << laser_data_[2] <<endl;
}

void Filtro_Particulas::odomCallback (const nav_msgs::OdometryConstPtr& msg)
{
	pose_x_ = msg->pose.pose.position.x;// + gaussian(0.0, move_noise_);
	pose_y_ = msg->pose.pose.position.y;// + gaussian(0.0, move_noise_);
	pose_theta_ = tf::getYaw(msg->pose.pose.orientation);// + gaussian(0.0, move_noise_); //em radianos

	twist_x_ = msg->twist.twist.linear.x;
	//cout<<"TWIST: "<<twist_x_<<endl;

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


/*			rand_xy = rand() % num_free_; //random de 0 a num_free_

			pose_x = 12.0;//(free_xy_[rand_xy])/10000; //separa x de y
			pose_y = 38.0;//(free_xy_[rand_xy])%10000;

			single_pose_.x = pose_x * res_ ; //1 pixel -> 0.05m
			single_pose_.y = pose_y * res_;
			//single_pose_.theta = (rand() % 360 + 0) - 180; //em graus
			single_pose_.theta = 0; //em radianos

			particle_pose_[0] = single_pose_;

			//cout<<"x: "<<single_pose_.x<<" ; y: "<<single_pose_.y<<" ; theta: "<<single_pose_.theta<<endl;
			//cout<<"particle_pose["<<i<<"]:\n"<<particle_pose_[i]<<endl;
*/

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
		cloud();
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

	for (int i = 0; i < num_part_; i++)
	{
		//probt = 1.0;
		probt = 0.0;

		double it = M_PI / (qtdd_laser_);

		for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
		{
			fake_laser_pose_[num_laser].theta = ((ang_min_) + (num_laser * it)) + particle_pose_[i].theta;
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
			int iteracao = range_max_fakelaser / passo_base; // 5.6 / 0.06 = 112

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
			weight_part_laser_[i][num_laser] = passo; //DISTANCIA FAKE DE CADA FEIXE DO LASER VIRTUAL!!!
			//weight_part_laser_[i][num_laser] += gaussian(0.0, laser_noise_);

			//cout<<"Part["<<i<<"]["<<num_laser<<"] = "<<passo<<" | laser_data["<<num_laser<<"] = "<<laser_data_[num_laser]<<endl;
			//usleep(100000);

			if(laser_data_[num_laser] > 0)
				measurementProb(i,num_laser);
			//else cout<<"Valores NAN e INF nao entram no measurementProb!!! Part: "<<num_part_<<"  Laser: "<<num_laser<<endl;
			//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;
			//usleep(100000);
			//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;
			//cout<<"fake_laser_data_["<<i<<"]"<<"["<<num_laser<<"]: "<<fake_laser_data_[i][num_laser]<<endl;
		}
		//cout<<"\n"<<endl;
		probt = 1/probt;

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
	probt +=  fabs(weight_part_laser_[particleMP][laserMP] - laser_data_[laserMP]);

	//probt *= gaussian(laser_data_[laserMP], laser_noise_, weight_part_laser_[particleMP][laserMP]);
	//usleep(250000);
	//cout<<"laser_virtual_["<<particleMP<<"]["<<laserMP<<"]: "<<weight_part_laser_[particleMP][laserMP]<<" | "<<laser_noise_<<" | "<<laser_data_[laserMP]<<" | prob: "<<probt<<endl;
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
		//usleep(200000);
		//cout<<"Normaliz: "<<n<<" ; prob: "<<weight_part_[n]<<" ; Soma: "<<soma<<endl;

		if(weight_part_[n] > max_w){
			max_w = weight_part_[n];
			index_max_w_ = n;
			//cout<<"max_w: "<<max_w<<endl;
			//cout<<"weight_part_ "<<n<<" : "<<weight_part_[n]<<" Particle_pose-> x: "<<particle_pose_[n].x<<" y: "<<particle_pose_[n].y<<" theta: "<<particle_pose_[n].theta<<endl;
			//usleep(200000);
		}
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
		//usleep(2000000);
		//cout<<"partic: "<<p<<" indexi: "<<indexi<<" beta: "<<beta<<" weight_part_: "<<weight_part_[indexi]<<" Particle_pose-> x: "<<particle_pose_[indexi].x<<" y: "<<particle_pose_[indexi].y<<" theta: "<<particle_pose_[indexi].theta<<endl;
	}
	for(int n = 0 ; n < num_part_ ; n++){
		particle_pose_[n] = particle_resample_[n];
		//cloud();
		//usleep(100000);
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
	hipot_ = sqrt((delta_pose_.x * delta_pose_.x) + (delta_pose_.y * delta_pose_.y));
	arctan_ = atan(delta_pose_.y / delta_pose_.x);

	//cout<<"pose_x_: "<<pose_x_<<" ; pose_y_: "<<pose_y_<<" ; pose_theta_: "<<pose_theta_<<endl;
	//cout<<"pose_anterior_.x: "<<pose_anterior_.x<<" ; pose_anterior_.y: "<<pose_anterior_.y<<" ; pose_anterior_.theta: "<<pose_anterior_.theta<<endl;
	//cout<<"delta_pose.x: "<<delta_pose_.x<<" ; delta_pose_.y: "<<delta_pose_.y<<" ; delta_pose_.theta: "<<delta_pose_.theta<<endl<<endl;

	pose_anterior_.x = pose_x_;
	pose_anterior_.y = pose_y_;
	pose_anterior_.theta = pose_theta_;
	//cout<<"pose_anterior_.x: "<<pose_anterior_.x<<" ; pose_anterior_.y: "<<pose_anterior_.y<<" ; pose_anterior_.theta: "<<pose_anterior_.theta<<endl;

	int p = 0;
	if(delta_pose_.x != 0 || delta_pose_.y != 0 || delta_pose_.theta != 0){
		//cout<<"deltas != 0"<<endl;

		//cout<<"pose_x_: "<<pose_x_<<" ; pose_y_: "<<pose_y_<<" ; pose_theta_: "<<pose_theta_<<endl;
		//cout<<"pose_anterior_.x: "<<pose_anterior_.x<<" ; pose_anterior_.y: "<<pose_anterior_.y<<" ; pose_anterior_.theta: "<<pose_anterior_.theta<<endl;
		//cout<<"delta_pose.x: "<<delta_pose_.x<<" ; delta_pose_.y: "<<delta_pose_.y<<" ; delta_pose_.theta: "<<delta_pose_.theta<<endl<<endl;

		for(p = 0; p < num_part_; p++)
		{
			particle_pose_[p].x += sign(twist_x_) * hipot_ * cos(particle_pose_[p].theta) + (gaussian(0.0, move_noise_) / reduz_gauss_);
			particle_pose_[p].y += sign(twist_x_) * hipot_ * sin(particle_pose_[p].theta) + (gaussian(0.0, move_noise_) / reduz_gauss_);

			//particle_pose_[p].x += delta_pose_.x; //+ gaussian(0.0, move_noise_);
			//particle_pose_[p].y += delta_pose_.y; //+ gaussian(0.0, move_noise_);

			//(twist_x_*cos(particle_pose_[p].theta)) + gaussian(0.0, move_noise_);// - (delta_pose_.y * sin(particle_pose_[p].theta))
			//(twist_x_*sin(particle_pose_[p].theta)) + gaussian(0.0, move_noise_);// + (delta_pose_.y * cos(particle_pose_[p].theta))

			//cout<<"sx: "<<sign(delta_pose_.x)<<" | sy: "<<sign(delta_pose_.y)<<endl;

			//cout<<"cos: "<<cos(particle_pose_[p].theta)<<" | sen: "<<sin(particle_pose_[p].theta)<<endl;
			//cout<<"N_Part: "<<p<<" | Theta: "<<(particle_pose_[p].theta)*180.0/M_PI<<" | cos: "<<cos(particle_pose_[p].theta)<<" | sen: "<<sin(particle_pose_[p].theta)<<endl;

			particle_pose_[p].theta += delta_pose_.theta + (gaussian(0.0, turn_noise_) / reduz_gauss_); //(delta_pose_.theta * gaussian(0.0, turn_noise_));

			if(particle_pose_[p].theta > M_PI)
				particle_pose_[p].theta -= 2.0 * M_PI;
			if(particle_pose_[p].theta <= - M_PI)
				particle_pose_[p].theta += 2.0 * M_PI;


			//cout<<"twist: "<<twist_x_<<endl;
			//cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p]<<endl;
			//usleep(150000);
			//cout<<"MoveParticle: "<<p<<" | x: "<<particle_pose_[p].x<<" ; y: "<<particle_pose_[p].y<<" ; theta: "<<particle_pose_[p].theta<<endl;
			//cout<<"delta_pose.x: "<<delta_pose_.x<<" ; delta_pose_.y: "<<delta_pose_.y<<" ; delta_pose_.theta: "<<delta_pose_.theta<<endl<<endl;


		}
		//cout<<endl;
		//cout<<"MoveParticle: "<<p-1<<" | x: "<<particle_pose_[p-1].x<<" ; y: "<<particle_pose_[p-1].y<<" ; theta: "<<particle_pose_[p-1].theta<<endl;
		//cout<<"Deltas_pose: "<<delta_pose_.x<<" | "<<delta_pose_.y<<" | "<<delta_pose_.theta<<endl;
		//cout<<"Gaussians(move e turn): "<<gaussian(0.0, move_noise_)<<" | "<<gaussian(0.0, turn_noise_)<<endl<<endl;
		//usleep(250000);

		//cout<<"particle_pose["<<p<<"]:\n"<<particle_pose_[p-1]<<endl;
		//cout<<"moveParticulas"<<endl;

		cloud();
		fakeLaser();
		//cout<<"fakeLaser()"<<endl;
		resample();
		//cout<<"resample()"<<endl;
		pubInicialPose();
		//cout<<"pubinitialPose()"<<endl;
	}
}

void Filtro_Particulas::pubInicialPose()
{
	double xmedia = 0.0;
	double ymedia = 0.0;
	double thetamedia = 0.0;
	double thetapos = 0.0;
	double thetaneg = 0.0;
	double pos = 0.0;
	double neg = 0.0;
	sum = 0.0;

	for (int i = 0; i < num_part_ ; i++)
	{
		xmedia += particle_pose_[i].x;
		ymedia += particle_pose_[i].y;
		thetamedia = particle_pose_[i].theta;

		if(thetamedia < 0.0)
		{
			neg++;
			thetaneg += particle_pose_[i].theta;
		}else
		{
			pos++;
			thetapos += particle_pose_[i].theta;
		}
	}

	xmedia = xmedia / (double)num_part_;
	ymedia = ymedia / (double)num_part_;
	if (neg > pos)
	{
		thetaneg = thetaneg / neg;
		thetamedia = thetaneg;
	}else{
		thetapos = thetapos / pos;
		thetamedia = thetapos;
	}

	if(thetamedia > M_PI)
		thetamedia -= 2.0 * M_PI;
	if(thetamedia <= - M_PI)
		thetamedia += 2.0 * M_PI;
	if(convergiu_ == 0)
	{
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
	}

	if(sum < error_particles_)
	{
		//Para publicar o pose médio.
		initial_pose2_.x = xmedia;
		initial_pose2_.y = ymedia;
		initial_pose2_.theta = thetamedia;

		//para publicar o pose da partícula com maior peso.
/*		initial_pose2_.x = particle_pose_[index_max_w_].x;
		initial_pose2_.y = particle_pose_[index_max_w_].y;
		initial_pose2_.theta = particle_pose_[index_max_w_].theta;

		convergiu_++;
*/
		initial_pose_pub_.publish(initial_pose2_);
		//cout<<"x: "<<xmedia<<" | y: "<<ymedia<<" | theta: "<<thetamedia<<endl;

		reduz_gauss_ = 2.0;

	}
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

void Filtro_Particulas::createGrids()
{
	x = 0;
	y = 0;
	xi = 0;
	yi = 0;
	i = 0;
	//num_laser = 0;
	achou = 0;
	total = 0;

	int qtdd_orient = 6; //quantidade de giros no mesmo pose
	double ang_it = 2 * M_PI / qtdd_orient;
	int pose_x = 0;
	int pose_y = 0;
	double it = M_PI / (qtdd_laser_);

	for (int i = 0; i < num_free_; i++)
	{
		//ROS_INFO("For das celulas free ");
		//carrega o xy no grid[] com indice fator=6
		grid_pose_energy_[i*qtdd_orient].xy = free_xy_[i];
		pose_x = free_xy_[i] / 10000;
		pose_y = free_xy_[i] % 10000;

		for(int ang_inc = 0 ; ang_inc < qtdd_orient ; ang_inc++)
		{
			//ROS_INFO("For do ang_inc");
			//carrega os outros 6 grid[].xy com o mesmo pose_xy
			grid_pose_energy_[(i*qtdd_orient) + ang_inc + 1].xy = free_xy_[i];
			grid_pose_energy_[(i*qtdd_orient) + ang_inc].energy = 0.0;
			grid_pose_energy_[(i*qtdd_orient) + ang_inc].sum = 0.0;

			for(num_laser = 0 ; num_laser < qtdd_laser_ ; num_laser++)
			{
				//ROS_INFO("For do num_laser ");
				//carrega cada grid.theta com um giro diferente para cada um dos 6 pose.xy
				grid_pose_energy_[(i*qtdd_orient) + ang_inc].theta = (ang_it * ang_inc) + gaussian(0.0, turn_noise_);

				//carrega o fake_laser[].theta com cada theta-ésimo ponto do fake_laser para o mesmo grid.theta
				fake_laser_pose_[num_laser].theta = ((ang_min_) + (num_laser * it)) + grid_pose_energy_[(i*qtdd_orient) + ang_inc].theta;
				if(fake_laser_pose_[num_laser].theta > M_PI)
					fake_laser_pose_[num_laser].theta -= 2.0 * M_PI;
				if(fake_laser_pose_[num_laser].theta <= - M_PI)
					fake_laser_pose_[num_laser].theta += 2.0 * M_PI;

				fake_laser_pose_[num_laser].x = pose.x;
				fake_laser_pose_[num_laser].y = pose.y;

				//começa a varredura de cada theta-ésimo ponto do fake_laser
				passo = 0;
				int iteracao = range_max_fakelaser / passo_base; // 5.6 / 0.05 = 112

				for(int p = 1; p <= iteracao; p++)
				{
					//ROS_INFO("For do p-interacao ");
					//varredura do fake_laser
					passo = passo_base * p;
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
						if (obstacle_finded_ == true)
						{
							fake_laser_data_[i][num_laser] = obstacle_;

							//cout<<"Dist-> Particula: "<<i<<" ; num_laser: "<<num_laser<<" ; passo: "<<weight_part_laser_[i][num_laser]<<endl;
							p = iteracao;

						}
						else
						{
							//não achou o obstáculo ou ele está a mais de 5.6 metros de distância
							fake_laser_data_[i][num_laser] = -999;
							passo = -9999;
						}
					}
				}
				//passo = DISTANCIA FAKE DE CADA FEIXE DO LASER VIRTUAL!!!
				if(passo >= 0.0)
				{
					//somando todas as distâncias para cada giro de cada pose
					grid_pose_energy_[(i*qtdd_orient) + ang_inc].sum += passo;
					num_energy_++;
					//cout<<"num_energy_: "<<num_energy_<<endl;

					//Fazendo cálculo da energia (Ge)
					grid_pose_energy_[(i*qtdd_orient) + ang_inc].energy += (1.0 - (passo / range_max_fakelaser));
				}
				else
				{
					grid_pose_energy_[(i*qtdd_orient) + ang_inc].sum = -9999;
					grid_pose_energy_[(i*qtdd_orient) + ang_inc].energy = -9999;
				}
			}
			//normalizando a energia
			grid_pose_energy_[(i*qtdd_orient) + ang_inc].energy = grid_pose_energy_[(i*qtdd_orient) + ang_inc].energy / qtdd_laser_;
		}
	}
	cout<<"num_energy_: "<<num_energy_<<endl;
	grids_ok_ = true;
}

void Filtro_Particulas::spin()
{
	ros::Rate loopRate(freq_);
	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();

		if (free_ok_ == true && occ_ok_ == true)
			//cout<<"free_ok: "<<free_ok_<<" | occ_ok: "<<occ_ok_<<" | grids_ok: "<<grids_ok_<<endl;
		{
			if(grids_ok_ == false)
			{
				ROS_INFO("Inicio do createGrids()");
				createGrids();
				ROS_INFO("Fim do createGrids()");
				//cout<<"grids_ok: "<<grids_ok_<<endl;

			}else if(grids_ok_ == false)
			{
				createParticles();

				if(create_particle_ok_ == 0  && odom_ok_ == true && laser_ok_ == true && zerar_deltas_ == false){

					//zerando os deltas do pose
					pose_anterior_.x = pose_x_;
					pose_anterior_.y = pose_y_;
					pose_anterior_.theta = pose_theta_;

					zerar_deltas_ = true;

					moveParticles();
					//cout<<"moveParticles()"<<endl;
				}else if(create_particle_ok_ == 0  && odom_ok_ == true && laser_ok_ == true && zerar_deltas_ == true){
					moveParticles();

				}
			}
		}
	}
}
