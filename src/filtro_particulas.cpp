#include "filtro_particulas.h"

Filtro_Particulas::Filtro_Particulas(ros::NodeHandle n)
{
	n_ = n;

	coordx_sub_ = n.subscribe("coordx", 4000, &Filtro_Particulas::coordxCallback, this);
	coordy_sub_ = n.subscribe("coordy", 4000, &Filtro_Particulas::coordyCallback, this);

	once_ = true;
	l_ = 0;
	min_x_ = 10000;
	min_y_ = 10000;
	max_x_ = -10000;
	max_y_ = -10000;

	particle_pose_[30];
	single_pose_.x = 0;
	single_pose_.y = 0;
	single_pose_.theta = 0;

}

Filtro_Particulas::~Filtro_Particulas()
{
	coordx_sub_.shutdown();
	coordy_sub_.shutdown();
}

void Filtro_Particulas::coordxCallback (const std_msgs::Int32MultiArray::ConstPtr& coordx)
{
	//Carregar os valores de x dos landmarks.
	std::vector<int>::const_iterator cx = coordx->data.begin();
	std::vector<int>::const_iterator cxm = coordx->data.begin()+1;

	min_x_ = *cx;
	max_x_ = *cxm;

	l_ = 0;

	for(std::vector<int>::const_iterator it = coordx->data.begin()+2 ; it != coordx->data.end(); ++it){

		landmarks_[l_][0] = *it;
		l_++;
	}
	return;
}

void Filtro_Particulas::coordyCallback (const std_msgs::Int32MultiArray::ConstPtr& coordy)
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

void Filtro_Particulas::createParticles()
{
	//Criando partículas randômicas.
	if(once_ == true)
	{
		int offset_x = max_x_ - min_x_ + 1;
		int offset_y = max_y_ - min_y_ + 1;

		for (int i = 0; i < 30; i++)
		{
			single_pose_.x = rand() % offset_x + min_x_; //random de min_x_ até max_x_
			single_pose_.y = rand() % offset_y + min_y_;
			single_pose_.theta = (rand() % 180 + 0) - 90;
			particle_pose_[i] = single_pose_;

			cout<<"x: "<<single_pose_.x<<" ; y: "<<single_pose_.y<<" ; theta: "<<single_pose_.theta<<endl;
			//cout<<"particle_pose["<<i<<"]:\n"<<particle_pose_[i]<<endl;
		}
	}
}

void Filtro_Particulas::readLandmarks()
{
	if(once_ == true)
	{
		//Teste para verificar se a matriz foi carregada corretamente.
		for(int f=0;f<l_;f++){
			cout << "[[" << landmarks_[f][0] << ", " << landmarks_[f][1] << "]]";

					}
		cout<<"\nl_: "<<l_<<endl;
		cout << "max: " << max_x_ << "; " << max_y_ << endl;
		cout << "min: " << min_x_ << "; " << min_y_ << endl;
		cout<<endl;
		cout<<endl;
		cout<<endl;
		cout<<endl;
	}
}

void Filtro_Particulas::fakeLaser()
{


}

void Filtro_Particulas::spin()
{
	ros::Rate loopRate(10);

	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();
		readLandmarks();
		createParticles();

		once_ = false;
	}

}
