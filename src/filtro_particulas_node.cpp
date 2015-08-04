#include "filtro_particulas.h"

int main (int argc, char** argv)
{
	ros::init( argc, argv, "filtro_particulas_node" );

	ros::NodeHandle n;

	Filtro_Particulas fp(n, 0.05);

	fp.spin();

	return 0;

}
