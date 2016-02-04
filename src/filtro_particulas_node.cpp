#include "filtro_particulas.h"

int main (int argc, char** argv)
{
	ros::init( argc, argv, "filtro_particulas_node" );

	ros::NodeHandle n;

	Filtro_Particulas fp(n);

	fp.spin();

	return 0;

}
