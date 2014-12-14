
#include <complex>
#include <stdio.h>
#include "swarm_formation.h"


velocity_t avoid(float posx, float posy, float obsx, float obsy)
{

	float wallfield = 10;
	float karmdist = 0.9;
	float vm[2] = {0, 0};
	float l;
	velocity_t vel;
	vel.v1 = 0;
	vel.v2 = 0;


	l = sqrt((obsx-posx)*(obsx-posx)+(obsy-posy)*(obsy-posy));
	vm[0] = wallfield*(posx-obsx)/l*(karmdist-l)*fabs((karmdist-l));
	vm[1] = wallfield*(posy-obsy)/l*(karmdist-l)*fabs((karmdist-l));
	if(l > karmdist){
                vm[0]= 0;
	        vm[1]= 0;
	       }

	vel.v1 = vm[0];
	vel.v2 = vm[1];

	return vel;
}

