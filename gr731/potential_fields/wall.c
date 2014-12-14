
#include <complex>
#include <stdio.h>
#include "swarm_formation.h"


velocity_t wall(float posx, float posy)
{

	float ax;
	float ay;
	float bx;
	float by;
	float projx;
	float projy;
	float obsx;
	float obsy;
	float vm[2] = {0, 0};
	float lb;
	float l;
	float dotab;
	float wallfield = 10;
	float karmdist = 0.7;

	float wall1x [6] = {-1.500, -1.500, -1.500, -1.500 , 0.800,  1.500};
	float wall2x [6] = {-1.500, 1.500, , -0.800, 1.500 , 1.500, 1.500};
	float wall1y [6] = {-1.500, -1.500,   0.f,  1.500 , 0.f,   -1.500};
	float wall2y [6] = {1.500, -1.500,     0.f,   1.500 , 0.f,   1.500};
	velocity_t vel;
	vel.v1 = (float)0;
	vel.v2 = (float)0;


    for(int o = 0; o <= 5; ++o)
    {
        ax = posx - wall1x[o];
        ay = posy - wall1y[o];
        bx = wall2x[o]- wall1x[o];
        by = wall2y[o]- wall1y[o];
        lb = sqrt((wall2x[o]-wall1x[o])*(wall2x[o]-wall1x[o])+(wall2y[o]-wall1y[o])*(wall2y[o]-wall1y[o]));
        dotab = ax*bx+ay*by;
        projx = dotab/(lb*lb)*bx;
        projy = dotab/(lb*lb)*by;
        obsx = projx + wall1x[o];
        obsy = projy + wall1y[o];
        if(obsx < wall1x[o] || (obsx == wall1x[o] && obsy < wall1y[o]))
        {
            obsx = wall1x[o];
            obsy = wall1y[o];
        }
        if(obsx > wall2x[o] || (obsx == wall2x[o] && obsy > wall2y[o]))
        {
            obsx = wall2x[o];
            obsy = wall2y[o];
        }
        

    	l = sqrt((obsx-posx)*(obsx-posx)+(obsy-posy)*(obsy-posy));
        vm[0] = wallfield*(posx-obsx)/l*(karmdist-l)*fabs((karmdist-l));
        vm[1] = wallfield*(posy-obsy)/l*(karmdist-l)*fabs((karmdist-l));

		if(l > karmdist)
		    {
		      vm[0]= (float)0;
	               vm[1]= (float)0;
			}

		vel.v1 = vel.v1 + vm[0];
		vel.v2 = vel.v2 + vm[1];
	}
 	return vel;
}

