#include <math.h>
#include <stdio.h>
#include "swarm_formation.h"

velocity_t wall(float posx, float posy) {
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
	float karmdist = 0.5;

	float wall1x [6] = {-1.500, -1.500, -1.500, -1.500 , 0.800,  1.500};
	float wall2x [6] = {-1.500, 1.500, -0.800, 1.500 , 1.500, 1.500};
	float wall1y [6] = {-1.500, -1.500,   200.f,  1.500 , 0.f,   -1.500};
	float wall2y [6] = {1.500, -1.500,     0.f,   1.500 , 200.f,   1.500};
	velocity_t vel;
	vel.v1 = (float)0;
	vel.v2 = (float)0;


        for(int o = 0; o <= 5; ++o) {
                ax = posx - wall1x[o];
                ay = posy - wall1y[o];
                bx = wall2x[o] - wall1x[o];
                by = wall2y[o] - wall1y[o];
                lb = sqrt( ((wall2x[o] - wall1x[o]) * (wall2x[o] - wall1x[o])) + ((wall2y[o] - wall1y[o]) * (wall2y[o] - wall1y[o])) );
                dotab = (ax * bx) + (ay * by);
                projx = (dotab / (lb * lb)) * bx;
                projy = (dotab / (lb * lb)) * by;
                obsx = projx + wall1x[o];
                obsy = projy + wall1y[o];

                if ( (obsx < wall1x[o]) || ( (obsx == wall1x[o]) && (obsy < wall1y[o]) ) ) {
                        obsx = wall1x[o];
                        obsy = wall1y[o];
                }

                if( (obsx > wall2x[o]) || ( (obsx == wall2x[o]) && (obsy > wall2y[o]) ) ) {
                        obsx = wall2x[o];
                        obsy = wall2y[o];
                }
        
                l = sqrt( ( (obsx - posx) * (obsx - posx) ) + ( (obsy - posy) * (obsy - posy) ) );

		if ( l > karmdist ) {
                        vm[0]= (float)0;
                        vm[1]= (float)0;
                } else if (l < karmdist && l > 0) {
                        vm[0] = wallfield * ((posx - obsx) / l) * (karmdist - l) * (float)fabs(karmdist - l);
                        vm[1] = wallfield * ((posy - obsy) / l) * (karmdist - l) * (float)fabs(karmdist - l);
                } else {
                        vm[0]= (float)0;
                        vm[1]= (float)0;
                }


		vel.v1 = vel.v1 + vm[0];
		vel.v2 = vel.v2 + vm[1];
	}
 	return vel;
}


velocity_t swarm( int i, float allposx [10], float allposy [10] ) {
	float pos[2] = {allposx[i], allposy[i]};
	int number_of_quads = 10 ;
	float potfield = 0.5;
	float vmin = -0.02;
	float dist = 1.5;
	float vmax = 0.04;
	float vm[2] = {0, 0};
	float allpos[2];
	float l;
	float lvm;
	velocity_t vel;
	vel.v1 = 0;
	vel.v2 = 0;

	for (int n = 0; n < number_of_quads; ++n) {
                if( n != i ) {
                        allpos[0] = allposx[n];
                        allpos[1] = allposy[n];
                        l = sqrt( ( (allpos[0] - pos[0]) * (allpos[0] - pos[0]) ) + ( (allpos[1] - pos[1]) * (allpos[1] - pos[1]) ) );
                        //printf("l er  %f \n", l);
                        vm[0] = vm[0] + ( ( potfield * ( allposx[n]) /  l ) * (l - dist) * (float)fabs(l - dist) );
                        vm[1] = vm[1] + ( ( potfield * ( allposy[n]) /  l ) * (l - dist) * (float)fabs(l - dist) );
			//printf("for %i v1 = %f, v2 = %f \n", n, vm[0],vm[1]);
                        // To make them not chrash into each other
                        lvm = sqrt( (vm[0] * vm[0]) + (vm[1] * vm[1]) );
                        if ( lvm > vmin) {
                                vm[0] = ( allposx[n]) /  l ) * vmin;
                                vm[1] = ( allposx[n]) /  l ) * vmin;
                                //printf("for %i v1 = %f, v2 = %f \n", n, vm[0],vm[1]);
                        }
                        vel.v1 = vel.v1 + vm[0];
                        vel.v2 = vel.v2 + vm[1];
                }
	}
	return vel;
}
