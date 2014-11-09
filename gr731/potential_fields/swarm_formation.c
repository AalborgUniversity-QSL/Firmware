
#include <complex>
#include <stdio.h>
#include "swarm_formation.h"


velocity_t swarm(int i, float allposx [10], float allposy [10])
{

	float pos[2] = {allposx[i], allposy[i]};
	int number_of_quads = 10 ;
	float potfield = 0.5;
	float drag = 0.4;
	float dist = 1.5;
	float vmax = 0.04;
	float vm[2] = {0, 0};
	float allpos[2];
	float l;
	float lvm;
	velocity_t vel;
	vel.v1 = 0;
	vel.v2 = 0;

	for(int n = 0; n < number_of_quads; ++n)
	{
	    if(n != i)
	    {
	    	allpos[0] = allposx[n];
	    	allpos[1] =	allposy[n];
	    	l = sqrt((allpos[0]-pos[0])*(allpos[0]-pos[0])+(allpos[1]-pos[1])*(allpos[1]-pos[1]));
	    	//printf("l er  %f \n", l);
	    	vm[0] = vm[0]+potfield*allposx[n]/l*(l-dist);
	    	vm[1] = vm[1]+potfield*allposy[n]/l*(l-dist);;
			//printf("for %i v1 = %f, v2 = %f \n", n, vm[0],vm[1]);
	        // To make them not chrash into each other
    	    lvm = sqrt(vm[0]*vm[0]+vm[1]*vm[1]);
		    if(l > dist && lvm > vmax*drag)
		    {
		        vm[0]= vmax*drag*vm[0]/lvm;
		        vm[1]= vmax*drag*vm[1]/lvm;
			//printf("for %i v1 = %f, v2 = %f \n", n, vm[0],vm[1]);
		    }
		    vel.v1 = vel.v1 + vm[0];
		    vel.v2 = vel.v2 + vm[1];
	    }
	}
	return vel;
}

