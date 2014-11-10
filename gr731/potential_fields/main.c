
#include "swarm_formation.h"
#include <stdio.h>

int main()
{
	int i = 0;
	float vx;
	float vy;
	float allposx[10] = {10.2, 2, 3, 4, 5, 6, 7, 8, 4, 10};
	float allposy[10] = {7, 2, 3, 4, 5, 6, 7, 8, 9, 1};
	velocity_t vel_sw = {0, 0};
	velocity_t vel_ob = {0, 0};
	velocity_t vel_wl = {0, 0};
	vel_sw = swarm(i, allposx, allposy);
	vel_ob = avoid(allposx[i],allposx[i], allposx[i]+0.5,allposx[i]-0.5);
	vel_wl = wall(allposx[i],allposx[i]);
	vx = vel_sw.v1+vel_ob.v1+vel_wl.v1;
	vy = vel_sw.v2+vel_ob.v2+vel_wl.v2;
	printf("v1 = %f, v2 = %f \n", vx,vy);
	return 0;
}