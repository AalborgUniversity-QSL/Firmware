#ifndef SWARM_FORMATION_H
#define SWARM_FORMATION_H

typedef struct v {
	float v1;
	float v2;
} velocity_t; 

velocity_t swarm(int i, float allposx [10], float allposy [10]);
velocity_t avoid(float posx, float posy, float obsx, float obsy);

#endif /* SWARM_FORMATION_H */