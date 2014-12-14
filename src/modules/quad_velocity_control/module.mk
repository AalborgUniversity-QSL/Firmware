#
# Quadcopter velocity control app
#

MODULE_COMMAND	= quad_velocity_control
SRCS		= quad_velocity_control_main.c \
		  swarm_formation.c
MODULE_STACKSIZE = 1200
