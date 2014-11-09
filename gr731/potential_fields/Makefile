CC	=	g++
CFLAGS	=	-Wall #Flag til compiling af Objects
LFLAGS	=	-Wall #Flag til linkeren
TARGET	=	potential_fields
SRC	=	main.c swarm_formation.c obstacle_avoidance.c
OBJS	=	${SRC:.c=.o}

.PHONY: clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o $@

.c.o:
	$(CC) -c $(CFLAGS) $<

clean:
	rm $(OBJS) $(TARGET)
