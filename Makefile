# macOS 
# CCFLAGS = -std=c++11 -g -O3 -I /usr/local/include `pkg-config --cflags glfw3` -framework OpenGL
# LDFLAGS = -L/usr/local/lib `pkg-config --libs glfw3` -lbox2d

#CCFLAGS = -std=c++11 -g -O3 -I /usr/local/include -framework OpenGL
#LDFLAGS = -L/usr/local/lib -l glfw3 -lbox2d

# Linux
# This list of dependencies works around
# the new (as of Spring 2018) and inconvenient Box2D building
CCFLAGS = -std=c++11 -g -O3 `pkg-config --cflags glfw3`
LDFLAGS = `pkg-config --libs glfw3` -lBox2D -lGL -lX11 -lXrandr -lXinerama -lXxf86vm -lXcursor -lpthread -ldl -O1


SRC = main.cc world.cc robot.cc box.cc guiworld.cc polygon.cc
HDR = push.hh

# Change this to wherever your Box2D source code is
# Note the multiple levels of directories named 'Box2D'
export CPATH=/home/adam/Documents/packages/Box2D_v2.3.0/Box2D

all: push

push: $(SRC) $(HDR)
	g++ $(CCFLAGS) $(SRC) $(LDFLAGS) -o $@

clean:
	rm -f push 
	rm -f *.o
