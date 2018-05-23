CXX = g++
CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -Iinclude -DHAVE_CONFIG_H 
OBJS = Solver.o Particle.o ParticleToy.o RodConstraint.o SpringForce.o CircularWireConstraint.o imageio.o

project1: $(OBJS)
	$(CXX) -o $@ $^ -lglut -lGLU -lGL -lpng
clean:
	rm $(OBJS) project1
