CXX = g++
CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -I include -DHAVE_CONFIG_H 
OBJS = System.o Solver.o Particle.o ParticleToy.o Constraint.o RodConstraint.o Force.o SpringForce.o GravityForce.o CircularWireConstraint.o Cloth.o imageio.o

project1: $(OBJS)
	$(CXX) -o $@ $^ -lglut -lGLU -lGL -lpng
clean:
	rm $(OBJS) project1
