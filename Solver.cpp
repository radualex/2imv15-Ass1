#include "Particle.h"
#include "Force.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "System.h"
#include "Solver.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand() % 2000) / 1000.f) - 1.f)

using namespace Eigen;
using namespace std;

void applyEuler(System *system, float h) {
    if (false) {
        //implicit(system, h);
    } else {
        VectorXf oldState = system->getState();
        VectorXf deriv = system->derivEval();
        VectorXf newState = oldState + h * deriv;

        system->setState(newState);
    }
}

void applyMidpoint(System *sys, float h) {

    VectorXf oldState = sys->getState();
    
    VectorXf deriv = sys->derivEval();
    VectorXf midPointState = oldState + h * 0.5f * deriv;
    
    sys->setState(midPointState);
    deriv = sys->derivEval();
    VectorXf newState = oldState + h * deriv;

    sys->setState(newState);
}

void applyRungeKutta(System *sys, float h) {

    VectorXf oldState = sys->getState();
    VectorXf deriv = sys->derivEval();
    VectorXf k1 = h * deriv;
    VectorXf newState = oldState + k1 / 2;
    deriv = sys->derivEval();
    VectorXf k2 = h * deriv;
    newState = oldState + k2 / 2;
    sys->setState(newState);

    deriv = sys->derivEval();
    VectorXf k3 = h * deriv;
    newState = oldState + k3;
    sys->setState(newState);

    deriv = sys->derivEval();
    VectorXf k4 = h * deriv;

    //Final state
    newState = oldState + 1.0f / 6.0f * k1 + 1.0f / 3.0f * k2 + 1.0f / 3.0f * k3 + 1.0f / 6.0f * k4;

    sys->setState(newState);
}

void simulation_step(System* sys, float dt, int solver)
{
    // for(int i = 0; i < sys->pVector.size(); i++)
    // {
    //     std::cout << sys->pVector[i]->m_Position[0] << " " << sys->pVector[i]->m_Position[1] << std::endl;
    // }

    if(solver == 1) 
    {
    applyEuler(sys,dt);
    }
    else if(solver == 2) 
    {
        applyMidpoint(sys, dt);
    }
    else if(solver == 3) 
    {
        applyRungeKutta(sys, dt);
    }

	// int ii, size = sys->pVector.size();

    
	//  for (ii = 0; ii < size; ii++)
	//  {
	//  	sys->pVector[ii]->m_Position += dt * sys->pVector[ii]->m_Velocity;
	//  	sys->pVector[ii]->m_Velocity = DAMP * sys->pVector[ii]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
	//  }
    
}