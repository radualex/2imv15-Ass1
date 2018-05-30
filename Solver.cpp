#include "Particle.h"
#include "Force.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "ParticleManipulator.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand() % 2000) / 1000.f) - 1.f)

using namespace Eigen;
using namespace std;

// It seems that the fix points can explode.
// no matter which scheme is applied.

void applyEuler(ParticleManipulator *pm, float dt) {
    VectorXf oldState = pm->getState();
    VectorXf deriv = pm->derivEval();
    VectorXf newState = oldState + dt * deriv;

    pm->setState(newState);
}

void applyMidpoint(ParticleManipulator *pm, float dt) {

    VectorXf oldState = pm->getState();
    
    VectorXf deriv = pm->derivEval();
    VectorXf midPointState = oldState + dt * 0.5f * deriv;
    
    pm->setState(midPointState);
    deriv = pm->derivEval();
    VectorXf newState = oldState + dt * deriv;

    pm->setState(newState);
}

void applyRungeKutta(ParticleManipulator *pm, float dt) {

    VectorXf oldState = pm->getState();

    VectorXf deriv = pm->derivEval();
    VectorXf k1 = dt * deriv;
    VectorXf newState = oldState + k1 / 2;

    deriv = pm->derivEval();
    VectorXf k2 = dt * deriv;
    newState = oldState + k2 / 2;
    pm->setState(newState);

    deriv = pm->derivEval();
    VectorXf k3 = dt * deriv;
    newState = oldState + k3;
    pm->setState(newState);

    deriv = pm->derivEval();
    VectorXf k4 = dt * deriv;
    
    newState = oldState + 1.0f / 6.0f * k1 + 1.0f / 3.0f * k2 + 1.0f / 3.0f * k3 + 1.0f / 6.0f * k4;

    pm->setState(newState);
}

void simulation_step(ParticleManipulator* pm, float dt, int solver)
{
    // for(int i = 0; i < pm->pVector.size(); i++)
    // {
    //     std::cout << pm->pVector[i]->m_Position[0] << " " << pm->pVector[i]->m_Position[1] << std::endl;
    // }

    if(solver == 1) 
    {
        applyEuler(pm,dt);
    }
    else if(solver == 2) 
    {
        applyMidpoint(pm, dt);
    }
    else if(solver == 3) 
    {
        applyRungeKutta(pm, dt);
    }

	// int ii, size = pm->pVector.size();

    
	//  for (ii = 0; ii < size; ii++)
	//  {
	//  	pm->pVector[ii]->m_Position += dt * pm->pVector[ii]->m_Velocity;
	//  	pm->pVector[ii]->m_Velocity = DAMP * pm->pVector[ii]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
	//  }
    
}