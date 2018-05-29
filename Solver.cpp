#include "Particle.h"
#include "Force.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "System.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand() % 2000) / 1000.f) - 1.f)

using namespace Eigen;
using namespace std;

 enum TYPE {
        EXPLICIT,
        IMPLICIT,
    };

TYPE type;

void simulation_step(System *system, float dt, int solver)
{
    if(solver == 1) applyEuler(system,dt);
    else if(solver == 2) applyMidpoint(system, dt);
    else if(solver == 3) applyRungeKutta(system, dt);

    /*
	int ii, size = system->particles.size();

	for (ii = 0; ii < size; ii++)
	{
		system->particles[ii]->m_Position += dt * pVector[ii]->m_Velocity;
		system->particles[ii]->m_Velocity = DAMP * pVector[ii]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
	}
    */
}


void applyEuler(System *system, float h) {
    if (type == IMPLICIT) {
        implicit(system, h);
    } else {
        VectorXf oldState = system->getState();
        VectorXf deriv = system->derivEval();
        VectorXf newState = oldState + h * deriv;

        if (system->wallExists) {
            newState = system->checkWallCollision(oldState, newState);
        }
        system->setState(newState, system->getTime() + h);
    }
}

void implicit(System *sys, float h) {
    // Get old state
    VectorXf oldState = sys->getState();
    sys->derivEval();

    // Fill mass matrix
    SparseMatrix<float> M(sys->getDim() / 2, sys->getDim() / 2);

    std::vector<Triplet<float>> MtripletList;
    MtripletList.reserve(sys->getDim() / 2);
    for (int i = 0; i < sys->particles.size() * 2; i += 2) {
        MtripletList.push_back(Triplet<float>(i+0, i+0, sys->particles[i / 2]->mass));
        MtripletList.push_back(Triplet<float>(i+1, i+1, sys->particles[i / 2]->mass));
    }
    M.setFromTriplets(MtripletList.begin(), MtripletList.end());

    // Fill jx and jy matrix based
    SparseMatrix<float> jx(sys->getDim() / 2, sys->getDim() / 2);
    MatrixXf jv(sys->getDim() / 2, sys->getDim() / 2);

    // Initialize emsysy map to compute jx
    auto jxm = map<int, map<int, float>>();
    unsigned long entries = 0;
    for (Force *f : sys->forces) {
        // Compute map for every force and update jxm appropriately
        auto fjx = f->jx();
        for (auto const &i1 : fjx) {
            for (auto const &i2 : i1.second) {
                if (jxm.count(i1.first) && jxm[i1.first].count(i2.first)) {
                    
                    jxm[i1.first][i2.first] += i2.second;
                } else {
                    jxm[i1.first][i2.first] = i2.second;
                    entries++;
                }
            }
        }

        if (f->particles.size() == 2) {
            MatrixXf fjv = f->jv();
            jv.block(f->particles[0]->index * 2, f->particles[1]->index * 2, fjv.cols(), fjv.rows()) = fjv;
            jv.block(f->particles[1]->index * 2, f->particles[0]->index * 2, fjv.cols(), fjv.rows()) = fjv;
        }
    }

    std::vector<Triplet<float>> JxTripletList;
    JxTripletList.reserve(entries);
    for (auto const &i1 : jxm) {
        for (auto const &i2 : i1.second) {
            JxTripletList.push_back(Triplet<float>(i1.first, i2.first, i2.second));
        }
    }
    jx.setFromTriplets(JxTripletList.begin(), JxTripletList.end());

    // Get fold and vold
    SparseVector<float> fold(sys->getDim() / 2);
    SparseVector<float> vold(sys->getDim() / 2);


    for (int i = 0; i < sys->particles.size(); i++) {
        Particle *p = sys->particles[i];
        vold.coeffRef(i * 2 + 0) = p->m_Velocity[0];
        vold.coeffRef(i * 2 + 1) = p->m_Velocity[1];
        fold.coeffRef(i * 2 + 0) = p->m_Force[0];
        fold.coeffRef(i * 2 + 1) = p->m_Force[1];
    }

    // Compute A
    SparseMatrix<float> A = M - h * h * jx;// - h * jv;
    SparseVector<float> b = h * (fold + h * jx * vold);

    // Solve for dy
    ConjugateGradient<SparseMatrix<float>, Lower|Upper> cg;
    cg.compute(A);
    SparseVector<float> dy = cg.solve(b);
    
    // Set new state
    VectorXf newState(sys->getDim());
    for (int i = 0; i < dy.size(); i += 2) {
        int si = i * 2; // State index
        newState[si + 0] = oldState[si + 0] + (oldState[si + 2] + dy.coeff(i+0)) * h;    // dX = (V0 + dV) * h
        newState[si + 1] = oldState[si + 1] + (oldState[si + 3] + dy.coeff(i+1)) * h;
        newState[si + 2] = oldState[si + 2] + dy.coeff(i+0);// * h;        // Update velocity
        newState[si + 3] = oldState[si + 3] + dy.coeff(i+1);// * h;
    }

    if (sys->wallExists) {
        newState = sys->checkWallCollision(oldState, newState);
    }
    sys->setState(newState);
}

void applyMidpoint(System *sys, float h) {

    VectorXf oldState = sys->getState();
    VectorXf deriv = sys->derivEval();
    VectorXf midPointState = oldState + h * 0.5f * deriv;

    sys->setState(midPointState, sys->getTime() + h);
    deriv = sys->derivEval();
    VectorXf newState = oldState + h * deriv;

    if(sys->wallExists) {
        newState = sys->checkWallCollision(oldState, newState);
    }
    sys->setState(newState, sys->getTime() + h);
}

void applyRungeKutta(System *sys, float h) {

    VectorXf oldState = sys->getState();
    float oldTime = sys->getTime();
    VectorXf deriv = sys->derivEval();
    VectorXf k1 = h * deriv;
    VectorXf newState = oldState + k1 / 2;
    deriv = sys->derivEval();
    VectorXf k2 = h * deriv;
    newState = oldState + k2 / 2;
    sys->setState(newState, oldTime + h / 2);

    deriv = sys->derivEval();
    VectorXf k3 = h * deriv;
    newState = oldState + k3;
    sys->setState(newState, oldTime + h);

    deriv = sys->derivEval();
    VectorXf k4 = h * deriv;

    //Final state
    newState = oldState + 1.0f / 6.0f * k1 + 1.0f / 3.0f * k2 + 1.0f / 3.0f * k3 + 1.0f / 6.0f * k4;

    if(sys->wallExists) {
        newState = sys->checkWallCollision(oldState, newState);
    }

    sys->setState(newState, oldTime + h);
}