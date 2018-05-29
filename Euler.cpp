#include "Euler.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulateStep(System *system, float h) {
    if (type == IMPLICIT) {
        implicit(system, h);
    } else {
        // Get the old state
        VectorXf oldState = system->getState();

        // Evaluate derivative
        VectorXf deriv = system->derivEval();

        // Compute the new state
        VectorXf newState = oldState + h * deriv;

        if (type == SEMI) {
            // If we are running semi implicit euler, use the new velocity instead
            VectorXf semiImpl(system->getDim());
            for (int i = 0; i < newState.size(); i += 6) {
                semiImpl[i + 0] = oldState[i + 0] + h * newState[i + 3];  // Xnew implicit, using Vnew
                semiImpl[i + 1] = oldState[i + 1] + h * newState[i + 4];  // Xold + h * Vnew
                semiImpl[i + 2] = oldState[i + 2] + h * newState[i + 5];
                semiImpl[i + 3] = newState[i + 3];  // Keep Vnew
                semiImpl[i + 4] = newState[i + 4];
                semiImpl[i + 5] = newState[i + 5];
            }

            // Set the new state, using semi implicit computation

            //check collisions
            if (system->wallExists) {
                semiImpl = system->checkWallCollision(oldState, semiImpl);
            }
            system->setState(semiImpl, system->getTime() + h);
        } else {
            if (system->wallExists) {
                newState = system->checkWallCollision(oldState, newState);
            }
            system->setState(newState, system->getTime() + h);
        }
    }
}

void Euler::implicit(System *sys, float h) {
    // Get old state
    VectorXf oldState = sys->getState();
    sys->derivEval();

    // Fill mass matrix
    SparseMatrix<float> M(sys->getDim() / 2, sys->getDim() / 2);

    std::vector<Triplet<float>> MtripletList;
    MtripletList.reserve(sys->getDim() / 2);
    for (int i = 0; i < sys->particles.size() * 3; i += 3) {
        MtripletList.push_back(Triplet<float>(i+0, i+0, sys->particles[i / 3]->mass));
        MtripletList.push_back(Triplet<float>(i+1, i+1, sys->particles[i / 3]->mass));
        MtripletList.push_back(Triplet<float>(i+2, i+2, sys->particles[i / 3]->mass));
    }
    M.setFromTriplets(MtripletList.begin(), MtripletList.end());

    // Fill jx and jy matrix based
    SparseMatrix<float> jx(sys->getDim() / 2, sys->getDim() / 2);
    MatrixXf jv(sys->getDim() / 2, sys->getDim() / 2);

    // Initialize empty map to compute jx
    auto jxm = map<int, map<int, float>>();
    unsigned long entries = 0;
    for (Force *f : sys->forces) {
        // Compute map for every force and update jxm appropriately
        auto fjx = f->jx();
        for (auto const &i1 : fjx) {
            for (auto const &i2 : i1.second) {
                if (jxm.count(i1.first) && jxm[i1.first].count(i2.first)) {
                    // todo maybe check if we can remove this, it should never happen?
                    // i1 and i2 exist
                    // Hence, we update the already existing value
                    jxm[i1.first][i2.first] += i2.second;
                } else {
                    // No value yet exists, since i1 or i2 does not exist
                    // Hence we set a new value
                    jxm[i1.first][i2.first] = i2.second;
                    entries++;
                }
            }
        }

        if (f->particles.size() == 2) {
            MatrixXf fjv = f->jv();
            jv.block(f->particles[0]->index * 3, f->particles[1]->index * 3, fjv.cols(), fjv.rows()) = fjv;
            jv.block(f->particles[1]->index * 3, f->particles[0]->index * 3, fjv.cols(), fjv.rows()) = fjv;
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
        vold.coeffRef(i * 3 + 0) = p->velocity[0];
        vold.coeffRef(i * 3 + 1) = p->velocity[1];
        vold.coeffRef(i * 3 + 2) = p->velocity[2];
        fold.coeffRef(i * 3 + 0) = p->force[0];
        fold.coeffRef(i * 3 + 1) = p->force[1];
        fold.coeffRef(i * 3 + 2) = p->force[2];
    }

    // Compute A
    SparseMatrix<float> A = M - h * h * jx;// - h * jv;
    SparseVector<float> b = h * (fold + h * jx * vold);

    // Solve for dy
    ConjugateGradient<SparseMatrix<float>, Lower|Upper> cg;
    cg.compute(A);
    SparseVector<float> dy = cg.solve(b);

//    std::cout << "#iterations:     " << cg.iterations() << std::endl;
//    std::cout << "estimated error: " << cg.error()      << std::endl;

    // Set new state
    VectorXf newState(sys->getDim());
    for (int i = 0; i < dy.size(); i += 3) {
        int si = i * 2; // State index
        newState[si + 0] = oldState[si + 0] + (oldState[si + 3] + dy.coeff(i+0)) * h;    // dX = (V0 + dV) * h
        newState[si + 1] = oldState[si + 1] + (oldState[si + 4] + dy.coeff(i+1)) * h;
        newState[si + 2] = oldState[si + 2] + (oldState[si + 5] + dy.coeff(i+2)) * h;
        newState[si + 3] = oldState[si + 3] + dy.coeff(i+0);// * h;        // Update velocity
        newState[si + 4] = oldState[si + 4] + dy.coeff(i+1);// * h;
        newState[si + 5] = oldState[si + 5] + dy.coeff(i+2);// * h;
    }

    if (sys->wallExists) {
        newState = sys->checkWallCollision(oldState, newState);
    }
    sys->setState(newState);
}
