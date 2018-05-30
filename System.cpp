#include "System.h"
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

System::System() {}

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

void System::free_data(void)
{
	pVector.clear();
	fVector.clear();
	cVector.clear();
}

void System::clear_data(void)
{
	int ii, size = pVector.size();

	for (ii = 0; ii < size; ii++)
	{
		pVector[ii]->reset();
	}
}


int System::getPositionOfParticle(Particle *p)
{
	int pos = std::find(pVector.begin(), pVector.end(), p) - pVector.begin();
	if (pos < pVector.size())
	{

		return pos;
	}
	return -1;
}


void System::apply_constraints(float ks, float kd)
{
	const int dimensions = 2;
	int vectorSize = pVector.size() * dimensions;
	int constraintsSize = cVector.size();

	VectorXf q = VectorXf::Zero(vectorSize);
	VectorXf Q = VectorXf::Zero(vectorSize);
	MatrixXf M = MatrixXf::Zero(vectorSize, vectorSize);
	MatrixXf W = MatrixXf::Zero(vectorSize, vectorSize);
	VectorXf C = VectorXf::Zero(constraintsSize);
	VectorXf Cder = VectorXf::Zero(constraintsSize);
	MatrixXf J = MatrixXf::Zero(constraintsSize, vectorSize);
	MatrixXf Jt = MatrixXf::Zero(vectorSize, constraintsSize);
	MatrixXf Jder = MatrixXf::Zero(constraintsSize, vectorSize);

  
	for (int i = 0; i < vectorSize; i += dimensions)
	{
		Particle *p = pVector[i / dimensions];
		for (int d = 0; d < dimensions; d++)
		{
			M(i + d,i + d) = p->mass;
			Q[i + d] = p->m_Force[d];
			q[i + d] = p->m_Velocity[d];
		}
	}
	W = M.inverse();

	for (int i = 0; i < constraintsSize; i++)
	{
		Constraint *c = cVector[i];

		C[i] = c->constraint();
		Cder[i] = c->constraintDerivative();
		std::vector<Vec2f> j = c->J();
		std::vector<Vec2f> jd = c->JDerivative();

		std::vector<Particle *> currentParticles = c->particles;
		for (int k = 0; k < currentParticles.size(); k++)
		{
			int currentPos = getPositionOfParticle(currentParticles[k]);
			if (currentPos != -1)
			{
				int pIndex = currentPos * dimensions;
				for (int d = 0; d < dimensions; d++)
				{
					Jder(i,pIndex + d) = jd[k][d];
					J(i,pIndex + d) = j[k][d];
					Jt(pIndex + d,i) = j[k][d];
				}
			}
			else
			{
				std::cout << "Error position -1";
			}
		}
	}
	MatrixXf JW = J * W;
	MatrixXf JWJt = JW * Jt;
	VectorXf Jderq = Jder * q;
	VectorXf JWQ = JW * Q;
	VectorXf KsC = ks * C;
	VectorXf KdCd = kd * Cder;
	VectorXf rhs = - Jderq - JWQ - KsC - KdCd;

	ConjugateGradient<MatrixXf, Lower|Upper> cg;
	auto lambda = cg.compute(JWJt).solve(rhs);

	VectorXf Qhat = Jt * lambda;

	for (int i = 0; i < pVector.size(); i++)
	{
		Particle *p = pVector[i];
		int index = i * dimensions;

		p->m_Force[0] += Qhat[index];
		p->m_Force[1] += Qhat[index + 1];
	}
}

void System::apply_forces()
{
	for (Force *f : fVector)
	{
		f->apply();
	}
}

void System::clearForces()
{
	for (Particle *p : pVector)
	{
		p->clearForce();
	}
}

void System::addParticle(Particle *p) 
{
    pVector.push_back(p);
}

void System::addForce(Force *f) 
{
    fVector.push_back(f);
}

void System::addConstraint(Constraint *c) 
{
    cVector.push_back(c);
}

VectorXf System::derivEval() {
    clearForces();
    apply_forces();
    apply_constraints(100.0f, 10.0f);
    return computeDerivative();
}

VectorXf System::computeDerivative() {
    VectorXf dst(this->getDim());
    for (int i = 0; i < pVector.size(); i++) {
        Particle *p = pVector[i];
        dst[i * 4 + 0] = p->m_Velocity[0];
        dst[i * 4 + 1] = p->m_Velocity[1];
        dst[i * 4 + 2] = p->m_Force[0] / p->mass;
        dst[i * 4 + 3] = p->m_Force[1] / p->mass;
    }
    return dst;
}

unsigned long System::getDim() {
    return pVector.size() * 2 * 2;
}

VectorXf System::getState() {
    VectorXf r(this->getDim());
    for (int i = 0; i < this->pVector.size(); i++) {
        Particle *p = pVector[i];
        r[i * 4 + 0] = p->m_Position[0];
        r[i * 4 + 1] = p->m_Position[1];
        r[i * 4 + 2] = p->m_Velocity[0];
        r[i * 4 + 3] = p->m_Velocity[1];
    }
    return r;
}

void System::setState(VectorXf newState) {
    for (int i = 0; i < pVector.size(); i++) {
        pVector[i]->m_Position[0] = newState[i * 4 + 0];
        pVector[i]->m_Position[1] = newState[i * 4 + 1];
        pVector[i]->m_Velocity[0] = newState[i * 4 + 2];
        pVector[i]->m_Velocity[1] = newState[i * 4 + 3];
    }
}
