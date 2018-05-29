#include "Cloth.h"

Cloth::Cloth(int xn,int yn, std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector) : dimX(xn),dimY(yn),particleMass(0.6f),damping(5.0f),spring(120.0f)
{
    deltaX = 2.5f/dimX;
    deltaY = 2.5f/dimY;
    init(pVector, fVector, cVector);
    applyForces(pVector, fVector);
    applyConstraints(pVector,cVector);
}

void Cloth::init(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector)
{
    const Vec2f center(0.0, 0.0);
    //pVector.clear();
    //pVector.resize(dimX * dimY);
    //fVector.resize(2*dimX*dimY - dimX - dimY + 2*(dimX-1)*(dimY-1) + 1);
    //cVector.resize(2);
   // std::cout << particles.size() << "|";
    //resize fVector and pVector
    for(int i = 0; i < dimX; i++){
        for(int j = 0; j < dimY; j++){
            auto p = new Particle(Vec2f(deltaX*i, deltaY*j), particleMass);
            pVector.push_back(p);
        }
    }
}

void Cloth::reset()
{

}

void Cloth::draw()
{

}

void Cloth::applyForces(std::vector<Particle*> &pVector, std::vector<Force*> &fVector)
{
    auto gravity = new GravityForce(pVector, Vec2f(0, -9.81f));
    fVector.push_back(gravity);
    float diagonalDist = sqrt(pow(deltaX,2) + pow(deltaY,2));

    for(int i = 0; i< dimX - 1; i++){
        for(int j = 0; j < dimY; j++){
            //all particles have a spring connecting them to the particle on the right
            auto s = new SpringForce(pVector, j*dimX + i, j * dimX + (i+1),deltaX,spring,damping);
            fVector.push_back(s);
        }
    }

    for(int i = 0; i< dimX; i++){
        for(int j = 0; j < dimY - 1; j++){
            //all particles have a spring connecting them to the particle on the bottom
            auto s = new SpringForce(pVector, j*dimX + i, (j + 1) * dimX + i,deltaY,spring,damping);
            fVector.push_back(s);
        }
    }

    for(int i = 0; i < dimX - 1; i++){
        for(int j = 0; j < dimY - 1; j++){
            //all particles have a spring connecting to the bottom right diagonal
            auto s1 = new SpringForce(pVector, j*dimX + i, (j+1)*dimX + (i+1), diagonalDist,spring,damping);
            fVector.push_back(s1);
            //all particles have a spring connecting to the bottom left diagonal
            auto s2 = new SpringForce(pVector, j*dimX + (i+1), (j+1)*dimX + i, diagonalDist,spring,damping);
            fVector.push_back(s2);
        }
    }

}

void Cloth::applyConstraints(std::vector<Particle*> &pVector, std::vector<Constraint*> &cVector)
{
    double radius = 0.2f;
    //Make sure the top edge particles don't move too much
    auto c1 = new CircularWireConstraint(pVector[0], pVector[0]->m_ConstructPos, radius);
    cVector.push_back(c1);
    auto c2 = new CircularWireConstraint(pVector[dimX - 1],pVector[dimX - 1]->m_ConstructPos, radius);
    cVector.push_back(c2);
    //TO DO: try with whole first row
    //TO DO: add rod constraint between particles
}