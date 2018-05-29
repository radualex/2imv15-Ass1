#include "Cloth.h"

Cloth::Cloth(int xn,int yn) : dimX(xn),dimY(yn),particleMass(0.1f),damping(5.0f),spring(120.0f)
{
    deltaX = 2.5f/dimX;
    deltaY = 2.5f/dimY;
    init();
}

void Cloth::init()
{
    particles.resize(dimX * dimY);
    //resize fVector and pVector
    for(int i = 0; i < dimX; i++){
        for(int j = 0; j < dimY; j++){
            auto p = new Particle(Vec2f(deltaX*i, deltaY*j), particleMass);
            particles.push_back(p);
        }
    }
}

void Cloth::reset()
{

}

void Cloth::draw()
{

}

void Cloth::applyForces()
{
    auto gravity = new GravityForce(particles, Vec2f(0, -9.81f));
    fVector.push_back(gravity);
    float diagonalDist = sqrt(pow(deltaX,2) + pow(deltaY,2));

    for(int i = 0; i< dimX - 1; i++){
        for(int j = 0; j < dimY; j++){
            //all particles have a spring connecting them to the particle on the right
            auto s = new SpringForce(particles, j*dimX + i, j * dimX + (i+1),deltaX,spring,damping);
            fVector.push_back(s);
        }
    }

    for(int i = 0; i< dimX; i++){
        for(int j = 0; j < dimY - 1; j++){
            //all particles have a spring connecting them to the particle on the bottom
            auto s = new SpringForce(particles, j*dimX + i, (j + 1) * dimX + i,deltaY,spring,damping);
            fVector.push_back(s);
        }
    }

    for(int i = 0; i < dimX - 1; i++){
        for(int j = 0; j < dimY - 1; j++){
            //all particles have a spring connecting to the bottom right diagonal
            auto s1 = new SpringForce(particles, j*dimX + i, (j+1)*dimX + (i+1), diagonalDist,spring,damping);
            fVector.push_back(s1);
            //all particles have a spring connecting to the bottom left diagonal
            auto s2 = new SpringForce(particles, j*dimX + (i+1), (j+1)*dimX + i, diagonalDist,spring,damping);
            fVector.push_back(s2);
        }
    }

}

void Cloth::applyConstraints()
{

}