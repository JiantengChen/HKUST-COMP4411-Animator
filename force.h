#ifndef FORCE_H
#define FORCE_H

#include "vec.h"
#include "particle.h"
class Particle;

class Force
{
public:
    virtual void addForce(Particle *parti) = 0;
};

class Gravity : public Force
{
public:
    Gravity(Vec3d g) : gravity(g) {};
    Vec3d gravity;
    virtual void addForce(Particle *parti);
};
class Viscous : public Force
{
public:
    Viscous(double v) : k(v) {}
    double k; // k of the force
    virtual void addForce(Particle *particle);
};
#endif // FORCE_H