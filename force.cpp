#include "force.h"
#include "particle.h"

void Gravity::addForce(Particle *parti)
{
    parti->setF(parti->getF() + gravity * parti->getM());
}

void Viscous::addForce(Particle *parti)
{
    parti->setF(parti->getF() - k * parti->getV());
}