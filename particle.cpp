// SAMPLE_SOLUTION
#include "particle.h"
#include "FL/gl.h"
#include "modelerdraw.h"
#include <GL/glu.h>

void Particle::update(float dt)
{
    for (std::vector<Force *>::iterator it = ForcesTobeApplied.begin(); it != ForcesTobeApplied.end(); it++)
    {
        (*it)->addForce(this);
    }

    // euler method
    velocity += netF / mass * dt;
    position += velocity * dt;
    life -= dt;

    // std::cout << "dt" << dt << "life" << life << std::endl;
    // std::cout << "update" << "pos:" << position << "vel:" << velocity << std::endl;
}

void Particle::draw()
{
    float r, g, b;
    r = rand() % 100 / 100.0;
    g = rand() % 100 / 100.0;
    b = rand() % 100 / 100.0;
    setDiffuseColor(r, g, b);
    glPushMatrix();
    glTranslated(position[0], position[1], position[2]);
    drawSphere(0.1);
    glPopMatrix();
}
