#pragma warning(disable : 4786)

#include "particleSystem.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>
#include "mat.h"

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem()
{
	// TODO
	srand(time(0));
	force_vec.push_back(new Gravity(Vec3d(0, -9.8, 0)));
	force_vec.push_back(new Viscous(0.1));
}

/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem()
{
	// TODO
	particle_vec.clear();
	force_vec.clear();
}

/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{

	// TODO
	bake_start_time = t;

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;
}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{

	// TODO

	bake_end_time = t;
	// These values are used by the UI
	simulate = false;
	dirty = true;
}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{

	// TODO

	// These values are used by the UI
	simulate = false;
	dirty = true;
}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	// TODO
	bake_fps = t - curr_time;
	curr_time = t;
	if (isSimulate() && !isBakedAt(t))
	{
		// std::cout << "update" << std::endl;
		std::vector<Particle>::iterator itLastDead = particle_vec.begin();
		for (auto it = particle_vec.begin(); it != particle_vec.end(); it++)
		{
			it->update(bake_fps);
			if (it->getLife() < 0)
				itLastDead = it;
		}
		particle_vec.erase(particle_vec.begin(), itLastDead);
		bakeParticles(t);
	}
}

/** Render particles */
void ParticleSystem::drawParticles(float t)
{

	// TODO
	if (isBakedAt(t))
	{
		map<float, std::vector<Particle>>::iterator iter = bake_storage.find(t);
		for (std::vector<Particle>::iterator it = iter->second.begin(); it != iter->second.end(); it++)
			it->draw();
	}
	else if (isSimulate())
	{
		for (auto it = particle_vec.begin(); it != particle_vec.end(); it++)
		{
			it->draw();
		}
	}
}

/** Adds the current configuration of particles to
 * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t)
{
	// TODO
	bake_storage.insert(std::pair<float, std::vector<Particle>>(t, particle_vec));
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
	bake_storage.clear();
}

// add
bool ParticleSystem::isBakedAt(float t)
{
	auto it = bake_storage.find(t);
	return (it != bake_storage.end());
}

void ParticleSystem::spawnParticles(Mat4f CameraM, Mat4f CurrModelM)
{
	Mat4f WorldM = CameraM.inverse() * CurrModelM;
	Vec4f WorldP4 = WorldM * Vec4f(0, 0, 0, 1);

	if (isSimulate())
	{
		if (!isBakedAt(curr_time + bake_fps))
		{
			for (int i = 0; i < 20; i++)
			{
				Vec3d WorldP(WorldP4[0], WorldP4[1], WorldP4[2]);
				double mass = 3;
				float lifetime = 0.8f + rand() / (RAND_MAX / 0.2f);
				// std::cout << "lifetime" << lifetime << std::endl;
				Particle newP(WorldP, mass, lifetime);
				float initialSpeed = rand() / (RAND_MAX / 2.0f);
				float theta = rand() / (RAND_MAX / 360.0f);
				float xSpeed = cos(theta) * initialSpeed;
				float zSpeed = sin(theta) * initialSpeed;
				Vec3d velocity(xSpeed, 5, zSpeed);

				newP.setV(velocity);

				for (std::vector<Force *>::iterator it = force_vec.begin(); it != force_vec.end(); it++)
				{
					newP.addForce(*it);
				}
				particle_vec.push_back(newP);
			}
		}
	}
}