#pragma once

#include <vector>
#include "ParticleObject.h"

class CParticleContainer
{
public:
	CParticleContainer();
	virtual ~CParticleContainer();

	int addObject(CParticleObject* it);
	CParticleObject* getObject(int objectID,bool getAlsoTheOnesFlaggedForDestruction);
	void removeAllObjects();
	void removeObject(int objectID);
	void** getParticles(int index,int* particlesCount,int* objectType,float** cols);

	bool addParticlesIfNeeded();
	void removeKilledParticles();
	void removeAllParticles();
	void updateParticlesPosition(float simulationTime);

	void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

private:
    std::vector<CParticleObject*> _allObjects; // can contain nullptr!
};
