#include "ParticleContainer.h"
#include "simLib.h"

CParticleContainer::CParticleContainer()
{
	
}

CParticleContainer::~CParticleContainer()
{
	removeAllObjects();
}

CParticleObject* CParticleContainer::getObject(int objectID,bool getAlsoTheOnesFlaggedForDestruction)
{
	if ( (objectID>=0)&&(objectID<int(_allObjects.size())) )
	{
		if (getAlsoTheOnesFlaggedForDestruction)
			return(_allObjects[objectID]);
        if (_allObjects[objectID]!=nullptr)
		{
			if (_allObjects[objectID]->isFlaggedForDestruction())
                return(nullptr);
			return(_allObjects[objectID]);
		}
        return(nullptr);
	}
    return(nullptr);
}

int CParticleContainer::addObject(CParticleObject* it)
{
	int newID=0;
    while (getObject(newID,true)!=nullptr)
		newID++;
	it->setObjectID(newID);
	if (newID>=int(_allObjects.size()))
        _allObjects.push_back(nullptr);
	_allObjects[newID]=it;
	if ((it->getLifeTime()<0.0f)&&(it->getSize()<-100.0f))
		return(131183);
	return(newID);
}

void CParticleContainer::removeAllObjects()
{
	for (int i=0;i<int(_allObjects.size());i++)
        delete _allObjects[i]; // Can be nullptr!
	_allObjects.clear();
}

void CParticleContainer::removeObject(int objectID)
{ // objectID is the index
	if ( (objectID>=0)&&(objectID<int(_allObjects.size())) )
	{
        if (_allObjects[objectID]!=nullptr)
		{
			if (!_allObjects[objectID]->isFlaggedForDestruction())
			{
				if (_allObjects[objectID]->canBeDestroyed())
				{
					delete _allObjects[objectID];
                    _allObjects[objectID]=nullptr;
				}
				else
					_allObjects[objectID]->flagForDestruction();
			}
		}
	}
}

void** CParticleContainer::getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
	if (index>=int(_allObjects.size()))
	{
		particlesCount[0]=-1;
        return(nullptr);
	}
    if ( (_allObjects[index]!=nullptr)&&(!_allObjects[index]->isFlaggedForDestruction()) )
		return(_allObjects[index]->getParticles(particlesCount,objectType,cols));
	particlesCount=0;
    return(nullptr);
}

bool CParticleContainer::addParticlesIfNeeded()
{ // return value indicates if there are particles that need to be simulated
	bool particlesPresent=false;
	for (int i=0;i<int(_allObjects.size());i++)
	{
        if ( (_allObjects[i]!=nullptr)&&(!_allObjects[i]->isFlaggedForDestruction()) )
			particlesPresent|=_allObjects[i]->addParticlesIfNeeded();
	}
	return(particlesPresent);
}

void CParticleContainer::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity)
{
	for (int i=0;i<int(_allObjects.size());i++)
	{
        if ( (_allObjects[i]!=nullptr)&&(!_allObjects[i]->isFlaggedForDestruction()) )
			_allObjects[i]->handleAntiGravityForces_andFluidFrictionForces(gravity);
	}
}

void CParticleContainer::removeKilledParticles()
{ // beware, _allObjects[i] can be nullptr!
	for (int i=0;i<int(_allObjects.size());i++)
	{
        if (_allObjects[i]!=nullptr)
		{
			if (_allObjects[i]->isFlaggedForDestruction())
			{
				_allObjects[i]->removeAllParticles();
				delete _allObjects[i];
                _allObjects[i]=nullptr;
			}
			else
				_allObjects[i]->removeKilledParticles();
		}
	}
}

void CParticleContainer::removeAllParticles()
{ // beware, _allObjects[i] can be nullptr!
	for (int i=0;i<int(_allObjects.size());i++)
	{
        if (_allObjects[i]!=nullptr)
		{
			if (_allObjects[i]->isFlaggedForDestruction())
			{
				_allObjects[i]->removeAllParticles();
				delete _allObjects[i];
                _allObjects[i]=nullptr;
			}
			else
				_allObjects[i]->removeAllParticles();
		}
	}
}

void CParticleContainer::updateParticlesPosition(float simulationTime)
{
	for (int i=0;i<int(_allObjects.size());i++)
	{
        if ( (_allObjects[i]!=nullptr)&&(!_allObjects[i]->isFlaggedForDestruction()) )
			_allObjects[i]->updateParticlesPosition(simulationTime);
	}
}



