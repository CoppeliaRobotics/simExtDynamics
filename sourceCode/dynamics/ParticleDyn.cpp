#include "ParticleDyn.h"
#include "RigidBodyContainerDyn.h"
#include "4Vector.h"
#include "simLib.h"

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3])
{
	_initializationState=0;
	_currentPosition=position;
	_initialVelocityVector=velocity;
	_objectType=objType;
	_size=size;
	_massOverVolume=massOverVolume;
	_killTime=killTime;
    if (addColor!=nullptr)
	{
		_additionalColor[0]=addColor[0];
		_additionalColor[1]=addColor[1];
		_additionalColor[2]=addColor[2];
	}
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(float parameters[18],int objectID)
{
	return(false);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
}

void CParticleDyn::removeFromEngine()
{
}

void CParticleDyn::updatePosition()
{
}

bool CParticleDyn::didTimeOut(float simulationTime)
{
	return(simulationTime>_killTime);
}

int CParticleDyn::getInitializationState()
{
	return(_initializationState);
}

int CParticleDyn::getUniqueID()
{
	return(_uniqueID);
}

void CParticleDyn::setUniqueID(int id)
{
	_uniqueID=id;
}

bool CParticleDyn::getRenderData(float* pos,float* size,int* objType,float** additionalColor)
{
	if (_initializationState==1)
	{
		_currentPosition.getInternalData(pos);
		objType[0]=_objectType;
		size[0]=_size;
		additionalColor[0]=_additionalColor;
		return(true);
	}
	return(false);
}
