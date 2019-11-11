#pragma once

#include "ParticleDyn.h"
#include "ode/ode.h"

class CParticleDyn_ode : public CParticleDyn
{
public:
	CParticleDyn_ode(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]);
	virtual ~CParticleDyn_ode();

	bool addToEngineIfNeeded(float parameters[18],int objectID);
	void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff);
	void updatePosition();
	void removeFromEngine();

protected:	
	dBodyID _odeRigidBody;
	dGeomID _odeGeom;
};
