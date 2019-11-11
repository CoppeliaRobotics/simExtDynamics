#pragma once

#include "ParticleDyn.h"
namespace Vx
{
	class VxUniverse;
	class VxPart;
}

class CParticleDyn_vortex : public CParticleDyn
{
public:
	CParticleDyn_vortex(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]);
	virtual ~CParticleDyn_vortex();

	bool addToEngineIfNeeded(float parameters[18],int objectID);
	void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff);
	void updatePosition();
	void removeFromEngine();

protected:	
	Vx::VxPart* _vortexRigidBody;
};
