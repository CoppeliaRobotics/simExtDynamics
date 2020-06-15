#pragma once

#include "ParticleDyn.h"
#include "btBulletDynamicsCommon.h"

class CParticleDyn_bullet283 : public CParticleDyn
{
public:
    CParticleDyn_bullet283(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]);
    virtual ~CParticleDyn_bullet283();

    bool addToEngineIfNeeded(float parameters[18],int objectID);
    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff);
    void updatePosition();
    void removeFromEngine();

protected:    
    btRigidBody* _rigidBody;
    btCollisionShape* _collShape;
};
