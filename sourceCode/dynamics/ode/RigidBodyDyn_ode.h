#pragma once

#include "RigidBodyDyn.h"
#include "ode/ode.h"

class CRigidBodyDyn_ode : public CRigidBodyDyn
{
public:
	CRigidBodyDyn_ode(CDummyShape* shape,CCollShapeDyn* collShapeDyn,bool forceStatic,bool forceNonRespondable,dWorldID odeWorld); // ODE
	virtual ~CRigidBodyDyn_ode();

	dBodyID getOdeRigidBody();

	C7Vector getInertiaFrameTransformation();
	C7Vector getShapeFrameTransformation();
	void reportVelocityToShape(CDummyShape* shape);
	void handleAdditionalForcesAndTorques(CDummyShape* shape);
	void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
	void applyCorrectEndConfig_forKinematicBody();

protected:	
	dBodyID _odeRigidBody;
};
