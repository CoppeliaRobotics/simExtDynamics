#pragma once

#include "RigidBodyDyn.h"
#include "btBulletDynamicsCommon.h"

class CRigidBodyDyn_bullet278 : public CRigidBodyDyn
{
public:
	CRigidBodyDyn_bullet278(CDummyShape* shape,CCollShapeDyn* collShapeDyn,bool forceStatic,bool forceNonRespondable,btDiscreteDynamicsWorld* bulletWorld); // Bullet
	virtual ~CRigidBodyDyn_bullet278();

	btRigidBody* getBtRigidBody();

	C7Vector getInertiaFrameTransformation();
	C7Vector getShapeFrameTransformation();
	void reportVelocityToShape(CDummyShape* shape);
	void handleAdditionalForcesAndTorques(CDummyShape* shape);
	void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
	void applyCorrectEndConfig_forKinematicBody();

protected:	
	btRigidBody* _rigidBody;
};
