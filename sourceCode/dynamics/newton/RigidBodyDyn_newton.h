#pragma once

#include "RigidBodyDyn.h"
#include "Newton.h"

class CRigidBodyDyn_newton : public CRigidBodyDyn
{
public:
	CRigidBodyDyn_newton(CDummyShape* const shape,CCollShapeDyn* const collShapeDyn,bool const forceStatic,bool const forceNonRespondable,NewtonWorld* const world); // newton
	virtual ~CRigidBodyDyn_newton();

	NewtonBody* getNewtonRigidBody() const;

	C7Vector getNewtonMatrix() const;
	static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
	static void ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex);

	C7Vector getInertiaFrameTransformation();
	C7Vector getShapeFrameTransformation();
	void reportVelocityToShape(CDummyShape* shape);
	void handleAdditionalForcesAndTorques(CDummyShape* shape);
	void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
	void applyCorrectEndConfig_forKinematicBody();

protected:	
	void _setNewtonParameters(CDummyShape* shape);

	NewtonBody* _newtonBody;
	C3Vector m_externForce;
	C3Vector m_externTorque;
	void* _newtonBodyUserData[5];// shapeID,this,stat. friction,kin. friction, restitution
	float _newtonStaticFriction;
	float _newtonKineticFriction;
	float _newtonRestitution;
	float _newtonLinearDrag;
	float _newtonAngularDrag;
	bool _newtonFastMoving;
};
