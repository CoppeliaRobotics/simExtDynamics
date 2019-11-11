#pragma once

#include "ConstraintDyn.h"
#include "ode/ode.h"

class CConstraintDyn_ode : public CConstraintDyn
{
public:
	// ShapeA - joint - shapeB
	CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,dWorldID odeWorldID); // ODE

	// ShapeA - joint - dummyA - dummyB - shapeB
	CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,dWorldID odeWorldID); // ODE

	// ShapeA - dummyA - dummyB - shapeB
	CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild,dWorldID odeWorldID); // ODE

	// ShapeA - force sensor - shapeB
	CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,dWorldID odeWorldID); // ODE

	// ShapeA - force sensor - dummyA - dummyB - shapeB
	CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,dWorldID odeWorldID); // ODE

	virtual ~CConstraintDyn_ode();

	void reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
	void reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
	dynReal getSliderPositionScaled(); // important! The slider pos is not initialized when added (Bullet)!
	dynReal getHingeAngle();

protected:
	void _handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
	void _handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);

	void _handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
	void _handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);

	void _setRevoluteJointLimits(CDummyJoint* joint);
	void _setPrismaticJointLimits(CDummyJoint* joint);

	void _setForceSensorBrokenUnbrokenConstraints_ode(dJointID odeConstr,CDummyForceSensor* forceSensor);
	dJointID _odeConstraint;
	dJointFeedback* _odeJointFeedbackStructure;
};
