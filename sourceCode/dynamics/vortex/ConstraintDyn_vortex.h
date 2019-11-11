#pragma once

#include "ConstraintDyn.h"

namespace Vx
{
    class VxUniverse;
    class VxConstraint;
}

class CConstraintDyn_vortex : public CConstraintDyn
{
public:
	// ShapeA - joint - shapeB
	CConstraintDyn_vortex(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint, Vx::VxUniverse*); // VORTEX

	// ShapeA - joint - dummyA - dummyB - shapeB
	CConstraintDyn_vortex(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,Vx::VxUniverse*); // VORTEX

	// ShapeA - dummyA - dummyB - shapeB
	CConstraintDyn_vortex(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild,Vx::VxUniverse*); // VORTEX

	// ShapeA - force sensor - shapeB
	CConstraintDyn_vortex(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,Vx::VxUniverse*); // VORTEX

	// ShapeA - force sensor - dummyA - dummyB - shapeB
	CConstraintDyn_vortex(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,Vx::VxUniverse*); // VORTEX

	virtual ~CConstraintDyn_vortex();

	bool getVortexDependencyInfo(int& linkedJoint,double& fact, double& off);
	Vx::VxConstraint* getVortexConstraint();

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

	void _setForceSensorBrokenUnbrokenConstraints_vortex(Vx::VxConstraint* vortexConstr,CDummyForceSensor* forceSensor);
	void _setVortexParameters(CDummyJoint* joint);
	Vx::VxConstraint* _vortexConstraint;
	int _vortexDependencyJointId;
	double _vortexDependencyFact;
	double _vortexDependencyOff;
};
