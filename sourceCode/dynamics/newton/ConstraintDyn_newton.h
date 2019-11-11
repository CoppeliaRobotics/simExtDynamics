#pragma once

#include "ConstraintDyn.h"
#include "Newton.h"

class CustomJoint;

class CConstraintDyn_newton : public CConstraintDyn
{
    class csimNewtonRevoluteJoint;
    class csimNewtonPrismaticJoint;
    class csimNewtonCommonJointData;
    class csimNewtonForceSensorJoint;
public:
	// ShapeA - joint - shapeB
	CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody, CDummyJoint* joint, NewtonWorld* world); // Newton

	// ShapeA - joint - dummyA - dummyB - shapeB
	CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB, NewtonWorld* world); // Newton

	// ShapeA - dummyA - dummyB - shapeB
	CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild, NewtonWorld* world); // Newton

	// ShapeA - force sensor - shapeB
	CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor, NewtonWorld* world); // Newton

	// ShapeA - force sensor - dummyA - dummyB - shapeB
	CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB, NewtonWorld* world); // Newton

	virtual ~CConstraintDyn_newton();

	CRigidBodyDyn* _getChild() const;
	CRigidBodyDyn* _getParent() const;
	CustomJoint* _getNewtonJoint() const;
	bool _isAcyclicJoint() const;
	void _setNewtonParameters(CDummyJoint* joint);
	bool getNewtonDependencyInfo(int& linkedJoint,float& fact, float& off);

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

	void _notifySekeletonRebuild();
	void _setForceSensorBrokenUnbrokenConstraints_newton (CDummyForceSensor* forceSensor);
	CustomJoint* _newtonConstraint;
	bool _isAcyclic;
	dynReal _newtonJointOffset;	// internally, we initialize Newton joints at 0, so this is the actual joint value when the joint is added
	int _newtonDependencyJointId;
	float _newtonDependencyFact;
	float _newtonDependencyOff;
};
