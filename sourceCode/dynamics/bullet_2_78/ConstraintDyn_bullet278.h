#pragma once

#include "ConstraintDyn.h"
#include "btBulletDynamicsCommon.h"

class CConstraintDyn_bullet278 : public CConstraintDyn
{
public:
    // ShapeA - joint - shapeB
    CConstraintDyn_bullet278(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,btDiscreteDynamicsWorld* bulletWorld); // Bullet

    // ShapeA - joint - dummyA - dummyB - shapeB
    CConstraintDyn_bullet278(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,btDiscreteDynamicsWorld* bulletWorld); // Bullet

    // ShapeA - dummyA - dummyB - shapeB
    CConstraintDyn_bullet278(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild,btDiscreteDynamicsWorld* bulletWorld); // Bullet

    // ShapeA - force sensor - shapeB
    CConstraintDyn_bullet278(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,btDiscreteDynamicsWorld* bulletWorld); // Bullet

    // ShapeA - force sensor - dummyA - dummyB - shapeB
    CConstraintDyn_bullet278(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,btDiscreteDynamicsWorld* bulletWorld); // Bullet

    virtual ~CConstraintDyn_bullet278();

    btTypedConstraint* getBtTypedConstraint();

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

    void _setForceSensorBrokenUnbrokenConstraints_bullet(btGeneric6DofConstraint* bulletConstr,CDummyForceSensor* forceSensor);
    btTypedConstraint* _constraint;
};
