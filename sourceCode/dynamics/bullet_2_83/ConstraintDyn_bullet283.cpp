#include "ConstraintDyn_bullet283.h"
#include "RigidBodyContainerDyn.h"
#include "RigidBodyDyn_bullet283.h"
#include "simLib.h"

CConstraintDyn_bullet283::CConstraintDyn_bullet283(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,btDiscreteDynamicsWorld* bulletWorld)
{
	_parentBody=parentBody;
	_childBody=childBody;
	_jointOrForceSensorLoopClosureLinkedDummyAChildID=-1; // not used (non-looped case)
	_jointOrForceSensorLoopClosureLinkedDummyBChildID=-1; // not used (non-looped case)
	_nonCyclicRevoluteJointPositionOffset=0.0f;
	_jointIsCyclic=!_simGetJointPositionInterval(joint,&_nonCyclicRevoluteJointPositionMinimum,&_nonCyclicRevoluteJointPositionRange);
	_dummyID=-1;
	_forceSensorID=-1;
	_constraintID=-1;
	_jointID=_simGetObjectID(joint);
	_lastEffortOnJoint=0.0f;
	_dynPassCount=0;
	_lastJointPosSet=false;
	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	// Following 4 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
	CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
	CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
	parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
	childBody->reportConfigurationToShape((CDummyShape*)childShape);

	_simGetObjectLocalTransformation(joint,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
	C7Vector tmpTr1;
	C7Vector tmpTr2;
	_simGetObjectCumulativeTransformation(childShape,tmpTr1.X.data,tmpTr1.Q.data,false);
	_simGetObjectCumulativeTransformation(joint,tmpTr2.X.data,tmpTr2.Q.data,false);
	_secondInitialLocalTransform=tmpTr1.getInverse()*tmpTr2;

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	C7Vector jtr,jtr2;
	_simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);
	_simGetObjectCumulativeTransformation(joint,jtr2.X.data,jtr2.Q.data,false);

	C3X3Matrix m;
	m.setIdentity();
	m.buildYRotation(1.5707963267f);
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{ // in Bullet the moving direction is the x-axis (not like CoppeliaSim the z-axis)
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*m.getQuaternion();
	}
	m.setIdentity();
	m.buildYRotation(piValue);
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{ // in Bullet the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
		C3X3Matrix jointOffsetThing;
		jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
		{ // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
			_nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5f;
			jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
			_jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
		}
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
	}

	C7Vector batr;
	C7Vector bbtr;
	batr.setIdentity();
	bbtr.setIdentity();
	C7Vector jtrRelToBodyA;
	C7Vector jtrRelToBodyB;
	_bodyAID=-1;
	_bodyBID=-1;

	btRigidBody* parentRigidBody=((CRigidBodyDyn_bullet283*)parentBody)->getBtRigidBody();
	btRigidBody* childRigidBody=((CRigidBodyDyn_bullet283*)childBody)->getBtRigidBody();

	if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		parentRigidBody->setActivationState(DISABLE_DEACTIVATION);
		parentBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		childRigidBody->setActivationState(DISABLE_DEACTIVATION);
		childBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING
    float stopERP=simGetEngineFloatParameter(sim_bullet_joint_stoperp,-1,joint,nullptr);
    float stopCFM=simGetEngineFloatParameter(sim_bullet_joint_stopcfm,-1,joint,nullptr);
    float normalCFM=simGetEngineFloatParameter(sim_bullet_joint_normalcfm,-1,joint,nullptr);
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{
		btHingeConstraint* hinge;
		btTransform jtrA;
		jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
		jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
		btTransform jtrB;
		jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
		hinge=new btHingeConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
		_constraint=hinge;
		// You have to modify the btHingeConstraint code and remove the -pi;+pi limitation in the setLimit routine!!!

		hinge->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		hinge->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		hinge->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{
		btSliderConstraint* slider;

		btTransform jtrA;
		jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
		jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));

		btTransform jtrB;
		jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));

		slider=new btSliderConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
		_constraint=slider;

		slider->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		slider->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		slider->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}
	if (_simGetJointType(joint)==sim_joint_spherical_subtype)
	{
		btPoint2PointConstraint* ballSocket;
		ballSocket=new btPoint2PointConstraint(*parentRigidBody,*childRigidBody,btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)),btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		_constraint=ballSocket;

		ballSocket->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		ballSocket->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		ballSocket->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}
	_constraint->setJointFeedback(&_bulletJointfeedbackStructure);
	_constraint->enableFeedback(true);
	bulletWorld->addConstraint(_constraint);
	handleMotorControl(joint,0,0);
}

CConstraintDyn_bullet283::CConstraintDyn_bullet283(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,btDiscreteDynamicsWorld* bulletWorld)
{
	_parentBody=parentBody;
	_childBody=childBody;
	_jointOrForceSensorLoopClosureLinkedDummyAChildID=_simGetObjectID(loopClosureDummyA);
	_jointOrForceSensorLoopClosureLinkedDummyBChildID=_simGetObjectID(loopClosureDummyB);

	_nonCyclicRevoluteJointPositionOffset=0.0f;
	_jointIsCyclic=!_simGetJointPositionInterval(joint,&_nonCyclicRevoluteJointPositionMinimum,&_nonCyclicRevoluteJointPositionRange);
	_dummyID=-1;
	_forceSensorID=-1;
	_constraintID=-1;
	_jointID=_simGetObjectID(joint);
	_lastEffortOnJoint=0.0f;
	_dynPassCount=0;
	_lastJointPosSet=false;
	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	// Following 4 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
	CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
	CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
	parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
	childBody->reportConfigurationToShape((CDummyShape*)childShape);

	_simGetObjectLocalTransformation(joint,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);

	_simGetObjectLocalTransformation(loopClosureDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data,false);
	_simGetObjectLocalTransformation(loopClosureDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data,false);

	// Following is DIFFERENT from the regular situation (non-looped):
	_simGetObjectLocalTransformation(loopClosureDummyB,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data,false);

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	C7Vector jtr;
	_simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);

	// Following is DIFFERENT from the regular situation (non-looped):
	C7Vector tmpTr1;
	_simGetObjectCumulativeTransformation(loopClosureDummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
	C7Vector jtr2(tmpTr1*_linkedDummyAInitialLocalTransform.getInverse());

	C3X3Matrix m;
	m.setIdentity();
	m.buildYRotation(1.5707963267f);
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{ // in Bullet the moving direction is the x-axis (not like CoppeliaSim the z-axis)
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*m.getQuaternion();
	}
	m.setIdentity();
	m.buildYRotation(piValue);
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{ // in Bullet the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
		C3X3Matrix jointOffsetThing;
		jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
		{ // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
			_nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5f;
			jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
			_jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
		}
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
	}

	C7Vector batr;
	C7Vector bbtr;
	batr.setIdentity();
	bbtr.setIdentity();
	C7Vector jtrRelToBodyA;
	C7Vector jtrRelToBodyB;
	_bodyAID=-1;
	_bodyBID=-1;

	btRigidBody* parentRigidBody=((CRigidBodyDyn_bullet283*)parentBody)->getBtRigidBody();
	btRigidBody* childRigidBody=((CRigidBodyDyn_bullet283*)childBody)->getBtRigidBody();

	if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		parentRigidBody->setActivationState(DISABLE_DEACTIVATION);
		parentBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		childRigidBody->setActivationState(DISABLE_DEACTIVATION);
		childBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING
    float stopERP=simGetEngineFloatParameter(sim_bullet_joint_stoperp,-1,joint,nullptr);
    float stopCFM=simGetEngineFloatParameter(sim_bullet_joint_stopcfm,-1,joint,nullptr);
    float normalCFM=simGetEngineFloatParameter(sim_bullet_joint_normalcfm,-1,joint,nullptr);
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{
		btHingeConstraint* hinge;
		btTransform jtrA;
		jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
		jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
		btTransform jtrB;
		jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
		hinge=new btHingeConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
		_constraint=hinge;

		// you have to modify the btHingeConstraint code and remove the -pi;+pi limitation in the setLimit routine!!!

		hinge->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		hinge->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		hinge->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{
		btSliderConstraint* slider;

		btTransform jtrA;
		jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
		jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));

		btTransform jtrB;
		jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));

		slider=new btSliderConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
		_constraint=slider;

		slider->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		slider->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		slider->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}
	if (_simGetJointType(joint)==sim_joint_spherical_subtype)
	{
		btPoint2PointConstraint* ballSocket;
		ballSocket=new btPoint2PointConstraint(*parentRigidBody,*childRigidBody,btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)),btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
		_constraint=ballSocket;

		ballSocket->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
		ballSocket->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
		ballSocket->setParam(BT_CONSTRAINT_CFM,normalCFM);
	}

	_constraint->setJointFeedback(&_bulletJointfeedbackStructure);
	_constraint->enableFeedback(true);
	bulletWorld->addConstraint(_constraint);
	handleMotorControl(joint,0,0);
}

CConstraintDyn_bullet283::CConstraintDyn_bullet283(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild,btDiscreteDynamicsWorld* bulletWorld)
{
	_parentBody=parentBody;
	_childBody=childBody;
	_jointOrForceSensorLoopClosureLinkedDummyAChildID=-1; // not used here
	_jointOrForceSensorLoopClosureLinkedDummyBChildID=-1; // not used here

	_nonCyclicRevoluteJointPositionOffset=0.0f;
	_jointID=-1;
	_forceSensorID=-1;
	_constraintID=-1;
	_dummyID=_simGetObjectID(dummyOnParent);
	_linkedDummyID=_simGetObjectID(dummyOnChild);
	_lastEffortOnJoint=0.0f;
	_dynPassCount=0;
	_lastJointPosSet=false;
	_jointIsCyclic=false;
	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	_simGetObjectLocalTransformation(dummyOnParent,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
	_simGetObjectLocalTransformation(dummyOnChild,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data,true);

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	// Following 4 lines are important when a dummy is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
	CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
	CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
	parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
	childBody->reportConfigurationToShape((CDummyShape*)childShape);

	C7Vector dtr,dtr2;
	_simGetObjectCumulativeTransformation(dummyOnParent,dtr.X.data,dtr.Q.data,true);
	_simGetObjectCumulativeTransformation(dummyOnChild,dtr2.X.data,dtr2.Q.data,true);

	C7Vector batr;
	C7Vector bbtr;
	batr.setIdentity();
	bbtr.setIdentity();
	C7Vector dtrRelToBodyA;
	C7Vector dtrRelToBodyB;
	_bodyAID=-1;
	_bodyBID=-1;

	btRigidBody* parentRigidBody=((CRigidBodyDyn_bullet283*)parentBody)->getBtRigidBody();
	btRigidBody* childRigidBody=((CRigidBodyDyn_bullet283*)childBody)->getBtRigidBody();

	if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		parentRigidBody->setActivationState(DISABLE_DEACTIVATION);
		parentBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	batr=parentBody->getInertiaFrameTransformation();
	dtrRelToBodyA=batr.getInverse()*dtr;
	_bodyAID=parentBody->getRigidBodyID();

	if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		childRigidBody->setActivationState(DISABLE_DEACTIVATION);
		childBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	dtrRelToBodyB=bbtr.getInverse()*dtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	dtrRelToBodyA.X*=linScaling; // ********** SCALING
	dtrRelToBodyB.X*=linScaling; // ********** SCALING

	btGeneric6DofConstraint* generalConstraint;
	btTransform dtrA;
	dtrA.setOrigin(btVector3(dtrRelToBodyA.X(0),dtrRelToBodyA.X(1),dtrRelToBodyA.X(2)));
	dtrA.setRotation(btQuaternion(dtrRelToBodyA.Q(1),dtrRelToBodyA.Q(2),dtrRelToBodyA.Q(3),dtrRelToBodyA.Q(0)));
	btTransform dtrB;
	dtrB.setOrigin(btVector3(dtrRelToBodyB.X(0),dtrRelToBodyB.X(1),dtrRelToBodyB.X(2)));
	dtrB.setRotation(btQuaternion(dtrRelToBodyB.Q(1),dtrRelToBodyB.Q(2),dtrRelToBodyB.Q(3),dtrRelToBodyB.Q(0)));
	generalConstraint=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,dtrA,dtrB,true);

	// Lock all axes (translations are locked by default):
	generalConstraint->getRotationalLimitMotor(0)->m_loLimit=0.0f;
	generalConstraint->getRotationalLimitMotor(0)->m_hiLimit=0.0f;
	generalConstraint->getRotationalLimitMotor(1)->m_loLimit=0.0f;
	generalConstraint->getRotationalLimitMotor(1)->m_hiLimit=0.0f;
	generalConstraint->getRotationalLimitMotor(2)->m_loLimit=0.0f;
	generalConstraint->getRotationalLimitMotor(2)->m_hiLimit=0.0f;

	_constraint=generalConstraint;
	_constraint->setJointFeedback(&_bulletJointfeedbackStructure);
	_constraint->enableFeedback(true);
	bulletWorld->addConstraint(_constraint);
}

CConstraintDyn_bullet283::CConstraintDyn_bullet283(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,btDiscreteDynamicsWorld* bulletWorld)
{
	_parentBody=parentBody;
	_childBody=childBody;
	_jointOrForceSensorLoopClosureLinkedDummyAChildID=-1; // not used (non-looped case)
	_jointOrForceSensorLoopClosureLinkedDummyBChildID=-1; // not used (non-looped case)

	_nonCyclicRevoluteJointPositionOffset=0.0f;
	_dummyID=-1;
	_constraintID=-1;
	_jointID=-1;
	_forceSensorID=_simGetObjectID(forceSensor);
	_lastEffortOnJoint=0.0f;
	_dynPassCount=0;
	_lastJointPosSet=false;
	_jointIsCyclic=false;
	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	// Following 4 lines are important when a force sensor is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
	CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
	CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
	parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
	childBody->reportConfigurationToShape((CDummyShape*)childShape);

	_simGetObjectLocalTransformation(forceSensor,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
	C7Vector tmpTr1,tmpTr2;
	_simGetObjectCumulativeTransformation(childShape,tmpTr1.X.data,tmpTr1.Q.data,false);
	_simGetObjectCumulativeTransformation(forceSensor,tmpTr2.X.data,tmpTr2.Q.data,false);
	_secondInitialLocalTransform=tmpTr1.getInverse()*tmpTr2;



	 // since 2010/02/13:
	if (_simIsForceSensorBroken(forceSensor))
	{
		_simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
		_secondInitialLocalTransform*=tmpTr1;
	}

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	C7Vector jtr,jtr2;
	_simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
	_simGetObjectCumulativeTransformation(forceSensor,jtr2.X.data,jtr2.Q.data,false);

	if (_simIsForceSensorBroken(forceSensor)) // since 2010/02/13
	{
		_simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
		jtr2*=tmpTr1;
	}

	C7Vector batr;
	C7Vector bbtr;
	batr.setIdentity();
	bbtr.setIdentity();
	C7Vector jtrRelToBodyA;
	C7Vector jtrRelToBodyB;
	_bodyAID=-1;
	_bodyBID=-1;

	btRigidBody* parentRigidBody=((CRigidBodyDyn_bullet283*)parentBody)->getBtRigidBody();
	btRigidBody* childRigidBody=((CRigidBodyDyn_bullet283*)childBody)->getBtRigidBody();

	if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		parentRigidBody->setActivationState(DISABLE_DEACTIVATION);
		parentBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		childRigidBody->setActivationState(DISABLE_DEACTIVATION);
		childBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	btGeneric6DofConstraint* constr;
	btTransform jtrA;
	jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
	jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
	btTransform jtrB;
	jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
	jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
	constr=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);

	constr->setUseFrameOffset(false);

	_setForceSensorBrokenUnbrokenConstraints_bullet(constr,forceSensor);

	_constraint=constr;
	_constraint->setJointFeedback(&_bulletJointfeedbackStructure);
	_constraint->enableFeedback(true);
	bulletWorld->addConstraint(_constraint);
}

CConstraintDyn_bullet283::CConstraintDyn_bullet283(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,btDiscreteDynamicsWorld* bulletWorld)
{
	_parentBody=parentBody;
	_childBody=childBody;
	_jointOrForceSensorLoopClosureLinkedDummyAChildID=_simGetObjectID(loopClosureDummyA);
	_jointOrForceSensorLoopClosureLinkedDummyBChildID=_simGetObjectID(loopClosureDummyB);

	_nonCyclicRevoluteJointPositionOffset=0.0f;
	_dummyID=-1;
	_constraintID=-1;
	_jointID=-1;
	_forceSensorID=_simGetObjectID(forceSensor);
	_lastEffortOnJoint=0.0f;
	_dynPassCount=0;
	_lastJointPosSet=false;
	_jointIsCyclic=false;
	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	// Following 4 lines are important when a force sensor is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
	CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
	CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
	parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
	childBody->reportConfigurationToShape((CDummyShape*)childShape);

	_simGetObjectLocalTransformation(forceSensor,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
	_simGetObjectLocalTransformation(loopClosureDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data,false);
	_simGetObjectLocalTransformation(loopClosureDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data,false);

	// Following is DIFFERENT from the regular situation (non-looped):
	_simGetObjectLocalTransformation(loopClosureDummyB,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data,false);

	// Following is not needed for looped cases (_secondInitialLocalTransform is not used in that case), but anyway:
	if (_simIsForceSensorBroken(forceSensor)) // since 2010/02/13
	{
		C7Vector tmpTr1;
		_simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
		_secondInitialLocalTransform*=tmpTr1;
	}

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	C7Vector jtr;
	_simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
	// Following is DIFFERENT from the regular situation (non-looped):
	C7Vector tmpTr1;
	_simGetObjectCumulativeTransformation(loopClosureDummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
	C7Vector jtr2(tmpTr1*_linkedDummyAInitialLocalTransform.getInverse());

	 // since 2010/02/13
	if (_simIsForceSensorBroken(forceSensor))
	{
		C7Vector tmpTr1;
		_simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
		jtr2*=tmpTr1;
	}

	C7Vector batr;
	C7Vector bbtr;
	batr.setIdentity();
	bbtr.setIdentity();
	C7Vector jtrRelToBodyA;
	C7Vector jtrRelToBodyB;
	_bodyAID=-1;
	_bodyBID=-1;

	btRigidBody* parentRigidBody=((CRigidBodyDyn_bullet283*)parentBody)->getBtRigidBody();
	btRigidBody* childRigidBody=((CRigidBodyDyn_bullet283*)childBody)->getBtRigidBody();

	if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		parentRigidBody->setActivationState(DISABLE_DEACTIVATION);
		parentBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
	{ // this is a dynamic object
		childRigidBody->setActivationState(DISABLE_DEACTIVATION);
		childBody->setDefaultActivationState(DISABLE_DEACTIVATION);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	btGeneric6DofConstraint* constr;
	btTransform jtrA;
	jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
	jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
	btTransform jtrB;
	jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
	jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
	constr=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);

	constr->setUseFrameOffset(false);

	_setForceSensorBrokenUnbrokenConstraints_bullet(constr,forceSensor);

	_constraint=constr;
	_constraint->setJointFeedback(&_bulletJointfeedbackStructure);
	_constraint->enableFeedback(true);
	bulletWorld->addConstraint(_constraint);
}

CConstraintDyn_bullet283::~CConstraintDyn_bullet283()
{
	delete _constraint;
}

void CConstraintDyn_bullet283::_setRevoluteJointLimits(CDummyJoint* joint)
{
	btHingeConstraint* hinge;
	hinge=(btHingeConstraint*)_constraint;
    if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
		hinge->setLimit(+1.0f,-1.0f); // no limits
	else
	{ // Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems  (revolute joints only)
		if (_nonCyclicRevoluteJointPositionRange<=359.0f*piValue*2.0f/360.0f)
		{ // when the range is <359, we keep the limits on all the time
			hinge->setLimit(-_nonCyclicRevoluteJointPositionRange*0.5f,+_nonCyclicRevoluteJointPositionRange*0.5f); // active limits. Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
		}
		else
		{ // Bullet doesn't support a range > 360. So we manually turn limits on/off as needed:
			// That doesn't work in Bullet. We leave it unlimited for now:
			hinge->setLimit(+1.0f,-1.0f); // no limits
		}
	}
}

void CConstraintDyn_bullet283::_setPrismaticJointLimits(CDummyJoint* joint)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float jiMin,jiRange;
	_simGetJointPositionInterval(joint,&jiMin,&jiRange);

	btSliderConstraint* slider;
	slider=(btSliderConstraint*)_constraint;
	slider->setLowerLinLimit(jiMin*linScaling); // ********** SCALING
	slider->setUpperLinLimit((jiMin+jiRange)*linScaling); // ********** SCALING
}

void CConstraintDyn_bullet283::_handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

	btHingeConstraint* hinge;
	hinge=(btHingeConstraint*)_constraint;

	_targetPositionToHoldAtZeroVelOn_velocityMode=false;

	float e;

    if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
		e=getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint)+_nonCyclicRevoluteJointPositionOffset,getHingeAngle()); // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
	else
		e=_simGetDynamicMotorTargetPosition(joint)+_nonCyclicRevoluteJointPositionOffset-getHingeAngle(); // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)

	int auxV=0;
	if (_dynPassCount==0)
		auxV|=1;
	int inputValuesInt[5];
	inputValuesInt[0]=passCnt;
	inputValuesInt[1]=totalPasses;
	inputValuesInt[2]=0; // reserved for future ext.
	inputValuesInt[3]=0; // reserved for future ext.
	inputValuesInt[4]=0; // reserved for future ext.
	float inputValuesFloat[7];

	inputValuesFloat[0]=getHingeAngle()-_nonCyclicRevoluteJointPositionOffset;

	inputValuesFloat[1]=_lastEffortOnJoint;
	inputValuesFloat[2]=dynStepSize;
	inputValuesFloat[3]=e;
	inputValuesFloat[4]=0.0f; // reserved for future ext.
	inputValuesFloat[5]=0.0f; // reserved for future ext.
	inputValuesFloat[6]=0.0f; // reserved for future ext.
	float outputValues[5];
	_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
	float velocityToApply=outputValues[0];
	float forceToApply=outputValues[1];

	hinge->enableAngularMotor(true,velocityToApply,forceToApply*torqueScaling*dynStepSize); // ********** SCALING
}


void CConstraintDyn_bullet283::_handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

	btHingeConstraint* hinge;
	hinge=(btHingeConstraint*)_constraint;

	int param=0;
	simGetObjectIntParameter(_jointID,2030,&param);
	bool lockModeWhenInVelocityControl=(param!=0);

	float vel=_simGetDynamicMotorTargetVelocity(joint);

	if ( (lockModeWhenInVelocityControl)&&(vel==0.0f)&&(_simIsDynamicMotorEnabled(joint)!=0) )
	{
		if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
			_targetPositionToHoldAtZeroVel_velocityMode=((btHingeConstraint*)_constraint)->getHingeAngle();
//			_targetPositionToHoldAtZeroVel_velocityMode=getHingeAngle()-_nonCyclicRevoluteJointPositionOffset;
		_targetPositionToHoldAtZeroVelOn_velocityMode=true;
		hinge->setLimit(_targetPositionToHoldAtZeroVel_velocityMode,_targetPositionToHoldAtZeroVel_velocityMode);
		hinge->enableAngularMotor(false,0.0f,0.0f); // ********** SCALING
	}
	else
	{
		_targetPositionToHoldAtZeroVelOn_velocityMode=false;
		hinge->enableAngularMotor(_simIsDynamicMotorEnabled(joint)!=0,vel,_simGetDynamicMotorMaxForce(joint)*torqueScaling*dynStepSize); // ********** SCALING
	}
}

void CConstraintDyn_bullet283::_handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float linVelocityScaling=CRigidBodyContainerDyn::getLinearVelocityScalingFactorDyn();
	float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

	btSliderConstraint* slider;
	slider=(btSliderConstraint*)_constraint;

	_targetPositionToHoldAtZeroVelOn_velocityMode=false;
	float e=_simGetDynamicMotorTargetPosition(joint)-getSliderPositionScaled()/linScaling; // ********** SCALING

	int auxV=0;
	if (_dynPassCount==0)
		auxV|=1;
	int inputValuesInt[5];
	inputValuesInt[0]=passCnt;
	inputValuesInt[1]=totalPasses;
	inputValuesInt[2]=0; // reserved for future ext.
	inputValuesInt[3]=0; // reserved for future ext.
	inputValuesInt[4]=0; // reserved for future ext.
	float inputValuesFloat[7];
	inputValuesFloat[0]=getSliderPositionScaled()/linScaling; // Scaling was forgotten and added on 22/8/2013, thanks to Ruediger Dehmel.
	inputValuesFloat[1]=_lastEffortOnJoint;
	inputValuesFloat[2]=dynStepSize;
	inputValuesFloat[3]=e;
	inputValuesFloat[4]=0.0f; // reserved for future ext.
	inputValuesFloat[5]=0.0f; // reserved for future ext.
	inputValuesFloat[6]=0.0f; // reserved for future ext.
	float outputValues[5];
	_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
	float velocityToApply=outputValues[0];
	float forceToApply=outputValues[1];

	slider->setPoweredLinMotor(true);
	slider->setTargetLinMotorVelocity(velocityToApply*linVelocityScaling); // ********** SCALING
	slider->setMaxLinMotorForce(forceToApply*forceScaling);//*dynStepSize*dynStepSize); // ********** SCALING
}

void CConstraintDyn_bullet283::_handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float linVelocityScaling=CRigidBodyContainerDyn::getLinearVelocityScalingFactorDyn();
	float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

	btSliderConstraint* slider;
	slider=(btSliderConstraint*)_constraint;

	int param=0;
	simGetObjectIntParameter(_jointID,2030,&param);
	bool lockModeWhenInVelocityControl=(param!=0);
	float vel=_simGetDynamicMotorTargetVelocity(joint);

	if ( (lockModeWhenInVelocityControl)&&(vel==0.0f)&&(_simIsDynamicMotorEnabled(joint)!=0) )
	{
		if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
			_targetPositionToHoldAtZeroVel_velocityMode=getSliderPositionScaled()/linScaling;
		_targetPositionToHoldAtZeroVelOn_velocityMode=true;
		slider->setPoweredLinMotor(false); // motor off
		slider->setLowerLinLimit(_targetPositionToHoldAtZeroVel_velocityMode*linScaling); // ********** SCALING
		slider->setUpperLinLimit(_targetPositionToHoldAtZeroVel_velocityMode*linScaling); // ********** SCALING
	}
	else
	{
		_targetPositionToHoldAtZeroVelOn_velocityMode=false;
		slider->setPoweredLinMotor(_simIsDynamicMotorEnabled(joint)!=0);
		slider->setTargetLinMotorVelocity(vel*linVelocityScaling); // ********** SCALING
		slider->setMaxLinMotorForce(_simGetDynamicMotorMaxForce(joint)*forceScaling);//*dynStepSize*dynStepSize); // ********** SCALING
	}
}

dynReal CConstraintDyn_bullet283::getSliderPositionScaled()
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))
//	dynReal linScaling=(dynReal)CRigidBodyContainerDyn::getPositionScalingFactorDyn();

	dynReal linScaling=(dynReal)CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	C7Vector p(_parentBody->getShapeFrameTransformation());
	C7Vector c(_childBody->getShapeFrameTransformation());
	p=p*_initialLocalTransform;
	c=c*_secondInitialLocalTransform;
	if (_jointOrForceSensorLoopClosureLinkedDummyAChildID!=-1)
		c=c*_linkedDummyAInitialLocalTransform.getInverse(); // Non-regular case (looped) (bug correction on 2010/10/08)
//		return((dynReal)((p.getInverse()*c.X)(2)*((dynReal)linScaling)); // scaling is here needed
	return((dynReal)((p.getInverse()*c.X)(2)*linScaling)); // scaling is here needed
}

dynReal CConstraintDyn_bullet283::getHingeAngle()
{
	dynReal retVal=(dynReal)0.0;
	if (true)
	{ // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
		dynReal jointPos=(dynReal)0.0;

		jointPos=((btHingeConstraint*)_constraint)->getHingeAngle();

		if (_jointIsCyclic)
		{ // turn count not needed here
			retVal=jointPos;
		}
		else
		{
			if (_lastJointPosSet)
			{
				dynReal dx=jointPos-_lastJointPos;
				if (dx>=0.0)
					dx=fmod(dx+dynReal(3.14159265),dynReal(6.28318531))-dynReal(3.14159265);
				else
					dx=fmod(dx-dynReal(3.14159265),dynReal(6.28318531))+dynReal(3.14159265);
				_jointPosAlt+=dx;
			}
			retVal=_jointPosAlt;
		}
		_lastJointPos=jointPos;
		_lastJointPosSet=true;
	}
	return(retVal);
}

void CConstraintDyn_bullet283::reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{ // totalPassesCount is 0 when we need to accumulate the values, and diff. from zero when we need to average!
	// If totalPassesCount is -1, then we do not report the forces and torques!
	if (_simGetDynamicsFullRefreshFlag(forceSensor))
		return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
	if (_forceSensorID!=-1)
	{
//		float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
		_simSetObjectLocalTransformation(forceSensor,_initialLocalTransform.X.data,_initialLocalTransform.Q.data);

        if (linkedDummyA!=nullptr)
		{ // special case (looped)
			_simSetObjectLocalTransformation(linkedDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data);
			_simSetObjectLocalTransformation(linkedDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data);
		}

		float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
		float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();
//		float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();
		// Now report forces and torques acting on the force sensor:
		C3Vector forces;
		forces.clear();
		C3Vector torques;
		torques.clear();

		if (_simIsForceSensorBroken(forceSensor)==0)
		{
			btJointFeedback* jfb=_constraint->getJointFeedback();
			C3Vector absF(jfb->m_appliedForceBodyB.getX(),jfb->m_appliedForceBodyB.getY(),jfb->m_appliedForceBodyB.getZ());
			absF/=forceScaling; // ********** SCALING
			C3Vector absT(jfb->m_appliedTorqueBodyB.getX(),jfb->m_appliedTorqueBodyB.getY(),jfb->m_appliedTorqueBodyB.getZ());
			absT/=torqueScaling; // ********** SCALING
			C7Vector childBodyAbsConf(_childBody->getInertiaFrameTransformation());
			C7Vector parentShapeAbsConf(_parentBody->getShapeFrameTransformation());
			C7Vector sensorAbsConf(parentShapeAbsConf*_initialLocalTransform);
			C3Vector absCorrectionV(sensorAbsConf.X-childBodyAbsConf.X);
			absT+=absF^absCorrectionV;
			C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
			forces=sensorAbsInverseQ*(absF*-1.0f);
			torques=sensorAbsInverseQ*(absT*-1.0f);
		}

		_simAddForceSensorCumulativeForcesAndTorques(forceSensor,forces.data,torques.data,totalPassesCount);

		if (totalPassesCount>0)
			_setForceSensorBrokenUnbrokenConstraints_bullet((btGeneric6DofConstraint*)_constraint,forceSensor);
	}
}

void CConstraintDyn_bullet283::reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{ // totalPassesCount is 0 when we need to accumulate the values, and diff. from zero when we need to average!
	// If totalPassesCount is -1, then we do not report the forces and torques!
	if (_simGetDynamicsFullRefreshFlag(joint))
		return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
	if (_jointID!=-1)
	{
//		float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();

		// Is following correct for joints?? Dummies are actualized after joints! Decided to keep this on 1/6/2011
        if (linkedDummyA!=nullptr)
		{ // special case (looped)
			_simSetObjectLocalTransformation(linkedDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data);
			_simSetObjectLocalTransformation(linkedDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data);
		}

		float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
		float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();
//		float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();
		// Now report forces and torques acting on the joint:
		float forceOrTorque=0.0f;

		float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();
		if (_simGetJointType(joint)!=sim_joint_spherical_subtype)
		{ // Spherical joints are not supported here
			if (_simGetJointType(joint)==sim_joint_revolute_subtype)
				forceOrTorque=-_constraint->getAppliedImpulse()/(torqueScaling*dynStepSize);
			if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
			{
				btJointFeedback* jfb=_constraint->getJointFeedback();
				C3Vector absF(jfb->m_appliedForceBodyB.getX(),jfb->m_appliedForceBodyB.getY(),jfb->m_appliedForceBodyB.getZ());
				absF/=forceScaling; // ********** SCALING
				C7Vector childBodyAbsConf(_childBody->getInertiaFrameTransformation());
				C7Vector parentShapeAbsConf(_parentBody->getShapeFrameTransformation());
				C7Vector sensorAbsConf(parentShapeAbsConf*_initialLocalTransform);
				C3Vector absCorrectionV(sensorAbsConf.X-childBodyAbsConf.X);
				C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
				C3Vector forces(sensorAbsInverseQ*(absF*-1.0f));
				forceOrTorque=forces(2);
			}
		}

		_lastEffortOnJoint=forceOrTorque;
		_simAddJointCumulativeForcesOrTorques(joint,forceOrTorque,totalPassesCount);
	}
}

void CConstraintDyn_bullet283::_setForceSensorBrokenUnbrokenConstraints_bullet(btGeneric6DofConstraint* bulletConstr,CDummyForceSensor* forceSensor)
{
	btTranslationalLimitMotor* m=bulletConstr->getTranslationalLimitMotor();
	btRotationalLimitMotor* r[3];
	r[0]=bulletConstr->getRotationalLimitMotor(0);
	r[1]=bulletConstr->getRotationalLimitMotor(1);
	r[2]=bulletConstr->getRotationalLimitMotor(2);

	for (int i=0;i<3;i++) // First translational constraints:
	{
		if (_simIsForceSensorBroken(forceSensor)==0)
		{
			m->m_lowerLimit[i]=0.0f;
			m->m_upperLimit[i]=0.0f;
			m->m_enableMotor[i]=true;
			m->m_targetVelocity[i]=0.0f;
			m->m_maxMotorForce[i]=SIM_MAX_FLOAT;
		}
		else
		{
			m->m_lowerLimit[i]=1.0f;
			m->m_upperLimit[i]=-1.0f;
			m->m_enableMotor[i]=false;
		}
	}

	for (int i=0;i<3;i++) // Now rotational constraints:
	{
		if (_simIsForceSensorBroken(forceSensor)==0)
		{
			r[i]->m_loLimit=0.0f;
			r[i]->m_hiLimit=0.0f;
			r[i]->m_enableMotor=true;
			r[i]->m_targetVelocity=0.0f;
			r[i]->m_maxMotorForce=SIM_MAX_FLOAT;
		}
		else
		{
			r[i]->m_loLimit=1.0f;
			r[i]->m_hiLimit=-1.0f;
			r[i]->m_enableMotor=false;
		}
	}
}

btTypedConstraint* CConstraintDyn_bullet283::getBtTypedConstraint()
{
	return(_constraint);
}
