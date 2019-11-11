#include "ConstraintDyn_ode.h"
#include "RigidBodyContainerDyn.h"
#include "RigidBodyDyn_ode.h"
#include "simLib.h"

CConstraintDyn_ode::CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,dWorldID odeWorldID)
{
    _odeJointFeedbackStructure=nullptr; // Only used with ODE force sensors or joints (1/6/2011)
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
	C7Vector tmpTr1,tmpTr2;
	_simGetObjectCumulativeTransformation(childShape,tmpTr1.X.data,tmpTr1.Q.data,false);
	_simGetObjectCumulativeTransformation(joint,tmpTr2.X.data,tmpTr2.Q.data,false);
	_secondInitialLocalTransform=tmpTr1.getInverse()*tmpTr2;

	_parentShapeID=parentBody->getShapeID();
	_childShapeID=childBody->getShapeID();

	C7Vector jtr,jtr2;
	_simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);
	_simGetObjectCumulativeTransformation(joint,jtr2.X.data,jtr2.Q.data,false);

	C3X3Matrix m;
	m.buildYRotation(piValue);
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{ // in ODE the rotation direction is the negative z-axis (not like CoppeliaSim the z-axis)
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*m.getQuaternion();
	}
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{ // in ODE the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
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

	dBodyID parentRigidBody=((CRigidBodyDyn_ode*)parentBody)->getOdeRigidBody();
	dBodyID childRigidBody=((CRigidBodyDyn_ode*)childBody)->getOdeRigidBody();

	//	if ((parentBody->isBodyKinematic())
	{ // this is a dynamic object
		dBodySetAutoDisableFlag(parentRigidBody,0);
		dBodyEnable(parentRigidBody);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

//	if ((childBody->isBodyKinematic())
	{ // this is a dynamic object
		dBodySetAutoDisableFlag(childRigidBody,0);
		dBodyEnable(childRigidBody);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();


	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	jtr.X*=linScaling; // ********** SCALING
	jtr2.X*=linScaling; // ********** SCALING
	C3X3Matrix jtrm(jtr.Q);

	// Following parameter retrieval is OLD. Use instead following functions:
	// - simGetEngineFloatParameter
	// - simGetEngineInt32Parameter
	// - simGetEngineBoolParameter
	float stopERP,stopCFM,bounce,fudge,normalCFM;
	_simGetJointOdeParameters(joint,&stopERP,&stopCFM,&bounce,&fudge,&normalCFM);
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{
		_odeConstraint=dJointCreateHinge(odeWorldID,0);
		dJointSetHingeParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetHingeParam(_odeConstraint,dParamBounce,bounce);
		dJointSetHingeParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetHingeParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetHingeParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);


		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetHingeAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));
		dJointSetHingeAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// New since 1/6/2011:
		_odeJointFeedbackStructure=new dJointFeedback;
		dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
	}
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{
		_odeConstraint=dJointCreateSlider(odeWorldID,0);
		dJointSetSliderParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetSliderParam(_odeConstraint,dParamBounce,bounce);
		dJointSetSliderParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetSliderParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetSliderParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetSliderAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// New since 1/6/2011:
		_odeJointFeedbackStructure=new dJointFeedback;
		dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
	}
	if (_simGetJointType(joint)==sim_joint_spherical_subtype)
	{
		_odeConstraint=dJointCreateBall(odeWorldID,0);
		dJointSetBallParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetBallParam(_odeConstraint,dParamBounce,bounce);
		dJointSetBallParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetBallParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetBallParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetBallAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=nullptr;
	}
	handleMotorControl(joint,0,0);
}

CConstraintDyn_ode::CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,dWorldID odeWorldID)
{
    _odeJointFeedbackStructure=nullptr; // Only used with ODE force sensors or joints (1/6/2011)
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
	m.buildYRotation(piValue);
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{ // in ODE the rotation direction is the negative z-axis (not like CoppeliaSim the z-axis)
		jtr.Q=jtr.Q*m.getQuaternion();
		jtr2.Q=jtr2.Q*m.getQuaternion();
	}
	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{ // in ODE the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
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

	dBodyID parentRigidBody=((CRigidBodyDyn_ode*)parentBody)->getOdeRigidBody();
	dBodyID childRigidBody=((CRigidBodyDyn_ode*)childBody)->getOdeRigidBody();

	//	if ((parentBody->isBodyKinematic())
	{ // this is a dynamic object
		dBodySetAutoDisableFlag(parentRigidBody,0);
		dBodyEnable(parentRigidBody);
	}
	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

//	if ((childBody->isBodyKinematic())
	{ // this is a dynamic object
		dBodySetAutoDisableFlag(childRigidBody,0);
		dBodyEnable(childRigidBody);
	}
	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	jtr.X*=linScaling; // ********** SCALING
	jtr2.X*=linScaling; // ********** SCALING
	C3X3Matrix jtrm(jtr.Q);

	// Following parameter retrieval is OLD. Use instead following functions:
	// - simGetEngineFloatParameter
	// - simGetEngineInt32Parameter
	// - simGetEngineBoolParameter
	float stopERP,stopCFM,bounce,fudge,normalCFM;
	_simGetJointOdeParameters(joint,&stopERP,&stopCFM,&bounce,&fudge,&normalCFM);

	if (_simGetJointType(joint)==sim_joint_revolute_subtype)
	{
		_odeConstraint=dJointCreateHinge(odeWorldID,0);
		dJointSetHingeParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetHingeParam(_odeConstraint,dParamBounce,bounce);
		dJointSetHingeParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetHingeParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetHingeParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);


		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetHingeAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));
		dJointSetHingeAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// New since 1/6/2011:
		_odeJointFeedbackStructure=new dJointFeedback;
		dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
	}
	if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
	{
		_odeConstraint=dJointCreateSlider(odeWorldID,0);
		dJointSetSliderParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetSliderParam(_odeConstraint,dParamBounce,bounce);
		dJointSetSliderParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetSliderParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetSliderParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetSliderAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// New since 1/6/2011:
		_odeJointFeedbackStructure=new dJointFeedback;
		dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
	}
	if (_simGetJointType(joint)==sim_joint_spherical_subtype)
	{
		_odeConstraint=dJointCreateBall(odeWorldID,0);
		dJointSetBallParam(_odeConstraint,dParamFudgeFactor,fudge);
		dJointSetBallParam(_odeConstraint,dParamBounce,bounce);
		dJointSetBallParam(_odeConstraint,dParamCFM,normalCFM);
		dJointSetBallParam(_odeConstraint,dParamStopERP,stopERP);
		dJointSetBallParam(_odeConstraint,dParamStopCFM,stopCFM);

		// Set the configuration of the child as if the joint was at 0 position:
		dBodyID cb=childRigidBody;
		C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
		C7Vector alpha(jtr2.getInverse()*cb_a);
		C7Vector cb_b(jtr*alpha);
		dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
		dQuaternion dQ;
		dQ[0]=cb_b.Q.data[0];
		dQ[1]=cb_b.Q.data[1];
		dQ[2]=cb_b.Q.data[2];
		dQ[3]=cb_b.Q.data[3];
		dBodySetQuaternion(cb,dQ);

		// Attach the joint to the 2 bodies
		dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
		dJointSetBallAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));

		// Reset the configuration of the child as it is now:
		dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
		dQ[0]=cb_a.Q.data[0];
		dQ[1]=cb_a.Q.data[1];
		dQ[2]=cb_a.Q.data[2];
		dQ[3]=cb_a.Q.data[3];
		dBodySetQuaternion(cb,dQ);
	}
		handleMotorControl(joint,0,0);
}

CConstraintDyn_ode::CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild,dWorldID odeWorldID)
{
    _odeJointFeedbackStructure=nullptr; // Only used with ODE force sensors or joints (1/6/2011)
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

	_bodyAID=parentBody->getRigidBodyID();

	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();

	dtr.X*=linScaling; // ********** SCALING
	dtr2.X*=linScaling; // ********** SCALING

	_odeConstraint=dJointCreateFixed(odeWorldID,0);

	dBodyID parentRigidBody=((CRigidBodyDyn_ode*)parentBody)->getOdeRigidBody();
	dBodyID childRigidBody=((CRigidBodyDyn_ode*)childBody)->getOdeRigidBody();

	// Set the configuration of the child as if the dummies were overlapping:
	dBodyID cb=childRigidBody;
	C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
	C7Vector x(cb_a.getInverse()*dtr2);
	C7Vector cb_b(dtr*x.getInverse());
	dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
	dQuaternion dQ;
	dQ[0]=cb_b.Q.data[0];
	dQ[1]=cb_b.Q.data[1];
	dQ[2]=cb_b.Q.data[2];
	dQ[3]=cb_b.Q.data[3];
	dBodySetQuaternion(cb,dQ);

	// Attach the fixed joint to the 2 bodies:
	dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
	dJointSetFixed(_odeConstraint);

	// Reset the configuration of the child as it is now:
	dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
	dQ[0]=cb_a.Q.data[0];
	dQ[1]=cb_a.Q.data[1];
	dQ[2]=cb_a.Q.data[2];
	dQ[3]=cb_a.Q.data[3];
	dBodySetQuaternion(cb,dQ);
}

CConstraintDyn_ode::CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,dWorldID odeWorldID)
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

	// since 2010/02/13
	if (_simIsForceSensorBroken(forceSensor))
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

	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	jtr.X*=linScaling; // ********** SCALING
	jtr2.X*=linScaling; // ********** SCALING
	C3X3Matrix jtrm(jtr.Q);

	dBodyID parentRigidBody=((CRigidBodyDyn_ode*)parentBody)->getOdeRigidBody();
	dBodyID childRigidBody=((CRigidBodyDyn_ode*)childBody)->getOdeRigidBody();

	_odeConstraint=dJointCreateFixed(odeWorldID,0);
	dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
	dJointSetFixed(_odeConstraint);

	_odeJointFeedbackStructure=new dJointFeedback;
	dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);

	_setForceSensorBrokenUnbrokenConstraints_ode(_odeConstraint,forceSensor);
}

CConstraintDyn_ode::CConstraintDyn_ode(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB,dWorldID odeWorldID)
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

	batr=parentBody->getInertiaFrameTransformation();
	jtrRelToBodyA=batr.getInverse()*jtr;
	_bodyAID=parentBody->getRigidBodyID();

	bbtr=childBody->getInertiaFrameTransformation();
	jtrRelToBodyB=bbtr.getInverse()*jtr2;
	_bodyBID=childBody->getRigidBodyID();

	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	jtrRelToBodyA.X*=linScaling; // ********** SCALING
	jtrRelToBodyB.X*=linScaling; // ********** SCALING

	jtr.X*=linScaling; // ********** SCALING
	jtr2.X*=linScaling; // ********** SCALING

	_odeConstraint=dJointCreateFixed(odeWorldID,0);

	dBodyID parentRigidBody=((CRigidBodyDyn_ode*)parentBody)->getOdeRigidBody();
	dBodyID childRigidBody=((CRigidBodyDyn_ode*)childBody)->getOdeRigidBody();

	// Set the configuration of the child as if the joint was at 0 position:
	dBodyID cb=childRigidBody;
	C7Vector cb_a(C4Vector(((float*)dBodyGetQuaternion(cb))),C3Vector(((float*)dBodyGetPosition(cb))));
	C7Vector alpha(jtr2.getInverse()*cb_a);
	C7Vector cb_b(jtr*alpha);
	dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
	dQuaternion dQ;
	dQ[0]=cb_b.Q.data[0];
	dQ[1]=cb_b.Q.data[1];
	dQ[2]=cb_b.Q.data[2];
	dQ[3]=cb_b.Q.data[3];
	dBodySetQuaternion(cb,dQ);

	// Attach the joint to the 2 bodies
	dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
	dJointSetFixed(_odeConstraint);

	// Reset the configuration of the child as it is now:
	dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
	dQ[0]=cb_a.Q.data[0];
	dQ[1]=cb_a.Q.data[1];
	dQ[2]=cb_a.Q.data[2];
	dQ[3]=cb_a.Q.data[3];
	dBodySetQuaternion(cb,dQ);

	_odeJointFeedbackStructure=new dJointFeedback;
	dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);

	_setForceSensorBrokenUnbrokenConstraints_ode(_odeConstraint,forceSensor);
}

CConstraintDyn_ode::~CConstraintDyn_ode()
{
	dJointDestroy(_odeConstraint);
	delete _odeJointFeedbackStructure;
}

void CConstraintDyn_ode::_setRevoluteJointLimits(CDummyJoint* joint)
{
    if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
	{ // no limits
		dJointSetHingeParam(_odeConstraint,dParamLoStop,-dInfinity);
		dJointSetHingeParam(_odeConstraint,dParamHiStop,+dInfinity);
	}
	else
	{
		// Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems  (revolute joints only)
		if (_nonCyclicRevoluteJointPositionRange<=359.0f*piValue*2.0f/360.0f)
		{ // when the range is <359, we keep the limits on all the time
			dJointSetHingeParam(_odeConstraint,dParamLoStop,-_nonCyclicRevoluteJointPositionRange*0.5f);
			dJointSetHingeParam(_odeConstraint,dParamHiStop,+_nonCyclicRevoluteJointPositionRange*0.5f);
		}
		else
		{ // since 23/3/2014: some rev. joints need to be limited to a range>360deg (e.g. for motion planning), so we keep it pseudo-cyclic on the dynamic side
			// ODE doesn't support a range > 360. So we manually turn limits on/off as needed:
			// That doesn't work in ODE. We leave it unlimited:
			// float a=getHingeAngle();
			if (false)//a<-_nonCyclicRevoluteJointPositionRange*0.5f+3.14159265f)
			{
				float m=fmod(-_nonCyclicRevoluteJointPositionRange*0.5f,3.14159265f);
				dJointSetHingeParam(_odeConstraint,dParamLoStop,m); // low limit on
			}
			else
				dJointSetHingeParam(_odeConstraint,dParamLoStop,-dInfinity); // low limit off
			if (false)//a>_nonCyclicRevoluteJointPositionRange*0.5f-3.14159265f)
				dJointSetHingeParam(_odeConstraint,dParamHiStop,+_nonCyclicRevoluteJointPositionRange*0.5f); // high limit on
			else
				dJointSetHingeParam(_odeConstraint,dParamHiStop,+dInfinity); // high limit off
		}
	}
}

void CConstraintDyn_ode::_setPrismaticJointLimits(CDummyJoint* joint)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float jiMin,jiRange;
	_simGetJointPositionInterval(joint,&jiMin,&jiRange);

	dJointSetSliderParam(_odeConstraint,dParamLoStop,jiMin*linScaling); // ********** SCALING
	dJointSetSliderParam(_odeConstraint,dParamHiStop,(jiMin+jiRange)*linScaling); // ********** SCALING
}

void CConstraintDyn_ode::_handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

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

	dJointSetHingeParam(_odeConstraint,dParamFMax,forceToApply*torqueScaling); // ********** SCALING
	dJointSetHingeParam(_odeConstraint,dParamVel,velocityToApply);
}


void CConstraintDyn_ode::_handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float torqueScaling=CRigidBodyContainerDyn::getTorqueScalingFactorDyn();

	int param=0;
	simGetObjectIntParameter(_jointID,2030,&param);
	bool lockModeWhenInVelocityControl=(param!=0);

	float vel=_simGetDynamicMotorTargetVelocity(joint);

	if ( (lockModeWhenInVelocityControl)&&(vel==0.0f)&&(_simIsDynamicMotorEnabled(joint)!=0) )
	{
		if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
			_targetPositionToHoldAtZeroVel_velocityMode=dJointGetHingeAngle(_odeConstraint);
//			_targetPositionToHoldAtZeroVel_velocityMode=getHingeAngle()-_nonCyclicRevoluteJointPositionOffset;
		_targetPositionToHoldAtZeroVelOn_velocityMode=true;
		dJointSetHingeParam(_odeConstraint,dParamLoStop,_targetPositionToHoldAtZeroVel_velocityMode);
		dJointSetHingeParam(_odeConstraint,dParamHiStop,_targetPositionToHoldAtZeroVel_velocityMode);
		dJointSetHingeParam(_odeConstraint,dParamFMax,0.0f); // Motor off
	}
	else
	{
		_targetPositionToHoldAtZeroVelOn_velocityMode=false;
		if (_simIsDynamicMotorEnabled(joint))
		{ // Motor on
			dJointSetHingeParam(_odeConstraint,dParamFMax,_simGetDynamicMotorMaxForce(joint)*torqueScaling); // ********** SCALING
			dJointSetHingeParam(_odeConstraint,dParamVel,vel);
		}
		else
			dJointSetHingeParam(_odeConstraint,dParamFMax,0.0f); // Motor off
	}
}

void CConstraintDyn_ode::_handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float linVelocityScaling=CRigidBodyContainerDyn::getLinearVelocityScalingFactorDyn();
	float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

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

	dJointSetSliderParam(_odeConstraint,dParamFMax,forceToApply*forceScaling); // ********** SCALING
	dJointSetSliderParam(_odeConstraint,dParamVel,velocityToApply*linVelocityScaling); // ********** SCALING
}

void CConstraintDyn_ode::_handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float linVelocityScaling=CRigidBodyContainerDyn::getLinearVelocityScalingFactorDyn();
	float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();

	int param=0;
	simGetObjectIntParameter(_jointID,2030,&param);
	bool lockModeWhenInVelocityControl=(param!=0);
	float vel=_simGetDynamicMotorTargetVelocity(joint);

	if ( (lockModeWhenInVelocityControl)&&(vel==0.0f)&&(_simIsDynamicMotorEnabled(joint)!=0) )
	{
		if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
			_targetPositionToHoldAtZeroVel_velocityMode=getSliderPositionScaled()/linScaling;
		_targetPositionToHoldAtZeroVelOn_velocityMode=true;
		dJointSetSliderParam(_odeConstraint,dParamFMax,0.0f); // Motor off
		dJointSetSliderParam(_odeConstraint,dParamLoStop,_targetPositionToHoldAtZeroVel_velocityMode*linScaling); // ********** SCALING
		dJointSetSliderParam(_odeConstraint,dParamHiStop,_targetPositionToHoldAtZeroVel_velocityMode*linScaling); // ********** SCALING
	}
	else
	{
		_targetPositionToHoldAtZeroVelOn_velocityMode=false;
		if (_simIsDynamicMotorEnabled(joint))
		{ // Motor on
			dJointSetSliderParam(_odeConstraint,dParamFMax,_simGetDynamicMotorMaxForce(joint)*forceScaling);// ********** SCALING
			dJointSetSliderParam(_odeConstraint,dParamVel,vel*linVelocityScaling); // ********** SCALING
		}
		else
			dJointSetSliderParam(_odeConstraint,dParamFMax,0.0f); // Motor off
	}
}

dynReal CConstraintDyn_ode::getSliderPositionScaled()
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))
//	dynReal linScaling=(dynReal)CRigidBodyContainerDyn::getPositionScalingFactorDyn();

	return(dJointGetSliderPosition(_odeConstraint)); // Corrected on 18/11/2014: no scaling here (is already scaled!)
}

dynReal CConstraintDyn_ode::getHingeAngle()
{
	dynReal retVal=(dynReal)0.0;
	if (true)
	{ // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
		dynReal jointPos=(dynReal)0.0;

		jointPos=dJointGetHingeAngle(_odeConstraint);
		
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

void CConstraintDyn_ode::reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
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
			C3Vector absF(_odeJointFeedbackStructure->f2[0],_odeJointFeedbackStructure->f2[1],_odeJointFeedbackStructure->f2[2]);
			absF/=forceScaling; // ********** SCALING
			C3Vector absT(_odeJointFeedbackStructure->t2[0],_odeJointFeedbackStructure->t2[1],_odeJointFeedbackStructure->t2[2]);
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
			_setForceSensorBrokenUnbrokenConstraints_ode(_odeConstraint,forceSensor);
	}
}

void CConstraintDyn_ode::reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
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

		if (_simGetJointType(joint)!=sim_joint_spherical_subtype)
		{ // Spherical joints are not supported here!
			C3Vector absF(_odeJointFeedbackStructure->f2[0],_odeJointFeedbackStructure->f2[1],_odeJointFeedbackStructure->f2[2]);
			absF/=forceScaling; // ********** SCALING
			C3Vector absT(_odeJointFeedbackStructure->t2[0],_odeJointFeedbackStructure->t2[1],_odeJointFeedbackStructure->t2[2]);
			absT/=torqueScaling; // ********** SCALING
			C7Vector childBodyAbsConf(_childBody->getInertiaFrameTransformation());

			C7Vector parentShapeAbsConf(_parentBody->getShapeFrameTransformation());
			C7Vector sensorAbsConf(parentShapeAbsConf*_initialLocalTransform);

			C3Vector absCorrectionV(sensorAbsConf.X-childBodyAbsConf.X);
			absT+=absF^absCorrectionV;

			C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
			C3Vector forces(sensorAbsInverseQ*(absF*-1.0f));
			C3Vector torques(sensorAbsInverseQ*(absT*-1.0f));
			if (_simGetJointType(joint)==sim_joint_revolute_subtype)
			{
				forceOrTorque=torques(2);
			}
			if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
			{
				forceOrTorque=forces(2);
			}

		}
		_lastEffortOnJoint=forceOrTorque;
		_simAddJointCumulativeForcesOrTorques(joint,forceOrTorque,totalPassesCount);
	}
}

void CConstraintDyn_ode::_setForceSensorBrokenUnbrokenConstraints_ode(dJointID odeConstr,CDummyForceSensor* forceSensor)
{
	if (_simIsForceSensorBroken(forceSensor))
        dJointAttach(odeConstr,nullptr,nullptr);
}
