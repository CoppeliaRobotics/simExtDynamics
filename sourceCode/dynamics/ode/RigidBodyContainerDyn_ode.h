#pragma once

#include "RigidBodyContainerDyn.h"
#include "ode/ode.h"

struct SOdeContactData
{
	dJointID jointID;
	int objectID1;
	int objectID2;
	C3Vector positionScaled;
	C3Vector normalVector;
};

class CRigidBodyContainerDyn_ode : public CRigidBodyContainerDyn
{
public:

	CRigidBodyContainerDyn_ode();
	virtual ~CRigidBodyContainerDyn_ode();

	int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
	void applyGravity();
	void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

	dWorldID getWorld();
	dSpaceID getOdeSpace();

protected:
	void _stepDynamics(float dt,int pass);
	void _createDependenciesBetweenJoints();
	void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

	static void _odeCollisionCallbackStatic(void* data,dGeomID o1,dGeomID o2);
	void _odeCollisionCallback(void* data,dGeomID o1,dGeomID o2);
	dWorldID _odeWorld;
	dSpaceID _odeSpace;
	dJointGroupID _odeContactGroup;
	std::vector<SOdeContactData> _odeContactsRegisteredForFeedback;
};
