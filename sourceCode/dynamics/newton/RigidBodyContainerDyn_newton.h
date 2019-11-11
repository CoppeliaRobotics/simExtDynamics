#pragma once

#include "RigidBodyContainerDyn.h"
#include "Newton.h"
#include "dMatrix.h"
#include "CustomJoint.h"
#include "CustomHinge.h"
#include "CustomSlider.h"
#include "CustomBallAndSocket.h"
#include "CustomHingeActuator.h"
#include "CustomSliderActuator.h"

class CRigidBodyContainerDyn_newton : public CRigidBodyContainerDyn
{
public:

	CRigidBodyContainerDyn_newton();
	virtual ~CRigidBodyContainerDyn_newton();

	int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
	void applyGravity();
	void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

	NewtonWorld* getWorld();

	void _notifySekeletonRebuild ();
	void _rebuildSkeletonList();
	static void* NewtonAllocMemory (int sizeInBytes);
	static void NewtonFreeMemory (void* const ptr, int sizeInBytes);

	bool _rebuildSkeletons;
protected:
	void _stepDynamics(float dt,int pass);
	void _createDependenciesBetweenJoints();
	void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

	void _addNewtonContactPoints(int dynamicPassNumber);
	typedef void (*NewtonSkeletontDestructor) (const NewtonSkeletonContainer* const me);
	static void NewtonOnUserContacts (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
	static int NewtonOnAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);

	NewtonWorld* _world;
};
