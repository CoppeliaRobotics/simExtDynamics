#pragma once

#include "RigidBodyContainerDyn.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedObjectArray.h"

typedef bool (*ContactAddedCallback)(
	btManifoldPoint& cp,
	const btCollisionObject* colObj0,
	int partId0,
	int index0,
	const btCollisionObject* colObj1,
	int partId1,
	int index1);

class CRigidBodyContainerDyn_bullet278 : public CRigidBodyContainerDyn
{
public:
	CRigidBodyContainerDyn_bullet278();
	virtual ~CRigidBodyContainerDyn_bullet278();

	int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
	void applyGravity();
	void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

	btDiscreteDynamicsWorld* getWorld();
	void addBulletContactPoints(int dynamicPassNumber);

protected:
	void _stepDynamics(float dt,int pass);
	void _createDependenciesBetweenJoints();
	void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

	static bool _bulletContactCallback(btManifoldPoint& cp,const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);

	btDiscreteDynamicsWorld* _dynamicsWorld;
	btBroadphaseInterface* _broadphase;
	btCollisionDispatcher* _dispatcher;
	btConstraintSolver* _solver;
	btDefaultCollisionConfiguration* _collisionConfiguration;
	btOverlapFilterCallback* _filterCallback;
	static bool _bulletContactCallback_useCustom;
	static float _bulletContactCallback_combinedFriction;
	static float _bulletContactCallback_combinedRestitution;
};
