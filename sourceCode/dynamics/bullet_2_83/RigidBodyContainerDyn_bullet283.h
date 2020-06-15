#pragma once

#include "RigidBodyContainerDyn.h"
#include "btBulletDynamicsCommon.h"

class CRigidBodyContainerDyn_bullet283 : public CRigidBodyContainerDyn
{
public:
    CRigidBodyContainerDyn_bullet283();
    virtual ~CRigidBodyContainerDyn_bullet283();

    int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
    void applyGravity();
    void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

    btDiscreteDynamicsWorld* getWorld();
    void addBulletContactPoints(int dynamicPassNumber);

protected:
    void _stepDynamics(float dt,int pass);
    void _createDependenciesBetweenJoints();
    void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

    static bool _bulletContactCallback(btManifoldPoint& cp,const btCollisionObjectWrapper* colObjWrap0,int partId0,int index0,const btCollisionObjectWrapper* colObjWrap1,int partId1,int index1);

    btBroadphaseInterface* _broadphase;
    btCollisionDispatcher* _dispatcher;

    btDiscreteDynamicsWorld* _dynamicsWorld;
    btConstraintSolver* _solver;

    btDefaultCollisionConfiguration* _collisionConfiguration;
    btOverlapFilterCallback* _filterCallback;
    static bool _bulletContactCallback_useCustom;
    static float _bulletContactCallback_combinedFriction;
    static float _bulletContactCallback_combinedRestitution;
};
