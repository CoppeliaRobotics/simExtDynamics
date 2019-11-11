#include "RigidBodyContainerDyn_bullet283.h"
#include "CollShapeDyn_bullet283.h"
#include "RigidBodyDyn_bullet283.h"
#include "ConstraintDyn_bullet283.h"
#include "simLib.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
//#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

bool CRigidBodyContainerDyn_bullet283::_bulletContactCallback_useCustom;
float CRigidBodyContainerDyn_bullet283::_bulletContactCallback_combinedFriction;
float CRigidBodyContainerDyn_bullet283::_bulletContactCallback_combinedRestitution;

CRigidBodyContainerDyn_bullet283::CRigidBodyContainerDyn_bullet283()
{
	_dynamicsCalculationPasses=0;
    _allRigidBodiesIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);
    _allConstraintsIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);

	_collisionConfiguration=new btDefaultCollisionConfiguration();
	_dispatcher=new	btCollisionDispatcher(_collisionConfiguration);
	_broadphase=new btDbvtBroadphase();
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    int solverType=simGetEngineInt32Parameter(sim_bullet_global_constraintsolvertype,-1,nullptr,nullptr);

	if (solverType==sim_bullet_constraintsolvertype_sequentialimpulse)
		_solver=new btSequentialImpulseConstraintSolver();
	if (solverType==sim_bullet_constraintsolvertype_nncg)
		_solver=new btNNCGConstraintSolver(); // in that case, use iteration count of 500 or more
	if (solverType==sim_bullet_constraintsolvertype_dantzig)
		_solver=new btMLCPSolver(new btDantzigSolver());
	if (solverType==sim_bullet_constraintsolvertype_projectedgaussseidel)
		_solver = new btMLCPSolver(new btSolveProjectedGaussSeidel());
//	_solver=new btMLCPSolver(new btLemkeSolver());

	_dynamicsWorld=new btDiscreteDynamicsWorld(_dispatcher,_broadphase,_solver,_collisionConfiguration);
    _dynamicsWorld->getSolverInfo().m_numIterations=simGetEngineInt32Parameter(sim_bullet_global_constraintsolvingiterations,-1,nullptr,nullptr);
	_dynamicsWorld->getSolverInfo().m_solverMode=SOLVER_SIMD+SOLVER_USE_WARMSTARTING+SOLVER_RANDMIZE_ORDER+SOLVER_ENABLE_FRICTION_DIRECTION_CACHING+SOLVER_USE_2_FRICTION_DIRECTIONS;
	// From Bullet 2.83 on, we use SOLVER_USE_2_FRICTION_DIRECTIONS. So no more "sticky contact" option as previously

	//register algorithm
	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(_dynamicsWorld->getDispatcher());
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
	struct _myCollisionCallback : public btOverlapFilterCallback
	{
		virtual ~_myCollisionCallback()
		{}
		// return true when pairs need collision
		virtual bool needBroadphaseCollision(btBroadphaseProxy* childProxy0,btBroadphaseProxy* childProxy1) const
		{
			btBroadphaseProxy* multiProxy0 =childProxy0;
			btBroadphaseProxy* multiProxy1 =childProxy1;

			bool collides = (multiProxy0->m_collisionFilterGroup & multiProxy1->m_collisionFilterMask) != 0;
			collides = collides && (multiProxy1->m_collisionFilterGroup & multiProxy0->m_collisionFilterMask);

			if (collides)
			{
				int objID1;
				int objID2;
				btRigidBody* a=(btRigidBody*)multiProxy0->m_clientObject;
				btRigidBody* b=(btRigidBody*)multiProxy1->m_clientObject;
				int dataA=(unsigned long long)a->getUserPointer();
				int dataB=(unsigned long long)b->getUserPointer();
				CDummyShape* shapeA=(CDummyShape*)_simGetObject(dataA);
				CDummyShape* shapeB=(CDummyShape*)_simGetObject(dataB);
				bool canCollide=false;
                if ( (shapeA==nullptr)||(shapeB==nullptr) )
				{ // particle-shape or particle-particle case:
                    if ( (shapeA==nullptr)&&(shapeB==nullptr) )
					{ // particle-particle case:
						CParticleObject* pa=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
						CParticleObject* pb=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
						objID1=dataA;
						objID2=dataB;
                        if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
							canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
						else
							return(false);
					}
					else
					{ // particle-shape case:
						unsigned int collFA=0;
                        if (shapeA!=nullptr)
						{
							collFA=_simGetDynamicCollisionMask(shapeA);
							canCollide=_simIsShapeDynamicallyRespondable(shapeA)!=0;
							objID1=_simGetObjectID(shapeA);
						}
						else
						{
							CParticleObject* po=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                            if (po!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
								collFA=po->getShapeRespondableMask();
							else
								return(false);
							objID1=dataA;
						}
						unsigned int collFB=0;
                        if (shapeB!=nullptr)
						{
							collFB=_simGetDynamicCollisionMask(shapeB);
							canCollide=_simIsShapeDynamicallyRespondable(shapeB)!=0;
							objID2=_simGetObjectID(shapeB);
						}
						else
						{
							CParticleObject* po=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                            if (po!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
								collFB=po->getShapeRespondableMask();
							else
								return(false);
							objID2=dataB;
						}
						canCollide=(canCollide&&(collFA&collFB&0xff00)); // we are global
					}
				}
				else
				{ // regular case (shape-shape)
					unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
					unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
					canCollide=(_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB));
					CDummy3DObject* lastPA=(CDummy3DObject*)_simGetLastParentForLocalGlobalCollidable(shapeA);
					CDummy3DObject* lastPB=(CDummy3DObject*)_simGetLastParentForLocalGlobalCollidable(shapeB);
					if (lastPA==lastPB)
						canCollide=(canCollide&&(collFA&collFB&0x00ff)); // we are local
					else
						canCollide=(canCollide&&(collFA&collFB&0xff00)); // we are global
					objID1=_simGetObjectID(shapeA);
					objID2=_simGetObjectID(shapeB);
				}
				if (canCollide)
				{
					int dataInt[3]={0,0,0};
					float dataFloat[14]={1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

					int customHandleRes=_simHandleCustomContact(objID1,objID2,sim_physics_bullet,dataInt,dataFloat);
					bool canReallyCollide=(customHandleRes!=0);
					if (customHandleRes>0)
					{
						_bulletContactCallback_useCustom=true;
						_bulletContactCallback_combinedFriction=dataFloat[0];
						_bulletContactCallback_combinedRestitution=dataFloat[1];
					}
					return(canReallyCollide);
				}
			}
			return(false);
		}
	};
	_filterCallback = new _myCollisionCallback();
	_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setOverlapFilterCallback(_filterCallback);
	gContactAddedCallback=_bulletContactCallback; // For Bullet's custom contact callback

	// Now flag all objects and geoms as "_dynamicsFullRefresh":
	for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
		_simSetDynamicsFullRefreshFlag(_simGetObjectFromIndex(sim_handle_all,i),true);
	for (int i=0;i<_simGetObjectListSize(sim_object_shape_type);i++)
		_simSetGeomProxyDynamicsFullRefreshFlag((void*)_simGetGeomProxyFromShape(_simGetObjectFromIndex(sim_object_shape_type,i)),true);

	_nextRigidBodyID=0;
}

CRigidBodyContainerDyn_bullet283::~CRigidBodyContainerDyn_bullet283()
{
	while (_allRigidBodiesList.size()!=0)
		_removeRigidBody(_allRigidBodiesList[0]->getRigidBodyID());
	
	for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
	{
		CDummy3DObject* it=(CDummy3DObject*)_simGetObjectFromIndex(sim_handle_all,i);
		_simSetDynamicSimulationIconCode(it,sim_dynamicsimicon_none);
	}

	particleCont.removeAllParticles();
	delete _dynamicsWorld;
	delete _solver;
	delete _broadphase;
	delete _dispatcher;
	delete _collisionConfiguration;
	delete _filterCallback;

	// Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
	particleCont.removeAllObjects();
}

int CRigidBodyContainerDyn_bullet283::getEngineInfo(int& engine,int data1[4],char* data2,char* data3)
{
	engine=sim_physics_bullet;
	data1[0]=283;
	strcpy(data2,"2.83.7");
	return(DYNAMICS_PLUGIN_VERSION);
}

bool CRigidBodyContainerDyn_bullet283::_bulletContactCallback(btManifoldPoint& cp,const btCollisionObjectWrapper* colObjWrap0,int partId0,int index0,const btCollisionObjectWrapper* colObjWrap1,int partId1,int index1)
{
	if (_bulletContactCallback_useCustom)
	{ // We want a custom handling of this contact!
		cp.m_combinedFriction=_bulletContactCallback_combinedFriction;
		cp.m_combinedRestitution=_bulletContactCallback_combinedRestitution;
		return(true);
	}
	return(false);
}

void CRigidBodyContainerDyn_bullet283::applyGravity()
{ // gravity is scaled here!!

	C3Vector gravity;
	_simGetGravity(gravity.data);

	float s=CRigidBodyContainerDyn::getGravityScalingFactorDyn(); // ********** SCALING

    if (_dynamicsWorld!=nullptr) // probably not needed anymore!
		_dynamicsWorld->setGravity(btVector3(gravity(0)*s,gravity(1)*s,gravity(2)*s));

	gravityVectorLength=(gravity*s).getLength();
}

btDiscreteDynamicsWorld* CRigidBodyContainerDyn_bullet283::getWorld()
{
	return(_dynamicsWorld);
}

void CRigidBodyContainerDyn_bullet283::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
    if (_dynamicsWorld==nullptr) // probably not needed
		return;
	btDefaultSerializer* serializer=new btDefaultSerializer(maxSerializeBufferSize);
	_dynamicsWorld->serialize(serializer);
	const unsigned char* buff=serializer->getBufferPointer();
	FILE* f;
	f=fopen(filenameAndPath.c_str(),"wt");
    if (f!=nullptr)
	{
		fwrite(buff,1,serializer->getCurrentBufferSize(),f);
		fclose(f);
	}
}

void CRigidBodyContainerDyn_bullet283::_createDependenciesBetweenJoints()
{
}

void CRigidBodyContainerDyn_bullet283::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
}

void CRigidBodyContainerDyn_bullet283::addBulletContactPoints(int dynamicPassNumber)
{
	float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
	float forceScaling=CRigidBodyContainerDyn::getForceScalingFactorDyn();
	float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

	int numManifolds=_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold=_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		btRigidBody* obA=(btRigidBody*)(contactManifold->getBody0());
		btRigidBody* obB=(btRigidBody*)(contactManifold->getBody1());

		int numContacts=contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt=contactManifold->getContactPoint(j);
			btVector3 ptA=pt.getPositionWorldOnA();
			btVector3 ptB=pt.getPositionWorldOnB();

			SContactInfo ci;
			ci.subPassNumber=i;
			ci.objectID1=(unsigned long long)obA->getUserPointer();
			ci.objectID2=(unsigned long long)obB->getUserPointer();
			C3Vector pt1(ptA.x(),ptA.y(),ptA.z());
			C3Vector pt2(ptB.x(),ptB.y(),ptB.z());
			C3Vector avrg((pt1+pt2)*0.5f);
			ci.position=avrg/linScaling; // ********** SCALING

			float force=pt.m_appliedImpulse/(forceScaling*dynStepSize); // ********** SCALING
			C3Vector d(pt.m_normalWorldOnB.x(),pt.m_normalWorldOnB.y(),pt.m_normalWorldOnB.z());
			ci.directionAndAmplitude=d*force;
			ci.surfaceNormal=d;
			ci.subPassNumber=dynamicPassNumber;
			_contactInfo.push_back(ci);

			_contactPoints.push_back(avrg(0)/linScaling);
			_contactPoints.push_back(avrg(1)/linScaling);
			_contactPoints.push_back(avrg(2)/linScaling);
		}
	}
}

void CRigidBodyContainerDyn_bullet283::_stepDynamics(float dt,int pass)
{
	_dynamicsWorld->stepSimulation(dt,10000,dt);
	addBulletContactPoints(pass);
}
