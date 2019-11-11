#include "RigidBodyContainerDyn_newton.h"
#include "CollShapeDyn_newton.h"
#include "RigidBodyDyn_newton.h"
#include "ConstraintDyn_newton.h"
#include "simLib.h"

CRigidBodyContainerDyn_newton::CRigidBodyContainerDyn_newton()
{
	_dynamicsCalculationPasses=0;
    _allRigidBodiesIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);
    _allConstraintsIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);

	// Following parameter retrieval is OLD. Use instead following functions:
	// - simGetEngineFloatParameter
	// - simGetEngineInt32Parameter
	// - simGetEngineBoolParameter
	float floatParams[2];
	int intParams[2];
	int ver=0;
    _simGetNewtonParameters(nullptr,&ver,floatParams,intParams);

	const float stepSize=floatParams[0];
	const float contactMergeTolerance=floatParams[1];

	const int newtonIterationsCount=intParams[0];
	const bool multithreaded=(intParams[1]&1)!=false;
	const bool exactSolver=(intParams[1]&2)!=false;
	const bool highJointAccuracy=(intParams[1]&4)!=false;

	// TODO_NEWTON_X2
	// Above settings from CoppeliaSim are not used to configure the engine:
	// contactMergeTolerance, exactSolver, highJointAccuracy

	// install the memory handle
	NewtonSetMemorySystem (NewtonAllocMemory, NewtonFreeMemory);

	// Create the Newton world
	_world = NewtonCreate ();

	// plus the user pointer call back
	NewtonWorldSetUserData (_world, this);

	// set the solver accuracy mode
	NewtonSetSolverModel (_world,newtonIterationsCount);


	if (multithreaded)
		NewtonSetThreadsCount(_world,4);
	else
		NewtonSetThreadsCount(_world,1);

	// set the solver to a high converge rate quality
	NewtonSetSolverConvergenceQuality (_world, 1);

	// set the Material call back for pair default-default of the material graph
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(_world);
    NewtonMaterialSetCollisionCallback (_world, defaultMaterialID, defaultMaterialID, nullptr, NewtonOnAABBOverlap, NewtonOnUserContacts);

	// set joint serialization call back
	CustomJoint::Initalize(_world);

	_rebuildSkeletons = true;

	// Now flag all objects and geoms as "_dynamicsFullRefresh":
	for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
		_simSetDynamicsFullRefreshFlag(_simGetObjectFromIndex(sim_handle_all,i),true);
	for (int i=0;i<_simGetObjectListSize(sim_object_shape_type);i++)
		_simSetGeomProxyDynamicsFullRefreshFlag((void*)_simGetGeomProxyFromShape(_simGetObjectFromIndex(sim_object_shape_type,i)),true);

	_nextRigidBodyID=0;
}

CRigidBodyContainerDyn_newton::~CRigidBodyContainerDyn_newton()
{
	while (_allRigidBodiesList.size()!=0)
		_removeRigidBody(_allRigidBodiesList[0]->getRigidBodyID());
	
	for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
	{
		CDummy3DObject* it=(CDummy3DObject*)_simGetObjectFromIndex(sim_handle_all,i);
		_simSetDynamicSimulationIconCode(it,sim_dynamicsimicon_none);
	}

	NewtonWaitForUpdateToFinish (_world);
	NewtonDestroyAllBodies (_world);
	NewtonDestroy(_world);

	// Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
	particleCont.removeAllObjects();
}

int CRigidBodyContainerDyn_newton::getEngineInfo(int& engine,int data1[4],char* data2,char* data3)
{
	engine=sim_physics_newton;
	data1[0]=NewtonWorldGetVersion();
	//strcpy(data2,"3.13");
	int major = data1[0]/100;
	sprintf (data2, "$d.%d", major, data1[0] - major * 100);
	return(DYNAMICS_PLUGIN_VERSION);
}

void CRigidBodyContainerDyn_newton::_addNewtonContactPoints(int dynamicPassNumber)
{
	dTree <NewtonJoint*, NewtonJoint*> filter;
	for (NewtonBody* body = NewtonWorldGetFirstBody(_world); body; body = NewtonWorldGetNextBody(_world, body))
	{
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint))
		{
			if (!filter.Find(joint))
			{
				filter.Insert (joint, joint);
				if (NewtonJointIsActive(joint))
				{
					NewtonBody* const newtonBodyA = NewtonJointGetBody0(joint);
					NewtonBody* const newtonBodyB = NewtonJointGetBody1(joint);
					_ASSERTE ((body == newtonBodyA) || (body == newtonBodyB));
					void** userDataA=(void**)NewtonBodyGetUserData(newtonBodyA);
					void** userDataB=(void**)NewtonBodyGetUserData(newtonBodyB);
					CRigidBodyDyn* const bodyA = (CRigidBodyDyn*)userDataA[1];
					CRigidBodyDyn* const bodyB = (CRigidBodyDyn*)userDataB[1];
					for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact))
					{
						dVector point;
						dVector normal;
						dVector dir0;
						dVector dir1;
						dVector force;
						NewtonMaterial* const material = NewtonContactGetMaterial(contact);
						NewtonMaterialGetContactPositionAndNormal(material, newtonBodyA, &point.m_x, &normal.m_x);
						NewtonMaterialGetContactForce (material, newtonBodyA, &force.m_x);
						NewtonMaterialGetContactTangentDirections(material, body, &dir0.m_x, &dir1.m_x);

						// We want the full contact force (including the friction-related force).
						// But keep following commented, in future we will also offer the normal force:
						// force = normal.Scale (normal % force);

						_contactPoints.push_back(point.m_x);
						_contactPoints.push_back(point.m_y);
						_contactPoints.push_back(point.m_z);

						SContactInfo ci;
						ci.subPassNumber = dynamicPassNumber;
						ci.objectID1 = ((int*)userDataA[0])[0];
						ci.objectID2 = ((int*)userDataB[0])[0];
						ci.position = C3Vector (point.m_x, point.m_y, point.m_z);
						C3Vector n(normal.m_x, normal.m_y, normal.m_z);
						n.normalize();
						C3Vector f(force.m_x, force.m_y, force.m_z);
						if (n*f<0.0f)
							n=n*-1.0f;
						ci.surfaceNormal =n;
						ci.directionAndAmplitude = f;
						_contactInfo.push_back(ci);
					}
				}
			}
		}
	}
}

int CRigidBodyContainerDyn_newton::NewtonOnAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{ // As far as I understood this, this is the broad-phase collision callback.
	// If it returns 0, no further collision is checked between those 2 bodies,
	// and they will not collide with each other.
	// So here, we check if those two bodies can collide

	// If we have a contact callback registered, we always return 1:
	if (_simGetContactCallbackCount()>0)
		return(1); // the narrow-phase routine will handle the callback!

	void** userDataA=(void**)NewtonBodyGetUserData(body0);
	void** userDataB=(void**)NewtonBodyGetUserData(body1);
	CDummyShape* shapeA=(CDummyShape*)_simGetObject(((int*)userDataA[0])[0]);
	CDummyShape* shapeB=(CDummyShape*)_simGetObject(((int*)userDataB[0])[0]);
	bool canCollide=false;
    if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
	{ // regular case (shape-shape)
		unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
		unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
		canCollide=(_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB))&&((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_respondable)!=0)&&((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_respondable)!=0);
		if (_simGetLastParentForLocalGlobalCollidable(shapeA)==_simGetLastParentForLocalGlobalCollidable(shapeB))
			canCollide=canCollide&&(collFA&collFB&0x00ff); // we are local
		else
			canCollide=canCollide&&(collFA&collFB&0xff00); // we are global
		if ( (_simIsShapeDynamicallyStatic(shapeA)||((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_dynamic)==0))&&
			(_simIsShapeDynamicallyStatic(shapeB)||((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_dynamic)==0)) )
			canCollide=false;
	}
	else
	{ // particle-shape or particle-particle case:
		int dataA=((int*)userDataA[0])[0];
		int dataB=((int*)userDataB[0])[0];
        if ( (shapeA==nullptr)&&(shapeB==nullptr) )
		{ // particle-particle case:
			CParticleObject* pa=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
			CParticleObject* pb=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);

            if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
				canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
		}
		else
		{ // particle-shape case:
            CDummyShape* shape=nullptr;
            CParticleObject* particle=nullptr;
            if (shapeA!=nullptr)
				shape=shapeA;
			else
				particle=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
            if (shapeB!=nullptr)
				shape=shapeB;
			else
				particle=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);

            if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
				canCollide=_simIsShapeDynamicallyRespondable(shape)&&(_simGetDynamicCollisionMask(shape)&particle->getShapeRespondableMask()&0xff00)&&((_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable)!=0); // we are global
		}
	}

	if (canCollide)
		return 1;
	return 0;
}

void CRigidBodyContainerDyn_newton::_rebuildSkeletonList()
{
	if (_rebuildSkeletons) {
		for (NewtonBody* body = NewtonWorldGetFirstBody(_world); body; body = NewtonWorldGetNextBody (_world, body)) {
			NewtonSkeletonContainer* const skeleton = NewtonBodyGetSkeleton(body);
			if (skeleton) {
				NewtonSkeletonContainerDelete (skeleton);
			}
		}

		dTree<CRigidBodyDyn*, CRigidBodyDyn*> filter;
		for (int i = 0; i < int(_allRigidBodiesList.size()); i++) {

			CRigidBodyDyn* root = _allRigidBodiesList[i];
			if (!filter.Find(root)) {
				bool hasAcyclicJoints = false;
				for (bool foundParent = true; foundParent; ) {
					foundParent = false;
					NewtonBody* const body = ((CRigidBodyDyn_newton*)root)->getNewtonRigidBody();
					dAssert(body);
					for (NewtonJoint* jointptr = NewtonBodyGetFirstJoint(body); jointptr; jointptr = NewtonBodyGetNextJoint(body, jointptr)) {
						CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (jointptr);
						CConstraintDyn* const constraint = (CConstraintDyn*)joint->GetUserData();
						if (((CConstraintDyn_newton*)constraint)->_isAcyclicJoint()) {
							hasAcyclicJoints = true;
							if (((CConstraintDyn_newton*)constraint)->_getChild() == root) {
								root = ((CConstraintDyn_newton*)constraint)->_getParent();
								foundParent = true;
								break;
							}
						}
					}
				}

				if (hasAcyclicJoints) {
					int stack = 1;
					CRigidBodyDyn* pool[64];
					pool[0] = root;

                    NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate (_world, ((CRigidBodyDyn_newton*)root)->getNewtonRigidBody(), nullptr);
					NewtonSkeletonSetSolverMode (skeleton, 1);
					while (stack) {
						stack --;
						CRigidBodyDyn* const root = pool[stack];
						filter.Insert(root);
						NewtonBody* const parentBone = ((CRigidBodyDyn_newton*)root)->getNewtonRigidBody();
						for (NewtonJoint* jointptr = NewtonBodyGetFirstJoint(parentBone); jointptr; jointptr = NewtonBodyGetNextJoint(parentBone, jointptr)) {
							CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (jointptr);
							CConstraintDyn* const constraint = (CConstraintDyn*)joint->GetUserData();
							if (((CConstraintDyn_newton*)constraint)->_isAcyclicJoint()) {
								if (((CConstraintDyn_newton*)constraint)->_getParent() == root) {
									CRigidBodyDyn* const child = ((CConstraintDyn_newton*)constraint)->_getChild();
									pool[stack] = child;
									stack ++;
									NewtonSkeletonContainerAttachBone (skeleton,((CRigidBodyDyn_newton*)child)->getNewtonRigidBody(), parentBone);
								}
							}
						}
					}
					NewtonSkeletonContainerFinalize (skeleton);
				}
			}
		}
	}
	_rebuildSkeletons = false;
}

void CRigidBodyContainerDyn_newton::_notifySekeletonRebuild ()
{
	_rebuildSkeletons = true;
}
/*
struct MaterialsPareamaters
{
	dFloat m_restitution;
	dFloat m_staticFriction;
	dFloat m_kineticFriction;
};
*/
/*
static MaterialsPareamaters materialGraph [4][4]=
{
                   // steel				  concrete				wood				asphalt
	// steel	   { {0.5f, 0.7f, 0.6f},  {0.5f, 0.7f, 0.6f},	{0.5f, 0.7f, 0.6f}, {0.5f, 0.7f, 0.6f} },
	// concrete  { {0.5f, 0.7f, 0.6f},  {0.5f, 0.7f, 0.6f},	{0.5f, 0.7f, 0.6f}, {0.5f, 0.7f, 0.6f} },
	// wood	   { {0.5f, 0.7f, 0.6f},  {0.5f, 0.7f, 0.6f},	{0.5f, 0.7f, 0.6f}, {0.5f, 0.7f, 0.6f} },
	// asphalt   { {0.5f, 0.7f, 0.6f},  {0.5f, 0.7f, 0.6f},	{0.5f, 0.7f, 0.6f}, {0.5f, 0.7f, 0.6f} },
};
*/
void CRigidBodyContainerDyn_newton::NewtonOnUserContacts(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{ // This is the narrow-phase collision checking and material setting if I understood it correctly
	// Here we actually (normally) want our objects to collide, unless a callback tells us otherwise:

	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

	void** userDataA=(void**)NewtonBodyGetUserData(body0);
	void** userDataB=(void**)NewtonBodyGetUserData(body1);

	CDummyShape* shapeA=(CDummyShape*)_simGetObject(((int*)userDataA[0])[0]);
	CDummyShape* shapeB=(CDummyShape*)_simGetObject(((int*)userDataB[0])[0]);
	bool collides=true; // was already checked previously, unless we have a user callback
	int id_A=((int*)userDataA[0])[0];
	int id_B=((int*)userDataB[0])[0];
	float statFriction_A=0.0f;
	float statFriction_B=0.0f;
	float kinFriction_A=0.0f;
	float kinFriction_B=0.0f;
	float restit_A=0.0f;
	float restit_B=0.0f;
    if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
	{ // regular case (shape-shape)
		statFriction_A=((float*)userDataA[2])[0];
		statFriction_B=((float*)userDataB[2])[0];
		kinFriction_A=((float*)userDataA[3])[0];
		kinFriction_B=((float*)userDataB[3])[0];
		restit_A=((float*)userDataA[4])[0];
		restit_B=((float*)userDataB[4])[0];
	}
	else
	{ // particle-shape or particle-particle case:
        if ( (shapeA==nullptr)&&(shapeB==nullptr) )
		{ // particle-particle case:
			CParticleObject* pa=particleCont.getObject(id_A-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
			CParticleObject* pb=particleCont.getObject(id_B-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);

            if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
			{
				// Get the particle's user data:
				statFriction_A=((float*)userDataA[2])[0];
				statFriction_B=((float*)userDataB[2])[0];
				kinFriction_A=((float*)userDataA[3])[0];
				kinFriction_B=((float*)userDataB[3])[0];
				restit_A=((float*)userDataA[4])[0];
				restit_B=((float*)userDataB[4])[0];
			}
			else
				collides=false; // not normal
		}
		else
		{ // particle-shape case:
            if (shapeA!=nullptr)
			{
				statFriction_A=((float*)userDataA[2])[0];
				kinFriction_A=((float*)userDataA[3])[0];
				restit_A=((float*)userDataA[4])[0];
			}
			else
			{
				CParticleObject* particle=particleCont.getObject(id_A-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
				{
					// Get the particle's user data:
					statFriction_B=((float*)userDataB[2])[0];
					kinFriction_B=((float*)userDataB[3])[0];
					restit_B=((float*)userDataB[4])[0];
				}
				else
					collides=false; // not normal
			}
            if (shapeB!=nullptr)
			{
				statFriction_B=((float*)userDataB[2])[0];
				kinFriction_B=((float*)userDataB[3])[0];
				restit_B=((float*)userDataB[4])[0];
			}
			else
			{
				CParticleObject* particle=particleCont.getObject(id_B-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
				{
					// Get the particle's user data:
					statFriction_A=((float*)userDataA[2])[0];
					kinFriction_A=((float*)userDataA[3])[0];
					restit_A=((float*)userDataA[4])[0];
				}
				else
					collides=false; // not normal
			}
		}
	}

	float statFriction=statFriction_A*statFriction_B;
	float kinFriction=kinFriction_A*kinFriction_B;
	float restit=(restit_A+restit_B)/2.0f;

	int dataInt[3]={0,0,0};
	float dataFloat[14]={statFriction,kinFriction,restit,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

	if (collides)
	{
		int flag=0;
#ifndef DG_USE_THREAD_EMULATION
		flag|=1024; // means: the callback scripts won't be called (e.g. when this thread is not the simulation thread)
#endif
		int customHandleRes=_simHandleCustomContact(id_A,id_B,sim_physics_newton+flag,dataInt,dataFloat);
		collides=(customHandleRes!=0);
		if (customHandleRes>0)
		{
			statFriction=dataFloat[0];
			kinFriction=dataFloat[1];
			restit=dataFloat[2];
		}
	}

	// TODO_NEWTON:
	// if collides is false, then we want to ignore this collision

	for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact))
	{
		NewtonMaterial* const material = NewtonContactGetMaterial(contact);
	/*
		NewtonCollision* const collision0 = NewtonMaterialGetBodyCollidingShape(material, body0);
		NewtonCollision* const collision1 = NewtonMaterialGetBodyCollidingShape(material, body1);

		CDummyGeomWrap* const info0 = (CDummyGeomWrap*)NewtonCollisionGetUserData(collision0);
		CDummyGeomWrap* const info1 = (CDummyGeomWrap*)NewtonCollisionGetUserData(collision1);

		//Julio not sure how CoppeliaSim use the material Newton use a contact graph by for now I will simple select the highest value the two
		//
		//float restitution = max (_simGetNewtonRestitution(info0), _simGetNewtonRestitution(info1));
		//float staticFriction = max (_simGetFriction(info0), _simGetFriction(info1));
		int index0 =  0;   //_simGetNewtonMaterialIndex(info0);
		int index1 =  0;   //_simGetNewtonMaterialIndex(info1);
		const MaterialsPareamaters& materialEdge = materialGraph[index0][index1];

		NewtonMaterialSetContactElasticity(material, materialEdge.m_restitution);
		NewtonMaterialSetContactFrictionCoef(material, materialEdge.m_staticFriction, materialEdge.m_kineticFriction, 0);
		NewtonMaterialSetContactFrictionCoef(material, materialEdge.m_staticFriction, materialEdge.m_kineticFriction, 1);
	*/
		NewtonMaterialSetContactElasticity(material, restit);
		NewtonMaterialSetContactFrictionCoef(material, statFriction, kinFriction, 0);
		NewtonMaterialSetContactFrictionCoef(material, statFriction, kinFriction, 1);

//		NewtonMaterialSetContactSoftness(material,0.7f); // 0.01-0.7
//		NewtonMaterialSetSurfaceThickness(NewtonBodyGetWorld(body0),NewtonBodyGetMaterialGroupID(body0),NewtonBodyGetMaterialGroupID(body1),0.001f);
	}
}

void CRigidBodyContainerDyn_newton::applyGravity()
{ // gravity is scaled here!!

	C3Vector gravity;
	_simGetGravity(gravity.data);

	// in newton the gravity and all external forces and torque are applied in the force and torque callback 

	gravityVectorLength=(gravity).getLength();
}

void CRigidBodyContainerDyn_newton::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
	// TODO_NEWTON
}

void CRigidBodyContainerDyn_newton::_createDependenciesBetweenJoints()
{
	// TODO_NEWTON
}

void CRigidBodyContainerDyn_newton::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
	// TODO_NEWTON
}

void* CRigidBodyContainerDyn_newton::NewtonAllocMemory(int sizeInBytes)
{
	//return malloc (sizeInBytes);
	return simCreateBuffer (sizeInBytes);
}

void CRigidBodyContainerDyn_newton::NewtonFreeMemory(void* const ptr, int sizeInBytes)
{
	//free (ptr);
	ptrSimReleaseBuffer((simChar*) ptr);
}

NewtonWorld* CRigidBodyContainerDyn_newton::getWorld()
{
	return(_world);
}

void CRigidBodyContainerDyn_newton::_stepDynamics(float dt,int pass)
{
	NewtonUpdate(_world,dt);
	_addNewtonContactPoints(pass);
}
