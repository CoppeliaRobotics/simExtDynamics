#include "RigidBodyContainerDyn.h"
#include "simLib.h"

#ifdef INCLUDE_BULLET_2_78_CODE
#include "RigidBodyContainerDyn_bullet278.h"
#include "CollShapeDyn_bullet278.h"
#include "RigidBodyDyn_bullet278.h"
#include "ConstraintDyn_bullet278.h"
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
#include "RigidBodyContainerDyn_bullet283.h"
#include "CollShapeDyn_bullet283.h"
#include "RigidBodyDyn_bullet283.h"
#include "ConstraintDyn_bullet283.h"
#endif

#ifdef INCLUDE_ODE_CODE
#include "RigidBodyContainerDyn_ode.h"
#include "CollShapeDyn_ode.h"
#include "RigidBodyDyn_ode.h"
#include "ConstraintDyn_ode.h"
#endif

#ifdef INCLUDE_NEWTON_CODE
#include "RigidBodyContainerDyn_newton.h"
#include "CollShapeDyn_newton.h"
#include "RigidBodyDyn_newton.h"
#include "ConstraintDyn_newton.h"
#endif

#ifdef INCLUDE_VORTEX_CODE
#include "RigidBodyContainerDyn_vortex.h"
#include "CollShapeDyn_vortex.h"
#include "RigidBodyDyn_vortex.h"
#include "ConstraintDyn_vortex.h"
#endif

CRigidBodyContainerDyn* CRigidBodyContainerDyn::currentRigidBodyContainerDynObject=nullptr; // for ODE's callback (and maybe Vortex too)
CParticleContainer CRigidBodyContainerDyn::particleCont;

float CRigidBodyContainerDyn::_positionScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_linearVelocityScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_massScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_masslessInertiaScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_forceScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_torqueScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_gravityScalingFactorDyn=0.0;
float CRigidBodyContainerDyn::_dynamicActivityRange=0.0;
float CRigidBodyContainerDyn::_dynamicsInternalStepSize=0.0;
int CRigidBodyContainerDyn::_dynamicParticlesIdStart=0;
int CRigidBodyContainerDyn::_3dObjectIdStart=0;
int CRigidBodyContainerDyn::_3dObjectIdEnd=0;

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
}

int CRigidBodyContainerDyn::getEngineInfo(int& engine,int data1[4],char* data2,char* data3)
{
	engine=-1;
	return(-1);
}

void CRigidBodyContainerDyn::applyGravity()
{
}

void CRigidBodyContainerDyn::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
}

void CRigidBodyContainerDyn::_stepDynamics(float dt,int pass)
{
}

void CRigidBodyContainerDyn::_createDependenciesBetweenJoints()
{
}

void CRigidBodyContainerDyn::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
}

void CRigidBodyContainerDyn::setPositionScalingFactorDyn(float f)
{
	_positionScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getPositionScalingFactorDyn()
{
	return(_positionScalingFactorDyn);
}

void CRigidBodyContainerDyn::setLinearVelocityScalingFactorDyn(float f)
{
	_linearVelocityScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getLinearVelocityScalingFactorDyn()
{
	return(_linearVelocityScalingFactorDyn);
}

void CRigidBodyContainerDyn::setMassScalingFactorDyn(float f)
{
	_massScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getMassScalingFactorDyn()
{
	return(_massScalingFactorDyn);
}

void CRigidBodyContainerDyn::setMasslessInertiaScalingFactorDyn(float f)
{
	_masslessInertiaScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getMasslessInertiaScalingFactorDyn()
{
	return(_masslessInertiaScalingFactorDyn);
}

void CRigidBodyContainerDyn::setForceScalingFactorDyn(float f)
{
	_forceScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getForceScalingFactorDyn()
{
	return(_forceScalingFactorDyn);
}

void CRigidBodyContainerDyn::setTorqueScalingFactorDyn(float f)
{
	_torqueScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getTorqueScalingFactorDyn()
{
	return(_torqueScalingFactorDyn);
}

void CRigidBodyContainerDyn::setGravityScalingFactorDyn(float f)
{
	_gravityScalingFactorDyn=f;
}

float CRigidBodyContainerDyn::getGravityScalingFactorDyn()
{
	return(_gravityScalingFactorDyn);
}

void CRigidBodyContainerDyn::setDynamicActivityRange(float f)
{
	_dynamicActivityRange=f;
}

float CRigidBodyContainerDyn::getDynamicActivityRange()
{
	return(_dynamicActivityRange);
}

void CRigidBodyContainerDyn::setDynamicsInternalTimeStep(float dt)
{
	_dynamicsInternalStepSize=dt;
}

float CRigidBodyContainerDyn::getDynamicsInternalTimeStep()
{
	return(_dynamicsInternalStepSize);
}

void CRigidBodyContainerDyn::setDynamicParticlesIdStart(int a)
{
	_dynamicParticlesIdStart=a;
}

int CRigidBodyContainerDyn::getDynamicParticlesIdStart()
{
	return(_dynamicParticlesIdStart);
}

void CRigidBodyContainerDyn::set3dObjectIdStart(int a)
{
	_3dObjectIdStart=a;
}

int CRigidBodyContainerDyn::get3dObjectIdStart()
{
	return(_3dObjectIdStart);
}

void CRigidBodyContainerDyn::set3dObjectIdEnd(int a)
{
	_3dObjectIdEnd=a;
}

int CRigidBodyContainerDyn::get3dObjectIdEnd()
{
	return(_3dObjectIdEnd);
}

int CRigidBodyContainerDyn::getDynamicsCalculationPasses()
{
	return(_dynamicsCalculationPasses);
}

void CRigidBodyContainerDyn::removeRigidBodyFromCollisionShapeDependency(int rigidBodyID)
{
	for (int i=0;i<int(_allCollisionShapes.size());i++)
	{
		if (_allCollisionShapes[i]->removeRigidBodyDependency(rigidBodyID))
		{
			delete _allCollisionShapes[i];
			_allCollisionShapes.erase(_allCollisionShapes.begin()+i);
			i--; // We need to reprocess this position
		}
	}
}

void CRigidBodyContainerDyn::_addOrUpdateRigidBody(CDummyShape* shape,bool forceStatic,bool forceNonRespondable)
{
	CRigidBodyDyn* body=getRigidBodyFromShapeID(_simGetObjectID(shape));

    if (body==nullptr)
	{ // We have to add that shape!
        CCollShapeDyn* collShape=nullptr;
//		CRigidBodyDyn* body=nullptr;

        CDummyGeomProxy* geom=nullptr; // nullptr means that this shape doesn't collide (but still dynamic)

// Following condition added to speed-up calculation with non-respondable shapes (in the past, the first arg of constructor CCollShapeDyn could be nullptr).
// But it happens that the condition caused a bug, noticed thanks to Ayberk �zg�r. So the condition was commented out on 6/3/2013 :
//		if (_simIsShapeDynamicallyRespondable(shape))
		geom=(CDummyGeomProxy*)_simGetGeomProxyFromShape(shape); // yes the shape collides

		// Following to display a warning with static respondable shapes build on a non-static construction:
		if ((_simIsShapeDynamicallyStatic(shape)||forceStatic)&&(_simIsShapeDynamicallyRespondable(shape)&&(!forceNonRespondable)))
		{ // explore its parents:
			CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(shape);
            while (parent!=nullptr)
			{
				if (_simGetObjectType(parent)==sim_object_shape_type)
				{
					CDummyShape* s((CDummyShape*)parent);
					if ((_simIsShapeDynamicallyStatic(s)==0)&&(_simGetTreeDynamicProperty(s)&sim_objdynprop_dynamic))
					{
						_simMakeDynamicAnnouncement(sim_announce_containsstaticshapesondynamicconstruction);
						break;
					}
				}
				// We don't wanna check for dynamic joints or force sensors, since we might have an intended force sensor or joint at the base (e.g. for later assembly with another model)
				parent=(CDummy3DObject*)_simGetParentObject(parent);
			}
		}

		CDummyGeomWrap* geomInfoFromShape=(CDummyGeomWrap*)_simGetGeomWrapFromGeomProxy((CDummyGeomProxy*)_simGetGeomProxyFromShape(shape));

		// Following to display a warning if using non-pure non-convex shapes:
        if ((_simGetPurePrimitiveType(geomInfoFromShape)==sim_pure_primitive_none)&&(geom!=nullptr)&&(_simIsGeomWrapConvex(geomInfoFromShape)==0)&&_simIsShapeDynamicallyRespondable(shape)&&(_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable))
			_simMakeDynamicAnnouncement(sim_announce_containsnonpurenonconvexshapes);

		// Check if the geomObject is already present as a collisionShape:
		collShape=getCollisionShapeFromGeomObject(geom);
        if (collShape==nullptr)
		{ // not yet there. We have to add it

#ifdef INCLUDE_BULLET_2_78_CODE
			collShape=new CCollShapeDyn_bullet278(geom);
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
			collShape=new CCollShapeDyn_bullet283(shape,geom);
#endif

#ifdef INCLUDE_ODE_CODE
			collShape=new CCollShapeDyn_ode(geom,((CRigidBodyContainerDyn_ode*)this)->getOdeSpace());
#endif

#ifdef INCLUDE_NEWTON_CODE
			bool willBeStatic=(_simIsShapeDynamicallyStatic(shape) || forceStatic);
			collShape=new CCollShapeDyn_newton(shape,geom,willBeStatic,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
			collShape=new CCollShapeDyn_vortex(shape,geom,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

			_allCollisionShapes.push_back(collShape);
            if (geom!=nullptr)
				_simSetGeomProxyDynamicsFullRefreshFlag(geom,false);
		}
		// Now create the rigid body and attach the collShape!

#ifdef INCLUDE_BULLET_2_78_CODE
		body=new CRigidBodyDyn_bullet278(shape,collShape,forceStatic,forceNonRespondable,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
		body=new CRigidBodyDyn_bullet283(shape,collShape,forceStatic,forceNonRespondable,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
		body=new CRigidBodyDyn_ode(shape,collShape,forceStatic,forceNonRespondable,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
		body = new CRigidBodyDyn_newton( shape, collShape, forceStatic, forceNonRespondable,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
		body=new CRigidBodyDyn_vortex(shape,collShape,forceStatic,forceNonRespondable,((CRigidBodyContainerDyn_vortex*)this)->getWorld(),0);
#endif // INCLUDE_VORTEX_CODE

		_addRigidBody(body);

		// Now add a dependent rigidBodyID to the collshape:
        if (collShape!=nullptr)
			collShape->addRigidBodyDependency(body->getRigidBodyID());

		_simSetDynamicSimulationIconCode(shape,sim_dynamicsimicon_objectisdynamicallysimulated);
	}
}

CRigidBodyDyn* CRigidBodyContainerDyn::getRigidBodyFromShapeID(int shapeID)
{
	return(_allRigidBodiesIndex[shapeID]);
}

CConstraintDyn* CRigidBodyContainerDyn::getConstraintFromJointID(int jointID)
{
	CDummyJoint* act=(CDummyJoint*)_simGetObject(jointID);
    if (act!=nullptr)
		return(_allConstraintsIndex[jointID]);
    return(nullptr);
}

CConstraintDyn* CRigidBodyContainerDyn::getConstraintFromDummyID(int dummyID)
{
	CDummyDummy* dum=(CDummyDummy*)_simGetObject(dummyID);
    if (dum!=nullptr)
		return(_allConstraintsIndex[dummyID]);
    return(nullptr);
}

CConstraintDyn* CRigidBodyContainerDyn::getConstraintFromForceSensorID(int forceSensorID)
{
	CDummyForceSensor* fs=(CDummyForceSensor*)_simGetObject(forceSensorID);
    if (fs!=nullptr)
		return(_allConstraintsIndex[forceSensorID]);
    return(nullptr);
}


CCollShapeDyn* CRigidBodyContainerDyn::getCollisionShapeFromGeomObject(CDummyGeomProxy* geomData)
{
	for (int i=0;i<int(_allCollisionShapes.size());i++)
	{
		CCollShapeDyn* collShape=_allCollisionShapes[i];
		if (geomData==collShape->getGeomData_nullForNonRespondable())
			return(collShape);
	}
    return(nullptr);
}

int CRigidBodyContainerDyn::_addRigidBody(CRigidBodyDyn* body)
{ // return value is the rigid body ID
	body->setRigidBodyID(_nextRigidBodyID);
	_allRigidBodiesList.push_back(body);
	_allRigidBodiesIndex[_simGetObjectID(body->getShape())]=body;
	return(_nextRigidBodyID++);
}

void CRigidBodyContainerDyn::_removeRigidBody(int rigidBodyID)
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];
		if (rigidBodyID==body->getRigidBodyID())
		{
			removeRigidBodyFromCollisionShapeDependency(rigidBodyID);
			_announceToConstraintsBodyWillBeDestroyed(rigidBodyID);

#ifdef INCLUDE_BULLET_2_78_CODE
			((CRigidBodyContainerDyn_bullet278*)this)->getWorld()->removeCollisionObject(((CRigidBodyDyn_bullet278*)body)->getBtRigidBody());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
			((CRigidBodyContainerDyn_bullet283*)this)->getWorld()->removeCollisionObject(((CRigidBodyDyn_bullet283*)body)->getBtRigidBody());
#endif

#ifdef INCLUDE_ODE_CODE
			// Empty!
#endif

#ifdef INCLUDE_NEWTON_CODE
			// Empty!
#endif

#ifdef INCLUDE_VORTEX_CODE
			// Empty!
#endif // INCLUDE_VORTEX_CODE

            _allRigidBodiesIndex[body->getShapeID()]=nullptr;
			delete body;
			_allRigidBodiesList.erase(_allRigidBodiesList.begin()+i);
			return;
		}
	}	
}

void CRigidBodyContainerDyn::_updateRigidBodiesFromSceneShapes()
{
	// 1. We have to remove all rigid bodies that have invalid uniqueShapeIDs and other
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];

		CDummyShape* shape=(CDummyShape*)_simGetObject(body->getShapeID());
        if (shape!=nullptr)
		{
			CDummyGeomProxy* geomData=(CDummyGeomProxy*)_simGetGeomProxyFromShape(shape);
			CDummyGeomProxy* geomData2=(CDummyGeomProxy*)_simGetGeomProxyFromShape(body->getShape());
			if ( (geomData==geomData2)||_simGetShapeIsStaticAndNotRespondableButDynamicTag(shape) ) // we have to check that the attached geomData is still the same!
			{
                if ( (body->getCollisionShapeDyn()->getGeomData_nullForNonRespondable()!=nullptr)&&_simGetGeomProxyDynamicsFullRefreshFlag(geomData) ) // added on 2009/10/16
                    shape=nullptr;
			}
			else
                shape=nullptr;
		}

        if (shape==nullptr)
		{ // we have to remove that body!
			_removeRigidBody(body->getRigidBodyID());
			i--; // we have to reprocess that position
		}
		else
		{ // we have to check if it is still valid:
			bool remove=_simGetDynamicsFullRefreshFlag(shape)!=0; // new since 2009/10/16
			int dp=_simGetTreeDynamicProperty(shape);
			if ((dp&sim_objdynprop_dynamic)==0)
			{ // All shapes should be static. Should that shape also be non-respondable?
				if ((dp&sim_objdynprop_respondable)==0)
					remove=true; // yes, we remove it
				else
				{ // Here we have only to remove the shape if it was previously non-static (it will be added again a bit further down)
					// Following instruction replaced on 6/5/2011 (was a bug before I think).
					if (!body->isBodyKinematic())
						remove=true;
				}
			}
			if (dp&sim_objdynprop_dynamic)
			{
				if (body->isBodyKinematic())
				{ // static
					if ( (_simIsShapeDynamicallyRespondable(shape)==0)&&(_simGetShapeIsStaticAndNotRespondableButDynamicTag(shape)==0) )
						remove=true;
					if (_simIsShapeDynamicallyStatic(shape)==0) // added on 2010/08/07
						remove=true; // That means: this is usually dynamic, but now static: we have overriden the dynamic characteristic!
				}
			}
			if (remove)
			{ // we have to remove that body!
				_removeRigidBody(body->getRigidBodyID());
				i--; // we have to reprocess that position
			}
		}
	}

	// 2. we have to add new shapes
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
	{
		CDummyShape* shape=(CDummyShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
		if ( _simIsShapeDynamicallyRespondable(shape)||(_simIsShapeDynamicallyStatic(shape)==0)||_simGetShapeIsStaticAndNotRespondableButDynamicTag(shape) )
		{
			int dp=_simGetTreeDynamicProperty(shape);
			if (dp&sim_objdynprop_dynamic) // Make sure the shape is enabled dynamically
				_addOrUpdateRigidBody(shape,false,(dp&sim_objdynprop_respondable)==0);
			else
			{ // that shape should be static
				if (dp&sim_objdynprop_respondable)
					_addOrUpdateRigidBody(shape,true,false); // add the shape, but let it appear static
				else
					_simSetDynamicSimulationIconCode(shape,sim_dynamicsimicon_none);
			}
		}
		_simSetDynamicsFullRefreshFlag(shape,false);
	}
}

bool CRigidBodyContainerDyn::isDynamicContentAvailable()
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		if (!_allRigidBodiesList[i]->isBodyKinematic())
			return(true);
	}
	return(false);
}

void CRigidBodyContainerDyn::handleDynamics(float dt,float simulationTime)
{
	currentRigidBodyContainerDynObject=this; // so that ODE's contact callback works! (and maybe Vortex too)


	_timeStepPassedToHandleDynamicsFunction=dt;
	float maxDynStep=0.0f;

#ifdef INCLUDE_BULLET_2_78_CODE
    maxDynStep=simGetEngineFloatParameter(sim_bullet_global_stepsize,-1,nullptr,nullptr);
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
    maxDynStep=simGetEngineFloatParameter(sim_bullet_global_stepsize,-1,nullptr,nullptr);
#endif

#ifdef INCLUDE_ODE_CODE
    maxDynStep=simGetEngineFloatParameter(sim_ode_global_stepsize,-1,nullptr,nullptr);
#endif

#ifdef INCLUDE_NEWTON_CODE
    maxDynStep=simGetEngineFloatParameter(sim_newton_global_stepsize,-1,nullptr,nullptr);
#endif

#ifdef INCLUDE_VORTEX_CODE
    maxDynStep=simGetEngineFloatParameter(sim_vortex_global_stepsize,-1,nullptr,nullptr);
#endif // INCLUDE_VORTEX_CODE

	_dynamicsCalculationPasses=int((dt/maxDynStep)+0.5f);
	if (_dynamicsCalculationPasses<1)
		_dynamicsCalculationPasses=1;
	float effStepSize=dt/float(_dynamicsCalculationPasses);
	setDynamicsInternalTimeStep(effStepSize);

	// Following is not for the visible contacts, but for the contacts callable from the API:
	_contactInfo.clear();

	int jointListSize=_simGetObjectListSize(sim_object_joint_type);
	int forceSensorListSize=_simGetObjectListSize(sim_object_forcesensor_type);
	for (int i=0;i<jointListSize;i++)
		_simSetDynamicJointLocalTransformationPart2IsValid((CDummyJoint*)_simGetObjectFromIndex(sim_object_joint_type,i),false);
	for (int i=0;i<forceSensorListSize;i++) // this and next line since 2010/02/13
		_simSetDynamicForceSensorLocalTransformationPart2IsValid((CDummyForceSensor*)_simGetObjectFromIndex(sim_object_forcesensor_type,i),false);


	bool particlesPresent=_updateDynamicWorld();
	_createDependenciesBetweenJoints();
	_contactPoints.clear(); // We have it here too in case we suddenly remove all dynamic content!

	if (isDynamicContentAvailable()||particlesPresent)
	{
		applyGravity();

		_calculateBodyToShapeTransformations_forKinematicBodies(dt);
		int passes=getDynamicsCalculationPasses(); // previously calculated
		for (int i=0;i<passes;i++)
		{
            int integers[4]={0,i+1,passes,0};
            float floats[1]={CRigidBodyContainerDyn::getDynamicsInternalTimeStep()};
            _simDynCallback(integers,floats);
			reportShapeConfigurations_forKinematicBodies(float(i+1)/float(passes),dt);
			_handleMotorControls(i+1,passes); // to enable/disable motors, to update target velocities, target positions and position control
			handleAdditionalForcesAndTorques(); // for shapes but also for "anti-gravity" particles or particels with fluid friction force!

			_contactPoints.clear(); // 2010/10/07

			_stepDynamics(effStepSize,i);

			int totalPassesCount=0;
			if (i==passes-1)
				totalPassesCount=passes;
			// Following moved inside the passes loop on 2009/11/29
			reportDynamicWorldConfiguration(totalPassesCount,false,simulationTime+float(i+1)*dt/float(passes));
            integers[3]=1;
            _simDynCallback(integers,floats);
		}
		_applyCorrectEndConfig_forKinematicBodies(); // Added on 2010/10/7 to correct for subtle things for ODE (maybe even for Bullet!)
	}


#ifdef INCLUDE_VORTEX_CODE
//	((CRigidBodyContainerDyn_vortex*)this)->handleDemoVersion();
#endif // INCLUDE_VORTEX_CODE

	clearAdditionalForcesAndTorques();

	// 1=respondable, 2=dynamic, 4=free, 8=motor, 16=pos control,32=force sensor, 64=loop closure dummy
	// Do following always, also when displaying the normal scene (so that when we switch during a pause, it looks correct)
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
	{
		CDummyShape* it=(CDummyShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
		CRigidBodyDyn* b=_allRigidBodiesIndex[_simGetObjectID(it)];
        if (b!=nullptr)
		{
			int flag=0;
			if (!b->isBodyKinematic())
				flag|=2;
			if ( _simIsShapeDynamicallyRespondable(it)&&(_simGetDynamicCollisionMask(it)!=0)&&((_simGetTreeDynamicProperty(it)&sim_objdynprop_respondable)!=0) )
				flag|=1;
			_simSetDynamicObjectFlagForVisualization(it,flag);
		}
	}
	for (int i=0;i<jointListSize;i++)
	{
		CDummyJoint* it=(CDummyJoint*)_simGetObjectFromIndex(sim_object_joint_type,i);
		CConstraintDyn* c=_allConstraintsIndex[_simGetObjectID(it)];
        if (c!=nullptr)
		{
			int flag=0;
			if (_simIsDynamicMotorEnabled(it))
			{
				if (_simIsDynamicMotorPositionCtrlEnabled(it))
					flag=16;
				else
					flag=8;					
			}
			else
				flag=4;
			_simSetDynamicObjectFlagForVisualization(it,flag);
		}
	}
	for (int i=0;i<forceSensorListSize;i++)
	{
		CDummyForceSensor* it=(CDummyForceSensor*)_simGetObjectFromIndex(sim_object_forcesensor_type,i);
		CConstraintDyn* c=_allConstraintsIndex[_simGetObjectID(it)];
        if (c!=nullptr)
			_simSetDynamicObjectFlagForVisualization(it,32);
	}
	int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
	for (int i=0;i<dummyListSize;i++)
	{
		CDummyDummy* it=(CDummyDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
		CConstraintDyn* c=_allConstraintsIndex[_simGetObjectID(it)];
        if (c!=nullptr)
			_simSetDynamicObjectFlagForVisualization(it,64);
	}
}

void CRigidBodyContainerDyn::reportDynamicWorldConfiguration(int totalPassesCount,bool doNotApplyJointIntrinsicPositions,float simulationTime)
{ // can also be called from outside (i.e. LuaScriptObject). In that case totalPassesCount should be -1 and doNotApplyJointIntrinsicPositions true
	reportConstraintConfigurations(totalPassesCount,doNotApplyJointIntrinsicPositions); // do this before reportRigidBodyConfigurations!!
	reportRigidBodyConfigurationsAndVelocities();
	reportConstraintSecondPartConfigurations(); // do this after reportRigidBodyConfigurations!!
	particleCont.updateParticlesPosition(simulationTime);
}

void CRigidBodyContainerDyn::reportRigidBodyConfigurationsAndVelocities()
{
	// first set all velocities to zero:
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
	{
		CDummyShape* it=(CDummyShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
        if (it!=nullptr)
			_simSetShapeDynamicVelocity(it,C3Vector::zeroVector.data,C3Vector::zeroVector.data);
	}
	// It is important that we update shapes from base to tip, otherwise we get wrong positions!
	std::vector<CDummy3DObject*> toExplore;
	int orphanListSize=_simGetObjectListSize(-1);
	for (int i=0;i<orphanListSize;i++)
	{
		toExplore.push_back((CDummy3DObject*)_simGetObjectFromIndex(-1,i));
		for (int j=0;j<int(toExplore.size());j++)
		{
			CDummy3DObject* it=toExplore[j];
			if (_simGetObjectType(it)==sim_object_shape_type)
			{
				CDummyShape* shape=(CDummyShape*)it;
				int shapeID=_simGetObjectID(shape);

				CRigidBodyDyn* rb=getRigidBodyFromShapeID(shapeID);
                if (rb!=nullptr)
				{
					rb->reportConfigurationToShape(shape);
					rb->reportVelocityToShape(shape);
				}
			}
			// Now its own children:
			int childrenCount;
			CDummy3DObject** childrenPointer=(CDummy3DObject**)_simGetObjectChildren(it,&childrenCount);
			if (childrenCount>0)
				toExplore.insert(toExplore.end(),childrenPointer,childrenPointer+childrenCount);
		}
		toExplore.clear();
	}
}

void CRigidBodyContainerDyn::reportConstraintConfigurations(int totalPassesCount,bool doNotApplyJointIntrinsicPositions)
{
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];

		CDummyJoint* joint=(CDummyJoint*)_simGetObject(constraint->getJointID());
        if (joint!=nullptr)
		{
			int ldummyA=constraint->getJointOrForceSensorLoopClosureLinkedDummyAChildID();
			int ldummyB=constraint->getJointOrForceSensorLoopClosureLinkedDummyBChildID();
            if ( (ldummyA==-1)||(ldummyB==-1) ) // bug until 16/10/2011, was ==nullptr before (but didn't cause crashes, or maybe very rarely)
			{
                constraint->reportConfigurationToJoint(joint,nullptr,nullptr,doNotApplyJointIntrinsicPositions); // regular case
                constraint->reportForcesToJoint(joint,nullptr,nullptr,totalPassesCount); // New since 1/6/2011, regular case
			}
			else
			{ // complex case (looped)
				CDummyDummy* a=(CDummyDummy*)_simGetObject(ldummyA); // can only be a dummy!
				CDummyDummy* b=(CDummyDummy*)_simGetObject(ldummyB); // can only be a dummy!
				constraint->reportConfigurationToJoint(joint,a,b,doNotApplyJointIntrinsicPositions);
				constraint->reportForcesToJoint(joint,a,b,totalPassesCount); // New since 1/6/2011
			}
		}

		CDummyForceSensor* forceSensor=(CDummyForceSensor*)_simGetObject(constraint->getForceSensorID());
        if (forceSensor!=nullptr)
		{
			int ldummyA=constraint->getJointOrForceSensorLoopClosureLinkedDummyAChildID();
			int ldummyB=constraint->getJointOrForceSensorLoopClosureLinkedDummyBChildID();
            if ( (ldummyA==-1)||(ldummyB==-1) ) // bug until 16/10/2011, was ==nullptr before (but didn't cause crashes, or maybe very rarely)
                constraint->reportConfigurationAndForcesToForceSensor(forceSensor,nullptr,nullptr,totalPassesCount); // regular case
			else
			{ // complex case (looped)
				CDummyDummy* a=(CDummyDummy*)_simGetObject(ldummyA); // can only be a dummy!
				CDummyDummy* b=(CDummyDummy*)_simGetObject(ldummyB); // can only be a dummy!
				constraint->reportConfigurationAndForcesToForceSensor(forceSensor,a,b,totalPassesCount);
			}
		}

		CDummyDummy* dummy=(CDummyDummy*)_simGetObject(constraint->getDummyID());
        if (dummy!=nullptr)
		{
			int theLinkedDummyID;
			if (_simGetDummyLinkType(dummy,&theLinkedDummyID)==sim_dummy_linktype_dynamics_loop_closure)
			{
				CDummyDummy* linkedDummy=(CDummyDummy*)_simGetObject(theLinkedDummyID);
                if (linkedDummy!=nullptr)
				{
					constraint->reportConfigurationToDummies(dummy,linkedDummy);
				}
			}
		}

	}
}

void CRigidBodyContainerDyn::reportConstraintSecondPartConfigurations()
{ // This is for joints and force sensors only
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];

		CDummyJoint* joint=(CDummyJoint*)_simGetObject(constraint->getJointID());
        if (joint!=nullptr)
			constraint->reportSecondPartConfigurationToJoint(joint);

		CDummyForceSensor* forceSensor=(CDummyForceSensor*)_simGetObject(constraint->getForceSensorID());
        if (forceSensor!=nullptr)
			constraint->reportSecondPartConfigurationToForceSensor(forceSensor);

		constraint->incrementDynPassCounter(); // 18/2/2013
	}
}

void CRigidBodyContainerDyn::reportShapeConfigurations_forKinematicBodies(float t,float cumulatedTimeStep)
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];
		if (body->isBodyKinematic())
		{
			CDummyShape* shape=(CDummyShape*)_simGetObject(body->getShapeID());
            if (shape!=nullptr)
				body->reportShapeConfigurationToRigidBody_forKinematicBody(shape,t,cumulatedTimeStep);
		}
	}
}

void CRigidBodyContainerDyn::_calculateBodyToShapeTransformations_forKinematicBodies(float dt)
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];
		if (body->isBodyKinematic())
		{
			CDummyShape* shape=(CDummyShape*)_simGetObject(body->getShapeID());
            if (shape!=nullptr)
				body->calculateBodyToShapeTransformation_forKinematicBody(shape,dt);
		}
	}
}


void CRigidBodyContainerDyn::_applyCorrectEndConfig_forKinematicBodies()
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];
		if (body->isBodyKinematic())
			body->applyCorrectEndConfig_forKinematicBody();
	}
}

void CRigidBodyContainerDyn::_announceToConstraintsBodyWillBeDestroyed(int rigidBodyID)
{
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constr=_allConstraintsList[i];
		if (constr->announceBodyWillBeDestroyed(rigidBodyID))
		{ // We need to remove this constraint!
			_removeConstraintFromIndex(i);
			i--; // this position needs to be reprocessed since erased
		}
	}
}

void CRigidBodyContainerDyn::_removeConstraintFromIndex(int indexPos)
{
	if ( (indexPos>=0)&&(int(_allConstraintsList.size())>indexPos) )
	{

#ifdef INCLUDE_BULLET_2_78_CODE
		((CRigidBodyContainerDyn_bullet278*)this)->getWorld()->removeConstraint(((CConstraintDyn_bullet278*)_allConstraintsList[indexPos])->getBtTypedConstraint());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
		((CRigidBodyContainerDyn_bullet283*)this)->getWorld()->removeConstraint(((CConstraintDyn_bullet283*)_allConstraintsList[indexPos])->getBtTypedConstraint());
#endif

#ifdef INCLUDE_ODE_CODE
		// Is ok to be empty (CConstraintDyn destruction will remove the joint from the world and destroy it)
#endif

#ifdef INCLUDE_NEWTON_CODE
		// Is ok to be empty (CConstraintDyn destruction will remove the joint from the world and destroy it)
#endif

#ifdef INCLUDE_VORTEX_CODE
		//_vortexWorld->removeConstraint(_allConstraintsList[indexPos]->getVortexConstraint());
#endif // INCLUDE_VORTEX_CODE

		int shapeId=_allConstraintsList[indexPos]->getJointID();
		if (shapeId==-1)
		{
			shapeId=_allConstraintsList[indexPos]->getDummyID();
			if (shapeId==-1)
			{
				shapeId=_allConstraintsList[indexPos]->getForceSensorID();
			}
		}
		if (shapeId!=-1)
            _allConstraintsIndex[shapeId]=nullptr;

		_removeDependenciesBetweenJoints(_allConstraintsList[indexPos]);

		delete _allConstraintsList[indexPos];
		_allConstraintsList.erase(_allConstraintsList.begin()+indexPos);
	}
}

void CRigidBodyContainerDyn::_updateConstraintsFromSceneJoints()
{
	// 1. We have to remove all constraints that have invalid uniqueJointIDs
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];
		int jointID=constraint->getJointID();
		if (jointID!=-1) // we could have a dummy-dummy or force sensor constraint!
		{ // not a dummy-dummy constraint, nor a force sensor constraint

			CDummyJoint* joint=(CDummyJoint*)_simGetObject(jointID);
            if (joint==nullptr)
			{ // we have to remove that constraint, the joint has disappeared!
				_removeConstraintFromIndex(i);
				i--; // we have to reprocess that position
			}
			else
			{ // We have to make sure the bodies are still there! (also with the same hierarchy relationship!)
				bool remove=true;
				CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(joint);
                if (parent!=nullptr)
				{
					int childrenCount;
					CDummy3DObject** childrenPointer=(CDummy3DObject**)_simGetObjectChildren(joint,&childrenCount); 
					if (childrenCount==1)
					{
						CDummy3DObject* child=childrenPointer[0];
						// Following new since 2010/03/16 (to be compatible with shape-joint-dummy-dummy-shape constraints)
						if (_simGetObjectID(parent)==constraint->getParentShapeID())
						{ // ok, the parent is still there (and has obviously to be a shape, otherwise its unique id wouldn't be the same)
							CDummyShape* shapeA=(CDummyShape*)parent;
							// Now we have 2 possibilities: child is a shape (regular case), or it is a dummy
							int childDummyID=constraint->getJointOrForceSensorLoopClosureLinkedDummyAChildID();
							if (childDummyID==-1)
							{ // we have the regular case here: child should be a shape
								if (_simGetObjectID(child)==constraint->getChildShapeID())
								{ // check now if at least one of the attached bodies is dynamic (a joint between two static shapes is NOT legal):
									CDummyShape* shapeB=(CDummyShape*)child; // is obviously a shape (it has the same unique ID)
									if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
										remove=_simGetDynamicsFullRefreshFlag(joint)!=0; // added on 2009/10/16 (was false before)
								}
							}
							else
							{ // here we have the more complex case: child is a dummy linked to another dummy whose parent is a shape
								int childChildListSize;
								_simGetObjectChildren(child,&childChildListSize);
								if ((_simGetObjectID(child)==childDummyID)&&(childChildListSize==0))
								{ 
									CDummyDummy* dummyA=(CDummyDummy*)child; // is obviously a dummy (it has the same unique ID)
									int dummyALinkedDummyID;
									int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
									CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyID);
                                    if (dummyB!=nullptr)
									{
										int dummyBChildListSize;
										_simGetObjectChildren(dummyB,&dummyBChildListSize);
										if ((dummyALinkType==sim_dummy_linktype_dynamics_loop_closure)&&(dummyBChildListSize==0))
										{ // ok, the two dummies are linked correctly. is the second dummy the same?
											int childDummyBID=constraint->getJointOrForceSensorLoopClosureLinkedDummyBChildID();
											if (childDummyBID==_simGetObjectID(dummyB))
											{ // yes, the second dummy is the same!
												CDummy3DObject* dummyBParent=(CDummy3DObject*)_simGetParentObject(dummyB);
                                                if (dummyBParent!=nullptr)
												{
													if (_simGetObjectID(dummyBParent)==constraint->getChildShapeID())
													{ // almost everything looks ok
														// check now if at least one of the attached bodies is dynamic (a joint between two static shapes is not YET supported):
														CDummyShape* shapeB=(CDummyShape*)dummyBParent; // is obviously a shape (it has the same unique ID)
														if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
															remove=_simGetDynamicsFullRefreshFlag(joint)!=0; // added on 2009/10/16 (was false before)
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
				if (!remove)
				{ // now make sure the joint is dynamic (or static) and not disabled
					if ((_simGetTreeDynamicProperty(joint)&sim_objdynprop_dynamic)==0)
						remove=true;
				}
				if (remove)
				{ // we have to remove that constraint!
					_removeConstraintFromIndex(i);
					i--; // we have to reprocess that position
				}
			}
		}
	}

	// 2. we have to add new joints
	int jointListSize=_simGetObjectListSize(sim_object_joint_type);
	for (int i=0;i<jointListSize;i++)
	{
		CDummyJoint* joint=(CDummyJoint*)_simGetObjectFromIndex(sim_object_joint_type,i);
		// Make sure the joint is dynamically enabled:
		if ( ((_simGetJointMode(joint)==sim_jointmode_force)||_simIsJointInHybridOperation(joint))&&(_simGetTreeDynamicProperty(joint)&sim_objdynprop_dynamic) )
			_addOrUpdateJointConstraint(joint);
		else
			_simSetDynamicSimulationIconCode(joint,sim_dynamicsimicon_none);
		_simSetDynamicsFullRefreshFlag(joint,false);
	}
}

void CRigidBodyContainerDyn::_updateConstraintsFromSceneDummies()
{
	// 1. We have to remove all constraints that have invalid uniqueDummyIDs
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];

		int dummyID=constraint->getDummyID();
		if (dummyID!=-1) // we could have a joint or force sensor constraint!
		{
			CDummyDummy* dummy=(CDummyDummy*)_simGetObject(dummyID);
            if (dummy==nullptr)
			{ // we have to remove that constraint!
				_removeConstraintFromIndex(i);
				i--; // we have to reprocess that position
			}
			else
			{ // We have to make sure the bodies are still there! (also with the same hierarchy relationship!)
				bool remove=true;
				CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(dummy);
				int linkedDummyID;
				int dummyLinkType=_simGetDummyLinkType(dummy,&linkedDummyID);
				CDummyDummy* linkedDummy=(CDummyDummy*)_simGetObject(linkedDummyID);
                if ((parent!=nullptr)&&(linkedDummy!=nullptr))
				{
					int dummyChildListSize;
					_simGetObjectChildren(dummy,&dummyChildListSize);
					int linkedDummyChildListSize;
					_simGetObjectChildren(linkedDummy,&linkedDummyChildListSize);
					if ((dummyLinkType==sim_dummy_linktype_dynamics_loop_closure)&&(dummyChildListSize==0)&&(linkedDummyChildListSize==0) ) // added the two ==0 on 2010/03/16
					{
						CDummy3DObject* child=(CDummy3DObject*)_simGetParentObject(linkedDummy);
                        if (child!=nullptr)
						{
							if ((_simGetObjectID(parent)==constraint->getParentShapeID())&&(_simGetObjectID(child)==constraint->getChildShapeID()))
							{ // now make sure that at least one of the parent is dynamic (a rigid joint between two static shapes is not YET supported)
								CDummyShape* shapeA=(CDummyShape*)parent;
								CDummyShape* shapeB=(CDummyShape*)child;
								if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
									remove=_simGetDynamicsFullRefreshFlag(dummy)!=0; // added on 2009/10/16 (was false before)
							}
						}
					}
				}
				if (!remove)
				{ // now make sure the two dummies are not disabled
					if (((_simGetTreeDynamicProperty(dummy)&sim_objdynprop_dynamic)==0)||((_simGetTreeDynamicProperty(linkedDummy)&sim_objdynprop_dynamic)==0) )
						remove=true;
				}
				if (remove)
				{ // we have to remove that constraint!
					_removeConstraintFromIndex(i);
					i--; // we have to reprocess that position
				}
			}
		}
	}

	// 2. we have to add new linked dummies
	int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
	for (int i=0;i<dummyListSize;i++)
	{
		CDummyDummy* dummy=(CDummyDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
		// Make sure the dummy is dynamic
		int linkedDummyID;
		int linkType=_simGetDummyLinkType(dummy,&linkedDummyID);
		if ( (linkedDummyID!=-1)&&(linkType==sim_dummy_linktype_dynamics_loop_closure)&&(_simGetTreeDynamicProperty(dummy)&sim_objdynprop_dynamic) )
			_addOrUpdateDummyConstraint(dummy);
		_simSetDynamicsFullRefreshFlag(dummy,false);
	}
}

void CRigidBodyContainerDyn::_updateConstraintsFromSceneForceSensors()
{
	// 1. We have to remove all constraints that have invalid uniqueForceSensorIDs
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];
		int forceSensID=constraint->getForceSensorID();
		if (forceSensID!=-1) // we could have a dummy or joint constraint!
		{
			CDummyForceSensor* forceSensor=(CDummyForceSensor*)_simGetObject(forceSensID);
            if (forceSensor==nullptr)
			{ // we have to remove that constraint, the force sensor has disappeared!
				_removeConstraintFromIndex(i);
				i--; // we have to reprocess that position
			}
			else
			{ // We have to make sure the bodies are still there! (also with the same hierarchy relationship!)
				bool remove=true;
				CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(forceSensor);
				int forceSensorChildrenCount;
				CDummy3DObject** forceSensorChildrenPointer=(CDummy3DObject**)_simGetObjectChildren(forceSensor,&forceSensorChildrenCount);
                if ( (parent!=nullptr)&&(forceSensorChildrenCount==1) )
				{
					CDummy3DObject* child=forceSensorChildrenPointer[0];
					// Following new since 2010-03-16 (to be compatible with shape-sensor-dummy-dummy-shape constraints)
					if (_simGetObjectID(parent)==constraint->getParentShapeID())
					{ // ok, the parent is still there (and has obviously to be a shape, otherwise its unique id wouldn't be the same)
						CDummyShape* shapeA=(CDummyShape*)parent;
						// Now we have 2 possibilities: child is a shape (regular case), or it is a dummy
						int childDummyID=constraint->getJointOrForceSensorLoopClosureLinkedDummyAChildID();
						if (childDummyID==-1)
						{ // we have the regular case here: child should be a shape
							if (_simGetObjectID(child)==constraint->getChildShapeID())
							{ // check now if at least one of the attached bodies is dynamic (a dynamic joint between two static shapes is NOT LEGAL):
								CDummyShape* shapeB=(CDummyShape*)child; // is obviously a shape (it has the same unique ID)
								if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
									remove=_simGetDynamicsFullRefreshFlag(forceSensor)!=0; // added on 2009/10/16 (was false before)
							}
						}
						else
						{ // here we have the more complex case: child is a dummy linked to another dummy whose parent is a shape
							int childChrildrenListSize;
							_simGetObjectChildren(child,&childChrildrenListSize);
							if ((_simGetObjectID(child)==childDummyID)&&(childChrildrenListSize==0))
							{ 
								CDummyDummy* dummyA=(CDummyDummy*)child; // is obviously a dummy (it has the same unique ID)
								int dummyALinkedDummyID;
								int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
								CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyID);
                                if (dummyB!=nullptr)
								{
									int dummyBChrildrenListSize;
									_simGetObjectChildren(dummyB,&dummyBChrildrenListSize);
									if ((dummyALinkType==sim_dummy_linktype_dynamics_loop_closure)&&(dummyBChrildrenListSize==0))
									{ // ok, the two dummies are linked correctly. is the second dummy the same?
										int childDummyBID=constraint->getJointOrForceSensorLoopClosureLinkedDummyBChildID();
										if (childDummyBID==_simGetObjectID(dummyB))
										{ // yes, the second dummy is the same!
											CDummy3DObject* dummyBParent=(CDummy3DObject*)_simGetParentObject(dummyB);
                                            if ((dummyBParent!=nullptr)&&(_simGetObjectID(dummyBParent)==constraint->getChildShapeID()))
											{ // almost everything looks ok
												// check now if at least one of the attached bodies is dynamic (a force sensor between two static shapes is not YET supported):
												CDummyShape* shapeB=(CDummyShape*)dummyBParent; // is obviously a shape (it has the same unique ID)
												if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
													remove=_simGetDynamicsFullRefreshFlag(forceSensor)!=0; // added on 2009/10/16 (was false before)
											}
										}
									}
								}
							}
						}
					}
				}
				if (!remove)
				{ // now make sure the force sensor is dynamic (or static) and not disabled
					if ((_simGetTreeDynamicProperty(forceSensor)&sim_objdynprop_dynamic)==0)
						remove=true;
				}
				if (remove)
				{ // we have to remove that constraint!
					_removeConstraintFromIndex(i);
					i--; // we have to reprocess that position
				}
			}
		}
	}

	// 2. we have to add new force sensors
	int forceSensorListSize=_simGetObjectListSize(sim_object_forcesensor_type);
	for (int i=0;i<forceSensorListSize;i++)
	{
		CDummyForceSensor* forceSensor=(CDummyForceSensor*)_simGetObjectFromIndex(sim_object_forcesensor_type,i);
		// Make sure the forceSensor is dynamically enabled:
		if ( _simGetTreeDynamicProperty(forceSensor)&sim_objdynprop_dynamic )
			_addOrUpdateForceSensorConstraint(forceSensor);
		else
			_simSetDynamicSimulationIconCode(forceSensor,sim_dynamicsimicon_none);
		_simSetDynamicsFullRefreshFlag(forceSensor,false);
	}
}

void CRigidBodyContainerDyn::_addOrUpdateJointConstraint(CDummyJoint* joint)
{
	CConstraintDyn* constraint=getConstraintFromJointID(_simGetObjectID(joint));
    if (constraint==nullptr)
	{ // We have to add that constraint (maybe)!
		bool successful=false;
		CDummy3DObject* parentObj=(CDummy3DObject*)_simGetParentObject(joint);
		int jointChildListSize;
		CDummy3DObject** jointChildrenPointer=(CDummy3DObject**)_simGetObjectChildren(joint,&jointChildListSize);
        if ( (jointChildListSize==1)&&(parentObj!=nullptr)&&(_simGetObjectType(parentObj)==sim_object_shape_type) )
		{
			CDummy3DObject* childObj=jointChildrenPointer[0];
			if (_simGetObjectType(childObj)==sim_object_shape_type)
			{ // regular case: the link is shape-joint-shape
				CRigidBodyDyn* parentBody(getRigidBodyFromShapeID(_simGetObjectID(parentObj)));
				CRigidBodyDyn* childBody(getRigidBodyFromShapeID(_simGetObjectID(childObj)));
                if ( (parentBody!=nullptr)&&(childBody!=nullptr) )
				{ // Now make sure we the child is dynamic:
					// following line new since 2010/03/17:
					if (!childBody->isBodyKinematic())
					{

#ifdef INCLUDE_BULLET_2_78_CODE
						constraint=new CConstraintDyn_bullet278(parentBody,childBody,joint,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
						constraint=new CConstraintDyn_bullet283(parentBody,childBody,joint,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
						constraint=new CConstraintDyn_ode(parentBody,childBody,joint,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
						constraint = new CConstraintDyn_newton(parentBody, childBody, joint,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
						constraint=new CConstraintDyn_vortex(parentBody,childBody,joint,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

						_allConstraintsList.push_back(constraint);
						_allConstraintsIndex[_simGetObjectID(joint)]=constraint;
						successful=true;
					}

				}
			}
			else
			{ // following added on 2010/03/16 (we might have a link: shape-joint-dummy-dummy-shape)
				if (_simGetObjectType(childObj)==sim_object_dummy_type)
				{ // the link might be the complex type (loop closure with joint):
					CDummyDummy* dummyA=(CDummyDummy*)childObj;
					int dummyALinkedDummyId;
					int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyId);
					CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyId);
                    if ((dummyB!=nullptr)&&(dummyALinkType==sim_dummy_linktype_dynamics_loop_closure))
					{
						int dummyAChildListSize,dummyBChildListSize;
						_simGetObjectChildren(dummyA,&dummyAChildListSize);
						_simGetObjectChildren(dummyB,&dummyBChildListSize);
						if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
						{
							CDummy3DObject* dummyBParent=(CDummy3DObject*)_simGetParentObject(dummyB);
                            if (dummyBParent!=nullptr)
							{
								CRigidBodyDyn* parentBody(getRigidBodyFromShapeID(_simGetObjectID(parentObj)));
								CRigidBodyDyn* childBody(getRigidBodyFromShapeID(_simGetObjectID(dummyBParent)));
                                if ( (parentBody!=nullptr)&&(childBody!=nullptr) )
								{ // Now make sure we have at least one dynamic body:

									if ( (!parentBody->isBodyKinematic())||(!childBody->isBodyKinematic()) )
									{ // ok, everything looks alright!

#ifdef INCLUDE_BULLET_2_78_CODE
										constraint=new CConstraintDyn_bullet278(parentBody,childBody,joint,dummyA,dummyB,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
										constraint=new CConstraintDyn_bullet283(parentBody,childBody,joint,dummyA,dummyB,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
										constraint=new CConstraintDyn_ode(parentBody,childBody,joint,dummyA,dummyB,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
										constraint = new CConstraintDyn_newton(parentBody, childBody,joint,dummyA,dummyB,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
										constraint=new CConstraintDyn_vortex(parentBody,childBody,joint,dummyA,dummyB,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

										_allConstraintsList.push_back(constraint);
										_allConstraintsIndex[_simGetObjectID(joint)]=constraint;
										successful=true;
									}
								}
							}
						}
					}
				}
			}
		}
		if (!successful)
			_simSetDynamicSimulationIconCode(joint,sim_dynamicsimicon_objectisnotdynamicallyenabled);
		else
			_simSetDynamicSimulationIconCode(joint,sim_dynamicsimicon_objectisdynamicallysimulated);
	}
}

void CRigidBodyContainerDyn::_addOrUpdateDummyConstraint(CDummyDummy* dummy)
{
	CConstraintDyn* constraint=getConstraintFromDummyID(_simGetObjectID(dummy));
    if (constraint==nullptr)
	{ // We have to add that constraint! (maybe)
		CDummy3DObject* parentObj=(CDummy3DObject*)_simGetParentObject(dummy);
        if ( (parentObj!=nullptr)&&(_simGetObjectType(parentObj)==sim_object_shape_type) )
		{
			int dummyLinkedDummyID;
			int dummyLinkType=_simGetDummyLinkType(dummy,&dummyLinkedDummyID);
			CDummyDummy* childDummy=(CDummyDummy*)_simGetObject(dummyLinkedDummyID);
            if ( (childDummy!=nullptr)&&(dummyLinkType==sim_dummy_linktype_dynamics_loop_closure) )
			{
				int dummyChildListSize,childDummyChildListSize;
				_simGetObjectChildren(dummy,&dummyChildListSize);
				_simGetObjectChildren(childDummy,&childDummyChildListSize);
				if ( (dummyChildListSize==0)&&(childDummyChildListSize==0) ) // condition added on 2010/03/17
				{
					CDummy3DObject* secondParentObj=(CDummy3DObject*)_simGetParentObject(childDummy);
                    if ( (secondParentObj!=nullptr)&&(_simGetObjectType(secondParentObj)==sim_object_shape_type) )
					{
						CRigidBodyDyn* parentBody(getRigidBodyFromShapeID(_simGetObjectID(parentObj)));
						CRigidBodyDyn* childBody(getRigidBodyFromShapeID(_simGetObjectID(secondParentObj)));
                        if ( (parentBody!=nullptr)&&(childBody!=nullptr) )
						{ // Now make sure that at least one body is dynamic:
							if ( (!parentBody->isBodyKinematic())||(!childBody->isBodyKinematic()) )
							{

#ifdef INCLUDE_BULLET_2_78_CODE
								constraint=new CConstraintDyn_bullet278(parentBody,childBody,dummy,childDummy,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
								constraint=new CConstraintDyn_bullet283(parentBody,childBody,dummy,childDummy,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
								constraint=new CConstraintDyn_ode(parentBody,childBody,dummy,childDummy,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
								constraint = new CConstraintDyn_newton(parentBody,childBody,dummy,childDummy,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
								constraint=new CConstraintDyn_vortex(parentBody,childBody,dummy,childDummy,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

								_allConstraintsList.push_back(constraint);
								_allConstraintsIndex[_simGetObjectID(dummy)]=constraint;
							}
						}
					}
				}
			}
		}
	}
}

void CRigidBodyContainerDyn::_addOrUpdateForceSensorConstraint(CDummyForceSensor* forceSensor)
{ 
	CConstraintDyn* constraint=getConstraintFromForceSensorID(_simGetObjectID(forceSensor));
    if (constraint==nullptr)
	{ // We have to add that constraint (maybe)!
		bool successful=false;
		CDummy3DObject* parentObj=(CDummy3DObject*)_simGetParentObject(forceSensor);
		int forceSensorChildListSize;
		CDummy3DObject** forceSensorChildrenPointer=(CDummy3DObject**)_simGetObjectChildren(forceSensor,&forceSensorChildListSize);
        if ( (forceSensorChildListSize==1)&&(parentObj!=nullptr)&&(_simGetObjectType(parentObj)==sim_object_shape_type) )
		{
			CDummy3DObject* childObj=forceSensorChildrenPointer[0];
			if (_simGetObjectType(childObj)==sim_object_shape_type)
			{ // regular case: the link is shape-sensor-shape
				CRigidBodyDyn* parentBody(getRigidBodyFromShapeID(_simGetObjectID(parentObj)));
				CRigidBodyDyn* childBody(getRigidBodyFromShapeID(_simGetObjectID(childObj)));
                if ( (parentBody!=nullptr)&&(childBody!=nullptr) )
				{ // Now make sure the child is dynamic:
					// Following line new since 2010/03/17:
					if (!childBody->isBodyKinematic())
					{

#ifdef INCLUDE_BULLET_2_78_CODE
						constraint=new CConstraintDyn_bullet278(parentBody,childBody,forceSensor,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
						constraint=new CConstraintDyn_bullet283(parentBody,childBody,forceSensor,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
						constraint=new CConstraintDyn_ode(parentBody,childBody,forceSensor,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
						constraint = new CConstraintDyn_newton(parentBody,childBody,forceSensor,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
						constraint=new CConstraintDyn_vortex(parentBody,childBody,forceSensor,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

						_allConstraintsList.push_back(constraint);
						_allConstraintsIndex[_simGetObjectID(forceSensor)]=constraint;
						successful=true;
					}
				}
			}
			else
			{ // following added on 2010/03/16 (we might have a link: shape-sensor-dummy-dummy-shape)
				if (_simGetObjectType(childObj)==sim_object_dummy_type)
				{ // the link might be the complex type (loop closure with joint):
					CDummyDummy* dummyA=(CDummyDummy*)childObj;
					int dummyALinkedDummyID;
					int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
					CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyID);
                    if ((dummyB!=nullptr)&&(dummyALinkType==sim_dummy_linktype_dynamics_loop_closure))
					{
						int dummyAChildListSize,dummyBChildListSize;
						_simGetObjectChildren(dummyA,&dummyAChildListSize);
						_simGetObjectChildren(dummyB,&dummyBChildListSize);
						if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
						{
							CDummy3DObject* dummyBParent=(CDummy3DObject*)_simGetParentObject(dummyB);
                            if (dummyBParent!=nullptr)
							{
								CRigidBodyDyn* parentBody(getRigidBodyFromShapeID(_simGetObjectID(parentObj)));
								CRigidBodyDyn* childBody(getRigidBodyFromShapeID(_simGetObjectID(dummyBParent)));
                                if ( (parentBody!=nullptr)&&(childBody!=nullptr) )
								{ // Now make sure we have at least one dynamic body:
									if ( (!parentBody->isBodyKinematic())||(!childBody->isBodyKinematic()) )
									{

#ifdef INCLUDE_BULLET_2_78_CODE
										constraint=new CConstraintDyn_bullet278(parentBody,childBody,forceSensor,dummyA,dummyB,((CRigidBodyContainerDyn_bullet278*)this)->getWorld());
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
										constraint=new CConstraintDyn_bullet283(parentBody,childBody,forceSensor,dummyA,dummyB,((CRigidBodyContainerDyn_bullet283*)this)->getWorld());
#endif

#ifdef INCLUDE_ODE_CODE
										constraint=new CConstraintDyn_ode(parentBody,childBody,forceSensor,dummyA,dummyB,((CRigidBodyContainerDyn_ode*)this)->getWorld());
#endif

#ifdef INCLUDE_NEWTON_CODE
										constraint = new CConstraintDyn_newton(parentBody,childBody,forceSensor,dummyA,dummyB,((CRigidBodyContainerDyn_newton*)this)->getWorld());
#endif

#ifdef INCLUDE_VORTEX_CODE
										constraint=new CConstraintDyn_vortex(parentBody,childBody,forceSensor,dummyA,dummyB,((CRigidBodyContainerDyn_vortex*)this)->getWorld());
#endif // INCLUDE_VORTEX_CODE

										_allConstraintsList.push_back(constraint);
										_allConstraintsIndex[_simGetObjectID(forceSensor)]=constraint;
										successful=true;
									}
								}
							}
						}
					}
				}
			}
		}
		if (!successful)
			_simSetDynamicSimulationIconCode(forceSensor,sim_dynamicsimicon_objectisnotdynamicallyenabled);
		else
			_simSetDynamicSimulationIconCode(forceSensor,sim_dynamicsimicon_objectisdynamicallysimulated);
	}
}

bool CRigidBodyContainerDyn::_updateDynamicWorld()
{ // return value indicates whether particles are present and need to be simulated
	bool particlesPresent=false;
	_temporarilyDisableShapesOutOfDynamicActivityRange(); // e.g. when shapes are falling too deep or flying too high, they are disabled
	_tagStaticAndNotCollidableButDynamicShapes(); // special: when a dynamic joint (or sensor!) connects a static non-collidable shape and a dynamic shape
	_updateRigidBodiesFromSceneShapes();
	_updateConstraintsFromSceneJoints();
	_updateConstraintsFromSceneDummies();
	_updateConstraintsFromSceneForceSensors();
	_updateHybridJointTargetPositions();

	particlesPresent|=particleCont.addParticlesIfNeeded();
	particleCont.removeKilledParticles();

#ifdef INCLUDE_NEWTON_CODE
	if (((CRigidBodyContainerDyn_newton*)this)->_rebuildSkeletons)
		((CRigidBodyContainerDyn_newton*)this)->_rebuildSkeletonList();
#endif

	return(particlesPresent);
}

void CRigidBodyContainerDyn::_temporarilyDisableShapesOutOfDynamicActivityRange()
{ // So that falling objects stop falling eventually! On 04/02/2011 extended the functionality to all dimensions and directions!
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
	{
		CDummy3DObject* it=(CDummy3DObject*)_simGetObjectFromIndex(sim_object_shape_type,i);
        if (_simGetParentObject(it)==nullptr) // condition since the end of the world: 21/12/2012
		{
			C7Vector tr;
			_simGetObjectCumulativeTransformation(it,tr.X.data,tr.Q.data,true);
			if (tr.X.getLength()>getDynamicActivityRange())
				_simDisableDynamicTreeForManipulation(it,true);
		}
	}
}

void CRigidBodyContainerDyn::_tagStaticAndNotCollidableButDynamicShapes()
{ // Here we don't care whether objects are dynamically enabled or disabled (through the hierarchy). This is just an additional tag
	// 1. We untag all shapes:
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
		_simSetShapeIsStaticAndNotRespondableButDynamicTag((CDummyShape*)_simGetObjectFromIndex(sim_object_shape_type,i),false);
	// 2. We check all dynamic joints and look if they connect a static non-collidable shape (parent) and a dynamic shape (child) (or the looped variant):
	int jointListSize=_simGetObjectListSize(sim_object_joint_type);
	for (int i=0;i<jointListSize;i++)
	{
		CDummyJoint* it=(CDummyJoint*)_simGetObjectFromIndex(sim_object_joint_type,i);
		if ((_simGetJointMode(it)==sim_jointmode_force)||_simIsJointInHybridOperation(it))
		{ // we have a dynamic joint here.
			int itChildListSize;
			CDummy3DObject** itChildrenPointer=(CDummy3DObject**)_simGetObjectChildren(it,&itChildListSize);
			if (itChildListSize==1)
			{
				CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(it);
                if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_object_shape_type) )
				{
					CDummyShape* parentShape=(CDummyShape*)parent;
					if (_simGetObjectType(itChildrenPointer[0])==sim_object_shape_type)
					{ // we might have a regular case (non-looped)
						CDummyShape* childShape=(CDummyShape*)itChildrenPointer[0];
						if (_simIsShapeDynamicallyStatic(childShape)==0)
						{
							if ( _simIsShapeDynamicallyStatic(parentShape)&&(_simIsShapeDynamicallyRespondable(parentShape)==0) )
								_simSetShapeIsStaticAndNotRespondableButDynamicTag(parentShape,true);
						}
					}
					else
					{ 
						if (_simGetObjectType(itChildrenPointer[0])==sim_object_dummy_type)
						{ // we might have a complex case (looped)
							CDummyDummy* dummyA=(CDummyDummy*)itChildrenPointer[0];
							int dummyALinkedDummyID;
							int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
							CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyID);
                            if ((dummyB!=nullptr)&&(dummyALinkType==sim_dummy_linktype_dynamics_loop_closure))
							{
								int dummyAChildListSize,dummyBChildListSize;
								_simGetObjectChildren(dummyA,&dummyAChildListSize);
								_simGetObjectChildren(dummyB,&dummyBChildListSize);
								if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
								{
									CDummy3DObject* par=(CDummy3DObject*)_simGetParentObject(dummyB);
                                    if ((par!=nullptr)&&(_simGetObjectType(par)==sim_object_shape_type))
									{
										CDummyShape* childShape=(CDummyShape*)par;
										if ((_simIsShapeDynamicallyStatic(childShape)==0)||(_simIsShapeDynamicallyStatic(parentShape)==0))
										{
											if ( _simIsShapeDynamicallyStatic(parentShape)&&(_simIsShapeDynamicallyRespondable(parentShape)==0) )
												_simSetShapeIsStaticAndNotRespondableButDynamicTag(parentShape,true);
											if ( _simIsShapeDynamicallyStatic(childShape)&&(_simIsShapeDynamicallyRespondable(childShape)==0) )
												_simSetShapeIsStaticAndNotRespondableButDynamicTag(childShape,true);
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	// 2. We check all force sensors and look if they connect a static non-collidable shape (parent) and a dynamic shape (child) (or the looped variant):
	int forceSensorListSize=_simGetObjectListSize(sim_object_forcesensor_type);
	for (int i=0;i<forceSensorListSize;i++)
	{
		CDummyForceSensor* it=(CDummyForceSensor*)_simGetObjectFromIndex(sim_object_forcesensor_type,i);
		int itChildListSize;
		CDummy3DObject** itChildrenPointer=(CDummy3DObject**)_simGetObjectChildren(it,&itChildListSize);
		if (itChildListSize==1)
		{
			CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(it);
            if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_object_shape_type) )
			{
				CDummyShape* parentShape=(CDummyShape*)parent;
				if (_simGetObjectType(itChildrenPointer[0])==sim_object_shape_type)
				{ // we might have a regular case (non-looped)
					CDummyShape* childShape=(CDummyShape*)itChildrenPointer[0];
					if (_simIsShapeDynamicallyStatic(childShape)==0)
					{
						if ( _simIsShapeDynamicallyStatic(parentShape)&&(_simIsShapeDynamicallyRespondable(parentShape)==0) )
							_simSetShapeIsStaticAndNotRespondableButDynamicTag(parentShape,true);
					}
				}
				else
				{ 
					if (_simGetObjectType(itChildrenPointer[0])==sim_object_dummy_type)
					{ // we might have a complex case (looped)
						CDummyDummy* dummyA=(CDummyDummy*)itChildrenPointer[0];
						int dummyALinkedDummyID;
						int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
						CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyID);
                        if ((dummyB!=nullptr)&&(dummyALinkType==sim_dummy_linktype_dynamics_loop_closure))
						{
							int dummyAChildListSize,dummyBChildListSize;
							_simGetObjectChildren(dummyA,&dummyAChildListSize);
							_simGetObjectChildren(dummyB,&dummyBChildListSize);
							if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
							{
								CDummy3DObject* par=(CDummy3DObject*)_simGetParentObject(dummyB);
                                if ((par!=nullptr)&&(_simGetObjectType(par)==sim_object_shape_type))
								{
									CDummyShape* childShape=(CDummyShape*)par;
									if ((_simIsShapeDynamicallyStatic(childShape)==0)||(_simIsShapeDynamicallyStatic(parentShape)==0))
									{
										if ( _simIsShapeDynamicallyStatic(parentShape)&&(_simIsShapeDynamicallyRespondable(parentShape)==0) )
											_simSetShapeIsStaticAndNotRespondableButDynamicTag(parentShape,true);
										if ( _simIsShapeDynamicallyStatic(childShape)&&(_simIsShapeDynamicallyRespondable(childShape)==0) )
											_simSetShapeIsStaticAndNotRespondableButDynamicTag(childShape,true);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	// 2. We check all linked dummies and look if they connect a static non-collidable shape and a dynamic shape (this section was forgotten, added it on 2010/03/18):
	int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
	for (int i=0;i<dummyListSize;i++)
	{
		CDummyDummy* dummyA=(CDummyDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
		int dummyALinkedDummyId;
		int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyId);
		CDummyDummy* dummyB=(CDummyDummy*)_simGetObject(dummyALinkedDummyId);
        if ((dummyB!=nullptr)&&(dummyALinkType==sim_dummy_linktype_dynamics_loop_closure))
		{
			int dummyAChildListSize,dummyBChildListSize;
			_simGetObjectChildren(dummyA,&dummyAChildListSize);
			_simGetObjectChildren(dummyB,&dummyBChildListSize);
			if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
			{
				CDummy3DObject* parent=(CDummy3DObject*)_simGetParentObject(dummyA);
				CDummy3DObject* child=(CDummy3DObject*)_simGetParentObject(dummyB);
                if ( (parent!=nullptr)&&(child!=nullptr)&&(_simGetObjectType(parent)==sim_object_shape_type)&&(_simGetObjectType(child)==sim_object_shape_type) )
				{
					CDummyShape* parentShape=(CDummyShape*)parent;
					CDummyShape* childShape=(CDummyShape*)child;
					if ((_simIsShapeDynamicallyStatic(childShape)==0)||(_simIsShapeDynamicallyStatic(parentShape)==0))
					{
						if ( _simIsShapeDynamicallyStatic(parentShape)&&(_simIsShapeDynamicallyRespondable(parentShape)==0) )
							_simSetShapeIsStaticAndNotRespondableButDynamicTag(parentShape,true);
						if ( _simIsShapeDynamicallyStatic(childShape)&&(_simIsShapeDynamicallyRespondable(childShape)==0) )
							_simSetShapeIsStaticAndNotRespondableButDynamicTag(childShape,true);
					}
				}
			}
		}
	}
}

void CRigidBodyContainerDyn::_handleMotorControls(int passCnt,int totalPasses)
{
	std::vector<CConstraintDyn*> constraintsToHandle;
	std::vector<CDummyJoint*> constraintsToHandle_joint;
	std::vector<int> constraintsToHandle_order;

	int jointListSize=_simGetObjectListSize(sim_object_joint_type);
	for (int i=0;i<jointListSize;i++)
	{
		CDummyJoint* it=(CDummyJoint*)_simGetObjectFromIndex(sim_object_joint_type,i);
        if (it!=nullptr)
		{
			CConstraintDyn* constraint=getConstraintFromJointID(_simGetObjectID(it));
            if (constraint!=nullptr)
			{
				constraintsToHandle.push_back(constraint);
				constraintsToHandle_joint.push_back(it);
				constraintsToHandle_order.push_back(_simGetJointCallbackCallOrder(it));
			}
		}
	}

	// handle first the higher priority joints:
	for (int i=0;i<int(constraintsToHandle.size());i++)
	{
		if (constraintsToHandle_order[i]<0)
			constraintsToHandle[i]->handleMotorControl(constraintsToHandle_joint[i],passCnt,totalPasses);
	}

	// now the normal priority joints:
	for (int i=0;i<int(constraintsToHandle.size());i++)
	{
		if (constraintsToHandle_order[i]==0)
			constraintsToHandle[i]->handleMotorControl(constraintsToHandle_joint[i],passCnt,totalPasses);
	}

	// now the low priority joints:
	for (int i=0;i<int(constraintsToHandle.size());i++)
	{
		if (constraintsToHandle_order[i]>0)
			constraintsToHandle[i]->handleMotorControl(constraintsToHandle_joint[i],passCnt,totalPasses);
	}
}

void CRigidBodyContainerDyn::_updateHybridJointTargetPositions()
{
	for (int i=0;i<int(_allConstraintsList.size());i++)
	{
		CConstraintDyn* constraint=_allConstraintsList[i];
		CDummyJoint* joint=(CDummyJoint*)_simGetObject(constraint->getJointID());
        if ( (joint!=nullptr)&&_simIsJointInHybridOperation(joint) ) // we could have a dummy constraint!
		{ 
			// Following 2 added on 2010/11/26 because dependent joints didn't work 
			if (_simGetJointMode(joint)==sim_jointmode_dependent)
				_simSetJointPosition(joint,0.0f); // value doesn't matter. We just wanna refresh the value that is dependent!
			_simSetDynamicMotorPositionControlTargetPosition(joint,_simGetJointPosition(joint));
		}
	}
}

float* CRigidBodyContainerDyn::getContactPoints(int* cnt)
{
	cnt[0]=_contactPoints.size()/3;
	if (cnt[0]==0)
        return(nullptr);
	return(&_contactPoints[0]);
}

bool CRigidBodyContainerDyn::getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float* contactInfo)
{
	int scanIndex=0;
	int ind=(index|sim_handleflag_extended)-sim_handleflag_extended;
	bool extended=((index&sim_handleflag_extended)!=0);
	for (int i=0;i<int(_contactInfo.size());i++)
	{
		if ( (_contactInfo[i].subPassNumber==dynamicPass)||(dynamicPass==sim_handle_all) )
		{
			if ( (_contactInfo[i].objectID1==objectHandle)||(_contactInfo[i].objectID2==objectHandle)||(sim_handle_all==objectHandle) )
			{
				if (ind==scanIndex)
				{
					contactInfo[0]=_contactInfo[i].position(0);
					contactInfo[1]=_contactInfo[i].position(1);
					contactInfo[2]=_contactInfo[i].position(2);
					if (_contactInfo[i].objectID2==objectHandle)
					{
						objectHandles[0]=_contactInfo[i].objectID2;
						objectHandles[1]=_contactInfo[i].objectID1;
						contactInfo[3]=-_contactInfo[i].directionAndAmplitude(0);
						contactInfo[4]=-_contactInfo[i].directionAndAmplitude(1);
						contactInfo[5]=-_contactInfo[i].directionAndAmplitude(2);
						if (extended)
						{
							contactInfo[6]=-_contactInfo[i].surfaceNormal(0);
							contactInfo[7]=-_contactInfo[i].surfaceNormal(1);
							contactInfo[8]=-_contactInfo[i].surfaceNormal(2);
						}
					}
					else
					{
						objectHandles[0]=_contactInfo[i].objectID1;
						objectHandles[1]=_contactInfo[i].objectID2;
						contactInfo[3]=_contactInfo[i].directionAndAmplitude(0);
						contactInfo[4]=_contactInfo[i].directionAndAmplitude(1);
						contactInfo[5]=_contactInfo[i].directionAndAmplitude(2);
						if (extended)
						{
							contactInfo[6]=_contactInfo[i].surfaceNormal(0);
							contactInfo[7]=_contactInfo[i].surfaceNormal(1);
							contactInfo[8]=_contactInfo[i].surfaceNormal(2);
						}
					}
					return(true);
				}
				scanIndex++;
			}
		}
	}
	return(false);
}

void CRigidBodyContainerDyn::clearAdditionalForcesAndTorques()
{
	int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
	for (int i=0;i<shapeListSize;i++)
	{
		CDummyShape* it=(CDummyShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
		_simClearAdditionalForceAndTorque(it);
	}
}

void CRigidBodyContainerDyn::handleAdditionalForcesAndTorques()
{
	for (int i=0;i<int(_allRigidBodiesList.size());i++)
	{
		CRigidBodyDyn* body=_allRigidBodiesList[i];
		CDummyShape* shape=(CDummyShape*)_simGetObject(body->getShapeID());
        if (shape!=nullptr)
			body->handleAdditionalForcesAndTorques(shape);
	}
	C3Vector gravity;
	_simGetGravity(gravity.data);
	particleCont.handleAntiGravityForces_andFluidFrictionForces(gravity);
}
