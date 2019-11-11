#pragma once

#include "RigidBodyDyn.h"
#include "CollShapeDyn.h"
#include "ConstraintDyn.h"
#include "ParticleContainer.h"
#include "dummyClasses.h"
#include "3Vector.h"
#include <vector>
#include <string>

struct SContactInfo
{
	int subPassNumber;
	int objectID1;
	int objectID2;
	C3Vector position;
	C3Vector surfaceNormal;
	C3Vector directionAndAmplitude;
};

class CRigidBodyContainerDyn  
{
public:
	CRigidBodyContainerDyn();
	virtual ~CRigidBodyContainerDyn();

	virtual int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
	virtual void applyGravity();
	virtual void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

	void removeRigidBodyFromCollisionShapeDependency(int rigidBodyID);
	CRigidBodyDyn* getRigidBodyFromShapeID(int shapeID);
	CConstraintDyn* getConstraintFromJointID(int jointID);
	CConstraintDyn* getConstraintFromDummyID(int dummyID);
	CConstraintDyn* getConstraintFromForceSensorID(int forceSensorID);
	CCollShapeDyn* getCollisionShapeFromGeomObject(CDummyGeomProxy* geomData);

	void handleDynamics(float dt,float simulationTime);
	bool isDynamicContentAvailable();

	void reportDynamicWorldConfiguration(int totalPassesCount,bool doNotApplyJointIntrinsicPositions,float simulationTime);

	void reportRigidBodyConfigurationsAndVelocities();
	void reportConstraintConfigurations(int totalPassesCount,bool doNotApplyJointIntrinsicPositions);
	void reportConstraintSecondPartConfigurations();

	void handleAdditionalForcesAndTorques();
	void clearAdditionalForcesAndTorques();

	bool getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float* contactInfo);

	int getDynamicsCalculationPasses();

	float* getContactPoints(int* cnt);

	float gravityVectorLength; // updated when handleDynamics is called

	static CParticleContainer particleCont;
	static CRigidBodyContainerDyn* currentRigidBodyContainerDynObject;

	static void setPositionScalingFactorDyn(float f);
	static float getPositionScalingFactorDyn();
	static void setLinearVelocityScalingFactorDyn(float f);
	static float getLinearVelocityScalingFactorDyn();
	static void setMassScalingFactorDyn(float f);
	static float getMassScalingFactorDyn();
	static void setMasslessInertiaScalingFactorDyn(float f);
	static float getMasslessInertiaScalingFactorDyn();
	static void setForceScalingFactorDyn(float f);
	static float getForceScalingFactorDyn();
	static void setTorqueScalingFactorDyn(float f);
	static float getTorqueScalingFactorDyn();
	static void setGravityScalingFactorDyn(float f);
	static float getGravityScalingFactorDyn();
	static void setDynamicActivityRange(float f);
	static float getDynamicActivityRange();
	static void setDynamicsInternalTimeStep(float dt);
	static float getDynamicsInternalTimeStep();
	static void setDynamicParticlesIdStart(int a);
	static int getDynamicParticlesIdStart();
	static void set3dObjectIdStart(int a);
	static int get3dObjectIdStart();
	static void set3dObjectIdEnd(int a);
	static int get3dObjectIdEnd();

protected:
	virtual void _stepDynamics(float dt,int pass);
	virtual void _createDependenciesBetweenJoints();
	virtual void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

	bool _updateDynamicWorld();

	void _temporarilyDisableShapesOutOfDynamicActivityRange();
	void _tagStaticAndNotCollidableButDynamicShapes();
	void _updateRigidBodiesFromSceneShapes();
	void _updateConstraintsFromSceneJoints();
	void _updateConstraintsFromSceneForceSensors();
	void _updateConstraintsFromSceneDummies();
	void _updateHybridJointTargetPositions();


	void _addOrUpdateRigidBody(CDummyShape* shape,bool forceStatic,bool forceNonRespondable);
	void _addOrUpdateJointConstraint(CDummyJoint* joint);
	void _addOrUpdateDummyConstraint(CDummyDummy* dummy);
	void _addOrUpdateForceSensorConstraint(CDummyForceSensor* forceSensor);
	void _handleMotorControls(int passCnt,int totalPasses);

	int _addRigidBody(CRigidBodyDyn* body);
	void _removeRigidBody(int rigidBodyID);

	// Following 3 go in pair:
	void _calculateBodyToShapeTransformations_forKinematicBodies(float dt);
	void reportShapeConfigurations_forKinematicBodies(float t,float cumulatedTimeStep);
	void _applyCorrectEndConfig_forKinematicBodies();

	void _announceToConstraintsBodyWillBeDestroyed(int rigidBodyID);
	void _removeConstraintFromIndex(int indexPos);

	std::vector<CRigidBodyDyn*> _allRigidBodiesList;
	std::vector<CRigidBodyDyn*> _allRigidBodiesIndex;
	std::vector<CConstraintDyn*> _allConstraintsList;
	std::vector<CConstraintDyn*> _allConstraintsIndex;

	std::vector<CCollShapeDyn*> _allCollisionShapes;

	int _nextRigidBodyID;

	// Following 2 updated when handleDynamics is called:
	float _timeStepPassedToHandleDynamicsFunction;
	int _dynamicsCalculationPasses;

	std::vector<float> _contactPoints;
	std::vector<SContactInfo> _contactInfo; // Not same as above!

	static float _positionScalingFactorDyn;
	static float _linearVelocityScalingFactorDyn;
	static float _massScalingFactorDyn;
	static float _masslessInertiaScalingFactorDyn;
	static float _forceScalingFactorDyn;
	static float _torqueScalingFactorDyn;
	static float _gravityScalingFactorDyn;
	static float _dynamicActivityRange;
	static float _dynamicsInternalStepSize;
	static int _dynamicParticlesIdStart;
	static int _3dObjectIdStart;
	static int _3dObjectIdEnd;
};
