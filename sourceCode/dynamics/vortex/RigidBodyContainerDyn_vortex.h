#pragma once

#include "RigidBodyContainerDyn.h"
#include "Vx/VxGearRatio.h"
class VortexIntersectSubscriber;
class VortexIntersectFilter;
namespace Vx
{
	class VxUniverse;
}
struct SVortexJointDependency
{
	Vx::VxGearRatio* gear;
	CConstraintDyn* constr1;
	CConstraintDyn* constr2;
};

class CRigidBodyContainerDyn_vortex : public CRigidBodyContainerDyn
{
public:

	CRigidBodyContainerDyn_vortex();
	virtual ~CRigidBodyContainerDyn_vortex();

	int getEngineInfo(int& engine,int data1[4],char* data2,char* data3);
	void applyGravity();
	void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

	Vx::VxUniverse* getWorld();
    //void handleDemoVersion();
	void licenseCheck();

    static bool _checkingLicense;
	static void _vortexCollisionCallbackStatic(void* data,Vx::VxCollisionGeometry* o1,Vx::VxCollisionGeometry* o2);
	void _vortexCollisionCallback(void* data,Vx::VxCollisionGeometry* o1,Vx::VxCollisionGeometry* o2);

protected:
	void _stepDynamics(float dt,int pass);
	void _createDependenciesBetweenJoints();
	void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

	void _addVortexContactPoints(int dynamicPassNumber);
	VortexIntersectSubscriber* vortexIntersectSubscriber;
	Vx::VxUniverse* _vortexWorld;
	VortexIntersectFilter* vortexIntersectFilter;
	std::vector<SVortexJointDependency> _gears;
};
