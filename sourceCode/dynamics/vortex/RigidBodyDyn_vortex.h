#pragma once

#include "RigidBodyDyn.h"
namespace Vx
{
	class VxPart;
	class VxUniverse;
	class VxIntersectFilter;
}

class CRigidBodyDyn_vortex : public CRigidBodyDyn
{
public:
	CRigidBodyDyn_vortex(CDummyShape* shape,CCollShapeDyn* collShapeDyn,bool forceStatic,bool forceNonRespondable,Vx::VxUniverse* vortexWorld,int aRandomValue); // VORTEX
	virtual ~CRigidBodyDyn_vortex();

	Vx::VxPart* getVortexRigidBody();

	// Following values accessed by callbacks:
	double vortex_skinThickness;
	bool vortex_autoSlip;
	double vortex_angularVelocityDamping;
	double vortex_autoAngularDampingTensionRatio;
	static void setVortexFilter(Vx::VxIntersectFilter* filter) { _sVortexIntersectFilter = filter; }

	C7Vector getInertiaFrameTransformation();
	C7Vector getShapeFrameTransformation();
	void reportVelocityToShape(CDummyShape* shape);
	void handleAdditionalForcesAndTorques(CDummyShape* shape);
	void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
	void applyCorrectEndConfig_forKinematicBody();

protected:	
	Vx::VxPart* _vortexRigidBody;
    static Vx::VxIntersectFilter* _sVortexIntersectFilter;
};
