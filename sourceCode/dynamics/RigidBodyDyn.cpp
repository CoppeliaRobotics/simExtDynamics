#include "RigidBodyDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"
#include "4X4FullMatrix.h"

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
}

CCollShapeDyn* CRigidBodyDyn::getCollisionShapeDyn()
{
	return(_collisionShapeDyn);
}


C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{
	return(C7Vector::identityTransformation);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{
	return(C7Vector::identityTransformation);
}

void CRigidBodyDyn::reportVelocityToShape(CDummyShape* shape)
{
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques(CDummyShape* shape)
{
}

void CRigidBodyDyn::reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep)
{
}

void CRigidBodyDyn::applyCorrectEndConfig_forKinematicBody()
{
}


void CRigidBodyDyn::setDefaultActivationState(int defState)
{
	_defaultActivationState=defState;
}

void CRigidBodyDyn::setRigidBodyID(int newID)
{
	_rigidBodyID=newID;
}

int CRigidBodyDyn::getRigidBodyID()
{
	return(_rigidBodyID);
}

bool CRigidBodyDyn::isBodyKinematic()
{
	return(_bodyIsKinematic);
}

int CRigidBodyDyn::getShapeID()
{
	return(_shapeID);
}

CDummyShape* CRigidBodyDyn::getShape()
{
	return(_shape);
}

void CRigidBodyDyn::reportConfigurationToShape(CDummyShape* shape)
{
	if (_simGetDynamicsFullRefreshFlag(shape)!=0)
		return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
	if (!_bodyIsKinematic)
	{ // dynamic
		if (_simGetParentFollowsDynamic(shape)!=0)
		{
			CDummy3DObject* it=(CDummy3DObject*)_simGetParentObject(shape);
            if (it!=nullptr)
			{
				C7Vector tr(getShapeFrameTransformation()*_originalLocalTransformation.getInverse());
				_simSetObjectCumulativeTransformation(it,tr.X.data,tr.Q.data,false);
			}
		}
		C7Vector tr(getShapeFrameTransformation());
		_simSetObjectCumulativeTransformation(shape,tr.X.data,tr.Q.data,false);
	}
}

void CRigidBodyDyn::calculateBodyToShapeTransformation_forKinematicBody(CDummyShape* shape,float dt)
{
	if (_bodyIsKinematic)
	{
		_bodyStart_kinematicBody=getInertiaFrameTransformation();
		C7Vector aax(_collisionShapeDyn->getLocalInertiaFrame_scaled());
		aax.X/=CRigidBodyContainerDyn::getPositionScalingFactorDyn(); // ********** SCALING

		C7Vector cumulTrp1;
		_simGetObjectCumulativeTransformation(shape,cumulTrp1.X.data,cumulTrp1.Q.data,true);
		_bodyEnd_kinematicBody=cumulTrp1*aax;
		_applyBodyToShapeTransf_kinematicBody=( ((_bodyStart_kinematicBody.X-_bodyEnd_kinematicBody.X).getLength()>0.00001f)||(_bodyStart_kinematicBody.Q.getAngleBetweenQuaternions(_bodyEnd_kinematicBody.Q)>0.05f*degToRad) ); // 0.01 to 0.05 on 26/11/2011 because the minGW compilation produces larger imprecisions!
		
		C3Vector dx;
		_simGetInitialDynamicVelocity(shape,dx.data);
		dx*=dt;
		C3Vector dxa;
		_simGetInitialDynamicAngVelocity(shape,dxa.data);
		if (dx.getLength()+dxa.getLength()>0.0f)
		{
			_simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it!
			_simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it!
			C7Vector tr;
			_simGetObjectCumulativeTransformation(shape,tr.X.data,tr.Q.data,true);
			tr.X+=dx;
			if (dxa.getLength()>0.0f)
			{ // following new since 25/03/2013
				C4Vector q(dxa(0),dxa(1),dxa(2));
				C4Vector angleAndAxis(q.getAngleAndAxis());
				q.setAngleAndAxis(angleAndAxis(0)*dt,C3Vector(angleAndAxis(1),angleAndAxis(2),angleAndAxis(3)));
				tr.Q=tr.Q*q;
			}
			_bodyEnd_kinematicBody=tr*aax;
			_applyBodyToShapeTransf_kinematicBody=true;
		}
	}
}
