#include "RigidBodyDyn_newton.h"
#include "RigidBodyContainerDyn_newton.h"
#include "CollShapeDyn_newton.h"
#include "NewtonConvertUtil.h"
#include "4X4FullMatrix.h"
#include "simLib.h"

CRigidBodyDyn_newton::CRigidBodyDyn_newton(CDummyShape* const shape,CCollShapeDyn* const collShapeDyn,bool const forceStatic,bool const forceNonRespondable, NewtonWorld* const world)
{
    _rigidBodyID = -1;
    _shapeID = _simGetObjectID(shape);
    _shape = shape;
    _collisionShapeDyn = collShapeDyn;
    _simGetObjectLocalTransformation(shape, _originalLocalTransformation.X.data, _originalLocalTransformation.Q.data, false); // needed for the "parent follows"-thing!

    C7Vector cumulPart1_scaled;
    _simGetObjectCumulativeTransformation(shape, cumulPart1_scaled.X.data, cumulPart1_scaled.Q.data, true);

    C7Vector tr(cumulPart1_scaled*collShapeDyn->getLocalInertiaFrame_scaled());
    CDummyGeomProxy* geomData = (CDummyGeomProxy*)_simGetGeomProxyFromShape(shape);
    CDummyGeomWrap* geomInfo = (CDummyGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);

    _bodyIsKinematic = (_simIsShapeDynamicallyStatic(shape) || forceStatic);
    float mass = _bodyIsKinematic ? 0.0f : _simGetMass(geomInfo);

    _setNewtonParameters(shape);

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(tr));
    _newtonBody = NewtonCreateDynamicBody (world,((CCollShapeDyn_newton*)_collisionShapeDyn)->getNewtonCollision(), &matrix[0][0]);
    _newtonBodyUserData[0]=&_shapeID;
    _newtonBodyUserData[1]=this;
    _newtonBodyUserData[2]=&_newtonStaticFriction;
    _newtonBodyUserData[3]=&_newtonKineticFriction;
    _newtonBodyUserData[4]=&_newtonRestitution;

    NewtonBodySetUserData(_newtonBody, _newtonBodyUserData);

    // disable auto sleep at all time
    NewtonBodySetAutoSleep(_newtonBody, 0);

    // set linear and angular damping
    NewtonBodySetLinearDamping(_newtonBody, _newtonLinearDrag);
    dVector angularDamp (_newtonAngularDrag, _newtonAngularDrag, _newtonAngularDrag, 0.0f);
    NewtonBodySetAngularDamping(_newtonBody, &angularDamp[0]);

    // attach the CDummyGeomWrap* as user dat of the collsion shape, thsi si so that we can apply material propertes in the collision callacks 
    NewtonCollision* const collision = NewtonBodyGetCollision(_newtonBody);
    NewtonCollisionSetUserData(collision, geomInfo);

    // initialize uninitialized variables
    _bodyStart_kinematicBody.setIdentity();
    _bodyEnd_kinematicBody.setIdentity();
    m_externForce.clear();
    m_externTorque.clear();

    if (!_bodyIsKinematic)
    {
        // this function will set a full inertia matrix with origin at the center of the collision shape
        NewtonBodySetMassProperties(_newtonBody, mass,((CCollShapeDyn_newton*)_collisionShapeDyn)->getNewtonCollision());

        C3Vector im;
        _simGetPrincipalMomentOfInertia(geomInfo,im.data);
        im*=mass;
        // here inertia will be set to the principal axis, but the origin is still at the center of collision shape.
        NewtonBodySetMassMatrix(_newtonBody,mass,im(0),im(1),im(2));

        // Following not needed since the inertia tensor orientation is still relative
        // to the rigid body:
        /*
        // The COM-centered tensor:
        C3X3Matrix tensor;
        tensor.clear();
        tensor.axis[0](0)=im(0);
        tensor.axis[1](1)=im(1);
        tensor.axis[2](2)=im(2);
        tensor=localInertiaFrame.Q.getMatrix()*tensor*inverseLocalInertiaFrame.Q.getMatrix();
        // The tensor that includes the COM:
//        C3X3Matrix D;
//        D.setIdentity();
//        D*=(inverseLocalInertiaFrame.X*inverseLocalInertiaFrame.X);
//        D.axis[0]-=inverseLocalInertiaFrame.X*inverseLocalInertiaFrame.X(0);
//        D.axis[1]-=inverseLocalInertiaFrame.X*inverseLocalInertiaFrame.X(1);
//        D.axis[2]-=inverseLocalInertiaFrame.X*inverseLocalInertiaFrame.X(2);
//        tensor+=D;
        // Set the inertia tensor:
        dMatrix matrix(dVector(tensor.axis[0](0),tensor.axis[0](1),tensor.axis[0](2),0.0f),
                      dVector(tensor.axis[1](0),tensor.axis[1](1),tensor.axis[1](2),0.0f),
                      dVector(tensor.axis[2](0),tensor.axis[2](1),tensor.axis[2](2),0.0f),
                      dVector(0.0f,0.0f,0.0f,1.0f));
        NewtonBodySetFullMassMatrix (_newtonBody,mass,&matrix[0][0]);
//*/
        // Here we simply need to reset the COM to the origin of the rigid body:
        C3Vector com;
        com.clear();
        NewtonBodySetCentreOfMass (_newtonBody, com.data);

        NewtonBodySetTransformCallback(_newtonBody, TransformCallback);
        NewtonBodySetForceAndTorqueCallback(_newtonBody, ApplyExtenalForceCallback);

        C3Vector v;
        _simGetInitialDynamicVelocity(shape, v.data);
        NewtonBodySetVelocity(_newtonBody, v.data);

        _simGetInitialDynamicAngVelocity(shape, v.data);
        NewtonBodySetOmega(_newtonBody, v.data);
        _simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        _simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it

        if (_newtonFastMoving)
            NewtonBodySetContinuousCollisionMode(_newtonBody,1); // fast moving

    }

    CRigidBodyContainerDyn_newton* const rigidBodyContainerDyn = (CRigidBodyContainerDyn_newton*) NewtonWorldGetUserData(world);
    rigidBodyContainerDyn->_notifySekeletonRebuild();
}

CRigidBodyDyn_newton::~CRigidBodyDyn_newton()
{
    if (NewtonBodyGetSkeleton(_newtonBody))
    {
        NewtonWorld* const world = NewtonBodyGetWorld(_newtonBody);
        CRigidBodyContainerDyn_newton* const rigidBodyContainerDyn = (CRigidBodyContainerDyn_newton*) NewtonWorldGetUserData(world);
        rigidBodyContainerDyn->_notifySekeletonRebuild();
    }
    NewtonDestroyBody (_newtonBody);
}

C7Vector CRigidBodyDyn_newton::getInertiaFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    tr = getNewtonMatrix();
    return(tr);
}

C7Vector CRigidBodyDyn_newton::getShapeFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    if (_collisionShapeDyn!=nullptr)
    {
        tr = getNewtonMatrix();
        tr*=_collisionShapeDyn->getInverseLocalInertiaFrame_scaled();
    }
    else
        tr.setIdentity();
    return(tr);
}

NewtonBody* CRigidBodyDyn_newton::getNewtonRigidBody() const
{
    return _newtonBody;
}

void CRigidBodyDyn_newton::reportVelocityToShape(CDummyShape* shape)
{
    C3Vector lv,av;

    NewtonBodyGetOmega(_newtonBody, &av(0));
    NewtonBodyGetVelocity(_newtonBody, &lv(0));

    _simSetShapeDynamicVelocity(shape,lv.data,av.data);
}

void CRigidBodyDyn_newton::handleAdditionalForcesAndTorques(CDummyShape* shape)
{
    C3Vector vf,vt;
    _simGetAdditionalForceAndTorque(shape,vf.data,vt.data);

    m_externTorque.clear();
    m_externForce.clear();
    // In Newton bodies are never sleeping!
    if ((vf.getLength() != 0.0f) || (vt.getLength() != 0.0f))
    { // We should wake the body!!
        _bodyWasInitiallySleeping = false;
        m_externTorque=vt;
        m_externForce=vf;
    }
}

void CRigidBodyDyn_newton::reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep)
{
    if (_bodyIsKinematic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr;
        tr.buildInterpolation(_bodyStart_kinematicBody,_bodyEnd_kinematicBody,t);

        dMatrix posit;
        dQuaternion rot;
        NewtonBodyGetRotation(_newtonBody, &rot.m_q0);
        NewtonBodyGetMatrix(_newtonBody, &posit[0][0]);
        C4X4FullMatrix transform(tr.getMatrix());
        dMatrix matrix(&transform(0, 0));
        matrix = matrix.Transpose4X4();
        dQuaternion rot1 (matrix);

        float timestep = CRigidBodyContainerDyn::getDynamicsInternalTimeStep();
        dVector veloc ((matrix.m_posit - posit.m_posit).Scale (1.0f / timestep));
        dVector omega (rot.CalcAverageOmega(rot1, 1.0f / timestep));

        NewtonBodySetVelocity(_newtonBody, &veloc[0]);
        NewtonBodySetOmega(_newtonBody, &omega[0]);
        NewtonBodyIntegrateVelocity (_newtonBody, timestep);
        NewtonBodySetMatrix(_newtonBody, &matrix[0][0]);
    }
}

void CRigidBodyDyn_newton::applyCorrectEndConfig_forKinematicBody()
{
    if (_bodyIsKinematic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr(_bodyEnd_kinematicBody);

        dVector zeroVeloc (0.0f, 0.0f, 0.0f, 0.0f);
        NewtonBodySetVelocity(_newtonBody, &zeroVeloc[0]);
        NewtonBodySetOmega(_newtonBody, &zeroVeloc[0]);
    }
}

C7Vector CRigidBodyDyn_newton::getNewtonMatrix() const
{
    dMatrix matrix;
    NewtonBodyGetMatrix(_newtonBody, &matrix[0][0]);
    matrix = matrix.Transpose4X4();
    float tmp[4][4];
    memcpy(tmp, &matrix[0][0], sizeof (tmp));
    return C7Vector(tmp);
}

void CRigidBodyDyn_newton::_setNewtonParameters(CDummyShape* shape)
{
    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float floatParams[5];
    int intParams[1];
    int parVer=0;
    _simGetNewtonParameters(shape,&parVer,floatParams,intParams);

    const float staticFriction=floatParams[0];
    const float kineticFriction=floatParams[1];
    const float restitution=floatParams[2];
    const float linearDrag=floatParams[3];
    const float angularDrag=floatParams[4];

    const bool fastMoving=(intParams[0]&1)!=false;

    _newtonStaticFriction=staticFriction;
    _newtonKineticFriction=kineticFriction;
    _newtonRestitution=restitution;
    _newtonLinearDrag=linearDrag;
    _newtonAngularDrag=angularDrag;
    _newtonFastMoving=fastMoving;
}

void CRigidBodyDyn_newton::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
    //Julio: I am no sure if CoppeliaSim Collect the transforms after the update but iteration ov e bodies in a loop
}

void CRigidBodyDyn_newton::ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
{
    float mass;
    float Ixx;
    float Iyy;
    float Izz;
    C3Vector gravity;

    _simGetGravity(gravity.data);
    NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);
    gravity *= mass;

    void** userData=(void**)NewtonBodyGetUserData(body);
    CRigidBodyDyn* const me = (CRigidBodyDyn*)userData[1];
    gravity += ((CRigidBodyDyn_newton*)me)->m_externForce;
    NewtonBodyAddForce(body, &gravity.data[0]);
    NewtonBodyAddTorque(body, &((CRigidBodyDyn_newton*)me)->m_externTorque.data[0]);
/*
// for testing codename system alignments
if (mass != 0.0f) {
    dVector omega;
    NewtonBodyGetOmega (body, &omega[0]);
    dVector torque (dVector (0, 0, 1, 0) - omega.Scale (0.1f));
    NewtonBodyAddTorque(body, &torque[0]);
}
*/
}

