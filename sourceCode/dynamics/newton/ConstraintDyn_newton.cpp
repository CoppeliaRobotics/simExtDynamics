#include "ConstraintDyn_newton.h"
#include "RigidBodyContainerDyn_newton.h"
#include "RigidBodyDyn_newton.h"
#include "simLib.h"
#include "4X4FullMatrix.h"
#include "NewtonConvertUtil.h"

class CConstraintDyn_newton::csimNewtonForceSensorJoint: public CustomHinge
{
    public:
    csimNewtonForceSensorJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomHinge(pinAndPivotFrame, child, parent)
        , m_broken(false)
    {
        EnableLimits(true);
        CustomHinge::SetLimits(0.0f, 0.0f);
    }

    void SubmitConstraints(dFloat timestep, int threadIndex)
    {
        if (!m_broken) {
            CustomHinge::SubmitConstraints(timestep, threadIndex);
        }
    }

    C3Vector GetForce() const
    {
        return C3Vector(-NewtonUserJointGetRowForce(m_joint, 0), -NewtonUserJointGetRowForce(m_joint, 1), -NewtonUserJointGetRowForce(m_joint, 2));
    }

    C3Vector GetTorque() const
    {
        return C3Vector(-NewtonUserJointGetRowForce(m_joint, 5), -NewtonUserJointGetRowForce(m_joint, 3), -NewtonUserJointGetRowForce(m_joint, 4));
    }

    bool m_broken;
};

class CConstraintDyn_newton::csimNewtonCommonJointData
{
    public:
    enum JointMode
    {
        m_free,
        m_motor,
        m_position,
    };

    csimNewtonCommonJointData()
        :m_joint(nullptr)
        ,m_lowLimit(-1.0e10f)
        ,m_highLimit(1.0e10f)
        ,m_lowForce(1.0e10f)
        ,m_highForce(1.0e10f)
        ,m_motorSpeed(0.0f)
        ,m_targetPosition(0.0f)
        ,m_axisWasActive(false)
        ,m_lockOnTargetMode(false)
        ,m_mode(m_free)
    {
    }

    dynReal GetJointForce() const
    {
        return m_axisWasActive ? -NewtonUserJointGetRowForce(m_joint->GetJoint(), 5) : 0.0f;
    }

    void SetLimits(dynReal low, dynReal high)
    {
        m_lowLimit = low;
        m_highLimit = high;
    }

    void SetMotor(bool motorIsOn, dynReal speed, dynReal maxForce)
    {
        m_lockOnTargetMode = false;
        if (motorIsOn) {
            m_motorSpeed = speed;
            m_highForce = fabsf(maxForce);
            m_lowForce = -m_highForce;
            m_mode = m_motor;
        }
        else
        {
            m_mode = m_free;
        }
    }

    void LockOnTargetPosition(dynReal targetPosition)
    {
        m_lockOnTargetMode = true;
        m_targetPosition = targetPosition;
        m_mode = m_position;
    }

    CustomJoint* m_joint;
    dynReal m_lowLimit;
    dynReal m_highLimit;
    dynReal m_lowForce;
    dynReal m_highForce;
    dynReal m_motorSpeed;
    dynReal m_targetPosition;
    bool m_axisWasActive;
    bool m_lockOnTargetMode;
    JointMode m_mode;
};

class CConstraintDyn_newton::csimNewtonRevoluteJoint: public CustomHingeActuator
{
    public:
    csimNewtonRevoluteJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomHingeActuator(pinAndPivotFrame, child, parent)
        ,m_data()
    {
        m_data.m_joint = this;
        EnableLimits(true);
        CustomHinge::SetLimits(-1.0e10f, 1.0e10f);
        SetFriction(0.0f);

        // JULIO: By default joints disable collision on the object that the link, in general the application handle this by limits,
        // collision can be enable by calling the function which can be a joint option.
        // for now I am assuming they alway collide
        SetBodiesCollisionState(true);
    }

    void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
    {
        m_data.m_axisWasActive = false;
        //dTrace (("%f %f\n", GetJointAngle(), GetJointOmega()));

        switch (m_data.m_mode)
        {
            case csimNewtonCommonJointData::m_free:
            {
                CustomHinge::SetLimits(m_data.m_lowLimit, m_data.m_highLimit);
                CustomHinge::SubmitConstraintsFreeDof(timestep, matrix0, matrix1);
                m_data.m_axisWasActive = m_lastRowWasUsed;
                break;
            }

            case csimNewtonCommonJointData::m_position:
            {
                // I did not test this, but I believe is right
                m_data.m_axisWasActive = true;
                dynReal posit = GetJointAngle();
                dynReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_targetPosition, posit);
                NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                NewtonUserJointSetRowStiffness(m_joint, 1.0f);
                break;
            }

            case csimNewtonCommonJointData::m_motor:
            default:
            {
                dynReal posit = GetJointAngle();
                dynReal speed = GetJointOmega();
                dynReal accel = 0.5f * (m_data.m_motorSpeed - speed) / timestep;
                if (posit <= m_data.m_lowLimit)
                {
                    dynReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_lowLimit, posit);
                    NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed > 0.0f) && (posit1 > m_data.m_lowLimit))
                    if (m_data.m_motorSpeed > 0.0f)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else if (posit >= m_data.m_highLimit)
                {
                    dynReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_highLimit, posit);
                    NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed < 0.0f) && (posit1 > m_data.m_highLimit))
                    if (m_data.m_motorSpeed < 0.0f)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else
                {
                    NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
                    NewtonUserJointSetRowAcceleration(m_joint, accel);
                    NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                    NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                }
                NewtonUserJointSetRowStiffness(m_joint, 1.0f);
                m_data.m_axisWasActive = true;
                break;
            }
        }
    }

    csimNewtonCommonJointData m_data;
};

class CConstraintDyn_newton::csimNewtonPrismaticJoint: public CustomSliderActuator
{
    public:
    csimNewtonPrismaticJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomSliderActuator(pinAndPivotFrame, child, parent)
        ,m_data()
    {
        m_data.m_joint = this;
        EnableLimits(true);
        CustomSlider::SetLimits(-1.0e10f, 1.0e10f);

        // JULIO: By default joints disable collision on the object that the link, in general the application handle this by limits,
        // collision can be enable by calling the function which can be a joint option.
        // for now I am assuming they alway collide
        SetBodiesCollisionState(true);
    }

    void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
    {
        m_data.m_axisWasActive = false;
        //dTrace (("%f %f %f\n", GetJointPosit(), GetJointSpeed(), (m_data.m_motorSpeed - GetJointSpeed()) / timestep));
        switch (m_data.m_mode)
        {
            case csimNewtonCommonJointData::m_free:
            {
                CustomSlider::SetLimits(m_data.m_lowLimit, m_data.m_highLimit);
                CustomSlider::SubmitConstraintsFreeDof(timestep, matrix0, matrix1);
                m_data.m_axisWasActive = m_lastRowWasUsed;
                break;
            }
            case csimNewtonCommonJointData::m_position:
            {
                // I did not test this, but I believe is right
                m_data.m_axisWasActive = true;
                dynReal posit = GetJointPosit();
                dynReal step = m_data.m_targetPosition - posit;
                const dVector& p0 = matrix0.m_posit;
                dVector p1(p0 + matrix0.m_front.Scale(step));
                NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                NewtonUserJointSetRowStiffness(m_joint, 1.0f);
                break;
            }

            case csimNewtonCommonJointData::m_motor:
            default:
            {
                dynReal posit = GetJointPosit();
                dynReal speed = GetJointSpeed();
                dynReal accel = 0.5f * (m_data.m_motorSpeed - speed) / timestep;
                if (posit <= m_data.m_lowLimit)
                {
                    dynReal step = m_data.m_lowLimit - posit;
                    const dVector& p0 = matrix0.m_posit;
                    dVector p1(p0 + matrix0.m_front.Scale(step));
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed > 0.0f) && (posit1 > m_data.m_lowLimit))
                    if (m_data.m_motorSpeed > 0.0f)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else if (posit >= m_data.m_highLimit)
                {
                    dynReal step = m_data.m_highLimit - posit;
                    const dVector& p0 = matrix0.m_posit;
                    dVector p1(p0 + matrix0.m_front.Scale(step));
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed < 0.0f) && (posit1 < m_data.m_highLimit))
                    if (m_data.m_motorSpeed < 0.0f)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else
                {
                    const dVector& p0 = matrix0.m_posit;
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p0[0], &matrix0.m_front[0]);
                    NewtonUserJointSetRowAcceleration(m_joint, accel);
                    NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                    NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                }
                NewtonUserJointSetRowStiffness(m_joint, 1.0f);
                m_data.m_axisWasActive = true;
                break;
            }
        }
    }
    csimNewtonCommonJointData m_data;
};

CConstraintDyn_newton::CConstraintDyn_newton(CRigidBodyDyn* parentBody, CRigidBodyDyn* childBody, CDummyJoint* joint, NewtonWorld* world)
{
    _parentBody = parentBody;
    _childBody = childBody;
    _jointOrForceSensorLoopClosureLinkedDummyAChildID = -1; // not used (non-looped case)
    _jointOrForceSensorLoopClosureLinkedDummyBChildID = -1; // not used (non-looped case)
    _nonCyclicRevoluteJointPositionOffset = 0.0f;
    _jointIsCyclic = _simGetJointPositionInterval(joint, &_nonCyclicRevoluteJointPositionMinimum, &_nonCyclicRevoluteJointPositionRange)==0;
    _dummyID = -1;
    _forceSensorID = -1;
    _constraintID = -1;
    _jointID = _simGetObjectID(joint);
    _lastEffortOnJoint = 0.0f;
    _dynPassCount = 0;
    _lastJointPosSet = false;
    _targetPositionToHoldAtZeroVelOn_velocityMode = false;

    // Following 4 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    CDummy3DObject* parentShape = (CDummy3DObject*)_simGetObject(parentBody->getShapeID());
    CDummy3DObject* childShape = (CDummy3DObject*)_simGetObject(childBody->getShapeID());
    parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
    childBody->reportConfigurationToShape((CDummyShape*)childShape);

    _simGetObjectLocalTransformation(joint, _initialLocalTransform.X.data, _initialLocalTransform.Q.data, true);
    C7Vector tmpTr1;
    C7Vector tmpTr2;
    _simGetObjectCumulativeTransformation(childShape, tmpTr1.X.data, tmpTr1.Q.data, false);
    _simGetObjectCumulativeTransformation(joint, tmpTr2.X.data, tmpTr2.Q.data, false);
    _secondInitialLocalTransform = tmpTr1.getInverse()*tmpTr2;

    _parentShapeID = parentBody->getShapeID();
    _childShapeID = childBody->getShapeID();

    _newtonJointOffset = _simGetJointPosition(joint);

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint, jtr.X.data, jtr.Q.data, true);

    // in Newton the moving direction is the x-axis
    C3X3Matrix m;
    m.buildYRotation(-piValue*0.5f);
    jtr.Q=jtr.Q*m.getQuaternion();

    // wake up the parent body and disable deactivation for future:
    //NewtonBodySetSleepState(parentBody->getNewtonRigidBody(),1);
    //NewtonBodySetFreezeState(parentBody->getNewtonRigidBody(),0);
    //NewtonBodySetAutoSleep(parentBody->getNewtonRigidBody(),0);

    // wake up the child body and disable deactivation for future:
    //NewtonBodySetSleepState(childBody->getNewtonRigidBody(),1);
    //NewtonBodySetFreezeState(childBody->getNewtonRigidBody(),0);
    //NewtonBodySetAutoSleep(childBody->getNewtonRigidBody(),0);

    NewtonBody* parentRigidBody=((CRigidBodyDyn_newton*)parentBody)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn_newton*)childBody)->getNewtonRigidBody();

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    switch (_simGetJointType(joint))
    {
        case sim_joint_spherical_subtype:
        {
            _newtonConstraint = new CustomBallAndSocket(matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }

        case sim_joint_prismatic_subtype:
        {
            _newtonConstraint = new csimNewtonPrismaticJoint(matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }

        case sim_joint_revolute_subtype:
        {
            _newtonConstraint = new csimNewtonRevoluteJoint (matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }
        default:
        {
            _ASSERTE (0);
        }
    }

    _bodyAID = parentBody->getRigidBodyID();
    _bodyBID = childBody->getRigidBodyID();
    _isAcyclic = true;
    _setNewtonParameters(joint);
    _notifySekeletonRebuild();
    handleMotorControl(joint, 0, 0);
}

CConstraintDyn_newton::CConstraintDyn_newton(CRigidBodyDyn* parentBody, CRigidBodyDyn* childBody, CDummyJoint* joint, CDummyDummy* loopClosureDummyA, CDummyDummy* loopClosureDummyB, NewtonWorld* world)
{
    _parentBody = parentBody;
    _childBody = childBody;
    _jointOrForceSensorLoopClosureLinkedDummyAChildID = _simGetObjectID(loopClosureDummyA);
    _jointOrForceSensorLoopClosureLinkedDummyBChildID = _simGetObjectID(loopClosureDummyB);
    _nonCyclicRevoluteJointPositionOffset = 0.0f;
    _jointIsCyclic = _simGetJointPositionInterval(joint, &_nonCyclicRevoluteJointPositionMinimum, &_nonCyclicRevoluteJointPositionRange) == 0;
    _dummyID = -1;
    _forceSensorID = -1;
    _constraintID = -1;
    _jointID = _simGetObjectID(joint);
    _lastEffortOnJoint = 0.0f;
    _dynPassCount = 0;
    _lastJointPosSet = false;
    _targetPositionToHoldAtZeroVelOn_velocityMode = false;


    // Following 4 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    CDummy3DObject* parentShape = (CDummy3DObject*)_simGetObject(parentBody->getShapeID());
    CDummy3DObject* childShape = (CDummy3DObject*)_simGetObject(childBody->getShapeID());
    parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
    childBody->reportConfigurationToShape((CDummyShape*)childShape);

    _simGetObjectLocalTransformation(joint, _initialLocalTransform.X.data, _initialLocalTransform.Q.data, true);

//    C7Vector tmpTr1;
//    C7Vector tmpTr2;
//    _simGetObjectCumulativeTransformation(childShape, tmpTr1.X.data, tmpTr1.Q.data, false);
//    _simGetObjectCumulativeTransformation(joint, tmpTr2.X.data, tmpTr2.Q.data, false);
//    _secondInitialLocalTransform = tmpTr1.getInverse()*tmpTr2;

    _simGetObjectLocalTransformation(loopClosureDummyA, _linkedDummyAInitialLocalTransform.X.data, _linkedDummyAInitialLocalTransform.Q.data, false);
    _simGetObjectLocalTransformation(loopClosureDummyB, _linkedDummyBInitialLocalTransform.X.data, _linkedDummyBInitialLocalTransform.Q.data, false);

    // Following is DIFFERENT from the regular situation (non-looped):
    _simGetObjectLocalTransformation(loopClosureDummyB, _secondInitialLocalTransform.X.data, _secondInitialLocalTransform.Q.data, false);

    _parentShapeID = parentBody->getShapeID();
    _childShapeID = childBody->getShapeID();

    _newtonJointOffset =0.0f;// _simGetJointPosition(joint);


    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint, jtr.X.data, jtr.Q.data, true);

    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(loopClosureDummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_linkedDummyAInitialLocalTransform.getInverse());


    C3X3Matrix m;
    m.buildYRotation(-piValue*0.5f);
    jtr.Q = jtr.Q*m.getQuaternion();


    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
            _newtonJointOffset =_simGetJointPosition(joint);
    //        jointOffsetThing.buildZRotation(_newtonJointOffset);
        }
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
    }
    else
        jtr2.Q = jtr2.Q*m.getQuaternion();
    dMatrix jmatrix(GetDMatrixFromCoppeliaSimTransformation(jtr));

//----
    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;
    _bodyAID=-1;
    _bodyBID=-1;

    batr=parentBody->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;
    _bodyAID=parentBody->getRigidBodyID();

    bbtr=childBody->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;
    _bodyBID=childBody->getRigidBodyID();
//----

//----

    NewtonBody* parentRigidBody=((CRigidBodyDyn_newton*)parentBody)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn_newton*)childBody)->getNewtonRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
    dMatrix matrixT=matrixI.Transpose4X4();
    float tmp[4][4];
    memcpy(tmp,&matrixT[0][0],sizeof(tmp));
    C7Vector cb_a(tmp);
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);
//----

    switch (_simGetJointType(joint))
    {
        case sim_joint_spherical_subtype:
        {
            _newtonConstraint = new CustomBallAndSocket(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }

        case sim_joint_prismatic_subtype:
        {
            _newtonConstraint = new csimNewtonPrismaticJoint(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }

        case sim_joint_revolute_subtype:
        {
            _newtonConstraint = new csimNewtonRevoluteJoint(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }
        default:
        {
              _ASSERTE(0);
        }
    }

    //----
    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);
    //----

    _bodyAID = parentBody->getRigidBodyID();
    _bodyBID = childBody->getRigidBodyID();
    _isAcyclic = false;
    _setNewtonParameters(joint);
    _notifySekeletonRebuild();
    handleMotorControl(joint, 0, 0);
}

CConstraintDyn_newton::CConstraintDyn_newton(CRigidBodyDyn* parentBody, CRigidBodyDyn* childBody, CDummyDummy* dummyOnParent, CDummyDummy* dummyOnChild, NewtonWorld* world)
{ // This is a rigid link between 2 rigid bodies involved in a loop closure (i.e. body1 - dummy1 - dummy2 - body2)
    _parentBody=parentBody;
    _childBody=childBody;
    _jointOrForceSensorLoopClosureLinkedDummyAChildID=-1; // not used here
    _jointOrForceSensorLoopClosureLinkedDummyBChildID=-1; // not used here

    _nonCyclicRevoluteJointPositionOffset=0.0f;
    _jointID=-1;
    _forceSensorID=-1;
    _constraintID=-1;
    _dummyID=_simGetObjectID(dummyOnParent);
    _linkedDummyID=_simGetObjectID(dummyOnChild);
    _lastEffortOnJoint=0.0f;
    _dynPassCount=0;
    _lastJointPosSet=false;
    _jointIsCyclic=false;
    _targetPositionToHoldAtZeroVelOn_velocityMode=false;

    _simGetObjectLocalTransformation(dummyOnParent,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
    _simGetObjectLocalTransformation(dummyOnChild,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data,true);

    _parentShapeID=parentBody->getShapeID();
    _childShapeID=childBody->getShapeID();

    // Following 4 lines are important when a dummy is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    CDummy3DObject* parentShape=(CDummy3DObject*)_simGetObject(parentBody->getShapeID());
    CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(childBody->getShapeID());
    parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
    childBody->reportConfigurationToShape((CDummyShape*)childShape);

    C7Vector dtr,dtr2;
    _simGetObjectCumulativeTransformation(dummyOnParent,dtr.X.data,dtr.Q.data,true);
    _simGetObjectCumulativeTransformation(dummyOnChild,dtr2.X.data,dtr2.Q.data,true);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector dtrRelToBodyA;
    C7Vector dtrRelToBodyB;
    _bodyAID=-1;
    _bodyBID=-1;

    _bodyAID=parentBody->getRigidBodyID();

    _bodyBID=childBody->getRigidBodyID();


    NewtonBody* parentRigidBody=((CRigidBodyDyn_newton*)parentBody)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn_newton*)childBody)->getNewtonRigidBody();

    // Set the configuration of the child as if the dummies were overlapping:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
    dMatrix matrixT=matrixI.Transpose4X4();
    float tmp[4][4];
    memcpy(tmp,&matrixT[0][0],sizeof(tmp));
    C7Vector cb_a(tmp);
    C7Vector x(cb_a.getInverse()*dtr2);
    C7Vector cb_b(dtr*x.getInverse());
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);


    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(dtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);

    _notifySekeletonRebuild();
    _isAcyclic = false;

}

CConstraintDyn_newton::CConstraintDyn_newton(CRigidBodyDyn* parentBody, CRigidBodyDyn* childBody, CDummyForceSensor* forceSensor, NewtonWorld* world)
{
    _parentBody = parentBody;
    _childBody = childBody;
    _jointOrForceSensorLoopClosureLinkedDummyAChildID = -1; // not used (non-looped case)
    _jointOrForceSensorLoopClosureLinkedDummyBChildID = -1; // not used (non-looped case)

    _nonCyclicRevoluteJointPositionOffset = 0.0f;
    _dummyID = -1;
    _constraintID = -1;
    _jointID = -1;
    _forceSensorID = _simGetObjectID(forceSensor);
    _lastEffortOnJoint = 0.0f;
    _dynPassCount = 0;
    _lastJointPosSet = false;
    _jointIsCyclic=false;
    _targetPositionToHoldAtZeroVelOn_velocityMode = false;

    // Following 4 lines are important when a force sensor is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    CDummy3DObject* parentShape = (CDummy3DObject*)_simGetObject(parentBody->getShapeID());
    CDummy3DObject* childShape = (CDummy3DObject*)_simGetObject(childBody->getShapeID());
    parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
    childBody->reportConfigurationToShape((CDummyShape*)childShape);

    _simGetObjectLocalTransformation(forceSensor, _initialLocalTransform.X.data, _initialLocalTransform.Q.data, true);
    C7Vector tmpTr1, tmpTr2;
    _simGetObjectCumulativeTransformation(childShape, tmpTr1.X.data, tmpTr1.Q.data, false);
    _simGetObjectCumulativeTransformation(forceSensor, tmpTr2.X.data, tmpTr2.Q.data, false);
    _secondInitialLocalTransform = tmpTr1.getInverse()*tmpTr2;

    // since 2010/02/13:
    if (_simIsForceSensorBroken(forceSensor))
    {
        _simGetDynamicForceSensorLocalTransformationPart2(forceSensor, tmpTr1.X.data, tmpTr1.Q.data);
        _secondInitialLocalTransform *= tmpTr1;
    }

    _parentShapeID = parentBody->getShapeID();
    _childShapeID = childBody->getShapeID();

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(forceSensor,jtr2.X.data,jtr2.Q.data,false);

    // since 2010/02/13
    if (_simIsForceSensorBroken(forceSensor))
    {
        _simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
        jtr2*=tmpTr1;
    }

    NewtonBody* parentRigidBody=((CRigidBodyDyn_newton*)parentBody)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn_newton*)childBody)->getNewtonRigidBody();

    // wake up the parent body and disable deactivation for future:
    NewtonBodySetSleepState(parentRigidBody,1);
    NewtonBodySetFreezeState(parentRigidBody,0);
    NewtonBodySetAutoSleep(parentRigidBody,0);

    // wake up the child body and disable deactivation for future:
    NewtonBodySetSleepState(childRigidBody,1);
    NewtonBodySetFreezeState(childRigidBody,0);
    NewtonBodySetAutoSleep(childRigidBody,0);

    _bodyAID = parentBody->getRigidBodyID();
    _bodyBID = childBody->getRigidBodyID();

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    _isAcyclic = true;
    _setForceSensorBrokenUnbrokenConstraints_newton (forceSensor);
}

CConstraintDyn_newton::CConstraintDyn_newton(CRigidBodyDyn* parentBody, CRigidBodyDyn* childBody, CDummyForceSensor* forceSensor, CDummyDummy* loopClosureDummyA, CDummyDummy* loopClosureDummyB, NewtonWorld* world)
{
    _parentBody = parentBody;
    _childBody = childBody;
    _jointOrForceSensorLoopClosureLinkedDummyAChildID = _simGetObjectID(loopClosureDummyA);
    _jointOrForceSensorLoopClosureLinkedDummyBChildID = _simGetObjectID(loopClosureDummyB);

    _nonCyclicRevoluteJointPositionOffset = 0.0f;
    _jointIsCyclic = false;
    _dummyID = -1;
    _constraintID = -1;
    _jointID = -1;
    _forceSensorID=_simGetObjectID(forceSensor);
    _lastEffortOnJoint = 0.0f;
    _dynPassCount = 0;
    _lastJointPosSet = false;
    _targetPositionToHoldAtZeroVelOn_velocityMode = false;
    _newtonJointOffset =0.0f;


    // Following 4 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    CDummy3DObject* parentShape = (CDummy3DObject*)_simGetObject(parentBody->getShapeID());
    CDummy3DObject* childShape = (CDummy3DObject*)_simGetObject(childBody->getShapeID());
    parentBody->reportConfigurationToShape((CDummyShape*)parentShape);
    childBody->reportConfigurationToShape((CDummyShape*)childShape);

    _simGetObjectLocalTransformation(forceSensor,_initialLocalTransform.X.data,_initialLocalTransform.Q.data,true);
    _simGetObjectLocalTransformation(loopClosureDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data,false);
    _simGetObjectLocalTransformation(loopClosureDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data,false);


    // Following is DIFFERENT from the regular situation (non-looped):
    _simGetObjectLocalTransformation(loopClosureDummyB,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data,false);

    // Following is not needed for looped cases (_secondInitialLocalTransform is not used in that case), but anyway:
    if (_simIsForceSensorBroken(forceSensor)) // since 2010/02/13
    {
        C7Vector tmpTr1;
        _simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
        _secondInitialLocalTransform*=tmpTr1;
    }

    _parentShapeID=parentBody->getShapeID();
    _childShapeID=childBody->getShapeID();

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(loopClosureDummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_linkedDummyAInitialLocalTransform.getInverse());

    // since 2010/02/13
    if (_simIsForceSensorBroken(forceSensor))
    {
        C7Vector tmpTr1;
        _simGetDynamicForceSensorLocalTransformationPart2(forceSensor,tmpTr1.X.data,tmpTr1.Q.data);
        jtr2*=tmpTr1;
    }

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;
    _bodyAID=-1;
    _bodyBID=-1;

    batr=parentBody->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;
    _bodyAID=parentBody->getRigidBodyID();

    bbtr=childBody->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;
    _bodyBID=childBody->getRigidBodyID();

    NewtonBody* parentRigidBody=((CRigidBodyDyn_newton*)parentBody)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn_newton*)childBody)->getNewtonRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
    dMatrix matrixT=matrixI.Transpose4X4();
    float tmp[4][4];
    memcpy(tmp,&matrixT[0][0],sizeof(tmp));
    C7Vector cb_a(tmp);
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);
//----

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    //----
    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);
    //----

    _notifySekeletonRebuild();
    _setForceSensorBrokenUnbrokenConstraints_newton(forceSensor);
    _ASSERTE (loopClosureDummyA && loopClosureDummyB);
    _isAcyclic = false;
}

CConstraintDyn_newton::~CConstraintDyn_newton()
{
    _notifySekeletonRebuild();
    delete _newtonConstraint;
}

void CConstraintDyn_newton::_setRevoluteJointLimits(CDummyJoint* joint)
{
    csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*)_newtonConstraint;
    if (_simGetJointPositionInterval(joint, nullptr, nullptr) == 0)
        jointClass->m_data.SetLimits(-1.0e10f, 1.0e10f);
    else
        jointClass->m_data.SetLimits(_nonCyclicRevoluteJointPositionMinimum-_newtonJointOffset,_nonCyclicRevoluteJointPositionMinimum+_nonCyclicRevoluteJointPositionRange-_newtonJointOffset);
}

void CConstraintDyn_newton::_setPrismaticJointLimits(CDummyJoint* joint)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:
    float jiMin,jiRange;
    _simGetJointPositionInterval(joint,&jiMin,&jiRange);

    csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*)_newtonConstraint;
    if (_simGetJointPositionInterval(joint, nullptr, nullptr) == 0)
        jointClass->m_data.SetLimits(-1.0e10f, 1.0e10f);
    else
    {
        _ASSERTE(jiMin <= 0.0f);
        _ASSERTE(jiMin + jiRange >= 0.0f);
//            jointClass->m_data.SetLimits(jiMin * linScaling, (jiMin + jiRange) * linScaling);
        jointClass->m_data.SetLimits(jiMin-_newtonJointOffset,jiMin+jiRange-_newtonJointOffset);
    }
}

void CConstraintDyn_newton::_handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
    float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

    csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*)_newtonConstraint;

    _targetPositionToHoldAtZeroVelOn_velocityMode=false;

    float e;

    if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
        e=getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint), getHingeAngle());
    else
        e=_simGetDynamicMotorTargetPosition(joint)-getHingeAngle();

    int auxV=0;
    if (_dynPassCount==0)
        auxV|=1;
    int inputValuesInt[5];
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    inputValuesInt[2]=0; // reserved for future ext.
    inputValuesInt[3]=0; // reserved for future ext.
    inputValuesInt[4]=0; // reserved for future ext.
    float inputValuesFloat[7];

    inputValuesFloat[0]=getHingeAngle();

    inputValuesFloat[1]=_lastEffortOnJoint;
    inputValuesFloat[2]=dynStepSize;
    inputValuesFloat[3]=e;
    inputValuesFloat[4]=0.0f; // reserved for future ext.
    inputValuesFloat[5]=0.0f; // reserved for future ext.
    inputValuesFloat[6]=0.0f; // reserved for future ext.
    float outputValues[5];
    _simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    float velocityToApply=outputValues[0];
    float forceToApply=outputValues[1];

    jointClass->m_data.SetMotor(true,velocityToApply, forceToApply);
}


void CConstraintDyn_newton::_handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
    csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*)_newtonConstraint;

    int param=0;
    simGetObjectIntParameter(_jointID,2030,&param);
    bool lockModeWhenInVelocityControl=(param!=0);

    float vel=_simGetDynamicMotorTargetVelocity(joint);

    if ((lockModeWhenInVelocityControl) && (vel == 0.0f) && (_simIsDynamicMotorEnabled(joint) != 0))
    {
        if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
            _targetPositionToHoldAtZeroVel_velocityMode = ((CustomHinge*)_newtonConstraint)->GetJointAngle();
//            _targetPositionToHoldAtZeroVel_velocityMode = getHingeAngle()-_newtonJointOffset;
        _targetPositionToHoldAtZeroVelOn_velocityMode = true;
        jointClass->m_data.LockOnTargetPosition(_targetPositionToHoldAtZeroVel_velocityMode);
    }
    else
    {
        _targetPositionToHoldAtZeroVelOn_velocityMode = false;
        jointClass->m_data.SetMotor(_simIsDynamicMotorEnabled(joint)!=0,vel, _simGetDynamicMotorMaxForce(joint));
    }
}

void CConstraintDyn_newton::_handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
    float dynStepSize=CRigidBodyContainerDyn::getDynamicsInternalTimeStep();

    csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*)_newtonConstraint;

    _targetPositionToHoldAtZeroVelOn_velocityMode=false;
    float e=_simGetDynamicMotorTargetPosition(joint)-getSliderPositionScaled();

    int auxV=0;
    if (_dynPassCount==0)
        auxV|=1;
    int inputValuesInt[5];
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    inputValuesInt[2]=0; // reserved for future ext.
    inputValuesInt[3]=0; // reserved for future ext.
    inputValuesInt[4]=0; // reserved for future ext.
    float inputValuesFloat[7];
    inputValuesFloat[0]=getSliderPositionScaled();
    inputValuesFloat[1]=_lastEffortOnJoint;
    inputValuesFloat[2]=dynStepSize;
    inputValuesFloat[3]=e;
    inputValuesFloat[4]=0.0f; // reserved for future ext.
    inputValuesFloat[5]=0.0f; // reserved for future ext.
    inputValuesFloat[6]=0.0f; // reserved for future ext.
    float outputValues[5];
    _simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    float velocityToApply=outputValues[0];
    float forceToApply=outputValues[1];

    jointClass->m_data.SetMotor(true,velocityToApply, forceToApply);
}

void CConstraintDyn_newton::_handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
    csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*)_newtonConstraint;

    int param=0;
    simGetObjectIntParameter(_jointID,2030,&param);
    bool lockModeWhenInVelocityControl=(param!=0);
    float vel=_simGetDynamicMotorTargetVelocity(joint);

    if ((lockModeWhenInVelocityControl) && (vel == 0.0f) && (_simIsDynamicMotorEnabled(joint) != 0))
    {
        if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
            _targetPositionToHoldAtZeroVel_velocityMode = getSliderPositionScaled()-_newtonJointOffset;
        _targetPositionToHoldAtZeroVelOn_velocityMode = true;
        jointClass->m_data.LockOnTargetPosition(_targetPositionToHoldAtZeroVel_velocityMode);
    }
    else
    {
        _targetPositionToHoldAtZeroVelOn_velocityMode = false;
        jointClass->m_data.SetMotor(_simIsDynamicMotorEnabled(joint)!=0,vel,_simGetDynamicMotorMaxForce(joint));
    }
}

dynReal CConstraintDyn_newton::getSliderPositionScaled()
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))
    csimNewtonPrismaticJoint* const slider = (csimNewtonPrismaticJoint*) _newtonConstraint;
    return slider->GetJointPosit()+_newtonJointOffset;
}

dynReal CConstraintDyn_newton::getHingeAngle()
{
    dynReal retVal=(dynReal)0.0;
    if (true)
    { // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
        dynReal jointPos=(dynReal)0.0;

        CustomHinge* const hinge = (CustomHinge*) _newtonConstraint;
        jointPos = hinge->GetJointAngle();
        _lastJointPosSet = false;
        retVal = jointPos + _newtonJointOffset;
//        printf("%f (offset: %f)\n",retVal*180.0f/piValue,_newtonJointOffset*180.0f/piValue);
    }
    return(retVal);
}

void CConstraintDyn_newton::reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{ // totalPassesCount is 0 when we need to accumulate the values, and diff. from zero when we need to average!
    // If totalPassesCount is -1, then we do not report the forces and torques!
    if (_simGetDynamicsFullRefreshFlag(forceSensor))
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (_forceSensorID!=-1)
    {
        _simSetObjectLocalTransformation(forceSensor,_initialLocalTransform.X.data,_initialLocalTransform.Q.data);

        if (linkedDummyA!=nullptr)
        { // special case (looped)
            _simSetObjectLocalTransformation(linkedDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data);
            _simSetObjectLocalTransformation(linkedDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data);
        }

        // Now report forces and torques acting on the force sensor:
        C3Vector forces;
        forces.clear();
        C3Vector torques;
        torques.clear();

        if (!_simIsForceSensorBroken(forceSensor))
        {
            csimNewtonForceSensorJoint* const sensor = (csimNewtonForceSensorJoint*) _newtonConstraint;
            forces = sensor->GetForce();
            torques = sensor->GetTorque();
        }

        _simAddForceSensorCumulativeForcesAndTorques(forceSensor,forces.data,torques.data,totalPassesCount);

        if (totalPassesCount>0)
            _setForceSensorBrokenUnbrokenConstraints_newton (forceSensor);
    }
}

void CConstraintDyn_newton::reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{ // totalPassesCount is 0 when we need to accumulate the values, and diff. from zero when we need to average!
    // If totalPassesCount is -1, then we do not report the forces and torques!
    if (_simGetDynamicsFullRefreshFlag(joint))
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (_jointID!=-1)
    {
        // Is following correct for joints?? Dummies are actualized after joints! Decided to keep this on 1/6/2011
        if (linkedDummyA!=nullptr)
        { // special case (looped)
            _simSetObjectLocalTransformation(linkedDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data);
            _simSetObjectLocalTransformation(linkedDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data);
        }

        // Now report forces and torques acting on the joint:
        float forceOrTorque=0.0f;

        if (_simGetJointType(joint)==sim_joint_revolute_subtype)
        {
            csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*) _newtonConstraint;
            forceOrTorque = jointClass->m_data.GetJointForce();
        }
        else if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
        {
            csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*) _newtonConstraint;
            forceOrTorque = jointClass->m_data.GetJointForce();
        }

        _lastEffortOnJoint=forceOrTorque;
        _simAddJointCumulativeForcesOrTorques(joint,forceOrTorque,totalPassesCount);
    }
}

void CConstraintDyn_newton::_setForceSensorBrokenUnbrokenConstraints_newton(CDummyForceSensor* forceSensor)
{
    csimNewtonForceSensorJoint* const joint = (csimNewtonForceSensorJoint*)_newtonConstraint;
    joint->m_broken = _simIsForceSensorBroken(forceSensor) ? true : false;
}

CRigidBodyDyn* CConstraintDyn_newton::_getChild() const
{
    return _childBody;
}

CRigidBodyDyn* CConstraintDyn_newton::_getParent() const
{
    return _parentBody;
}


bool CConstraintDyn_newton::_isAcyclicJoint() const
{
    return _isAcyclic;
}

bool CConstraintDyn_newton::getNewtonDependencyInfo(int& linkedJoint,float& fact,float& off)
{
    if (_newtonDependencyJointId==-1)
        return(false);
    linkedJoint=_newtonDependencyJointId;
    fact=_newtonDependencyFact;
    off=_newtonDependencyOff;
    _newtonDependencyJointId=-1; // to indicate that it was processed
    return(true);
}

CustomJoint* CConstraintDyn_newton::_getNewtonJoint() const
{
    return _newtonConstraint;
}

void CConstraintDyn_newton::_notifySekeletonRebuild()
{
    CRigidBodyContainerDyn_newton* const rigidBodyContainerDyn = (CRigidBodyContainerDyn_newton*) NewtonWorldGetUserData(NewtonBodyGetWorld (_newtonConstraint->GetBody0()));
    rigidBodyContainerDyn->_notifySekeletonRebuild();
}

void CConstraintDyn_newton::_setNewtonParameters(CDummyJoint* joint)
{
    int jointType=_simGetJointType(joint);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float floatParams[2];
    int intParams[2];
    int parVer=0;
    _simGetNewtonParameters(joint,&parVer,floatParams,intParams);

    const float dependencyFactor=floatParams[0];
    const float dependencyOffset=floatParams[1];

    const int dependencyJointA_ID=intParams[0];
    const int dependencyJointB_ID=intParams[1]; // -1 if no dependent joint

    if (jointType==sim_joint_revolute_subtype)
    {
        // TODO_NEWTON:
    }

    if (jointType==sim_joint_prismatic_subtype)
    {
        // TODO_NEWTON:
    }

    if (jointType==sim_joint_spherical_subtype)
    {
        // TODO_NEWTON:
    }

    // TODO_NEWTON:
    // Store information about a dependent joint here. Constraint creation for that happens in the _createDependenciesBetweenJoints
    _newtonDependencyJointId=dependencyJointB_ID;
    _newtonDependencyFact=dependencyFactor;
    _newtonDependencyOff=dependencyOffset;
}

