#pragma once

#include "dummyClasses.h"
#include "CollShapeDyn.h"
#include "7Vector.h"

class CRigidBodyDyn  
{
public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    virtual C7Vector getInertiaFrameTransformation();
    virtual C7Vector getShapeFrameTransformation();
    virtual void reportVelocityToShape(CDummyShape* shape);
    virtual void handleAdditionalForcesAndTorques(CDummyShape* shape);
    virtual void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
    virtual void applyCorrectEndConfig_forKinematicBody();

    int getRigidBodyID();
    void setRigidBodyID(int newID);
    int getShapeID();
    CDummyShape* getShape();
    bool isBodyKinematic();
    CCollShapeDyn* getCollisionShapeDyn();
    void reportConfigurationToShape(CDummyShape* shape);
    void setDefaultActivationState(int defState);
    void calculateBodyToShapeTransformation_forKinematicBody(CDummyShape* shape,float dt);

protected:    
    int _rigidBodyID;
    CDummyShape* _shape;
    int _shapeID;
    int _defaultActivationState;
    CCollShapeDyn* _collisionShapeDyn;
    bool _bodyWasInitiallySleeping;

    C7Vector _originalLocalTransformation; // unscaled!!

    // Following used by calculateBodyToShapeTransformation and reportShapeConfigurationToRigidBody
    C7Vector _bodyStart_kinematicBody; // unscaled!
    C7Vector _bodyEnd_kinematicBody; // unscaled!
    bool _applyBodyToShapeTransf_kinematicBody;

    bool _bodyIsKinematic;
};
