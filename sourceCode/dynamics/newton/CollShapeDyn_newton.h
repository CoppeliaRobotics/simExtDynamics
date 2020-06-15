#pragma once

#include "CollShapeDyn.h"
#include "Newton.h"
#include "dMatrix.h"
#include "CustomJoint.h"
#include "CustomHinge.h"
#include "CustomSlider.h"
#include "CustomBallAndSocket.h"
#include "CustomHingeActuator.h"
#include "CustomSliderActuator.h"

class CCollShapeDyn_newton : public CCollShapeDyn
{
public:
    CCollShapeDyn_newton(CDummyShape* shape,CDummyGeomProxy* geomData,bool willBeStatic, NewtonWorld* const world); // newton
    virtual ~CCollShapeDyn_newton();

    NewtonCollision* getNewtonCollision();

protected:    
    std::vector<float> _newtonHeightfieldData;
    NewtonCollision* _shape;
    void _setNewtonParameters(CDummyShape* shape);
};
