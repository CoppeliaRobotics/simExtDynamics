#pragma once

#include "CollShapeDyn.h"
#include "btBulletDynamicsCommon.h"

class CCollShapeDyn_bullet278 : public CCollShapeDyn
{
public:
    CCollShapeDyn_bullet278(CDummyGeomProxy* geomData); // Bullet
    virtual ~CCollShapeDyn_bullet278();

    btCollisionShape* getBtCollisionShape();

protected:    
    btTriangleIndexVertexArray* _indexVertexArrays; // for meshes
    std::vector<btCollisionShape*> _compoundChildShapes;
    btCollisionShape* _collisionShape;
};
