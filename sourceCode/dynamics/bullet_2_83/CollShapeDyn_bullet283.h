#pragma once

#include "CollShapeDyn.h"
#include "btBulletDynamicsCommon.h"

class CCollShapeDyn_bullet283 : public CCollShapeDyn
{
public:
	CCollShapeDyn_bullet283(CDummyShape* shape,CDummyGeomProxy* geomData);
	virtual ~CCollShapeDyn_bullet283();

	btCollisionShape* getBtCollisionShape();

protected:	
	btTriangleIndexVertexArray* _indexVertexArrays; // for meshes
	std::vector<btCollisionShape*> _compoundChildShapes;
	btCollisionShape* _collisionShape;
};
