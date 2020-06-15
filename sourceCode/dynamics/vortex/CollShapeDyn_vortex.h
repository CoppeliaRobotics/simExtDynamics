#pragma once

#include "CollShapeDyn.h"
#include "Vx/VxCollisionGeometry.h"

namespace Vx
{
    class VxGeometry;
    class VxCollisionGeometry;
    class VxUniverse;
    class VxTriangleMeshBVTree;
    class VxTriangleMeshUVGrid;
    class VxConvexMesh;
}

class CCollShapeDyn_vortex : public CCollShapeDyn
{
public:
    CCollShapeDyn_vortex(CDummyShape* shape,CDummyGeomProxy* geomData, Vx::VxUniverse* universe); // VORTEX
    virtual ~CCollShapeDyn_vortex();
    Vx::VxCollisionGeometry* getVortexGeoms(int index);

protected:    
    Vx::VxGeometry* _createVortexSimpleGeometry(int pType, const C3Vector& s,float hollowScaling, CDummyGeomWrap* geomInfo, float linScaling);
    Vx::VxCollisionGeometry* _createVortexCollisionGeometry(Vx::VxUniverse* universe,CDummyGeomWrap* geomInfo,Vx::VxGeometry* vxGeometry,const Vx::VxTransform& vxTransform,const float* floatParams,const int* intParams);
    std::vector<Vx::VxCollisionGeometry*> _vortexGeoms; // if more than 1 element, then it is a compound object
    Vx::VxTriangleMeshBVTree* _createVortexBVTreeMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling);
    Vx::VxTriangleMeshUVGrid* _createVortexUVGridMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling);
    std::vector<float> _vortexConvexPlanes_scaled;
    std::vector<unsigned int> _vortexConvexPolygons;
    std::vector<std::vector<float>* > _vortexMmeshVertices_scaled;
    std::vector<std::vector<float>* > _vortexMconvexPlanes_scaled;
    std::vector<std::vector<unsigned int>* > _vortexMconvexPolygons;
};
