#pragma once

#include "CollShapeDyn.h"
#include "ode/ode.h"

class CCollShapeDyn_ode : public CCollShapeDyn
{
public:
	CCollShapeDyn_ode(CDummyGeomProxy* geomData,dSpaceID space);
	virtual ~CCollShapeDyn_ode();

	dGeomID getOdeGeoms(int index);
	void setOdeMeshLastTransform();

protected:	
	std::vector<dGeomID> _odeGeoms; // if more than 1 element, then it is a compound object
	dTriMeshDataID _trimeshDataID;
	dHeightfieldDataID _odeHeightfieldDataID;
	dReal* _odeMeshLastTransformThingMatrix;
	unsigned char _odeMeshLastTransformThingIndex;
	std::vector<float> _odeHeightfieldData_scaled;
	std::vector<dReal> _odeConvexPlanes_scaled;
	std::vector<unsigned int> _odeConvexPolygons;
	std::vector<std::vector<dReal>* > _odeMmeshVertices_scaled;
	std::vector<std::vector<dReal>* > _odeMconvexPlanes_scaled;
	std::vector<std::vector<unsigned int>* > _odeMconvexPolygons;
};
