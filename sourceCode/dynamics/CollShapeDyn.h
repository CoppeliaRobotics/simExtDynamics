#pragma once

#include "dummyClasses.h"
#include <vector>
#include "7Vector.h"

class CCollShapeDyn  
{
public:
	CCollShapeDyn();
	virtual ~CCollShapeDyn();

	int getObjectID();
	void setObjectID(int newID);
	void addRigidBodyDependency(int rigidBodyID);
	bool removeRigidBodyDependency(int rigidBodyID);
	bool isRigidBodyDependent(int rigidBodyID);
	C7Vector getLocalInertiaFrame_scaled();
	C7Vector getInverseLocalInertiaFrame_scaled();
	CDummyGeomProxy* getGeomData_nullForNonRespondable();

protected:	
	int _objectID;
	C7Vector _localInertiaFrame_scaled;
	C7Vector _inverseLocalInertiaFrame_scaled;
	CDummyGeomProxy* _geomData;
	std::vector<int> _rigidBodyDependencies;
	std::vector<dynReal> _meshVertices_scaled;
	std::vector<int> _meshIndices;
};
