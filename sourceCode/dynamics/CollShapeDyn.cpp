#include "CollShapeDyn.h"
#include "simLib.h"
#include "4X4FullMatrix.h"

CCollShapeDyn::CCollShapeDyn()
{
}

CCollShapeDyn::~CCollShapeDyn()
{
}

C7Vector CCollShapeDyn::getLocalInertiaFrame_scaled()
{
    return(_localInertiaFrame_scaled);
}

C7Vector CCollShapeDyn::getInverseLocalInertiaFrame_scaled()
{
    return(_inverseLocalInertiaFrame_scaled);
}

void CCollShapeDyn::setObjectID(int newID)
{
    _objectID=newID;
}

int CCollShapeDyn::getObjectID()
{
    return(_objectID);
}

void CCollShapeDyn::addRigidBodyDependency(int rigidBodyID)
{
    if (!isRigidBodyDependent(rigidBodyID)) // just in case
        _rigidBodyDependencies.push_back(rigidBodyID);
}

bool CCollShapeDyn::removeRigidBodyDependency(int rigidBodyID)
{ // return value true means this object should be removed (no more dependent rigid bodies)
    for (int i=0;i<int(_rigidBodyDependencies.size());i++)
    {
        if (_rigidBodyDependencies[i]==rigidBodyID)
        {
            _rigidBodyDependencies.erase(_rigidBodyDependencies.begin()+i);
            break;
        }
    }
    return(_rigidBodyDependencies.size()==0);
}

bool CCollShapeDyn::isRigidBodyDependent(int rigidBodyID)
{
    for (int i=0;i<int(_rigidBodyDependencies.size());i++)
    {
        if (_rigidBodyDependencies[i]==rigidBodyID)
            return(true);
    }
    return(false);
}

CDummyGeomProxy* CCollShapeDyn::getGeomData_nullForNonRespondable()
{
    return(_geomData);
}
