#include "CollShapeDyn_newton.h"
#include "simLib.h"
#include "4X4FullMatrix.h"
#include "NewtonConvertUtil.h"

CCollShapeDyn_newton::CCollShapeDyn_newton(CDummyShape* shape,CDummyGeomProxy* geomData,bool willBeStatic, NewtonWorld* const world)
{
    _geomData=geomData;
    _localInertiaFrame_scaled.setIdentity();
    _inverseLocalInertiaFrame_scaled.setIdentity();

    CDummyGeomWrap* geomInfo=(CDummyGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);

    _simGetLocalInertiaFrame(geomInfo,_localInertiaFrame_scaled.X.data,_localInertiaFrame_scaled.Q.data);
    _inverseLocalInertiaFrame_scaled=_localInertiaFrame_scaled.getInverse();

    // Do we have a pure primitive?
    int primType=_simGetPurePrimitiveType(geomInfo);
    if (primType!=sim_pure_primitive_none)
    {
        // We have a pure primitive here:
        if (!_simIsGeomWrapGeometric(geomInfo))
        {
            // We a have a pure MULTISHAPE!!
            int componentListSize=_simGetGeometricCount(geomInfo);
            CDummyGeometric** componentList=new CDummyGeometric*[componentListSize];
            _simGetAllGeometrics(geomInfo,(simVoid**)componentList);

            _shape = NewtonCreateCompoundCollision (world, 0);
            NewtonCompoundCollisionBeginAddRemove (_shape);    
            for (int i=0;i<componentListSize;i++)
            {
                CDummyGeometric* sc=componentList[i];
                int pType=_simGetPurePrimitiveType(sc);
                float hollowScaling=_simGetPureHollowScaling(sc);
                if (hollowScaling!=0.0f)
                    _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);
                C3Vector s;
                _simGetPurePrimitiveSizes(sc,s.data);
                //s*=linScaling; // ********** SCALING

                C7Vector aax;
                // for pure shapes, the vertice frame also indicates the pure shape origin
                _simGetVerticesLocalFrame(sc, aax.X.data, aax.Q.data); 
                C7Vector xxx(_inverseLocalInertiaFrame_scaled * aax);

                int subshapeType =_simGetPurePrimitiveType(sc);
                NewtonCollision* childShape = nullptr;
                switch (subshapeType) 
                {
                    case sim_pure_primitive_plane:
                    case sim_pure_primitive_cuboid:
                    {
                        dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(xxx));
                        float z=s(2);
                        if (z<0.0001f)
                            z=0.0001f;
                        childShape = NewtonCreateBox(world, s(0), s(1), z, 0, &localTransform[0][0]);
                        break;
                    }

                    case sim_pure_primitive_disc:
                    case sim_pure_primitive_cylinder:
                    {
                        C3X3Matrix rot;
                        rot.buildYRotation(piValue/2.0f);
                        C7Vector ilif(xxx);
                        ilif.Q*=rot.getQuaternion();
                        dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(ilif));
                        float z=s(2);
                        if (z<0.0001f)
                            z=0.0001f;
                        //childShape = NewtonCreateChamferCylinder(world, s(0) * 0.5f, z, 0, &localTransform[0][0]);
                        childShape = NewtonCreateCylinder(world, s(0) * 0.5f, z, 0, &localTransform[0][0]);
                        break;
                    }

                    case sim_pure_primitive_spheroid:
                    {
                        if ( ( ((s(0)-s(1))/s(0))>0.01f )||( ((s(0)-s(2))/s(0))>0.01f ) ) // Pure spheroids are not (yet) supported by ODE
                            _simMakeDynamicAnnouncement(sim_announce_purespheroidnotsupported);
                        dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(xxx));
                        childShape = NewtonCreateSphere(world, s(0) * 0.5f, 0, &localTransform[0][0]);
                        break;
                    }

                    case sim_pure_primitive_cone:
                    {
                        C3X3Matrix rot;
                        rot.buildYRotation(-piValue/2.0f);
                        C7Vector ilif(xxx);
                        ilif.Q*=rot.getQuaternion();
                        dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(ilif));
                        childShape = NewtonCreateCone(world,s(0) * 0.5f,s(2),0,&localTransform[0][0]);
                        break;
                    }
                    default:
                        _ASSERTE(0);
                }
        
                NewtonCompoundCollisionAddSubCollision (_shape, childShape);
                NewtonDestroyCollision(childShape);
            }
            delete[] componentList;
            NewtonCompoundCollisionEndAddRemove (_shape);    
        }
        else
        { 
            // we have a SIMPLE pure shape
            float hollowScaling=_simGetPureHollowScaling((CDummyGeometric*)geomInfo);
            if (hollowScaling!=0.0f)
                _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);
            C3Vector s;
            _simGetPurePrimitiveSizes(geomInfo,s.data);

            switch (primType)
            {

                case sim_pure_primitive_plane:
                case sim_pure_primitive_cuboid:
                {
                    dMatrix invMatrix (GetDMatrixFromCoppeliaSimTransformation(_inverseLocalInertiaFrame_scaled));
                    float z=s(2);
                    if (z<0.0001f)
                        z=0.0001f;
                    _shape = NewtonCreateBox (world, s(0), s(1), z, 0, &invMatrix[0][0]);
                    break;
                }
                
                case sim_pure_primitive_disc:
                case sim_pure_primitive_cylinder:
                {
                    C3X3Matrix rot;
                    rot.buildYRotation(piValue/2.0f);
                    C7Vector ilif(_inverseLocalInertiaFrame_scaled);
                    ilif.Q*=rot.getQuaternion();
                    dMatrix invMatrix (GetDMatrixFromCoppeliaSimTransformation(ilif));
                    float z=s(2);
                    if (z<0.0001f)
                        z=0.0001f;
                    // _shape = NewtonCreateChamferCylinder(world, s(0) * 0.5f, z, 0, &invMatrix[0][0]);
                    _shape = NewtonCreateCylinder(world, s(0) * 0.5f, z, 0, &invMatrix[0][0]);
                    break;
                }

                case sim_pure_primitive_spheroid:
                {
                    if ( ( ((s(0)-s(1))/s(0))>0.01f )||( ((s(0)-s(2))/s(0))>0.01f ) ) // Pure spheroids are not (yet) supported by ODE
                        _simMakeDynamicAnnouncement(sim_announce_purespheroidnotsupported);
                    dMatrix invMatrix (GetDMatrixFromCoppeliaSimTransformation(_inverseLocalInertiaFrame_scaled));
                    _shape = NewtonCreateSphere(world, s(0) * 0.5f, 0,&invMatrix[0][0]);
                    break;
                }

                case sim_pure_primitive_cone:
                {
                    C3X3Matrix rot;
                    rot.buildYRotation(-piValue/2.0f);
                    C7Vector ilif(_inverseLocalInertiaFrame_scaled);
                    ilif.Q*=rot.getQuaternion();
                    dMatrix invMatrix (GetDMatrixFromCoppeliaSimTransformation(ilif));
                    _shape = NewtonCreateCone(world, s(0) * 0.5f, s(2), 0, &invMatrix[0][0]);
                    break;
                }

                case sim_pure_primitive_heightfield:
                {
                    int xCnt,yCnt; // height values along x or y
                    float minH,maxH; // min and max heights (relative to frame)
                    // Heightfield x and y size is: s(0) and s(1)
                    // Heightfield pad x-size is: s(0)/(float(xCnt-1))
                    // Heightfield pad y-size is: s(1)/(float(yCnt-1))
                    const float* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH);
                    // hData contains xCnt*yCnt heights in following order: x0y0, x1,y0, ..., xn,y0,x0y1,x1y1, ...
                    _newtonHeightfieldData.resize(xCnt*yCnt);
                    for (int i=0;i<xCnt;i++)
                    {
                        for (int j=0;j<yCnt;j++)
                            _newtonHeightfieldData[(xCnt-1-i)+xCnt*j]=hData[i+xCnt*j];
                    }
                    C3X3Matrix rot1;
                    rot1.buildXRotation(-piValue/2.0f);
                    C3X3Matrix rot2;
                    rot2.buildZRotation(piValue);
                    C7Vector ilif;
                    ilif.setIdentity();
                    ilif.Q=rot2.getQuaternion()*ilif.Q;
                    ilif.Q=rot1.getQuaternion()*ilif.Q;
                    ilif.X+=C3Vector(s(0),0.0f,s(1))*0.5f;
                    ilif.inverse();
                    dMatrix invMatrix (GetDMatrixFromCoppeliaSimTransformation(ilif));
                    char* attributeMap=new char[xCnt*yCnt];
                    for (int i=0;i<xCnt*yCnt;i++)
                        attributeMap[i]=0;
                    _shape = NewtonCreateHeightFieldCollision(world,xCnt,yCnt,1,0,&_newtonHeightfieldData[0],attributeMap,1.0f,s(0)/(float(xCnt-1)),0);
                    NewtonCollisionSetMatrix(_shape,&invMatrix[0][0]);
                    break;
                }

                default:
                    _ASSERTE (0);
            }
        }
    }
    else
    {
        // Here we have either:
        // 1. a random shape/multishape
        // 2. a convex shape
        // 3. a convex multishape
        if ((_simIsGeomWrapConvex(geomInfo)==0)&&willBeStatic) // in Newton, random meshes can only be static! If not static, treat them as convex meshes
        {     // We have a general-type geom object (trimesh)
            float* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
            _meshIndices.assign(allIndices,allIndices+allIndicesSize);

            for (int i=0;i<allVerticesSize/3;i++)
            { // We need to take into account the position of the inertia frame
                C3Vector v(allVertices+3*i+0);
                //v*=linScaling; // ********** SCALING
                v*=_inverseLocalInertiaFrame_scaled;
                _meshVertices_scaled.push_back(v(0));
                _meshVertices_scaled.push_back(v(1));
                _meshVertices_scaled.push_back(v(2));
            }

            _shape = NewtonCreateTreeCollision(world, 0);
            NewtonTreeCollisionBeginBuild(_shape);
            for (int i = 0; i < int(_meshIndices.size()); i += 3)
            {
                float triangle[3][3];
                for (int j=0; j < 3; j++)
                {
                    int index = _meshIndices[i + j];
                    triangle[j][0] = _meshVertices_scaled[index * 3 + 0];
                    triangle[j][1] = _meshVertices_scaled[index * 3 + 1];
                    triangle[j][2] = _meshVertices_scaled[index * 3 + 2];
                }
                NewtonTreeCollisionAddFace (_shape, 3, &triangle[0][0], 3 * sizeof (float), 0);
            }
            NewtonTreeCollisionEndBuild (_shape, 0);


            simReleaseBuffer((simChar*)allVertices);
            simReleaseBuffer((simChar*)allIndices);
        }
        else
        { // We have a convex shape or multishape:
            if (_simIsGeomWrapConvex(geomInfo)==0)
            { // this shape is NOT convex. In Newton, only static non-convex shapes are supported. So treat non-convex dynamic shapes as convex and output a warning:
                _simMakeDynamicAnnouncement(sim_announce_newtondynamicrandommeshnotsupported);
            }

            if (!_simIsGeomWrapGeometric(geomInfo))
            { // We a have a convex MULTISHAPE!!
                int componentListSize=_simGetGeometricCount(geomInfo);
                CDummyGeometric** componentList = new CDummyGeometric*[componentListSize];
                _simGetAllGeometrics(geomInfo,(simVoid**)componentList);

                _shape = NewtonCreateCompoundCollision(world, 0);
                NewtonCompoundCollisionBeginAddRemove(_shape);
                for (int comp=0;comp<componentListSize;comp++)
                {
                    CDummyGeometric* sc=componentList[comp];

                    float* allVertices;
                    int allVerticesSize;
                    int* allIndices;
                    int allIndicesSize;
                    _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                    _meshVertices_scaled.clear();

                    C3Vector c;// this is the inside point
                    c.clear();
                    for (int i=0;i<allVerticesSize/3;i++)
                    {
                        C3Vector v(allVertices+3*i);
                        //v*=linScaling; // ********** SCALING
                        c+=v;
                    }
                    c/=float(allVerticesSize/3);

                    C7Vector tr;
                    tr.setIdentity();
                    tr.X=c;
                    C7Vector _inverseLocalInertiaFrame2_scaled(_inverseLocalInertiaFrame_scaled*tr);
                    dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(_inverseLocalInertiaFrame2_scaled));

                    for (int i=0;i<allVerticesSize/3;i++)
                    { // We need to take into account the position of the inertia frame
                        C3Vector v(allVertices+3*i+0);
                        //v*=linScaling; // ********** SCALING
                        v-=c; // we recenter the convex mesh (will be corrected further down)
                        _meshVertices_scaled.push_back(v(0));
                        _meshVertices_scaled.push_back(v(1));
                        _meshVertices_scaled.push_back(v(2));
                    }
                    simReleaseBuffer((simChar*)allVertices);
                    simReleaseBuffer((simChar*)allIndices);

                    NewtonCollision* const childShape = NewtonCreateConvexHull (world, _meshVertices_scaled.size()/3, &_meshVertices_scaled[0], sizeof(float)*3, 1.0e-3f, 0, &localTransform[0][0]);
                    NewtonCompoundCollisionAddSubCollision(_shape, childShape);
                    NewtonDestroyCollision(childShape);
                }
                delete[] componentList;
                NewtonCompoundCollisionEndAddRemove(_shape);
            }
            else
            { // We have a convex SHAPE
                float* allVertices;
                int allVerticesSize;
                int* allIndices;
                int allIndicesSize;
                _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                _meshIndices.assign(allIndices,allIndices+allIndicesSize);

                C3Vector c;
                c.clear();
                for (int i=0;i<allVerticesSize/3;i++)
                {
                    C3Vector v(allVertices+3*i);
                    //v*=linScaling; 
                    c+=v;
                }
                c/=float(allVerticesSize/3);

                C7Vector tr;
                tr.setIdentity();
                tr.X=c;
                C7Vector _inverseLocalInertiaFrame2_scaled(_inverseLocalInertiaFrame_scaled*tr);
                dMatrix localTransform(GetDMatrixFromCoppeliaSimTransformation(_inverseLocalInertiaFrame2_scaled));

                for (int i=0;i<allVerticesSize/3;i++)
                { // We need to take into account the position of the inertia frame
                    C3Vector v(allVertices+3*i+0);
                    //v*=linScaling; // ********** SCALING
                    v-=c; 
                    _meshVertices_scaled.push_back(v(0));
                    _meshVertices_scaled.push_back(v(1));
                    _meshVertices_scaled.push_back(v(2));
                }
                simReleaseBuffer((simChar*)allVertices);
                simReleaseBuffer((simChar*)allIndices);
                _shape = NewtonCreateConvexHull (world, _meshVertices_scaled.size()/3, &_meshVertices_scaled[0], sizeof(float)*3, 1.0e-3f, 0, &localTransform[0][0]);
            }
        }
    }
    _setNewtonParameters(shape);
}

CCollShapeDyn_newton::~CCollShapeDyn_newton()
{
    NewtonDestroyCollision (_shape);
}

void CCollShapeDyn_newton::_setNewtonParameters(CDummyShape* shape)
{ // This can probably be left empty! (same routine in CRigidBodyDyn)
/*
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
    */
}

NewtonCollision* CCollShapeDyn_newton::getNewtonCollision()
{
    return _shape;
}
