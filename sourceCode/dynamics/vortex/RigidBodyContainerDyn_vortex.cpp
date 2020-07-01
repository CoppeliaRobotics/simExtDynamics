#include "RigidBodyContainerDyn_vortex.h"
#include "CollShapeDyn_vortex.h"
#include "RigidBodyDyn_vortex.h"
#include "ConstraintDyn_vortex.h"
#include "simLib.h"
#include <iostream>
#include "Vx/VxFrame.h"
#include "Vx/VxUniverse.h"
#include "Vx/VxCollisionGeometry.h"
#include "Vx/VxPart.h"
#include "Vx/VxIntersectResult.h"
#include "Vx/VxDynamicsContactInput.h"
#include "Vx/VxDynamicsContact.h"
#include "Vx/VxContactMaterial.h"
#include "VortexConvertUtil.h"
#include "Vx/VxTriangleMeshBVTree.h"
#include "Vx/VxTriangleMeshUVGrid.h"
#include "Vx/VxSolverParameters.h"
#include "Vx/VxConstraint.h"
#include "Vx/VxRequest.h"
#include "Vx/VxIntersectFilter.h"
#include "Vx/VxMessage.h"
#include "Vx/VxPrismatic.h"
#include "Vx/VxVersion.h"
#include "Vx/VxBox.h"
#include "Vx/VxRigidBodyResponseModel.h"
#include "Vx/VxMaterialTable.h"
#ifdef LIN_SIM
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#endif

bool VxFrameReleased = true; // debug to use universe print content
bool gPrintContact = false;
bool gPrintConstraintDebug = false;
unsigned int vortexStartTimeTag;
bool CRigidBodyContainerDyn_vortex::_checkingLicense=false;

/* Those globals values are used to self parameterize Vortex so that it is stable without having to
 * tweak values.
 */
//const bool sAutoAngularDamping = true;
//const float sSkinThickness = 0.002;
//const bool sAutoSlip = true;
const Vx::VxReal spScaleL = 1; // scales default linear solver parameters
const Vx::VxReal spScaleA = 1; // scales default angular solver parameters

class VortexIntersectFilter : public Vx::VxIntersectFilter
{
public:
    bool isPairEnabled(Vx::VxCollisionGeometry* cg0, Vx::VxCollisionGeometry* cg1)
    {
        bool canCollide = true;
        CRigidBodyContainerDyn_vortex::_vortexCollisionCallbackStatic((void*)&canCollide, cg0, cg1);
        return canCollide;
    }

};
static int VortexResponsePart = 0;// Vx::VxUniverse::kResponsePart; TODO: why the compiler doesn't find this

class VortexIntersectSubscriber : public Vx::VxUniverse::IntersectSubscriber
{
public:
    virtual void notifyDisjoint(Vx::VxUniverse::eIntersectEventType , Vx::VxIntersectResult* )
    {
        // don't need
    }
    virtual void notifyIntersect(Vx::VxUniverse::eIntersectEventType type, Vx::VxIntersectResult* ires, Vx::VxDynamicsResponseInput* dres)
    {
        if (CRigidBodyContainerDyn_vortex::_checkingLicense)
            return;
        Vx::VxCollisionGeometry* cg[2];
        ires->getCollisionGeometryPair(cg, cg+1);
        Vx::VxPart* part[2];
        ires->getPartPair(part, part+1);

        CRigidBodyDyn* body1=(CRigidBodyDyn*)part[0]->userData().getData("csim1").getPointerVoid();
        CRigidBodyDyn* body2=(CRigidBodyDyn*)part[1]->userData().getData("csim1").getPointerVoid();
        double skin1=0.0;
        if (body1!=nullptr)
            skin1=((CRigidBodyDyn_vortex*)body1)->vortex_skinThickness;
        double skin2=0.0;
        if (body2!=nullptr)
            skin2=((CRigidBodyDyn_vortex*)body2)->vortex_skinThickness;
        bool autoSlip=(((body1!=nullptr)&&((CRigidBodyDyn_vortex*)body1)->vortex_autoSlip)||((body2!=nullptr)&&((CRigidBodyDyn_vortex*)body2)->vortex_autoSlip));
        double skinThickness=skin1;
        if (skin2>skinThickness)
            skinThickness=skin2;
        /*  If penetration < sSkinThickness, the contact stiffness will be proportional to the penetration.
         *  The goal is to help the solver to find the appropriate point of equilibrium during grasping.
         */
        if (skinThickness > 0.0)
        {
            Vx::VxDynamicsContactInput* vi = (Vx::VxDynamicsContactInput*)dres;
            Vx::VxDynamicsContactInput::DynamicsContactIterator it = vi->dynamicsContactBegin();
            Vx::VxDynamicsContactInput::DynamicsContactIterator ie = vi->dynamicsContactEnd();

            while (it != ie)
            {
                const Vx::VxReal p = (*it)->getPenetration();
                if (p <= skinThickness && p > 0)
                {
                    Vx::VxContactMaterial* m = (*it)->getContactMaterial();
                    const Vx::VxReal s = 1.0/m->getCompliance() * p;
                    m->setCompliance(1.0/s);
                    m->setDamping(s/10.0);

                }
                ++it;
            }
        }
        const float sScale = 0.01f;
        const float sGravity = CRigidBodyContainerDyn::currentRigidBodyContainerDynObject->gravityVectorLength;
        const float sMaxMassScale = 1000;
        if (autoSlip)
        {
            /*  The autoslip adapts the contact slip vs the pressure on the contacts.
             *  This contact slip may be problematic when a ligth object is under large pressure
             *  The slip is computed from the contact normal forces.
             */
            Vx::VxDynamicsContactInput* vi = (Vx::VxDynamicsContactInput*)dres;
            Vx::VxDynamicsContactInput::DynamicsContactIterator it = vi->dynamicsContactBegin();
            Vx::VxDynamicsContactInput::DynamicsContactIterator ie = vi->dynamicsContactEnd();
            int index = 0;
            int scale = 1;
            Vx::VxReal mass;
            if (part[0]->getControl() != Vx::VxPart::kControlDynamic)
            {
                index = 1;
                scale = -1;
                mass = part[1]->getMass();
            }
            else if (part[1]->getControl() != Vx::VxPart::kControlDynamic)
            {
                mass = part[0]->getMass();
            }
            else
            {
                mass = std::min<float>(part[0]->getMass(), part[1]->getMass());
            }
            Vx::VxReal dt = Vx::VxFrame::currentInstance()->getTimeStep();

            while (it != ie)
            {
                bool matched = (*it)->getMatched();
                if (matched)
                {
                    /* What we want to do here: the heavier the objects the smaller the slip we need.
                     * In the case of a car for example, the slip should not be based on the wheel but
                     * on the pressure the chassis exert on the wheel. This pressure would be read in the
                     * suspension constraint. The larger the pressure the smaller the slip. Now if the wheel
                     * is under a chassis with chassisMass > sMaxMassScale*wheelMass, we liimt the slip to avoid
                     * instability due to very large mass ratio.
                     *
                     * Similarly, in the case of the gripper, the slip decrease with the pressure.
                    */
                    Vx::VxVector3 n, f;
                    (*it)->getNormal(n);
                    (*it)->getForce(index, f);
                    const Vx::VxReal lambda = f.dot(n)*scale;
                    // compute apparent mass using lambda / gravity
                    Vx::VxReal apparentMass = lambda/sGravity;

                    {
                        // limit apparentMass vs the smallest mass in the pair of colliding parts.
                        apparentMass = std::max<float>(mass, apparentMass);
                        apparentMass = std::min<float>(sMaxMassScale*mass, apparentMass);
                        if (lambda > Vx::VX_MEDIUM_EPSILON)
                        {
                            const Vx::VxReal loss = sScale * dt / apparentMass;
                            //Vx::VxInfo(0, " computed loss = %g, original was %g\n", loss, (*it)->getContactMaterial()->getSlip(Vx::VxMaterial::kFrictionAxisLinearPrimary));
                            (*it)->getContactMaterial()->setSlip(Vx::VxMaterial::kFrictionAxisLinear, loss);
                        }
                    }
                }
                else
                {
                    const Vx::VxReal loss = sScale * dt / mass;
                    //Vx::VxInfo(0, " computed loss = %g, original was %g\n", loss, (*it)->getContactMaterial()->getSlip(Vx::VxMaterial::kFrictionAxisLinearPrimary));
                    (*it)->getContactMaterial()->setSlip(Vx::VxMaterial::kFrictionAxisLinear, loss);
                }
                ++it;
            }
        }

/* printing contacts:
        {
            Vx::VxDynamicsContactInput* vi = (Vx::VxDynamicsContactInput*)dres;
            Vx::VxDynamicsContactInput::DynamicsContactIterator it = vi->dynamicsContactBegin();
            Vx::VxDynamicsContactInput::DynamicsContactIterator ie = vi->dynamicsContactEnd();

            while (it != ie)
            {
                Vx::VxVector3 n;
                (*it)->getNormal(n);
                if (1)//fabs(n[2]) < 0.2)
                {
                    Vx::VxInfo(0, "%.3f %.3f %.3f p=%g stiff=%g\n", n[0], n[1], n[2], (*it)->getPenetration(), 1.0/(*it)->getContactMaterial()->getCompliance());
                }
                ++it;
            }
        } */
        // CRigidBodyContainerDyn::_vortexCollisionCallbackStatic((void*)dres, cg[0], cg[1]);
    }
};

void vortexInfoHandler(const int level, const char *const format,va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp+=format;
    vsnprintf(buff,sizeof(buff),tmp.c_str(),ap);
    simAddLog(LIBRARY_NAME,sim_verbosity_infos,buff);
}

void vortexWarningHandler(const int level, const char *const format,va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp+=format;
    vsnprintf(buff,sizeof(buff),tmp.c_str(),ap);
    simAddLog(LIBRARY_NAME,sim_verbosity_warnings,buff);
}

void vortexErrorHandler(const int level, const char *const format,va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp+=format;
    vsnprintf(buff,sizeof(buff),tmp.c_str(),ap);
    simAddLog(LIBRARY_NAME,sim_verbosity_errors,buff);
}

Vx::VxPart* getCollisionGeometryPart(Vx::VxCollisionGeometry* cg)
{
    while (cg->getPart() == nullptr && cg->getParent() != nullptr)
    {
        cg = (Vx::VxCollisionGeometry*)cg->getParent();
    }
    return cg->getPart();
}

int CRigidBodyContainerDyn_vortex::getEngineInfo(int& engine,int data1[4],char* data2,char* data3)
{
    engine=sim_physics_vortex;
    data1[0]=612;
    strcpy(data2,Vx::VxGetVersion());
    return(DYNAMICS_PLUGIN_VERSION);
}

CRigidBodyContainerDyn_vortex::CRigidBodyContainerDyn_vortex()
{
    int plugin_verbosity = sim_verbosity_default;
    simGetModuleInfo(LIBRARY_NAME,sim_moduleinfo_verbosity,nullptr,&plugin_verbosity);
    if (plugin_verbosity==sim_verbosity_none)
        Vx::LogSetLevel(Vx::kOff);
    if (plugin_verbosity>=sim_verbosity_errors)
        Vx::LogSetLevel(Vx::kError);
    if (plugin_verbosity>=sim_verbosity_warnings)
        Vx::LogSetLevel(Vx::kWarn);
    if (plugin_verbosity>=sim_verbosity_infos)
        Vx::LogSetLevel(Vx::kInfo);
    if (plugin_verbosity>=sim_verbosity_debug)
        Vx::LogSetLevel(Vx::kAll);
    _dynamicsCalculationPasses=0;
    _allRigidBodiesIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);
    _allConstraintsIndex.resize(CRigidBodyContainerDyn::get3dObjectIdEnd()-CRigidBodyContainerDyn::get3dObjectIdStart(),nullptr);

    vortexStartTimeTag=simGetSystemTimeInMs(-1);
    Vx::VxFrame* frame = Vx::VxFrame::instance();
    _vortexWorld = new Vx::VxUniverse();
    frame->addUniverse(_vortexWorld);
    // adjust contact matching epsilon
    for (int i=0; i<Vx::VxCollisionPairRequest::kDefaultRequestCount; ++i)
    {
        for (int j=0; j<=i; ++j)
        {
            Vx::VxCollisionPairRequest* req = _vortexWorld->getPartPartCollisionPairRequest(i, j);
            req->setContactMatchingEpsilon(0.01);
        }
    }
    // ?? Vx::VxDynamicsContact::setForceContactMatching(true);
    static const Vx::VxReal initialVelocityThresholdScale = Vx::VxDynamicsContact::getSlidingVelocityThresholdScale();
    Vx::VxDynamicsContact::setSlidingVelocityThresholdScale(initialVelocityThresholdScale);
    _vortexWorld->getSolverParameters(0)->setScaleBoxMaxIteration(2);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float floatParams[10];
    int intParams[1];
    _simGetVortexParameters(nullptr,3,floatParams,intParams);

//        const double stepSize=getVortexUnsignedDouble(floatParams[0]);
//        const double internalScalingFactor=getVortexUnsignedDouble(floatParams[1]);
    const double contactTolerance=getVortexUnsignedDouble(floatParams[2]);
    const double constraint_linear_compliance=getVortexUnsignedDouble(floatParams[3]);
    const double constraint_linear_damping=getVortexUnsignedDouble(floatParams[4]);
    const double constraint_linear_kineticLoss=getVortexUnsignedDouble(floatParams[5]);
    const double constraint_angular_compliance=getVortexUnsignedDouble(floatParams[6]);
    const double constraint_angular_damping=getVortexUnsignedDouble(floatParams[7]);
    const double constraint_angular_kineticLoss=getVortexUnsignedDouble(floatParams[8]);
    // floatParams[9] is RESERVED!! (used to be the auto angular damping tension ratio)

    const bool autoSleep=((intParams[0]&1)!=0);
    const bool multiThreading=((intParams[0]&2)!=0);
    // (intParams[0]&4) is for full internal scaling
    // intParams[0]&8 is RESERVED!! (used to be the auto angular damping)


    // In ODE and Bullet we disable the auto-sleep functionality, because with those engines,
    // if you remove the floor under disabled boxes, they will not automatically wake.
    // This is different with Vortex (correctly handled), thus, in Vortex, we
    // can keep the auto-sleep functionality!
    _vortexWorld->setAutoSleep(autoSleep);

    // Constraint solver parameters:
    Vx::VxSolverParameters* sp=_vortexWorld->getSolverParameters();
    sp->setConstraintLinearCompliance(constraint_linear_compliance); // Vortex default: 1.0e-10
    sp->setConstraintLinearDamping(constraint_linear_damping); // Vortex default: 8.33e+8
    sp->setConstraintLinearKineticLoss(constraint_linear_kineticLoss); // Vortex default: 6.0e-9
    sp->setConstraintAngularCompliance(constraint_angular_compliance); // Vortex default: 1.0e-10
    sp->setConstraintAngularDamping(constraint_angular_damping); // Vortex default: 8.33e+8
    sp->setConstraintAngularKineticLoss(constraint_angular_kineticLoss); // Vortex default: 6.0e-9

/*
    sp->setConstraintLinearCompliance(1.0e-10*spScaleL); // Vortex default: 1.0e-10
    sp->setConstraintLinearDamping(8.33e+8/spScaleL); // Vortex default: 8.33e+8
    sp->setConstraintLinearKineticLoss(6.0e-9*spScaleL); // Vortex default: 6.0e-9
    sp->setConstraintAngularCompliance(1.0e-10*spScaleA); // Vortex default: 1.0e-10
    sp->setConstraintAngularDamping(8.33e+8/spScaleA); // Vortex default: 8.33e+8
    sp->setConstraintAngularKineticLoss(6.0e-9*spScaleA); // Vortex default: 6.0e-9
*/
/*
    just before IROS/IREX. TODO for after:
    - Scale all vortex params (forgotten)
    - Add a box max force parameter (see email from Martin on 15/10/2013)
*/
    // Contact tolerance. Vortex default: 0.001
//    frame->setContactTolerance(contactTolerance*linScaling);
    _vortexWorld->setDefaultContactTolerance(contactTolerance);

    frame->setAutomaticTimeStep(false);
    _vortexWorld->setDynamicsMultithreaded(multiThreading);
    _vortexWorld->setCollisionMultithreaded(multiThreading);

    vortexIntersectSubscriber = new VortexIntersectSubscriber;
    _vortexWorld->addIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventFirst, vortexIntersectSubscriber, 0);
    _vortexWorld->addIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventActive, vortexIntersectSubscriber, 0);
    vortexIntersectFilter = new VortexIntersectFilter;
    CRigidBodyDyn_vortex::setVortexFilter(vortexIntersectFilter);

// Following instructions bring the RELEASE version to crash just after a message was output!
//----------------------------------------------------------------------
//        Vx::VxSetInfoHandler((Vx::VxHandler)vortexInfoHandler);
//        Vx::VxSetWarningHandler((Vx::VxHandler)vortexWarningHandler);
//        Vx::VxSetFatalErrorHandler((Vx::VxHandler)vortexErrorHandler);
//----------------------------------------------------------------------

    // Now flag all objects and geoms as "_dynamicsFullRefresh":
    for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
        _simSetDynamicsFullRefreshFlag(_simGetObjectFromIndex(sim_handle_all,i),true);
    for (int i=0;i<_simGetObjectListSize(sim_object_shape_type);i++)
        _simSetGeomProxyDynamicsFullRefreshFlag((void*)_simGetGeomProxyFromShape(_simGetObjectFromIndex(sim_object_shape_type,i)),true);

    _nextRigidBodyID=0;
}

CRigidBodyContainerDyn_vortex::~CRigidBodyContainerDyn_vortex()
{
    while (_allRigidBodiesList.size()!=0)
        _removeRigidBody(_allRigidBodiesList[0]->getRigidBodyID());
    
    for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
    {
        CDummy3DObject* it=(CDummy3DObject*)_simGetObjectFromIndex(sim_handle_all,i);
        _simSetDynamicSimulationIconCode(it,sim_dynamicsimicon_none);
    }

    particleCont.removeAllParticles();

    _vortexWorld->removeIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventFirst, vortexIntersectSubscriber);
    _vortexWorld->removeIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventActive, vortexIntersectSubscriber);
    delete vortexIntersectSubscriber;
    delete vortexIntersectFilter;
    CRigidBodyDyn_vortex::setVortexFilter(nullptr);
    // this will delete everything, frame, universe and all objects not removed from it
    Vx::VxFrame::currentInstance()->release();
    VxFrameReleased = true;
    _vortexWorld = nullptr;

    // Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
    particleCont.removeAllObjects();
}

void CRigidBodyContainerDyn_vortex::licenseCheck()
{
    static bool done=false;
    static bool licensePresent=false;

    if (!done)
    {
        simAddLog(LIBRARY_NAME,sim_verbosity_loadinfos,Vx::VxGetVersion());
        std::string year(Vx::VxGetVersion(),Vx::VxGetVersion()+4);

        _checkingLicense=true;
        if (std::stoi(year)<=2019)
        {
            Vx::VxPart* partDynamic=new Vx::VxPart(1.0);
            _vortexWorld->addPart(partDynamic);
            Vx::VxPart* partStatic=new Vx::VxPart(1.0);
            _vortexWorld->addPart(partStatic);
            partStatic->setControl(Vx::VxPart::kControlStatic);
            Vx::VxCollisionGeometry* cgDynamic=new Vx::VxCollisionGeometry(new Vx::VxBox(C3Vector2VxVector3(C3Vector(0.1f,0.1f,0.1f))),nullptr);
            partDynamic->addCollisionGeometry(cgDynamic);
            Vx::VxCollisionGeometry* cgStatic=new Vx::VxCollisionGeometry(new Vx::VxBox(C3Vector2VxVector3(C3Vector(0.1f,0.1f,0.1f))),nullptr);
            partStatic->addCollisionGeometry(cgStatic);
            partDynamic->setPosition(C3Vector2VxVector3(C3Vector(0,0,100.0)));
            partStatic->setPosition(C3Vector2VxVector3(C3Vector(0,0,99.0)));
            Vx::VxFrame::currentInstance()->setTimeStep(0.1);
            for (int i=0;i<10;i++)
                Vx::VxFrame::currentInstance()->step();
            double zCoord=partDynamic->getTransform().t()[2];
            _vortexWorld->removePart(partDynamic);
            _vortexWorld->removePart(partStatic);
            delete partDynamic;
            delete partStatic;
            licensePresent=(zCoord>98.0);
        }
        else
        {
            Vx::VxPart* partDynamic=new Vx::VxPart(1.0);
            _vortexWorld->addPart(partDynamic);
            partDynamic->setPosition(C3Vector2VxVector3(C3Vector(0,0,100.0)));
            Vx::VxFrame::currentInstance()->setTimeStep(0.1);
            for (int i=0;i<10;i++)
                Vx::VxFrame::currentInstance()->step();
            double zCoord=partDynamic->getTransform().t()[2];
            _vortexWorld->removePart(partDynamic);
            delete partDynamic;
            licensePresent=(zCoord<99.0);
        }
        done=true;
        _checkingLicense=false;
    }
    if (!licensePresent)
        _simMakeDynamicAnnouncement(sim_announce_vortexpluginisdemo);
}


void CRigidBodyContainerDyn_vortex::_vortexCollisionCallbackStatic(void* data,Vx::VxCollisionGeometry* o1,Vx::VxCollisionGeometry* o2)
{ // this function is static and will call the corresponding function of the current object:
    ((CRigidBodyContainerDyn_vortex*)currentRigidBodyContainerDynObject)->_vortexCollisionCallback(data,o1,o2);
}

void CRigidBodyContainerDyn_vortex::_vortexCollisionCallback(void* data,Vx::VxCollisionGeometry* o1,Vx::VxCollisionGeometry* o2)
{
    Vx::VxPart* b1=getCollisionGeometryPart(o1);
    Vx::VxPart* b2=getCollisionGeometryPart(o2);

    CDummyShape* shapeA = b1==nullptr?nullptr:(CDummyShape*)_simGetObject(b1->userData().getData("csim").getValueInteger());
    CDummyShape* shapeB = b2==nullptr?nullptr:(CDummyShape*)_simGetObject(b2->userData().getData("csim").getValueInteger());

    bool canCollide=false;
    int objID1;
    int objID2;

    if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
    { // regular case (shape-shape)
        unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
        unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
        canCollide=_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB)&&((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_respondable)!=0)&&((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_respondable)!=0);
        if (_simGetLastParentForLocalGlobalCollidable(shapeA)==_simGetLastParentForLocalGlobalCollidable(shapeB))
            canCollide=canCollide&&(collFA&collFB&0x00ff); // we are local
        else
            canCollide=canCollide&&(collFA&collFB&0xff00); // we are global
        if ( (_simIsShapeDynamicallyStatic(shapeA)||((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_dynamic)==0))&&
            (_simIsShapeDynamicallyStatic(shapeB)||((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_dynamic)==0)) )
            canCollide=false;
    }
    else
    { // particle-shape or particle-particle case:
        int dataA=(uint64_t) b1->userData().getData("csim").getValueInteger();
        int dataB=(uint64_t) b2->userData().getData("csim").getValueInteger();
        if ( (shapeA==nullptr)&&(shapeB==nullptr) )
        { // particle-particle case:
            CParticleObject* pa=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
            CParticleObject* pb=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);

            if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
            {
                canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
            }
        }
        else
        { // particle-shape case:
            CDummyShape* shape;
            CParticleObject* particle;
            if (shapeA!=nullptr)
            {
                shape=shapeA;
                objID1=_simGetObjectID(shapeA);
            }
            else
            {
                particle=particleCont.getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                objID1=dataA;
            }
            if (shapeB!=nullptr)
            {
                shape=shapeB;
                objID2=_simGetObjectID(shapeB);
            }
            else
            {
                particle=particleCont.getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                objID2=dataB;
            }

            if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
            {
                canCollide=_simIsShapeDynamicallyRespondable(shape)&&(_simGetDynamicCollisionMask(shape)&particle->getShapeRespondableMask()&0xff00)&&((_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable)!=0); // we are global
            }
        }
    }

    bool disableThisContact=true;
    if (canCollide)
    {
        int dataInt[3]={0,0,0};
        float dataFloat[14]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
        // For now we won't allow modification of contact parameters when using VORTEX
        bool canReallyCollide=(_simHandleCustomContact(objID1,objID2,sim_physics_vortex,dataInt,dataFloat)!=0);
        if (canReallyCollide)
            disableThisContact=false;
    }

    bool* pCanCollide = (bool*)data;
    *pCanCollide = !disableThisContact;

/*
    if (disableThisContact)
    {
        Vx::VxDynamicsContactInput* dres = (Vx::VxDynamicsContactInput*) data;
        Vx::VxDynamicsContactInput::DynamicsContactIterator it = dres->dynamicsContactBegin();
        Vx::VxDynamicsContactInput::DynamicsContactIterator ie = dres->dynamicsContactEnd();
        while (it != ie)
        {
            (*it)->enableContact(false); // if don't collide
            it++;
        }
    }
*/
}

void CRigidBodyContainerDyn_vortex::applyGravity()
{ // gravity is scaled here!!

    C3Vector gravity;
    _simGetGravity(gravity.data);

    _vortexWorld->setGravity(C3Vector2VxVector3(gravity));

    gravityVectorLength=(gravity).getLength();
}

void CRigidBodyContainerDyn_vortex::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{ 
    _vortexWorld->printContent(filenameAndPath.c_str());
}

Vx::VxUniverse* CRigidBodyContainerDyn_vortex::getWorld()
{
    return(_vortexWorld);
}


void CRigidBodyContainerDyn_vortex::_createDependenciesBetweenJoints()
{
    for (int i=0;i<int(_allConstraintsList.size());i++)
    {
        CConstraintDyn* constr1=_allConstraintsList[i];
        int linkedJoint;
        double fact,off;
        if (((CConstraintDyn_vortex*)constr1)->getVortexDependencyInfo(linkedJoint,fact,off))
        { // when we enter here, the dependency flag is automatically cleared in constr1
            fact*=-1.0f; // when fact is >0, we want to turn into the same direction!
            CConstraintDyn* constr2=getConstraintFromJointID(linkedJoint);
            if (constr2!=nullptr)
            {
                Vx::VxConstraint* joint1=((CConstraintDyn_vortex*)constr1)->getVortexConstraint();
                Vx::VxConstraint* joint2=((CConstraintDyn_vortex*)constr2)->getVortexConstraint();

                // We have following equation:
                // joint1=off+joint2*fact

                /*
                * The gear ratio constraint is setup to be initially at rest.
                * off doesn't really make sense unless there is a backlash and off < backlash/2
                * so that we can position the teeth relative position otherwise the gear will violently snap
                * back in equilibrium
                */

                // need to identified the gear parts. This is not great, we should have a better criteria
                int i0 = joint1->getPart(0)->getControl() == Vx::VxPart::kControlDynamic ? 0 : 1;
                int i1 = joint2->getPart(0)->getControl() == Vx::VxPart::kControlDynamic ? 0 : 1;

                // get gear parts attachment info
                Vx::VxVector3 a00, a01, a10, a11, p0, p1;
                joint1->getPartAttachmentPositionRel(i0, p0);
                joint2->getPartAttachmentPositionRel(i1, p1);
                joint1->getPartAttachmentAxesRel(i0, a00, a01);
                joint2->getPartAttachmentAxesRel(i1, a10, a11);

                // set gear parts
                Vx::VxGearRatio* gr = new Vx::VxGearRatio();
                gr->setPartAndAttachmentRel(0, joint1->getPart(i0), p0, a00, a01);
                gr->setPartAndAttachmentRel(1, joint2->getPart(i1), p1, a10, a11);

                // set the gears reference parts and attachments.
                // note here if we only support geears being attached to the same part, se can simply call gr->setengageMode(true) instead.
                const int other[2] = {1,0};
                gr->getPartAttachmentPosition(0, p0);
                gr->getPartAttachmentAxes(0, a00, a01);
                gr->getPartAttachmentPosition(1, p1);
                gr->getPartAttachmentAxes(1, a10, a11);
                gr->setPartAndAttachment(2, joint1->getPart(other[i0]), p0, a00, a01);
                gr->setPartAndAttachment(3, joint2->getPart(other[i1]), p1, a10, a11);

                // support linear and angular gear, a hinge supports a rotation gear while a prismatic supports a linear one
                if (joint1->isOfExactClassType(Vx::VxPrismatic::getStaticClassType()))
                {
                    gr->setMotionAngular(0, false);
                }
                if (joint2->isOfExactClassType(Vx::VxPrismatic::getStaticClassType()))
                {
                    gr->setMotionAngular(1, false);
                }

                // set this to false to simulate a belt that could be sliding on the gear.
                // use gr->setMaxTorque(maxTorque) and gr->setMinTorque(-maxTorque) to tune the belt friction force
                // note that setting the maxTorque on the positional gear allow simulate rubber teeth gear where teeth jumping may occur.
                gr->setUpdateMode(Vx::VxGearRatio::kPositional);

                // set the gear ratio. Not sure if I interpret correctly here as in the exemple the gear size not set, could be 1.0/fact
                gr->setGearRatio(fact);
                gr->resetPositions(); // for positional gear, this sets the gear at equilibrium position

                const Vx::VxReal backLash = 0;
                gr->setBacklash(backLash);
                if (fabs(off) < backLash)
                {
                    // not sure about the sign here
                    gr->setPartReferencePosition(0, off + gr->getPartReferencePosition(0));
                }

                // internally, the gear error is filter for smoother result. The is the constraint violation mean lifetime
                // which should be larger than the time step.
                gr->setPositionOffsetMeanLifeTime(Vx::VxFrame::currentInstance()->getTimeStep()*10);

                if (joint1->getUniverse())
                {
                    //Vx::VxInfo(0, "gr added");
                    joint1->getUniverse()->addConstraint(gr);
                }
                else
                {
                    // in this case th egear will never be added to the universe, I guess this cannot happen?!?
                }

                // this is used to destroy the gear for now...
                SVortexJointDependency dep;
                dep.gear=gr;
                dep.constr1=constr1;
                dep.constr2=constr2;
                _gears.push_back(dep);
            }
        }
    }
}

void CRigidBodyContainerDyn_vortex::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
    for (int i=0;i<int(_gears.size());i++)
    {
        if ( (_gears[i].constr1==theInvolvedConstraint)||(_gears[i].constr2==theInvolvedConstraint) )
        {
            Vx::VxConstraint* c=_gears[i].gear;
            if (c->getUniverse())
                c->getUniverse()->removeConstraint(c);
            delete c;
            _gears.erase(_gears.begin()+i);
            i--; // reprocess this position
        }
    }
}

void CRigidBodyContainerDyn_vortex::_addVortexContactPoints(int dynamicPassNumber)
{
    Vx::VxUniverse::DynamicsContactIterator it=_vortexWorld->dynamicsContactBegin();
    Vx::VxUniverse::DynamicsContactIterator ie=_vortexWorld->dynamicsContactEnd();

    while (it != ie)
    {
        if ((*it)->isEnabled())
        {
            Vx::VxReal3 pos,force, n;
            (*it)->getPosition(pos);
            (*it)->getNormal(n);
            C3Vector n2(VxVector32C3Vector(n));
            n2.normalize();
            C3Vector pos2(VxVector32C3Vector(pos));
            SContactInfo ci;
            ci.subPassNumber=dynamicPassNumber;
            Vx::VxPart* part1;
            Vx::VxPart* part2;
            (*it)->getPartPair(&part1,&part2);
            ci.objectID1=part1->userData().getData("csim").getValueInteger();
            ci.objectID2=part2->userData().getData("csim").getValueInteger();
            ci.position=pos2;
            if (part1->getControl()==Vx::VxPart::kControlDynamic)
                (*it)->getForce(0,force);
            else
                (*it)->getForce(1,force);
            C3Vector force2(VxVector32C3Vector(force));
            ci.directionAndAmplitude=force2;
            if (force2*n2<0.0f)
                n2=n2*-1.0f;
            ci.surfaceNormal=n2;
            _contactInfo.push_back(ci);

            _contactPoints.push_back(pos2(0));
            _contactPoints.push_back(pos2(1));
            _contactPoints.push_back(pos2(2));
        //   _contactPoints.push_back(pos2(0)+n[0]*0.01);
        //   _contactPoints.push_back(pos2(1)+n[1]*0.01);
        //   _contactPoints.push_back(pos2(2)+n[2]*0.01);
        }
        ++it;
    }

/*    if (gPrintConstraintDebug){
        // debug display de l'etat des contraintes
        int index = 0;
        const VxSet<Vx::VxConstraint *> &constraints = _vortexWorld->getConstraints();
        for (VxSet<Vx::VxConstraint *>::const_iterator it = constraints.begin(); it != constraints.end(); ++it)
        {
            Vx::VxConstraint* constraint = *it;
            if (constraint->getControl(0) != Vx::VxConstraint::kControlFree)
            {
                if (constraint->getLimitsActive(0) == true)
                {
                    Vx::VxReal pos = constraint->getCoordinateCurrentPosition(0);
                    Vx::VxReal mvel = constraint->getMotorDesiredVelocity(0);
                    Vx::VxReal vel = constraint->getCoordinateVelocity(0);
                    Vx::VxReal limLow = constraint->getLimitPosition(0, Vx::VxConstraint::kLimitLower);
                    Vx::VxReal limHi = constraint->getLimitPosition(0, Vx::VxConstraint::kLimitUpper);
                    char str[512];

                    if (constraint->getControl(0) == Vx::VxConstraint::kControlMotorized)
                        sprintf(str, "%d motor,  p=%g, v=%g, %g, low=%g, hi=%g", index, pos, vel, mvel, limLow, limHi);
                    else
                        sprintf(str, "%d locked, p=%g, lpos=%g lvel=%g low=%g, hi=%g", index, pos, constraint->getLockPosition(0), constraint->getLockVelocity(0), limLow, limHi);

                    if (constraint->isLimitExceeded(0))
                        Vx::VxInfo(0, "%s (exceeded) \n", str);
                    else
                        Vx::VxInfo(0, "%s \n", str);
                }
                else
                {
                    Vx::VxReal pos = constraint->getCoordinateCurrentPosition(0);
                    Vx::VxReal mvel = constraint->getMotorDesiredVelocity(0);
                    Vx::VxReal vel = constraint->getCoordinateVelocity(0);
                    char str[512];

                    if (constraint->getControl(0) == Vx::VxConstraint::kControlMotorized)
                        sprintf(str, "%d motor,  p=%g, v=%g, %g, low=%g, hi=%g", index, pos, vel, mvel);
                    else
                        sprintf(str, "%d locked, p=%g, lpos=%g lvel=%g low=%g, hi=%g", index, pos, constraint->getLockPosition(0), constraint->getLockVelocity(0));
                }
                index++;
            }
        }
    } */
#if 0
    Vx::VxVector3 sf(0,0,0), st(0,0,0);
        for (Vx::VxUniverse::DynamicsContactIterator it = _vortexWorld->dynamicsContactBegin(); it != _vortexWorld->dynamicsContactEnd(); ++it)
        {
            Vx::VxPart* p[2];
            (*it)->getPartPair(p,p+1);
            int i = p[0]->getControl() == Vx::VxPart::kControlDynamic ? 0 : 1;
            bool matched = (*it)->getMatched();
            bool slide = (*it)->getIsSliding();
            Vx::VxReal d0, d1;
            bool sd = (*it)->getIntegratedDisplacement(d0, d1);

            Vx::VxReal vth = (*it)->getSlidingVelocityThreshold(), sv0, sv1;

            (*it)->getSlidingVelocity(sv0, sv1);

            Vx::VxVector3 f, t, n;
            (*it)->getForce(i, f);
            (*it)->getNormal(n);
            t = f - n*f.dot(n);
            st +=t;
            sf+=f;

            Vx::VxInfo(0, "%s %s matched=%d, slide=%d, pen=%g nz=%g f=%.3f t=%.3f\n", p[0]->getName(), p[1]->getName(), matched, slide, (*it)->getPenetration(),n[2], f.dot(n), t.norm());
        }
        Vx::VxInfo(0, "sf=%.3f st=%.3f\n", sf.norm(), st.norm());
#endif
    /*  auto angular damping on the parts. Angular damping helps reducing possible instabilities on light object
    *  submitted to large pressure or tension from heavier objetcs.
    */
    for (Vx::VxPartSet::const_iterator it = _vortexWorld->getParts().begin(), itE = _vortexWorld->getParts().end(); it != itE; ++it)

    {
        Vx::VxPart* part = *it;
        if (part->getControl() != Vx::VxPart::kControlDynamic)
            continue;

        CRigidBodyDyn* body=(CRigidBodyDyn*)part->userData().getData("csim1").getPointerVoid();
        if ((body!=nullptr)&&(((CRigidBodyDyn_vortex*)body)->vortex_autoAngularDampingTensionRatio!=0.0))
        { // auto angular damping is enabled
            const Vx::VxReal pAutoAngularDampingTensionAdded = ((CRigidBodyDyn_vortex*)body)->vortex_angularVelocityDamping;
            const Vx::VxReal pAutoAngularDampingTensionTreshold = part->getMass();
            const Vx::VxReal pAutoAngularDampingTensionRatio = ((CRigidBodyDyn_vortex*)body)->vortex_autoAngularDampingTensionRatio;

            Vx::VxReal sumF = 0;
            for (int i=0; i<part->getConstraintCount(); ++i)
            {
                Vx::VxVector3 v;
                Vx::VxConstraint* c = part->getConstraint(i);
                int index = 0;
                if (c->getPart(0) != part)
                {
                    index = 1;
                }
                c->getPartForce(index, v);
                sumF += v.norm();
            }
            for (int i=0; i<part->getContactCount(); ++i)
            {
                Vx::VxVector3 v;
                Vx::VxDynamicsContact* c = part->getContact(i)->getContact();
                int index = 0;
                Vx::VxPart* p[2];
                c->getPartPair(p, p+1);
                if (p[0] != part)
                {
                    index = 1;
                }
                c->getForce(index, v);
                sumF += v.norm();
            }
            sumF /= gravityVectorLength;
            Vx::VxReal angularDamping = pAutoAngularDampingTensionAdded;
            if(sumF > pAutoAngularDampingTensionTreshold)
            {
                const Vx::VxReal excess = sumF - pAutoAngularDampingTensionTreshold;
                angularDamping += (excess * pAutoAngularDampingTensionRatio);
            }
            part->setAngularVelocityDamping(angularDamping);
        }
    }

#if 0
    // debug display des mesh

        // const VxSet<Vx::VxPart *> &parts = _vortexWorld->getParts();
        for (VxSet<Vx::VxPart *>::const_iterator it = parts.begin(); it != parts.end(); ++it)
        {
            Vx::VxPart* part = *it;

            for (VxSet<Vx::VxCollisionGeometry *>::const_iterator itcg = part->getCollisionGeometries().begin(); itcg < part->getCollisionGeometries().end(); ++itcg)
            {
                Vx::VxTriangleMeshBVTree* bv= dynamic_cast<Vx::VxTriangleMeshBVTree*>((*itcg)->getGeometry());
                if (bv)
                {
                    for (int k=0; k<bv->getTriangleCount(); ++k)
                    {
                        Vx::VxReal3Ptr v1, v2, v3;
                        bv->getTriangleVertexPtrs(k, &v1, &v2, &v3);
                        Vx::VxVector3 v, n;
                        v = (Vx::VxVector3(v1)+Vx::VxVector3(v2)+Vx::VxVector3(v3))/3.0;
                        bv->getTriangleNormal(k, &n);

                        v/=linScaling; // ********** SCALING
                        v = part->getTransform().transform((v));
                        _contactPoints.push_back(v[0]);
                        _contactPoints.push_back(v[1]);
                        _contactPoints.push_back(v[2]);
                        v += part->getTransform().rotate(Vx::VxVector3(n) * (0.01/linScaling));
                        _contactPoints.push_back(v[0]);
                        _contactPoints.push_back(v[1]);
                        _contactPoints.push_back(v[2]);
                    }
                }
                Vx::VxTriangleMeshUVGrid* uv= dynamic_cast<Vx::VxTriangleMeshUVGrid*>((*itcg)->getGeometry());
                if (uv)
                {
                    for (int k=0; k<uv->getTriangleCount(); ++k)
                    {
                        Vx::VxReal3Ptr v1, v2, v3, n;
                        uv->getTriangleVertexAndNormalPointers(k, &v1, &v2, &v3, &n);
                        Vx::VxVector3 v;
                        v = (Vx::VxVector3(v1)+Vx::VxVector3(v2)+Vx::VxVector3(v3))/3.0;

                        v/=linScaling; // ********** SCALING
                        v = part->getTransform().transform((v));
                        _contactPoints.push_back(v[0]);
                        _contactPoints.push_back(v[1]);
                        _contactPoints.push_back(v[2]);
                        v += part->getTransform().rotate(Vx::VxVector3(n) * (0.01/linScaling));
                        _contactPoints.push_back(v[0]);
                        _contactPoints.push_back(v[1]);
                        _contactPoints.push_back(v[2]);
                    }
                }
            }
        }
#endif
}

void CRigidBodyContainerDyn_vortex::_stepDynamics(float dt,int pass)
{
    if (VxFrameReleased)
    {
        VxFrameReleased = false;
        //_vortexWorld->printContent("vxuniverse.txt");
    }
    //Vx::VxInfo(0, "trial=%d, time remaining=%g\n", Vx::LicensingManager::isRunningTrial(), Vx::LicensingManager::getTrialRemainingSeconds());
    Vx::VxFrame::currentInstance()->setTimeStep((Vx::VxReal)dt);
    Vx::VxFrame::currentInstance()->step();
    _addVortexContactPoints(pass);
}
