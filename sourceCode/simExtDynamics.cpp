#include "simExtDynamics.h"
#ifdef INCLUDE_BULLET_2_78_CODE
#include "RigidBodyContainerDyn_bullet278.h"
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
#include "RigidBodyContainerDyn_bullet283.h"
#endif
#ifdef INCLUDE_ODE_CODE
#include "RigidBodyContainerDyn_ode.h"
#endif
#ifdef INCLUDE_NEWTON_CODE
#include "RigidBodyContainerDyn_newton.h"
#endif
#ifdef INCLUDE_VORTEX_CODE
#include "RigidBodyContainerDyn_vortex.h"
#endif
#include "simLib.h"
#include <iostream>
#include <cstdio>

#ifdef _WIN32
#include <direct.h>
#endif

#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
#endif

static LIBRARY simLib;

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
	 char curDirAndFile[1024];
 #ifdef _WIN32
	 _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
	 getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
	 std::string currentDirAndPath(curDirAndFile);

	 std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/coppeliaSim.dll";
 #elif defined (__linux)
     temp+="/libcoppeliaSim.so";
 #elif defined (__APPLE__)
     temp+="/libcoppeliaSim.dylib";
 #endif /* __linux || __APPLE__ */

     simLib=loadSimLibrary(temp.c_str());
     if (simLib==NULL)
	 {
         printf("simExt%s: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n",LIBRARY_NAME); // cannot use simAddLog here.
         return(0);
	 }
     if (getSimProcAddresses(simLib)==0)
	 {
         printf("simExt%s: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n",LIBRARY_NAME); // cannot use simAddLog here.
         unloadSimLibrary(simLib);
         return(0);
	 }

	return(DYNAMICS_PLUGIN_VERSION);
}

SIM_DLLEXPORT void simEnd()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
	return(NULL);
}

CRigidBodyContainerDyn* dynWorld=NULL;

SIM_DLLEXPORT char dynPlugin_startSimulation(int engine,int version,const float floatParams[20],const int intParams[20])
{
	CRigidBodyContainerDyn::setPositionScalingFactorDyn(floatParams[0]);
	CRigidBodyContainerDyn::setLinearVelocityScalingFactorDyn(floatParams[1]);
	CRigidBodyContainerDyn::setMassScalingFactorDyn(floatParams[2]);
	CRigidBodyContainerDyn::setMasslessInertiaScalingFactorDyn(floatParams[3]);
	CRigidBodyContainerDyn::setForceScalingFactorDyn(floatParams[4]);
	CRigidBodyContainerDyn::setTorqueScalingFactorDyn(floatParams[5]);
	CRigidBodyContainerDyn::setGravityScalingFactorDyn(floatParams[6]);

	CRigidBodyContainerDyn::setDynamicActivityRange(floatParams[7]);

	CRigidBodyContainerDyn::setDynamicParticlesIdStart(intParams[0]);
	CRigidBodyContainerDyn::set3dObjectIdStart(intParams[1]);
	CRigidBodyContainerDyn::set3dObjectIdEnd(intParams[2]);


    simAddLog(LIBRARY_NAME,sim_verbosity_infos,"initializing the physics engine...");

#ifdef INCLUDE_BULLET_2_78_CODE
	if ( (engine==sim_physics_bullet)&&(version==0) )
        dynWorld=new CRigidBodyContainerDyn_bullet278();
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
	if ( (engine==sim_physics_bullet)&&(version==283) )
        dynWorld=new CRigidBodyContainerDyn_bullet283();
#endif

#ifdef INCLUDE_ODE_CODE
	if ( (engine==sim_physics_ode)&&(version==0) )
        dynWorld=new CRigidBodyContainerDyn_ode();
#endif

#ifdef INCLUDE_NEWTON_CODE
	if ( (engine==sim_physics_newton)&&(version==0) )
        dynWorld=new CRigidBodyContainerDyn_newton();
#endif

#ifdef INCLUDE_VORTEX_CODE
    if ( (engine==sim_physics_vortex)&&(version==0) )
    {
        dynWorld=new CRigidBodyContainerDyn_vortex();
		((CRigidBodyContainerDyn_vortex*)dynWorld)->licenseCheck();
	}
#endif

	if (dynWorld!=NULL)
    {
        int	data1[4];
        char versionStr[256];
        dynWorld->getEngineInfo(engine,data1,versionStr,NULL);
        std::string tmp("engine version: ");
        tmp+=versionStr;
        tmp+=", plugin version: ";
        tmp+=std::to_string(DYNAMICS_PLUGIN_VERSION);
        simAddLog(LIBRARY_NAME,sim_verbosity_infos,tmp.c_str());
        simAddLog(LIBRARY_NAME,sim_verbosity_infos,"initialization successful.");
    }

	return(dynWorld!=NULL);
}

SIM_DLLEXPORT void dynPlugin_endSimulation()
{
	delete dynWorld;
	dynWorld=NULL;
}

SIM_DLLEXPORT void dynPlugin_step(float timeStep,float simulationTime)
{
	if (dynWorld!=NULL)
		dynWorld->handleDynamics(timeStep,simulationTime);
}

SIM_DLLEXPORT char dynPlugin_isDynamicContentAvailable()
{
	if (dynWorld!=NULL)
		return(dynWorld->isDynamicContentAvailable());
	return(0);
}

SIM_DLLEXPORT void dynPlugin_serializeDynamicContent(const char* filenameAndPath,int bulletSerializationBuffer)
{
	if (dynWorld!=NULL)
		dynWorld->serializeDynamicContent(filenameAndPath,bulletSerializationBuffer);
}

SIM_DLLEXPORT int dynPlugin_addParticleObject(int objectType,float size,float massOverVolume,const void* params,float lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission)
{
	if (dynWorld!=NULL)
	{
		CParticleObject* it=new CParticleObject(objectType,size,massOverVolume,params,lifeTime,maxItemCount);
		for (int i=0;i<9;i++)
			it->color[i]=0.25f;
		for (int i=9;i<12;i++)
			it->color[i]=0.0f;
		for (int i=0;i<3;i++)
		{
			if (ambient!=NULL)
				it->color[0+i]=ambient[i];
			if (specular!=NULL)
				it->color[6+i]=specular[i];
			if (emission!=NULL)
				it->color[9+i]=emission[i];
		}
		return(dynWorld->particleCont.addObject(it));
	}
	return(-1); // error
}

SIM_DLLEXPORT char dynPlugin_removeParticleObject(int objectHandle)
{
	if (dynWorld!=NULL)
	{
		if (objectHandle==sim_handle_all)
			dynWorld->particleCont.removeAllObjects();
		else
		{
			CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
			if (it==NULL)
				return(false); // error
			dynWorld->particleCont.removeObject(objectHandle);
		}
		return(true);
	}
	return(false); // error
}

SIM_DLLEXPORT char dynPlugin_addParticleObjectItem(int objectHandle,const float* itemData,float simulationTime)
{
	if (dynWorld!=NULL)
	{
		CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
		if (it==NULL)
			return(false); // error
		it->addParticle(simulationTime,itemData);
		return(true);
	}
	return(false); // error
}

SIM_DLLEXPORT int dynPlugin_getParticleObjectOtherFloatsPerItem(int objectHandle)
{
	int retVal=0;
	if (dynWorld!=NULL)
	{
		CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
		if (it!=NULL)
			retVal=it->getOtherFloatsPerItem();
		else if (objectHandle==-131183)
			retVal=61855195;
	}
	return(retVal);
}

SIM_DLLEXPORT float* dynPlugin_getContactPoints(int* count)
{
	float* retVal=NULL;
	count[0]=0;
	if (dynWorld!=NULL)
		retVal=dynWorld->getContactPoints(count);
	return(retVal);
}

SIM_DLLEXPORT void** dynPlugin_getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
	if (dynWorld==NULL)
	{
		particlesCount[0]=-1;
		return(NULL);
	}
	return(dynWorld->particleCont.getParticles(index,particlesCount,objectType,cols));
}

SIM_DLLEXPORT char dynPlugin_getParticleData(const void* particle,float* pos,float* size,int* objectType,float** additionalColor)
{
	if (particle==NULL)
		return(false);
	return(((CParticleDyn*)particle)->getRenderData(pos,size,objectType,additionalColor));
}

SIM_DLLEXPORT char dynPlugin_getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float contactInfo[6])
{
	if (dynWorld!=NULL)
		return(dynWorld->getContactForce(dynamicPass,objectHandle,index,objectHandles,contactInfo));
	return(false);
}

SIM_DLLEXPORT void dynPlugin_reportDynamicWorldConfiguration(int totalPassesCount,char doNotApplyJointIntrinsicPositions,float simulationTime)
{
	if (dynWorld!=NULL)
		dynWorld->reportDynamicWorldConfiguration(totalPassesCount,doNotApplyJointIntrinsicPositions!=0,simulationTime);
}

SIM_DLLEXPORT int dynPlugin_getDynamicStepDivider()
{
	if (dynWorld!=NULL)
		return(dynWorld->getDynamicsCalculationPasses());
	return(0);
}

SIM_DLLEXPORT int dynPlugin_getEngineInfo(int* engine,int* data1,char* data2,char* data3)
{
	if (dynWorld!=NULL)
		return(dynWorld->getEngineInfo(engine[0],data1,data2,data3));
	engine[0]=-1;
	return(-1);
}
