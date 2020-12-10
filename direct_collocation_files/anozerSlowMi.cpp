/* -- Include various simbody/ opensim source file */
#include "mex.h"
#include <OpenSim/OpenSim.h>
#include <iostream>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>

/* -- Use the OS namespaces to prevent double declarations */
using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* -- initiate model and receive states */
Model OsimModel("anal.osim");
SimTK::State &defaultStates 		= OsimModel.initSystem();
Array<string> stateNames 			= OsimModel.getStateVariableNames();
const ControllerSet &controllerSet 	= OsimModel.getControllerSet();
const ContactGeometrySet &geomSet 	= OsimModel.getContactGeometrySet();
const ForceSet &fSet 				= OsimModel.getForceSet();

/* -- The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
			     int nrhs, const mxArray *prhs[] )
{
	/* -- Initial paramater allocation */
	int stateSequence, actuatorIndx, indx, ap;

	/* -- Initiate model and receive states */
    double *parameters           	= mxGetPr(prhs[0]);
	double *locL 					= mxGetPr(prhs[1]);
    double *stateSequence_d   		= mxGetPr(prhs[2]);
    double *nFreeDOFs_d  	        = mxGetPr(prhs[3]);
		int nFreeDOFs 				= (int)nFreeDOFs_d[0];
	double *nActuators_d  	        = mxGetPr(prhs[4]);
		int nActuators 				= (int)nActuators_d[0];
	double *nSpheres_d 				= mxGetPr(prhs[5]);
		int nSpheres 				= (int)nSpheres_d[0];
	double *nPPN_d		  	        = mxGetPr(prhs[6]);
		int nPPN	 				= (int)nPPN_d[0];
	double *np_d		  	        = mxGetPr(prhs[7]);
		int np	 					= (int)np_d[0];			
	double *currentTimePercentage   = mxGetPr(prhs[8]);
    double *t0 			            = mxGetPr(prhs[9]);
    double *tf                     	= mxGetPr(prhs[10]);
			
	/* -- output variable declarations */
    plhs[0] 						= mxCreateDoubleMatrix(1, (2*nFreeDOFs), mxREAL);//derivatives of states 
	plhs[1] 						= mxCreateDoubleMatrix(1, nSpheres * 12,mxREAL);//3 forces and 3 torques for each contact pair
	double *derivatives 			= mxGetPr(plhs[0]);
    double *grfs 					= mxGetPr(plhs[1]);
   
    Vec3 locG = {0, parameters[np+nSpheres*2+4], 0};
	 // Vec3 locG = {0, parameters[np+nSpheres+1+(nSpheres-1)*3+nSpheres*2], 0};
	geomSet.get(0).setLocation(locG);

	/* -- Update sphere locations */
	for (ap=0; ap <= nSpheres-1; ap++)
	{
		// Set radius
		ContactSphere* cpl = dynamic_cast<ContactSphere*>( &geomSet.get(ap*2+2) );
		ContactSphere* cpr = dynamic_cast<ContactSphere*>( &geomSet.get(ap*2+1) );
		// cpl->setRadius(parameters[np+nSpheres*2+1]);
		// cpr->setRadius(parameters[np+nSpheres*2+1]);
		// if (ap == 0 || ap == nSpheres-1) {
		if (ap == 0) {
			// cpl->setRadius(parameters[np+nSpheres*3]);
			// cpr->setRadius(parameters[np+nSpheres*3]);
			cpl->setRadius(parameters[np+nSpheres*2+2]);
			cpr->setRadius(parameters[np+nSpheres*2+2]);
		} else {
			cpl->setRadius(parameters[np+nSpheres*2+3]);
			cpr->setRadius(parameters[np+nSpheres*2+3]);
			// cpl->setRadius(parameters[np+nSpheres*3+1]);
			// cpr->setRadius(parameters[np+nSpheres*3+1]);
		}
		
		// Parameters is indexed with *2 because paramaters is fed the planar sphere locations
		// if (ap == 0) {
			// Vec3 mv3l = {locL[ap*3], parameters[np], locL[ap*3+2]};
			// Vec3 mv3r = {locL[ap*3], parameters[np],-locL[ap*3+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
		// } else {
			// Vec3 mv3l = {parameters[np+1+(ap-1)*3], parameters[np+2+(ap-1)*3], parameters[np+3+(ap-1)*3]};
			// Vec3 mv3r = {parameters[np+1+(ap-1)*3], parameters[np+2+(ap-1)*3],-parameters[np+3+(ap-1)*3]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
		// }
			// Vec3 mv3l = {parameters[np+nSpheres], parameters[np+nSpheres+1], parameters[np+nSpheres+2]};
			// Vec3 mv3r = {parameters[np+nSpheres], parameters[np+nSpheres+1],-parameters[np+nSpheres+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
		// Set stiffness of ALL spheres
		// OpenSim::HuntCrossleyForce* hcfl = dynamic_cast<OpenSim::HuntCrossleyForce*> ( &fSet.get(nActuators+ap*2+1));
		// OpenSim::HuntCrossleyForce* hcfr = dynamic_cast<OpenSim::HuntCrossleyForce*> ( &fSet.get(nActuators+ap*2));
		// hcfl->setDissipation(parameters[np+nSpheres+(1+(nSpheres-1)*3+nSpheres)+ap]);
		// hcfr->setDissipation(parameters[np+nSpheres+(1+(nSpheres-1)*3+nSpheres)+ap]);
		// hcfl->setStiffness(parameters[np+nSpheres+(1+(nSpheres-1)*3)+ap]);
		// hcfr->setStiffness(parameters[np+nSpheres+(1+(nSpheres-1)*3)+ap]);
		// printf("%f:\t\n",parameters[np+nSpheres+4+ap]);
		// printf("%f:\t\n",parameters[np+nSpheres+6+ap]);

		
		
		 // if (ap == 1|| ap == 2) {
			// Vec3 mv3l = {parameters[np+nSpheres + (ap-1)*3], parameters[np+nSpheres+(ap-1)*3+1], parameters[np+nSpheres+(ap-1)*3+2]};
			// Vec3 mv3r = {parameters[np+nSpheres + (ap-1)*3], parameters[np+nSpheres+(ap-1)*3+1],-parameters[np+nSpheres+(ap-1)*3+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
			// printf("%f:\t\n",parameters[np+nSpheres+(ap-1)*3+1]);

		 // } else {
			// Vec3 mv3l = {locL[ap*3], locL[ap*3+1], locL[ap*3+2]};
			// Vec3 mv3r = {locL[ap*3], locL[ap*3+1],-locL[ap*3+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
		 // }
			
			
			// /* planar motion*/		
			if (ap == 0) {
				Vec3 mv3l = {parameters[np+ap*2], parameters[np+nSpheres*2], parameters[np+ap*2+1]};
				Vec3 mv3r = {parameters[np+ap*2], parameters[np+nSpheres*2],-parameters[np+ap*2+1]};
				geomSet.get(ap*2+1).setLocation(mv3r);
				geomSet.get(ap*2+2).setLocation(mv3l);
			} else {
				Vec3 mv3l = {parameters[np+ap*2], parameters[np+nSpheres*2+1], parameters[np+ap*2+1]};
				Vec3 mv3r = {parameters[np+ap*2], parameters[np+nSpheres*2+1],-parameters[np+ap*2+1]};
				geomSet.get(ap*2+1).setLocation(mv3r);
				geomSet.get(ap*2+2).setLocation(mv3l);
			}
			// Vec3 mv3l = {parameters[np+ap*2], parameters[np+nSpheres*2], parameters[np+ap*2+1]};
			// Vec3 mv3r = {parameters[np+ap*2], parameters[np+nSpheres*2],-parameters[np+ap*2+1]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
			
			// /* 3d motion */
			// Vec3 mv3l = {parameters[np+ap*3], parameters[np+ap*3+1], parameters[np+ap*3+2]};
			// Vec3 mv3r = {parameters[np+ap*3], parameters[np+ap*3+1],-parameters[np+ap*3+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);
		
			/* Z motion */
			// Vec3 mv3l = {locL[ap*3], parameters[np+nSpheres+ap], locL[ap*3+2]};
			// Vec3 mv3r = {locL[ap*3], parameters[np+nSpheres+ap],-locL[ap*3+2]};
			// geomSet.get(ap*2+1).setLocation(mv3r);
			// geomSet.get(ap*2+2).setLocation(mv3l);

		// printf("%f:\t\n",hcfl->getStiffness());
		// printf("%f:\t\n",locL[ap*3+1]);
		// printf("%f:\t\n",locL[ap*3+2]);

		// printf("%f:\t\n",parameters[np+ap]);
		// printf("%d:\t\n",ap+1+nSpheres);
		// printf("%f:\t\n",locL[ap*3]);
		// printf("%f:\t\n",locL[ap*3+2]);
		// printf("%d:\t\n",np+ap*2+1);
		// printf("%f:\t\n",locL[3*ap+1]);
		// printf("%f:\t\n",parameters[np+ap*2+1]);
	
	}
				// printf("%f:\t\n",parameters[np+nSpheres+8]);

    SimTK::State &defaultStates 		= OsimModel.initSystem();
	// printf("%c:\t\n",geomSet.get(1).getPropertyByIndex(5));
    defaultStates.setTime((*(tf)-*(t0))*(*(currentTimePercentage))+*(t0));
   	/* -- Set all state variables */
	for (indx=0; indx<= 2*(nFreeDOFs)- 1; indx++)
	{
	    stateSequence      = (int)stateSequence_d[indx];
		OsimModel.setStateVariable(defaultStates,stateNames[stateSequence],*(parameters + indx));
	}
	
    /* -- Set all control values */
	actuatorIndx 	= 0;
	PrescribedController* controller = dynamic_cast<PrescribedController*>( &controllerSet.get(0) );
	while (indx < 2*(nFreeDOFs)  + nActuators)
	{
		// printf("%d:\t\n",indx);
		// printf("%e:\t\n",*(parameters + indx));
		controller->prescribeControlForActuator(actuatorIndx,	new Constant(*(parameters + indx)));
		indx++;
		actuatorIndx++;
    }

	/* -- Calculate the state derivatives */
    SimTK::Vector drvsSimTKVec = OsimModel.computeStateVariableDerivatives(defaultStates);
    for (indx=0; indx<2*(nFreeDOFs); indx++)
	{
        *(derivatives+indx) = drvsSimTKVec[(int)stateSequence_d[indx]];
	}
   
	/* -- Calculate the foot=ground contact force */
	Array<double> Fc_R1 = OsimModel.getForceSet().get("Foot_Ground_R1").getRecordValues(defaultStates);
	Array<double> Fc_R2 = OsimModel.getForceSet().get("Foot_Ground_R2").getRecordValues(defaultStates);
	Array<double> Fc_R3 = OsimModel.getForceSet().get("Foot_Ground_R3").getRecordValues(defaultStates);
	Array<double> Fc_R4 = OsimModel.getForceSet().get("Foot_Ground_R4").getRecordValues(defaultStates);
	Array<double> Fc_R5 = OsimModel.getForceSet().get("Foot_Ground_R5").getRecordValues(defaultStates);
	Array<double> Fc_R6 = OsimModel.getForceSet().get("Foot_Ground_R6").getRecordValues(defaultStates);
	// Array<double> Fc_R7 = OsimModel.getForceSet().get("Foot_Ground_R7").getRecordValues(defaultStates);
	// Array<double> Fc_R8 = OsimModel.getForceSet().get("Foot_Ground_R8").getRecordValues(defaultStates);
	
	Array<double> Fc_L1 = OsimModel.getForceSet().get("Foot_Ground_L1").getRecordValues(defaultStates);
	Array<double> Fc_L2 = OsimModel.getForceSet().get("Foot_Ground_L2").getRecordValues(defaultStates);
	Array<double> Fc_L3 = OsimModel.getForceSet().get("Foot_Ground_L3").getRecordValues(defaultStates);
	Array<double> Fc_L4 = OsimModel.getForceSet().get("Foot_Ground_L4").getRecordValues(defaultStates);
	Array<double> Fc_L5 = OsimModel.getForceSet().get("Foot_Ground_L5").getRecordValues(defaultStates);
	Array<double> Fc_L6 = OsimModel.getForceSet().get("Foot_Ground_L6").getRecordValues(defaultStates);
	// Array<double> Fc_L7 = OsimModel.getForceSet().get("Foot_Ground_L7").getRecordValues(defaultStates);
	// Array<double> Fc_L8 = OsimModel.getForceSet().get("Foot_Ground_L8").getRecordValues(defaultStates);
	
	for (int fc=0;fc<=5;fc++)
	{
         *(grfs+fc) 	= Fc_R1[fc+6];
		 *(grfs+6+fc)  	= Fc_R2[fc+6];
         *(grfs+12+fc) 	= Fc_R3[fc+6];
		 *(grfs+18+fc) 	= Fc_R4[fc+6];
         *(grfs+24+fc) 	= Fc_R5[fc+6];
		 *(grfs+30+fc) 	= Fc_R6[fc+6];
		 // *(grfs+36+fc) 	= Fc_R7[fc+6];
		 // *(grfs+42+fc) 	= Fc_R8[fc+6];
		 
         *(grfs+36+fc) 	= Fc_L1[fc+6];
		 *(grfs+42+fc) 	= Fc_L2[fc+6];
         *(grfs+48+fc) 	= Fc_L3[fc+6];
		 *(grfs+54+fc) 	= Fc_L4[fc+6];
         *(grfs+60+fc) 	= Fc_L5[fc+6];
		 *(grfs+66+fc) 	= Fc_L6[fc+6];
		 // *(grfs+78+fc) 	= Fc_L7[fc+6];
		 // *(grfs+90+fc) 	= Fc_L8[fc+6];
	}

	// OsimModel.print("anal.osim");
}