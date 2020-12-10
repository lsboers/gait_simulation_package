/* -- Include various simbody/ opensim source file */
#include "mex.h"
#include <OpenSim/OpenSim.h>
#include <iostream>

/* -- Use the OS namespaces to prevent double declarations */
using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* -- initiate model and receive states */
Model OsimModel("analysis.osim");
SimTK::State &defaultStates 		= OsimModel.initSystem();
Array<string> stateNames 			= OsimModel.getStateVariableNames();
const ControllerSet &controllerSet 	= OsimModel.getControllerSet();

/* -- The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
			     int nrhs, const mxArray *prhs[] )
{
	/* -- Initial paramater allocation */
	int stateSequence, actuatorIndx, indx;

	/* -- Initiate model and receive states */
    double *parameters           	= mxGetPr(prhs[0]);
    double *stateSequence_d   		= mxGetPr(prhs[1]);
    double *nFreeDOFs_d  	        = mxGetPr(prhs[2]);
		int nFreeDOFs 				= (int)nFreeDOFs_d[0];
	double *nActuators_d  	        = mxGetPr(prhs[3]);
		int nActuators 				= (int)nActuators_d[0];
	double *nSpheres_d 				= mxGetPr(prhs[4]);
		int nSpheres 				= (int)nSpheres_d[0];
	double *currentTimePercentage   = mxGetPr(prhs[5]);
    double *t0 			            = mxGetPr(prhs[6]);
    double *tf                     	= mxGetPr(prhs[7]);

	/* -- output variable declarations */
    plhs[0] 						= mxCreateDoubleMatrix(1, (2*nFreeDOFs), mxREAL);//derivatives of states 
	plhs[1] 						= mxCreateDoubleMatrix(1, nSpheres * 12, mxREAL);//3 forces and 3 torques for each contact pair
	double *derivatives 			= mxGetPr(plhs[0]);
    double *grfs 					= mxGetPr(plhs[1]);
   
    defaultStates.setTime((*(tf)-*(t0))*(*(currentTimePercentage))+*(t0));
   	/* -- Set all state variables */
	for (indx=0; indx<= 2*(nFreeDOFs) - 1; indx++)
	{
	    stateSequence      = (int)stateSequence_d[indx];
		OsimModel.setStateVariable(defaultStates,stateNames[stateSequence],*(parameters + indx));
	}
	
    /* -- Set all control values */
	actuatorIndx 	= 0;
	PrescribedController* controller = dynamic_cast<PrescribedController*>( &controllerSet.get(0) );
	while (indx < 2*(nFreeDOFs) + nActuators)
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
	// Array<double> Fc_R5 = OsimModel.getForceSet().get("Foot_Ground_R5").getRecordValues(defaultStates);
	// Array<double> Fc_R6 = OsimModel.getForceSet().get("Foot_Ground_R6").getRecordValues(defaultStates);
	
	Array<double> Fc_L1 = OsimModel.getForceSet().get("Foot_Ground_L1").getRecordValues(defaultStates);
	Array<double> Fc_L2 = OsimModel.getForceSet().get("Foot_Ground_L2").getRecordValues(defaultStates);
	Array<double> Fc_L3 = OsimModel.getForceSet().get("Foot_Ground_L3").getRecordValues(defaultStates);
	Array<double> Fc_L4 = OsimModel.getForceSet().get("Foot_Ground_L4").getRecordValues(defaultStates);
	// Array<double> Fc_L5 = OsimModel.getForceSet().get("Foot_Ground_L5").getRecordValues(defaultStates);
	// Array<double> Fc_L6 = OsimModel.getForceSet().get("Foot_Ground_L6").getRecordValues(defaultStates);
	
	for (int fc=0;fc<=5;fc++)
	{
         *(grfs+fc) 	= Fc_R1[fc+6];
		 *(grfs+6+fc)  	= Fc_R2[fc+6];
         *(grfs+12+fc) 	= Fc_R3[fc+6];
		 *(grfs+18+fc) 	= Fc_R4[fc+6];
         // *(grfs+24+fc) 	= Fc_R5[fc+6];
		 // *(grfs+30+fc) 	= Fc_R6[fc+6];
		 
         *(grfs+24+fc) 	= Fc_L1[fc+6];
		 *(grfs+30+fc) 	= Fc_L2[fc+6];
         *(grfs+36+fc) 	= Fc_L3[fc+6];
		 *(grfs+42+fc) 	= Fc_L4[fc+6];
         // *(grfs+60+fc) 	= Fc_L5[fc+6];
		 // *(grfs+66+fc) 	= Fc_L6[fc+6];
	}


}