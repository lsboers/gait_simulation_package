% The script assumes correctly configured OpenSim and BTK installations. Latest
% edits of script were performed with Matlab 2017b and OpenSim 3.3

% To use the script properly, press the run button instead of sectionalized
% running or f9 testing. If only parts need to be accessed within the
% script, try using breakpoints or return statements.

%-------------------------------------------------------------------------%
%------------- Initialize workspace ---------------%
%-------------------------------------------------------------------------%
% Preparing command window, workspace and backgrounds
clearvars; close all; clc;
import org.opensim.modeling.*

% % whose subject-data are we using
% file_osim_array   = {'fullbody7.osim'};
% file_osim_array   = {'staticP032_fullbody7_SCALED.osim', ... 
%     'staticP052_fullbody7_SCALED.osim', ...
%     'staticP062_fullbody7_SCALED.osim'};
% path_osim   = 'C:\Users\lsboe\Documents\gait_simulation_package\skeletal_models\';
% file_c3d_array    = {'staticP122.c3d'};
% file_c3d_array    = {'normalP032.c3d', ...
%     'normalP052.c3d', ...
%     'normalP062.c3d'};
% path_c3d    = 'C:\Users\lsboe\Documents\gait_simulation_package\experimental_data\';
% massmat = [72];% % Shortcuts
% whose subject-data are we using
file_osim_array   = {'fullbody7.osim'};
file_osim_array   = {'staticP092_fullbody7_SCALED.osim'};
path_osim   = 'C:\Users\lsboe\Documents\gait_simulation_package\skeletal_models\';
file_c3d_array    = {'staticP042.c3d'};
file_c3d_array    = {'normalP092.c3d'};
path_c3d    = 'C:\Users\lsboe\Documents\gait_simulation_package\experimental_data\';
massmat = [72];% % Shortcuts
% massmat = [80;59.7;59.7;76.3;56.8;84.7];% % Shortcuts

for p = 1:length(file_c3d_array)
    
    % Consider adding paths add the beginning of the runall file, in a similar
    % fashion as to how the configureOpensim matlab script performs this task
    %%%%%%%%%%%%%%%%%%%
    % TUTORIAL;
    % http://biomechanical-toolkit.github.io/docs/Wrapping/Matlab/_tutorial.html
    path_btk        = 'C:\btk-0.2.1_Win7_MatlabR2009b_64bit\BTK\share\btk-0.2\Wrapping\Matlab\btk';
    path_scripts    = 'C:\Users\lsboe\Documents\gait_simulation_package\scripts';
    addpath(path_btk, path_scripts)
    clear path_btk path_scripts
    
    % Configure path settings (assuming the Run button was pressed)
    path_cur        = fileparts(mfilename('fullpath'));
    cd([path_cur,'/..']);
    path_main       = cd;
    clear path_cur
    
    % Select the current subject and it's data
    file_osim = file_osim_array{p};
    file_c3d = file_c3d_array{p};
    
    run_opensim_algorithms = 0;
    
    %% Preprocessing    : Locating and loading required experimental data files
    % Load the c3d file using BTK toolbox
    data_raw = btkReadAcquisition([path_c3d file_c3d]);
    
    % Create readable and editable data from c3d file as a struct using the prepData function
    data = read_c3d_data(data_raw, file_c3d);
    data.Mass = massmat; % our c3d files didn't have mass data, so we manually insert them
    
    % Create motion (.trc) & grf (.mot) files
    data = create_motion_and_grf_files(data, file_c3d, path_c3d, file_osim);
    [path_trc, Name, Handle] = fileparts(data.TRC_Filename);
    file_trc = [Name Handle];
    if isfield(data, 'GRF_Filename')
        [path_grfmot, Name, Handle] = fileparts(data.GRF_Filename);
        file_grfmot = [Name Handle];
    end
    clear Name Handle data_raw fileID nmaerkNames nmarkers S startTime subjName ...
        markerNames
    
    %% Processing data OS
    
    if run_opensim_algorithms == 0
        cd([path_main, '\RRA\results'])
        data.states  = [data.Name '_states.sto'];
        data.controls    = [data.Name '_controls.sto'];
        file_osimRAD       = [data.Name '_RAD.osim'];
    else
        %% Processing       : OS Scale function
        
        if ~strcmp(file_osim(end-10:end-5),'SCALED') % If current osim file ends with scaled, scip scaling
            % Create marker -and taskset files to be used during scaling
            % ... markerset; position of markers on osim model for scaling
            % ... taskset; give weights on how heavily to track markers
            cd([path_main,'/scaling']);
            data = create_marker_and_task_set_file(data);
            
            % Create measurement set scale controls to be applied during scaling.
            data = create_measurement_scale_file(data);
            
            % Create shortcuts to used files
            reqFiles.scaleMarkerSet        = data.markerSet_Filename;
            reqFiles.scaleTasks            = data.taskSet_Filename;
            reqFiles.scaleMeasurements     = data.measurementSet_Filename;
            
            % Initiate scale function
            disp('Initiating OS scale sequence')
            data = setup_scale_file(data, file_osim, file_trc, reqFiles);
            command = ['scale -S ' data.Name '_Setup_Scale.xml'];
            system(command);
            
            % Secondary scale function
            disp('Initiating second OS scale sequence')
            [~,y,z] = fileparts(data.newScaledOsim_Filename);
            data = setup_scale_file2(data, [y z], file_trc, reqFiles);
            command = ['scale -S ' data.Name '_Setup_Scale2.xml'];
            system(command);
            
            disp('OS Scale sequence complete')
            [~, x, y] = fileparts(data.newScaledOsim_Filename);
            file_osim = [x y];
            clear x y z ans
            return
        else
            disp('Model already scaled, continuing to IK')
        end
        
        %% Processing       : OS IK function
        % Create marker set AND task set file to be applied on the raw model
        cd([path_main,'/inverse_kinematics']);
        data = create_marker_and_task_set_file(data);
        
        % Shortcut to required file
        reqFiles.IKTasks               = data.taskSet_Filename;
        
        % Initiate IK function
        if run_opensim_algorithms == 0
            disp(['Motion data was found for ', data.Name, ', IK setup stopped'])
            data.IK_Filename = [data.Name '_ik.mot'];
        else
            data = setup_IK_file(data, file_osim, file_trc, reqFiles);
            command = ['ik -S ' data.Name '_Setup_InverseKinematics.xml'];
            system(command);
        end; clear ans command
        
        % Load and save kinematics in data struct
        [headersKinematics, dataKinematics] = read_dot_mot(data.IK_Filename);
        data.Kinematics = dataKinematics;
        data.KinematicsHeaders = headersKinematics;
        clear headersKinematics dataKinematics
        
        %% Processing       : OS RRA function
        % Create grf/ external loads file and the rra tasks file
        % ... RRA tasks; allocate tracking tasks for the algorithm
        cd([path_main '/RRA/'])
        firstStep = data.firstStep;
        data = create_external_loads_file(data, firstStep);
        data = create_RRA_task_file(data, [path_osim, '/', file_osim]);
        
        % Create variables containing coordinates (to actuate about) and
        % coordinates of coms for application of forces
        cd([path_main '/skeletal_models/'])
        model = Model(file_osim);
        bodSet = model.getBodySet;
        % ... Change mass of model to match subject
        currTotalMass = get_mass_of_model(model);
        massScaleFactor = data.Mass/currTotalMass;
        
        for i = 0:bodSet.getSize()-1
            currBodyMass = bodSet.get(i).getMass();
            newBodyMass = currBodyMass*massScaleFactor;
            bodSet.get(i).setMass(newBodyMass);
        end
        osimModel_rraMassChanges = model;
        file_osimRAD = [data.Name '_RAD.osim'];
        osimModel_rraMassChanges.print(file_osimRAD);
        model = Model(file_osimRAD);
        bodSet = model.getBodySet;
        coordSet = model.getCoordinateSet;
        
        % ... Get COMs of all all bodies found in the osim model
        coms = zeros(bodSet.getSize(),3);
        for i = 0:bodSet.getSize()-1
            massCenter = ArrayDouble.createVec3([0, 0, 0]);
            bodSet.get(i).getMassCenter(massCenter);
            massCenter = char(massCenter);
            coms(i+1,:) = str2num(massCenter(3:end-1));
        end; clear i bodSet massCenter
        % ... Get coordinates of osim model
        coords = cell(coordSet.getSize(),1);
        for i = 0:coordSet.getSize()-1
            coName = coordSet.get(i);
            if coName.get_locked
                coords(i+1,:) = cellstr('nan');
            else
                coStrName = char(coName);
                if strcmp('beta',coStrName(end-3:end))
                    coords(i+1,:) = cellstr('nan');
                else
                    coords(i+1,:) = cellstr(coStrName);
                end
            end
        end; clear i Coordinates coName coStrName osimMassCenterChanger
        coords(strcmp('nan',coords)) = '';
        
        % Now create RRA Actuator set xml file
        cd([path_main '/RRA/'])
        data = create_RRA_actuator_file(data,coms,coords, [path_osim, '/', file_osim]);
        disp('Done creating .osim files, continue with script')
        
        reqFiles.RRAgrfFile            = data.externalLoads_Filename;
        reqFiles.RRAactuatorFile       = data.RRA_Actuator_Filename;
                reqFiles.RRAconstraintFile     = 'constraints.xml';
        reqFiles.RRAtaskFile           = 'rra_tasks_walking.xml';
%         reqFiles.RRAtaskFile = data.RRA_Tasks_Filename;
        
        cd([path_main '/RRA'])
        % Run RRA
        data = setup_RRA_file(data, file_osimRAD, file_grfmot, reqFiles);
        command = ['rra -S ' data.Name '_Setup_ReduceResiduals.xml'];
        system(command);
        
        file_osimRAD = [data.Name '_RAD.osim'];
        cd([path_main '/RRA'])
        outlog = fileread('out.log');
        massChange = str2double(outlog(regexpi(outlog, 'total mass change: ', 'end'):regexpi(outlog, 'total mass change: .?[0-9]+[.][0-9]+', 'end')));
        disp(['walk, rra 1, dMass = ', num2str(massChange)])
        dCOM = outlog(regexpi(outlog, 'Mass Center \(COM\) adjustment:', 'end'):regexpi(outlog, 'Mass Center \(COM\) adjustment: .+]', 'end'));
        disp(['walk, rra 1, dCOM = ', dCOM])
        cd([path_main '/skeletal_models'])
        osimModel_rraMassChanges = Model(file_osimRAD);
        osimModel_rraMassChanges = set_model_masses(osimModel_rraMassChanges, massChange);
        osimModel_rraMassChanges.print(file_osimRAD);
        
        while abs(massChange) > .25

            % again
            % Now create RRA Actuator set xml file
            cd([path_main '/RRA/'])
            data = create_RRA_actuator_file(data,coms,coords, [path_osim, '/', file_osim]);
            disp('Done creating .osim files, continue with script')

            reqFiles.RRAgrfFile            = data.externalLoads_Filename;
            reqFiles.RRAactuatorFile       = data.RRA_Actuator_Filename;
                    reqFiles.RRAconstraintFile     = 'constraints.xml';
            reqFiles.RRAtaskFile           = 'rra_tasks_walking3.xml';

            cd([path_main '/RRA'])
            % Run RRA
            data = setup_RRA_file(data, file_osimRAD, file_grfmot, reqFiles);
            command = ['rra -S ' data.Name '_Setup_ReduceResiduals.xml'];
            system(command);

            file_osimRAD = [data.Name '_RAD.osim'];
            cd([path_main '/RRA'])
            outlog = fileread('out.log');
            massChange = str2double(outlog(regexpi(outlog, 'total mass change: ', 'end'):regexpi(outlog, 'total mass change: .?[0-9]+[.][0-9]+', 'end')));
            disp(['walk, rra 1, dMass = ', num2str(massChange)])
            dCOM = outlog(regexpi(outlog, 'Mass Center \(COM\) adjustment:', 'end'):regexpi(outlog, 'Mass Center \(COM\) adjustment: .+]', 'end'));
            disp(['walk, rra 1, dCOM = ', dCOM])
            cd([path_main '/skeletal_models'])
            osimModel_rraMassChanges = Model(file_osimRAD);
            osimModel_rraMassChanges = set_model_masses(osimModel_rraMassChanges, massChange);
            osimModel_rraMassChanges.print(file_osimRAD);

        end



        data.states = [data.Name '_states.sto'];
        data.controls   = [data.Name '_controls.sto'];

        
        
%     continue    
    end
    
    %% Processing       : Initiating, locating and preparing DC variables
    disp('-----> Step 1: Preparing working directory & variables for DC')
    % Clear workspace
    clearvars -except p data file_grfmot file_osim file_osimp file_osimRAD file_osimRAD2 file_trc ...
        path_c3d path_grfmot path_main path_osim path_trc logicRRA file_c3d file_c3dp massmat
    
    % Create set of required variables
    file_motion         = data.states;
    file_controls       = data.controls;
    path_motion         = cd;
    if exist('file_osimRAD')
        file_osim = file_osimRAD;
    end
    
    % Initial patient data
    subject_rra_file = file_osim(1:end-5);
    cd([path_main,'/skeletal_models']);
    osimModel = Model(file_osim);
    BW = get_mass_of_model(osimModel);
    
    % run logic
    parametrize_states_controls = 1;
    create_mex_wrapper          = 1;
    logicsCase                  = 2;
    disp('-----> Step 1: Preparation complete')
    
    %% Processing       : Parametrize data
    
    disp('-----> Step 2: Sample RRA states & controls in 80 Nodes')
    
    % Discretize the CMC data
    nNodes = 80;
    cd([path_main '/RRA/results'])
    if parametrize_states_controls
        fpEventOne = data.fpTimeTOZeroPRED;
        fpEventTwo = data.fpTimeHCThirdPRED;
        required_time_range = [fpEventOne fpEventTwo];
        fileResampledData   = parametrize_states_and_controls ...
            (osimModel, path_motion, file_motion, file_controls, nNodes, required_time_range);
    else
        fileResampledData   = ['Sample' num2str(nNodes) 'nodesfor',file_motion,'.mat'];
    end
    
    disp('-----> Step 2: All Resampled')
    
    %% Processing       : Create initial foot-ground layout
    
    disp('-----> Step 3: Add foot-ground model and ControllerSet to the OpenSim model')
    
    % Info on first step
    firstStep   = data.firstStep;
    firstIC     = data.fpTimeHCFirst;
    secondIC    = data.fpTimeHCSecond;
    firstTO     = data.fpTimeTOFirst;
    secondTO    = data.fpTimeTOSecond;
    
    % Get scaledset name + file
    try
        cd([path_main, '\scaling'])
        scaledfileName  = char(osimModel.getName);
        scaleSetName    = [scaledfileName(1:end-7), '_ScaleSet_Applied.xml'];
        scaleSetStruct  = xml2struct(scaleSetName);
    catch
        scaleSetStruct  = [];
    end
    
    % Add markers structure + define sphere props
    % cd([path_main, '\CMC\walk'])
    OsimModel_FG    = Model([path_osim, file_osim]);
    % .. addMarkers + the print below is purely for visualization
    [OsimModel_FG, locL, locR, rads]  = add_temporary_markers(OsimModel_FG, scaleSetStruct, logicsCase);
    OsimModel_FG.print([path_main, '/direct_collocation_files/', [file_osim(1:end-5), '_Marked.osim']]);
    % ..
    OsimModel_FG    = add_hc_spheres([path_osim, file_osim], locL, locR, rads, 0, logicsCase, 0,0);
    OsimModel_FG    = add_controllers(OsimModel_FG);
    file_osimFG     = [file_osim(1:end-5), '_FGContact.osim'];
    OsimModel_FG.print([path_main, '/direct_collocation_files/', file_osimFG]);
    copyfile([path_main, '/direct_collocation_files/', file_osimFG] ,[path_main, '/direct_collocation_files/analysis.osim']);
    clear scaleSetStruct
    
    disp('-----> Step 3: Donezo')
    
    %% Processing       : Prepare DVs for foot-ground model calibration
    
    disp('-----> Step 4: Prepare DVs for foot-ground model calibration')
    % Load resampled data to act as initial guess
    pathInitialGuess = [path_motion, '\', fileResampledData];
    dataInitialGuess = orderfields(load(pathInitialGuess));
    
    % Allocate data variables
    nameSVC     = dataInitialGuess.namesStateVarsControls;
    nSVC        = dataInitialGuess.numStateVarsControls; nPPN = nSVC;
    idxSVC      = dataInitialGuess.indexStateVarsControls;
    datSVC      = dataInitialGuess.matrixStatesControls;
    idxLocked   = dataInitialGuess.indexDOFLocked;
    idxBeta     = dataInitialGuess.indexDOFbeta;
    %
    nameSVF     = dataInitialGuess.namesStateVarsFree;
    nSVF        = dataInitialGuess.numStateVarsFree;
    idxSVF      = dataInitialGuess.indexStateVarsFree;
    idxSVFOS    = dataInitialGuess.indexStateVarsFreeOS;
    %
    namePos     = dataInitialGuess.namesPosition;
    nPos        = dataInitialGuess.numCoordinatesFree;
    idxPos      = dataInitialGuess.indexPosition;
    %
    nameVel     = dataInitialGuess.namesVelocity;
    nVel        = dataInitialGuess.numCoordinatesFree;
    idxVel      = dataInitialGuess.indexVelocity;
    %
    nameControl = dataInitialGuess.namesControls;
    nCon        = dataInitialGuess.numActuators;
    idxCon      = dataInitialGuess.indexControls;
    %
    nSpheres    = length(rads);
    
    % ... and create initial/ final timestamp + step data
    t   = dataInitialGuess.timeVector;
    t0  = t(1); tf = t(end);
    h   = diff(t);
    h   = mean(h);
    clear dataInitialGuess
    
    % Calculate optimum muscle volume and fiber lengths
    % based on eqs 2 and 3 in Correa et al (2011)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Transition to part 2
    cd([path_main, '/direct_collocation_files'])
    OsimModel_FG    = Model(file_osimFG);
    actuatorSet     = OsimModel_FG.getActuators();
    
    % Extract experimental GRFs
    dataFP = readMOT([path_grfmot, '\', file_grfmot]);
    fpData = dataFP.data;
    fpLabels = dataFP.labels;
    fpStartTime = find(t(1) <= fpData(:,1),1);
    fpEndTime = find(t(end) <= fpData(:,1),1);
    % Select both vx (both feet) kinetic data points in the grf.mot file
    fpData = fpData(fpStartTime:fpEndTime,[1,find(contains(fpLabels, '_v'))])';
    %     [b,a] = butter(4,10/500);
    tus = fpData(1,:);
    %     desiredFPs = filtfilt(b,a,fpData(2:end,:)');
    desiredFPs = [tus',fpData(2:end,:)'];
    desiredFPs = resamp3(desiredFPs', t)';
    desiredFPs(:,1)=[];
    if firstStep == 'l'
        desiredFPs = [desiredFPs(:,4:6), desiredFPs(:,1:3)];
    end
    for i = 0:OsimModel_FG.getForceSet.getSize-1
        trueTBD = startsWith(char(OsimModel_FG.getForceSet.get(i)), 'Foot_Ground_');
        if trueTBD == 1
            idxActuatorSphere = i;
            break
        end
    end
    clear dataFP fpData fpLabels
    
    % Extract experimental Powers
    RRAPower = [path_main '\RRA\results\normalP03_Actuation_Power.sto'];
    RRAPower = [path_main '\RRA\results\' data.Name '_Actuation_Power.sto'];
    powerRRA = readMOT(RRAPower);
    powerData = powerRRA.data;
    powerLabels = powerRRA.labels;
    powStartTime = find(t(1) <= powerData(:,1),1);
    powEndTime = find(t(end) <= powerData(:,1),1);
    powerData = powerData(powStartTime:powEndTime,:)';
    powerRRA = resamp3(powerData, t)';
    clear RRAPower powerData powerLabels powStartTime powEndTime
    
    
    residsPelvic = zeros(nNodes,6);
    residsPelvisZero(1:nNodes, 1:6) = residsPelvic;
    if ~(size(locL,2) == 1)
        locL = locL'; locL = locL(:);
    end
    
    %% Linear optimization of foot-model gradient
    
    % How to save /load data as:
    mat_sphere = ['sphereProps' num2str(nSpheres) 'spheres_' subject_rra_file '.mat'];
    
    % If the novel sphere orientation doesn't already exist, load it up
    if ~exist(mat_sphere, 'file')
        
        % Compile the slow version of the mex-file (the one that changes sphere
        % locations and propeties)
        if create_mex_wrapper
            clear mex_sphere_optimization
            mex mex_sphere_optimization.cpp ...
                -Lc:\OpenSim\sdk\lib\ -losimCommon -losimSimulation -losimAnalyses -losimActuators -losimTools -lOpenSim_SimTKcommon -lOpenSim_SimTKmath -lOpenSim_SimTKsimbody -losimJavaJNI -losimLepton -losimSimmFileWriter ...
                -Ic:\OpenSim\sdk\include ...
                -Ic:/OpenSim/sdk/include/SimTK/include -DWIN32 -D_WINDOWS -DNDEBUG
        else
            clear mex
        end
        
        % Get planar locations of sphere on current Y plane + 1 radius
        % acting as DVs for the gradient opt.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %         locLPlanar      = locL; locLPlanar(2:3:end) = [];
        %         locLY1          = locL(2);
        %         DVSphere        = [locLPlanar; repmat(1e+6,nSpheres,1); locLY1];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        locLPlanar      = locL;
        DVSphere        = [locLPlanar; repmat(1e+6,nSpheres,1); rads];
        
        % What are the desired states
        desiredStates 	= datSVC;
        numFGContacts   = OsimModel_FG.getNumContactGeometries()-1;
        nPPN            = size(datSVC,2);
        
        % Craft cost function (simple no gradient)
        poolobj = gcp('nocreate');
        delete(poolobj);
        Cost = @(DVs) optimize_sphere_positions(DVs, ...
            desiredStates, desiredFPs, residsPelvisZero,...
            t, t0, tf, h, nNodes, nPPN, ...
            nPos, numFGContacts, nSVF, nSpheres, ...
            idxSVFOS, idxActuatorSphere, ...
            locL, firstStep);
        
        % Create set of upper- and lower bounds
        ub              = inf(length(DVSphere),1); lb = -inf(length(DVSphere),1);
        lb(1:3:nSpheres*3) = .02;
        ub(1:3:nSpheres*3) = .225;
        lb(2:3:nSpheres*3) = -.005;
        ub(2:3:nSpheres*3) = .005;
        lb(3:3:nSpheres*3) = -0.04;
        ub(3:3:nSpheres*3) = 0.05;
        lb(nSpheres*3+1:nSpheres*4) = 1e+5;
        ub(nSpheres*3+1:nSpheres*4) = 1e+7;
        lb(nSpheres*4+1:nSpheres*5) = .02;
        ub(nSpheres*4+1:nSpheres*5) = .05;
        % Create optimset of fminsearch solver
        options = optimset('fminsearch');
        options = optimset(options, 'MaxIter', 2000, 'Display', 'iter', ...
            'tolFun', 500, 'tolX', 100);
        [sphereparams,~,eFlag,OUT] = fminsearch_bounded(Cost, DVSphere, lb, ub, options);
        save(mat_sphere, 'sphereparams')
        
        % Create new location and size matrices en remake the model
        locLPlanar      = sphereparams(1:nSpheres*3); % Z for heel and XYZ forefoot
        locL(1:3:end)   = locLPlanar(1:3:nSpheres*3);
        locL(2:3:end)   = locLPlanar(2:3:nSpheres*3);
        locL(3:3:end)   = locLPlanar(3:3:nSpheres*3);
        locR            = locL .* repmat([1 1 -1], 1, nSpheres)';
        stiffy          = sphereparams(nSpheres*3+1:nSpheres*4);
        rads            = sphereparams(nSpheres*4+1:nSpheres*5);
        
        % .. rebuild model with improved locations desune
        OsimModel_FG    = add_hc_spheres([path_osim, file_osim], locL, locR, rads, 0, logicsCase, 0, stiffy);
        OsimModel_FG    = add_controllers(OsimModel_FG);
        file_osimFG     = [file_osim(1:end-5), '_FGContact.osim'];
        OsimModel_FG.print([path_main, '/direct_collocation_files/', file_osimFG]);
        copyfile([path_main, '/direct_collocation_files/', file_osimFG] ,[path_main, '/direct_collocation_files/analysis.osim']);
    else
        
        % .. load the 'optimized' sphere parameters.
        paramsTBD = load(mat_sphere);
        sphereparams = paramsTBD.sphereparams;
        % .. rebuild model with improved locations desune
        % Create new location and size matrices en remake the model
        locLPlanar      = sphereparams(1:nSpheres*3); % Z for heel and XYZ forefoot
        locL(1:3:end)   = locLPlanar(1:3:nSpheres*3);
        locL(2:3:end)   = locLPlanar(2:3:nSpheres*3);
        locL(3:3:end)   = locLPlanar(3:3:nSpheres*3);
        locR            = locL .* repmat([1 1 -1], 1, nSpheres)';
        stiffy          = sphereparams(nSpheres*3+1:nSpheres*4);
        rads            = sphereparams(nSpheres*4+1:nSpheres*5);
        
        % .. rebuild model with improved locations desune
        OsimModel_FG    = add_hc_spheres([path_osim, file_osim], locL, locR, rads, 0, logicsCase, 0, stiffy);
        OsimModel_FG    = add_controllers(OsimModel_FG);
        file_osimFG     = [file_osim(1:end-5), '_FGContact.osim'];
        OsimModel_FG.print([path_main, '/direct_collocation_files/', file_osimFG]);
        copyfile([path_main, '/direct_collocation_files/', file_osimFG] ,[path_main, '/direct_collocation_files/analysis.osim']);
    end
    
    %% Using fMinCon to for normal walking RO
    
    mat_filename = ['fsolveOptStatesTracking_' num2str(nNodes) 'Nodes_' num2str(nNodes * 93) 'DVs_' subject_rra_file '.mat'];
    
    if create_mex_wrapper
        clear mex
        mex mex_calculate_state_derivatives.cpp ...
            -Lc:\OpenSim\sdk\lib\ -losimCommon -losimSimulation -losimAnalyses -losimActuators -losimTools -lOpenSim_SimTKcommon -lOpenSim_SimTKmath -lOpenSim_SimTKsimbody -losimJavaJNI -losimLepton -losimSimmFileWriter ...
            -Ic:\OpenSim\sdk\include ...
            -Ic:/OpenSim/sdk/include/SimTK/include -DWIN32 -D_WINDOWS -DNDEBUG
        mex mex_calculate_state_derivatives2.cpp ...
            -Lc:\OpenSim\sdk\lib\ -losimCommon -losimSimulation -losimAnalyses -losimActuators -losimTools -lOpenSim_SimTKcommon -lOpenSim_SimTKmath -lOpenSim_SimTKsimbody -losimJavaJNI -losimLepton -losimSimmFileWriter ...
            -Ic:\OpenSim\sdk\include ...
            -Ic:/OpenSim/sdk/include/SimTK/include -DWIN32 -D_WINDOWS -DNDEBUG
    else
        clear mex
    end
    
    if ~exist(mat_filename, 'file')
        
        designVars = datSVC'; designVars = designVars(:);
        nPPN = size(datSVC,2);
        
        % Create bounds constraints for problem
        boundsLower = -inf(nNodes,nPPN);
        boundsUpper = inf(nNodes,nPPN);
        % .. Qs and dQs
        
        for q = idxPos
            boundsLower(:,q) = datSVC(:,q) - .5*range(datSVC(:,q));
            boundsUpper(:,q) = datSVC(:,q) + .5*range(datSVC(:,q));
            if any(q == 4:6)
                boundsLower(:,q) = datSVC(:,q) - 10*std(datSVC(:,q));
                boundsUpper(:,q) = datSVC(:,q) + 10*std(datSVC(:,q));
            end
        end
        
        for v = idxVel
            boundsLower(:,v) = datSVC(:,v) - (.5*range(datSVC(:,v-nPos)))/h;
            boundsUpper(:,v) = datSVC(:,v) + (.5*range(datSVC(:,v-nPos)))/h;
            if any(v == 35:37)
                boundsLower(:,v) = datSVC(:,v) - (10*std(datSVC(:,v-nPos)))/h;
                boundsUpper(:,v) = datSVC(:,v) + (10*std(datSVC(:,v-nPos)))/h;
            end
        end
        
        for l = idxCon
            %             boundsLower(:,l) = -1;
            %             boundsUpper(:,l) = 1;
            boundsLower(:,l) = datSVC(:,l) - 3*range(datSVC(:,l));
            boundsUpper(:,l) = datSVC(:,l) + 3*range(datSVC(:,l));
            if any(l - [88:93] == 0)
                boundsLower(:,l) = -100;
                boundsUpper(:,l) = 100;
            end
        end
        
        ub = boundsUpper'; ub = ub(:);
        lb = boundsLower'; lb = lb(:);
        
        
        % -- Find gradient of cost and Jacobian of constraints using finite difference
        finitediffmethod = 'central';
        desiredStates = datSVC;
        numFGContacts = nSpheres*2;
        disp(['Working with ' num2str(nSpheres) ' spheres!'])
        % ..Solver settings
        initialX    = ones(length(desiredStates(:)),1);
        deltaX      = eps^(1/3);
        options_fmincon = optimset('fmincon');
        options_fmincon = optimset(options_fmincon, ...
            'Display','iter', ...
            'MaxIter', 200, ...
            'TolFun',1e-5, ...
            'TolX', 1e-5, ...
            'TolCon', 1e-5, ...
            'FinDiffType', 'central', ...
            'GradConstr','on', ...
            'GradObj','on', ...
            'DerivativeCheck','off',...
            'UseParallel', 'always', ...
            'SubproblemAlgorithm','cg', ...
            'OutputFcn', [], ...
            'FinDiffRelStep', deltaX, ...
            'TypicalX', initialX, ...
            'InitBarrierParam',1e-2);
        
        % Calculate initial predictions (X) and GRFs
        for n = 1:nNodes
            [derivs, ~] = mex_calculate_state_derivatives(...
                datSVC(n,:), idxSVFOS, ...
                nPos, (nCon), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
        end; clear n
        
        z =  (diff(datSVC(:, 1:nSVF), [], 1) - h/2 * (predFuncVal(1:end - 1, :) + predFuncVal(2:end, :)));
        max_z = max(abs(z));
        
        
        
        % We trackin
        [FUN, NONLCON] = createCCTrack0907(designVars, ...
            initialX, deltaX, ...
            desiredStates, powerRRA, residsPelvisZero, desiredFPs, max_z, ...
            t, t0, tf, h, nNodes, nPPN, nSpheres, ...
            nPos, nCon, 0, numFGContacts, nSVF, ...
            idxPos, idxVel, idxCon, idxSVFOS,firstStep, namePos, nameControl);
        
        
        poolobj = gcp('nocreate');
        delete(poolobj);
        % Call fmincon
        [DVs_fmincon,Defects,exitflag,output] = fmincon(FUN, designVars, [],[],[],[],lb,ub,NONLCON,options_fmincon);
        X = vec2mat(DVs_fmincon,nPPN);
        
        DefectErrs = Defects;
        
        % -- Save and plot the fsolve solution
        %      X = vec2mat(DVs_fsolve,nPPN);
        %             X(:, idxFib) = X(:, idxFib).*repmat(optFiber',1, nNodes)';
        datSVC_F = X;
        
        % Allocate prediction-variables
        predFuncVal             = zeros(nNodes, nSVF);
        grs                     = zeros(nNodes, nSpheres *12);
        matrixStatesControls    = vec2mat(DVs_fmincon(1:nNodes*nPPN), nPPN);
        %         matrixStatesControls    = matrixStatesControls(:,1:end-6);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for n = 1:nNodes
            [derivs, gr] = mex_calculate_state_derivatives(...
                matrixStatesControls(n,:), idxSVFOS, ...
                nPos, (nCon), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
            grs(n,:) = gr;
        end; clear n
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % -- Use Trapezoidal Method to calculate the defect errors (dimensional
        % reduction cuz of differentiating (z is approximation)?
        contactloads = grs;
        nFGContacts = nSpheres*2;
        if size(contactloads,1) == nNodes
            contactloads = contactloads';
        end
        contactforces = contactloads([1:6:nFGContacts/2*6, ...
            2:6:nFGContacts/2*6, ...
            3:6:nFGContacts/2*6,...
            4:6:nFGContacts/2*6, ...
            5:6:nFGContacts/2*6, ...
            6:6:nFGContacts/2*6, ...
            nFGContacts/2*6+1:6:nFGContacts*6, ...
            nFGContacts/2*6+2:6:nFGContacts*6, ...
            nFGContacts/2*6+3:6:nFGContacts*6, ...
            nFGContacts/2*6+4:6:nFGContacts*6, ...
            nFGContacts/2*6+5:6:nFGContacts*6, ...
            nFGContacts/2*6+6:6:nFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
        PredGRF = [sum(contactforces(1:nSpheres,:)); ...
            sum(contactforces(nSpheres+1:nSpheres*2,:)); ...
            sum(contactforces(nSpheres*2+1:nSpheres*3,:)); ...
            sum(contactforces(nSpheres*3+1:nSpheres*4,:)); ...
            sum(contactforces(nSpheres*4+1:nSpheres*5,:)); ...
            sum(contactforces(nSpheres*5+1:nSpheres*6,:)); ...
            sum(contactforces(nSpheres*6+1:nSpheres*7,:)); ...
            sum(contactforces(nSpheres*7+1:nSpheres*8,:)); ...
            sum(contactforces(nSpheres*8+1:nSpheres*9,:)); ...
            sum(contactforces(nSpheres*9+1:nSpheres*10,:)); ...
            sum(contactforces(nSpheres*10+1:nSpheres*11,:)); ...
            sum(contactforces(nSpheres*11+1:nSpheres*12,:))];
        
        fileName_states_fsolve_sto = ['fsolveOptStatesTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_contrl_fsolve_sto = ['fsolveOptContrlTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_ground_fsolve_mot = ['fsolveOptGroundTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        createStateAndControlSTO('analysis.osim', datSVC_F, nNodes, nPos, nCon, ...
            idxPos, idxVel, idxCon, fileName_states_fsolve_sto, filename_contrl_fsolve_sto, t, idxLocked, nameControl)
        
        saveAsSTOGRF('analysis.osim',PredGRF',[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, filename_ground_fsolve_mot,t, idxLocked)
        
        save(mat_filename,'DVs_fmincon','datSVC','datSVC_F')
        %     saveAsSTO('analysis.osim',datSVC_F,[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, sto_states_fileName,t, idxLocked)
    else
        load(mat_filename)
        
        %         datSVC_F = vec2mat(DVs_fmin(1:nNodes*nPPN),nPPN);
        %         save(mat_fileName,'DVs_fmin', 'datSVC','datSVC_F', 'Defects')
        %         DVs_fmincon = designVars;
        %         datSVC_F = matrixStatesControls;
        %         nPPN = 93;
        % Allocate prediction-variables
        predFuncVal             = zeros(nNodes, nSVF);
        grs                     = zeros(nNodes, nSpheres *12);
        nPPN = 93;
        matrixStatesControls    = vec2mat(DVs_fmincon(1:nNodes*nPPN), nPPN);
        % %         matrixStatesControls    = matrixStatesControls(:,1:end-6);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for n = 1:nNodes
            [derivs, gr] = mex_calculate_state_derivatives(...
                matrixStatesControls(n,:), idxSVFOS, ...
                nPos, (nCon), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
            grs(n,:) = gr;
        end; clear n
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % -- Use Trapezoidal Method to calculate the defect errors (dimensional
        % reduction cuz of differentiating (z is approximation)?
        contactloads = grs;
        nFGContacts = nSpheres*2;
        if size(contactloads,1) == nNodes
            contactloads = contactloads';
        end
        contactforces = contactloads([1:6:nFGContacts/2*6, ...
            2:6:nFGContacts/2*6, ...
            3:6:nFGContacts/2*6,...
            4:6:nFGContacts/2*6, ...
            5:6:nFGContacts/2*6, ...
            6:6:nFGContacts/2*6, ...
            nFGContacts/2*6+1:6:nFGContacts*6, ...
            nFGContacts/2*6+2:6:nFGContacts*6, ...
            nFGContacts/2*6+3:6:nFGContacts*6, ...
            nFGContacts/2*6+4:6:nFGContacts*6, ...
            nFGContacts/2*6+5:6:nFGContacts*6, ...
            nFGContacts/2*6+6:6:nFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
        PredGRF = [sum(contactforces(1:nSpheres,:)); ...
            sum(contactforces(nSpheres+1:nSpheres*2,:)); ...
            sum(contactforces(nSpheres*2+1:nSpheres*3,:)); ...
            sum(contactforces(nSpheres*3+1:nSpheres*4,:)); ...
            sum(contactforces(nSpheres*4+1:nSpheres*5,:)); ...
            sum(contactforces(nSpheres*5+1:nSpheres*6,:)); ...
            sum(contactforces(nSpheres*6+1:nSpheres*7,:)); ...
            sum(contactforces(nSpheres*7+1:nSpheres*8,:)); ...
            sum(contactforces(nSpheres*8+1:nSpheres*9,:)); ...
            sum(contactforces(nSpheres*9+1:nSpheres*10,:)); ...
            sum(contactforces(nSpheres*10+1:nSpheres*11,:)); ...
            sum(contactforces(nSpheres*11+1:nSpheres*12,:))];
        
        fileName_states_fsolve_sto = ['fsolveOptStatesTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_contrl_fsolve_sto = ['fsolveOptContrlTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_ground_fsolve_mot = ['fsolveOptGroundTracking_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        
        createStateAndControlSTO('analysis.osim', datSVC_F, nNodes, nPos, nCon, ...
            idxPos, idxVel, idxCon, fileName_states_fsolve_sto, filename_contrl_fsolve_sto, t, idxLocked, nameControl)
        saveAsSTOGRF('analysis.osim',PredGRF',[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, filename_ground_fsolve_mot,t, idxLocked)
        
    end
    
    
    % Use the STOs created during fsolve and fmincon in order to
    % coordinates of coms for application of forces
    cd([path_main '/skeletal_models/'])
    model = Model(file_osim);
    bodSet = model.getBodySet;
    coordSet = model.getCoordinateSet;
    % ... Get COMs of all all bodies found in the osim model
    coms = zeros(bodSet.getSize(),3);
    for i = 0:bodSet.getSize()-1
        massCenter = ArrayDouble.createVec3([0, 0, 0]);
        bodSet.get(i).getMassCenter(massCenter);
        massCenter = char(massCenter);
        coms(i+1,:) = str2num(massCenter(3:end-1));
    end; clear i bodSet massCenter
    % ... Get coordinates of osim model
    coords = cell(coordSet.getSize(),1);
    for i = 0:coordSet.getSize()-1
        coName = coordSet.get(i);
        if coName.get_locked
            coords(i+1,:) = cellstr('nan');
        else
            coStrName = char(coName);
            if strcmp('beta',coStrName(end-3:end))
                coords(i+1,:) = cellstr('nan');
            else
                coords(i+1,:) = cellstr(coStrName);
            end
        end
    end; clear i Coordinates coName coStrName osimMassCenterChanger
    coords(strcmp('nan',coords)) = '';
    
    % Now create RRA Actuator set xml file
    cd([path_main '/direct_collocation_files/'])
    data = createAnalyzeActuatorfile(data,coms,coords, [path_osim, '/', file_osim]);
    
    % reqFiles.RRAgrfFile            = data.externalLoads_Filename;
    reqFiles.FWDactuatorFile    = data.fwd_Actuator_Filename;
    
    
    controlsFsolve     = filename_contrl_fsolve_sto;
    statesFsolve       = fileName_states_fsolve_sto;
    groundFsolve       = filename_ground_fsolve_mot;
    data = setupAnalysisFile(data, file_osim, statesFsolve, controlsFsolve, reqFiles,t);
    command = ['analyze -S ' data.Name '_Setup_Analysis.xml'];
    system(command);
    DCfsolvePower = ['C:\Users\lsboe\Documents\gait_simulation_package\direct_collocation_files\DCResults\' data.Name '_fsolve_Actuation_Power.sto'];
    powerFminDat = readMOT(DCfsolvePower);
    MaxAbsTargetGRF = repmat(max(abs(desiredFPs),[],1),nNodes,1);
    powerFmin2 = powerFminDat.data;
    plot(sum(powerRRA,2)); hold on
    plot(sum(powerFmin2,2));
    %     figure;
    %     plot(PredGRF([1:3,7:9],:))
    
    
    % end
    
    %% Using fMinCon to induce trunk flexion
    
    mat_filename = ['FminconOptStates_' num2str(nNodes) 'Nodes_' num2str(nNodes * 87) 'DVs_' subject_rra_file '.mat'];
    
    if ~exist(mat_filename, 'file')
        
        % remove pelvic residuals
        nPPN = 87;
        nCon = 25;
        idxCon = idxCon(1:25);
        datSVC_F = datSVC_F(:,1:87);
        nPR = 6;
        
        % Create bounds constraints for problem
        boundsLower = -inf(nNodes,nPPN);
        boundsUpper = inf(nNodes,nPPN);
        
        
        if ~exist('desBackTraj.mat', 'file')
            fivecoeff = [5.02698798812004 -43.8920396008367 146.226930801761 -229.841474408676 167.783188264353 -44.3420776329154];
            desBackTraj = polyval(fivecoeff,t)';
            save('desBackTraj', 'desBackTraj')
        else
            load('desBackTraj')
        end
        desBackTraj = .85*(pchip(t0:((tf-t0)/(120-.5)):tf,desBackTraj,t)');
        
        if firstStep == 'r'
            maxv = max(desBackTraj);
            minv = min(desBackTraj);
            midv = maxv - (maxv-minv)/2;
            desBackTraj = desBackTraj - 2*(desBackTraj-midv);
        end
        desBackTrajD= (diff(desBackTraj))/h;
        desBackTrajD = [desBackTrajD;desBackTrajD(end)];
        desBackTraj = [desBackTraj, desBackTrajD];
        
        
        
        
        
        angle_initial_back = desBackTraj(1);
        angle_end_back = desBackTraj(end);
        if angle_end_back <= 0
            angle_end_back = -0.18;
            anle_initial_back = 0.18;
        else
            angle_end_back = 0.18;
            anle_initial_back = -0.18;
        end
        area_des_back = 14;
        
        idxTrunk            = find(startsWith(nameSVC, 'lumbar_bend'),2);
        %         datSVC_F(:,[20,51]) = desBackTraj;
        designVars = datSVC_F'; designVars = designVars(:);
        nPPN = size(datSVC_F,2);
        
        % Formulate desired states
        desiredStates   = datSVC_F;
        
        
        
        
        for q = idxPos
            boundsLower(:,q) = datSVC_F(:,q) - range(datSVC_F(:,q));
            boundsUpper(:,q) = datSVC_F(:,q) + range(datSVC_F(:,q));
            if any(q - [20] == 0)
                boundsLower(:,q) = -100;
                boundsUpper(:,q) = 100;
            end
        end
        
        for v = idxVel
            boundsLower(:,v) = datSVC_F(:,v) - (range(datSVC_F(:,v-nPos)))/h;
            boundsUpper(:,v) = datSVC_F(:,v) + (range(datSVC_F(:,v-nPos)))/h;
            if any(v - [51] == 0)
                boundsLower(:,v) = -100;
                boundsUpper(:,v) = 100;
            end
        end
        
        for l = idxCon
            boundsLower(:,l) = datSVC_F(:,l) - 3*range(datSVC_F(:,l));
            boundsUpper(:,l) = datSVC_F(:,l) + 3*range(datSVC_F(:,l));
            if any(l - [76] == 0)
                boundsLower(:,l) = -1;
                boundsUpper(:,l) = 1;
            end
        end
        
        
        
        ub = boundsUpper'; ub = ub(:);
        lb = boundsLower'; lb = lb(:);
        
        
        % -- Find gradient of cost and Jacobian of constraints using finite difference
        finitediffmethod    = 'central';
        % .. track pelvis motion during fmincon
        numFGContacts = nSpheres*2;
        %         nSpheres = numFGContacts/2;
        disp(['Working with ' num2str(nSpheres) ' spheres!'])
        
        
        % ..Differentiator settings
        initialX    = 10.^repmat(real(ceil(log10(range(abs(datSVC_F))))),nNodes,1)';
        initialX    = initialX(:);
        deltaX      = eps^(1/3);
        % 11-10-2019 -> we initiate the GRAND SWITCH to fmincon in order to easily
        % implement constraining rules on pelvic coordinates and grf acquidity
        options_fmincon = optimset('fmincon');
        options_fmincon = optimset(options_fmincon, ...
            'Display','iter', ...
            'MaxIter', 200, ...
            'TolFun',1e-5, ...
            'TolX', 1e-5, ...
            'TolCon', 1e-5, ...
            'FinDiffType', 'central', ...
            'GradConstr','on', ...
            'GradObj','on', ...
            'DerivativeCheck','off',...
            'UseParallel', 'always', ...
            'SubproblemAlgorithm','cg', ...
            'OutputFcn', [], ...
            'FinDiffRelStep', deltaX, ...
            'TypicalX', initialX, ...
            'InitBarrierParam', 1);
        
        % We trackin
        [FUN, NONLCON] = createCC283(designVars, ...
            initialX, deltaX, ...
            desBackTraj, desiredStates, area_des_back, powerFmin2, residsPelvisZero, desiredFPs, ...
            t, t0, tf, h, nNodes, nPPN, nSpheres, ...
            nPos, nCon, nPR, numFGContacts, nSVF, ...
            idxPos, idxVel, idxCon, idxSVFOS, idxTrunk, ...
            firstStep, angle_initial_back, angle_end_back);
        poolobj = gcp('nocreate');
        delete(poolobj);
        % Call fmincon
        [DVs_fmincon,Defects,exitflag,output] = fmincon(FUN, designVars, [],[],[],[],lb,ub,NONLCON,options_fmincon);
        X = vec2mat(DVs_fmincon,nPPN);
        
        DefectErrs = Defects;
        
        % -- Save and plot the fsolve solution
        %      X = vec2mat(DVs_fsolve,nPPN);
        %             X(:, idxFib) = X(:, idxFib).*repmat(optFiber',1, nNodes)';
        datSVC_F = X;
        
        % Allocate prediction-variables
        predFuncVal             = zeros(nNodes, nSVF);
        grs                     = zeros(nNodes, nSpheres *12);
        matrixStatesControls    = vec2mat(DVs_fmincon(1:nNodes*nPPN), nPPN);
        matrixStatesControls    = matrixStatesControls(:,1:end-6);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for n = 1:nNodes
            [derivs, gr] = mex_calculate_state_derivatives(...
                matrixStatesControls(n,:), idxSVFOS, ...
                nPos, (nCon), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
            grs(n,:) = gr;
        end; clear n
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % -- Use Trapezoidal Method to calculate the defect errors (dimensional
        % reduction cuz of differentiating (z is approximation)?
        contactloads = grs;
        nFGContacts = nSpheres*2;
        if size(contactloads,1) == nNodes
            contactloads = contactloads';
        end
        contactforces = contactloads([1:6:nFGContacts/2*6, ...
            2:6:nFGContacts/2*6, ...
            3:6:nFGContacts/2*6,...
            4:6:nFGContacts/2*6, ...
            5:6:nFGContacts/2*6, ...
            6:6:nFGContacts/2*6, ...
            nFGContacts/2*6+1:6:nFGContacts*6, ...
            nFGContacts/2*6+2:6:nFGContacts*6, ...
            nFGContacts/2*6+3:6:nFGContacts*6, ...
            nFGContacts/2*6+4:6:nFGContacts*6, ...
            nFGContacts/2*6+5:6:nFGContacts*6, ...
            nFGContacts/2*6+6:6:nFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
        PredGRF = [sum(contactforces(1:nSpheres,:)); ...
            sum(contactforces(nSpheres+1:nSpheres*2,:)); ...
            sum(contactforces(nSpheres*2+1:nSpheres*3,:)); ...
            sum(contactforces(nSpheres*3+1:nSpheres*4,:)); ...
            sum(contactforces(nSpheres*4+1:nSpheres*5,:)); ...
            sum(contactforces(nSpheres*5+1:nSpheres*6,:)); ...
            sum(contactforces(nSpheres*6+1:nSpheres*7,:)); ...
            sum(contactforces(nSpheres*7+1:nSpheres*8,:)); ...
            sum(contactforces(nSpheres*8+1:nSpheres*9,:)); ...
            sum(contactforces(nSpheres*9+1:nSpheres*10,:)); ...
            sum(contactforces(nSpheres*10+1:nSpheres*11,:)); ...
            sum(contactforces(nSpheres*11+1:nSpheres*12,:))];
        
        fileName_states_fsolve_sto = ['FminconOptStates_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_contrl_fsolve_sto = ['FminconOptContrl_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_ground_fsolve_mot = ['FminconOptGround_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        
        createStateAndControlSTO('analysis.osim', datSVC_F, nNodes, nPos, nCon, ...
            idxPos, idxVel, idxCon, fileName_states_fsolve_sto, filename_contrl_fsolve_sto, t, idxLocked, nameControl)
        saveAsSTOGRF('analysis.osim',PredGRF',[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, filename_ground_fsolve_mot,t, idxLocked)
        
        save(mat_filename,'DVs_fmincon','datSVC','datSVC_F')
        %     saveAsSTO('analysis.osim',datSVC_F,[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, sto_states_fileName,t, idxLocked)
    else
        load(mat_filename)
        
        %         datSVC_F = vec2mat(DVs_fmin(1:nNodes*nPPN),nPPN);
        %         save(mat_fileName,'DVs_fmin', 'datSVC','datSVC_F', 'Defects')
        
        %         nPPN = 93;
        % Allocate prediction-variables
        nPPN = 87;
        nCon = 25;
        idxCon = idxCon(1:25);
        predFuncVal             = zeros(nNodes, nSVF);
        grs                     = zeros(nNodes, nSpheres *12);
        matrixStatesControls    = vec2mat(DVs_fmincon(1:nNodes*nPPN), nPPN);
        %         matrixStatesControls    = matrixStatesControls(:,1:end-6);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for n = 1:nNodes
            [derivs, gr] = mex_calculate_state_derivatives(...
                matrixStatesControls(n,:), idxSVFOS, ...
                nPos, (nCon), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
            grs(n,:) = gr;
        end; clear n
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % -- Use Trapezoidal Method to calculate the defect errors (dimensional
        % reduction cuz of differentiating (z is approximation)?
        contactloads = grs;
        nFGContacts = nSpheres*2;
        if size(contactloads,1) == nNodes
            contactloads = contactloads';
        end
        contactforces = contactloads([1:6:nFGContacts/2*6, ...
            2:6:nFGContacts/2*6, ...
            3:6:nFGContacts/2*6,...
            4:6:nFGContacts/2*6, ...
            5:6:nFGContacts/2*6, ...
            6:6:nFGContacts/2*6, ...
            nFGContacts/2*6+1:6:nFGContacts*6, ...
            nFGContacts/2*6+2:6:nFGContacts*6, ...
            nFGContacts/2*6+3:6:nFGContacts*6, ...
            nFGContacts/2*6+4:6:nFGContacts*6, ...
            nFGContacts/2*6+5:6:nFGContacts*6, ...
            nFGContacts/2*6+6:6:nFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
        PredGRF = [sum(contactforces(1:nSpheres,:)); ...
            sum(contactforces(nSpheres+1:nSpheres*2,:)); ...
            sum(contactforces(nSpheres*2+1:nSpheres*3,:)); ...
            sum(contactforces(nSpheres*3+1:nSpheres*4,:)); ...
            sum(contactforces(nSpheres*4+1:nSpheres*5,:)); ...
            sum(contactforces(nSpheres*5+1:nSpheres*6,:)); ...
            sum(contactforces(nSpheres*6+1:nSpheres*7,:)); ...
            sum(contactforces(nSpheres*7+1:nSpheres*8,:)); ...
            sum(contactforces(nSpheres*8+1:nSpheres*9,:)); ...
            sum(contactforces(nSpheres*9+1:nSpheres*10,:)); ...
            sum(contactforces(nSpheres*10+1:nSpheres*11,:)); ...
            sum(contactforces(nSpheres*11+1:nSpheres*12,:))];
        
        
        fileName_states_fsolve_sto = ['FminconOptStates_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_contrl_fsolve_sto = ['FminconOptContrl_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        filename_ground_fsolve_mot = ['FminconOptGround_' num2str(nNodes) 'Nodes_' num2str(length(DVs_fmincon)) 'DVs_' subject_rra_file '.sto'];
        
        createStateAndControlSTO('analysis.osim', datSVC_F, nNodes, nPos, nCon, ...
            idxPos, idxVel, idxCon, fileName_states_fsolve_sto, filename_contrl_fsolve_sto, t, idxLocked, nameControl)
        saveAsSTOGRF('analysis.osim',PredGRF',[], nNodes, nPos, 0, 0, nCon, idxPos, idxVel, [], [], idxCon, filename_ground_fsolve_mot,t, idxLocked)
        
        save(mat_filename,'DVs_fmincon','datSVC','datSVC_F')
        %     saveAsSTO(
    end
    
    
    % Use the STOs created during fsolve and fmincon in order to
    % coordinates of coms for application of forces
    cd([path_main '/skeletal_models/'])
    model = Model(file_osim);
    bodSet = model.getBodySet;
    coordSet = model.getCoordinateSet;
    % ... Get COMs of all all bodies found in the osim model
    coms = zeros(bodSet.getSize(),3);
    for i = 0:bodSet.getSize()-1
        massCenter = ArrayDouble.createVec3([0, 0, 0]);
        bodSet.get(i).getMassCenter(massCenter);
        massCenter = char(massCenter);
        coms(i+1,:) = str2num(massCenter(3:end-1));
    end; clear i bodSet massCenter
    % ... Get coordinates of osim model
    coords = cell(coordSet.getSize(),1);
    for i = 0:coordSet.getSize()-1
        coName = coordSet.get(i);
        if coName.get_locked
            coords(i+1,:) = cellstr('nan');
        else
            coStrName = char(coName);
            if strcmp('beta',coStrName(end-3:end))
                coords(i+1,:) = cellstr('nan');
            else
                coords(i+1,:) = cellstr(coStrName);
            end
        end
    end; clear i Coordinates coName coStrName osimMassCenterChanger
    coords(strcmp('nan',coords)) = '';
    
    % Now create RRA Actuator set xml file
    cd([path_main '/direct_collocation_files/'])
    data = createAnalyzeActuatorfile(data,coms,coords, [path_osim, '/', file_osim]);
    
    % reqFiles.RRAgrfFile            = data.externalLoads_Filename;
    reqFiles.FWDactuatorFile    = data.fwd_Actuator_Filename;
    
    
    controlsFsolve     = filename_contrl_fsolve_sto;
    statesFsolve       = fileName_states_fsolve_sto;
    groundFsolve       = filename_ground_fsolve_mot;
    data = setupAnalysisFile(data, file_osim, statesFsolve, controlsFsolve, reqFiles,t);
    command = ['analyze -S ' data.Name '_Setup_Analysis.xml'];
    system(command);
    DCfsolvePower = ['C:\Users\lsboe\Documents\gait_simulation_package\direct_collocation_files\DCResults\' data.Name '_Fminco_Actuation_Power.sto'];
    powerFminDat = readMOT(DCfsolvePower);
    MaxAbsTargetGRF = repmat(max(abs(desiredFPs),[],1),nNodes,1);
    powerFmin = powerFminDat.data;
    
    %
    %         figure;plot(t, sum(powerFmin2,2));
    %         hold on
    %         plot(t, sum(powerFmin,2))
    %
end


%% Functions
% |------ Pre-Processing
function data = read_c3d_data(data_raw, file_c3d)

% Determining marker coordinates and frames
marker_data.Filename = file_c3d;
[markers, markersInfo] = btkGetMarkers(data_raw);
marker_data.Markers = markers;
marker_data.First_Frame = btkGetFirstFrame(data_raw);
marker_data.Last_Frame = btkGetLastFrame(data_raw);

% Convert data to millimeters if in meters (for whatever reason yaknow)
markerNames = fieldnames(markers);
if abs(mean(markers.(markerNames{1})(:,1))) > 5
    disp('Measurement units were acquired in MM, changed to M for calculations');
    for i = 1:length(markerNames)
        marker_data.Markers.(markerNames{i}) = marker_data.Markers.(markerNames{i})/1000;
    end
    markersInfo.units.ALLMARKERS = 'M';
else
end; clear markers idx

% Convert marker data to the OpenSim coordinate system. Coordinate system
% of OpenSim is structured as x (forward), z (to the right) and y (up). To
% rotate your personal coordinatesystem, enter it's orientation below.
gaitLabCoordinateSystem = 'xyz';
switch gaitLabCoordinateSystem
    case 'xyz'
        for i = 1:length(markerNames)
            marker_data.Markers.(markerNames{i}) = marker_data.Markers.(markerNames{i}) * rotx(90);
        end; clear i
        % If another coordinate system is used, think of the required rotation to
        % achieve the OpenSim coordinate system and write it down below.
    case 'xzy'
    case 'yzx'
    case 'yxz'
    case 'zxy'
    case 'zyx'
    case 'xyz'
    case 'xyz'
    case 'xyz'
    case 'xyz'
    case 'xyz'
end; clear ans i gaitLabCoordinateSystem

% Extend marker data structure and append to data structure
marker_data.Info = markersInfo;
marker_data.Info.NumFrames = btkGetPointFrameNumber(data_raw);
marker_data.Time = (1/marker_data.Info.frequency:...
    1/marker_data.Info.frequency:...
    marker_data.Info.NumFrames/marker_data.Info.frequency)';
marker_data.markerNames = markerNames;
data.marker_data = marker_data;
clear markersInfo marker_data markerNames

% Determining analog channel input
[analogs, analogsInfo] = btkGetAnalogs(data_raw);
analog_data.Channels = analogs;
analog_data.Info = analogsInfo;
analog.data.Info.NumFrames = btkGetAnalogFrameNumber(data_raw);
analog_data.Time = (1/analog_data.Info.frequency:1/analog_data.Info.frequency:...
    analog.data.Info.NumFrames/analog_data.Info.frequency)';
data.analog_data = analog_data;
clear analog analogs analogsInfo analog_data

% Determining forceplate output (if available)
grw_threshold = 10; % N.B.
[FPs, FPsInfo] = btkGetForcePlatforms(data_raw);
grw_data = btkGetGroundReactionWrenches(data_raw, grw_threshold);

% Since everything should be in Nm in this personal script, we divide all
% torque data by 1000
if mean(grw_data(1).P(:,2)) > 5
    grw_data(1).P(:,:) = grw_data(1).P(:,:)/1000;
    grw_data(2).P(:,:) = grw_data(2).P(:,:)/1000;
    grw_data(1).M(:,:) = grw_data(1).M(:,:)/1000;
    grw_data(2).M(:,:) = grw_data(2).M(:,:)/1000;
end

% Convert FP data to the OpenSim coordinate system. Coordinate system
% of OpenSim is structured as x (forward), -z (to the right) and y (up). To
% rotate your personal coordinatesystem, enter it's orientation below.
FPCoordinateSystem = 'xzy';
switch FPCoordinateSystem
    case 'xyz'
    case 'xzy'
        for i = 1:length(grw_data)
            fields = fieldnames(grw_data(i));
            for j = 1:numel(fields)
                grw_data(i).(fields{j}) = grw_data(i).(fields{j}) * rotx(90);
            end; clear j
        end; clear i
        % If another coordinate system is used, think of the required rotation to
        % achieve the OpenSim coordinate system and write it down below.
    case 'yzx'
    case 'yxz'
        for i = 1:length(grw_data)
            fields = fieldnames(grw_data(i));
            for j = 1:numel(fields)
                grw_data(i).(fields{j}) = grw_data(i).(fields{j}) * rotz(-90) * ...
                    [1 0 0;0 1 0;0 0 -1] * rotx(-90);
            end; clear j
        end; clear i
    case 'zxy'
    case 'zyx'
    case 'xyz'
    case 'xyz'
    case 'xyz'
    case 'xyz'
    case 'xyz'
end; clear fields
clear ans i FPCoordinateSystem

% Gather all data together and clear unneccesary data
fp_data.GRF_data = grw_data;
fp_data.Info = FPsInfo;
fp_data.FP_data = FPs;
fp_data.Time = (1/FPsInfo(1).frequency:1/FPsInfo(1).frequency:...
    length(grw_data(1).P)/FPsInfo(1).frequency)';
data.fp_data = fp_data;
clear fp_data grw_data grw_threshold FPs FPsInfo

% Detect footevents
if ~startsWith(file_c3d, 'static')
    % Foot contact event detection first foot
    fpEvents = nan(1,2);
    eventThreshold = .01;
    orderFPs = 1:length(data.fp_data.GRF_data);
    for i = orderFPs
        datForceX = data.fp_data.GRF_data(i).F(:,1);
        datForceY = data.fp_data.GRF_data(i).F(:,2);
        datMarker = data.marker_data.Markers;
        idxEvent = find(datForceY > eventThreshold*max(datForceY));
        
        % Check if it was a feasible time range translating into an actual
        % step:
        lenIdx = find(diff(idxEvent)>1);
        tbd = [];
        for j = 1:length(lenIdx)
            if j == 1
                if lenIdx(j) <= 100
                    tbd = [tbd, 1:lenIdx(j)];
                end
            else
                if lenIdx(j) - lenIdx(j-1) <= 100
                    tbd = [tbd, lenIdx(j-1)+1:lenIdx(j)];
                end
            end
        end; clear j
        idxEvent(tbd) = [];
        %         idxEvent = [(idxEvent(1):idxEvent(1)-1)'; idxEvent];
        
        % Situational logic
        if i == 1
            idxHCFirst = idxEvent(1);
            idxTOFirst = idxEvent(end) + 10;
            % Predicting HC next step
            if length(orderFPs) == 2
                [~,idxMaxPeakX] = max(datForceX);
                [~,idxMinPeakX] = min(datForceX);
                idxHCstepDetect = idxEvent ==  idxMaxPeakX;
                idxTOstepDetect = idxEvent ==  idxMinPeakX;
                idxHCSecondPRED = idxEvent(idxHCstepDetect) - 20;
                idxTOZeroPRED   = idxEvent(idxTOstepDetect) + 30;
                %                 idxEvent = idxEvent(1:idxNESTEDstepDetect - 20);
            end
            timeHCFirst      = idxHCFirst/ data.fp_data.Info(i).frequency;
            timeTOFirst      = idxTOFirst/ data.fp_data.Info(i).frequency;
            timeHCSecondPRED = idxHCSecondPRED/ data.fp_data.Info(i).frequency;
            timeTOZeroPRED   = idxTOZeroPRED/ data.fp_data.Info(i).frequency;
        elseif i == 2
            idxHCSecond = idxEvent(1);
            idxTOSecond = idxEvent(end) + 2;
            % Predicting HC next step
            %             if length(orderFPs) == 2
            %                 [~,idxPeakX] = max(datForceX);
            %                 idxNESTEDstepDetect = idxEvent ==  idxPeakX;
            %                 idxHCThirdPRED = idxEvent(idxNESTEDstepDetect) - 20;
            %                 idxEvent = idxEvent(1:find(idxNESTEDstepDetect));
            %             end
            if length(orderFPs) == 2
                [~,idxMaxPeakX] = max(datForceX);
                idxHCstepDetect = idxEvent ==  idxMaxPeakX;
                idxHCThirdPRED = idxEvent(idxHCstepDetect) - 20;
                idxEvent = idxEvent(1:find(idxHCstepDetect)-40);
            end
            timeHCSecond      = idxHCSecond/ data.fp_data.Info(i).frequency;
            timeTOSecond      = idxTOSecond/ data.fp_data.Info(i).frequency;
            timeHCThirdPRED   = idxHCThirdPRED/ data.fp_data.Info(i).frequency;
        end
        
        
        
        
        % Fill fpEvents with indexes of footcontact
        if isempty(idxEvent)
            
            % Basically throwing when the idxEvent variable is empty
            disp('No foot contact event detected, consider altering event threshold or evaluate fp data')
            fpEvents = [];
        elseif i == 1
            
            % Start at a point where we have GRFs for BOTH legs (meaning if
            % we start at TOzero we have GRFs for both legs as opposed to
            % the situation where the initial trailing leg is touching the
            % ground
            idxEvent = idxEvent(find(idxEvent==idxTOZeroPRED):end);
            
            % Rounding up eventTimes
            idxEventTimes = floor(idxEvent(1)*data.marker_data.Info.frequency/data.fp_data.Info(i).frequency):...
                ceil(idxEvent(end)*data.marker_data.Info.frequency/data.fp_data.Info(i).frequency);
            fpEvents(1,1) = min(min(fpEvents), idxEventTimes(1), 'omitnan');
            fpEvents(1,2) = max(max(fpEvents), idxEventTimes(end), 'omitnan');
            
            % Which foot goes first?
            if datMarker.RCAL(idxEventTimes(1)) > datMarker.LCAL(idxEventTimes(1))
                stepFirst = 'r';
            else
                stepFirst = 'l';
            end
        elseif i == 2
            
            %
            fcTwoMarker = floor(idxEvent(1)*data.marker_data.Info.frequency/data.fp_data.Info(i).frequency);
            idxEventTimes = fcTwoMarker:...
                ceil(idxEvent(end)*data.marker_data.Info.frequency/data.fp_data.Info(i).frequency);
            fpEvents(1,2) = idxEventTimes(end);
        end
    end; clear i tbd
    
    % Add to struct
    data.firstStep = stepFirst;
    data.fpEvents = fpEvents;
    data.fpTimeHCFirst = timeHCFirst;
    data.fpTimeHCSecond = timeHCSecond;
    data.fpTimeHCSecondPRED = timeHCSecondPRED;
    data.fpTimeHCThirdPRED = timeHCThirdPRED;
    data.fpTimeTOFirst = timeTOFirst;
    data.fpTimeTOSecond = timeTOSecond;
    data.fpIdxHCFirst = idxHCFirst;
    data.fpIdxHCSecond = idxHCSecond;
    data.fpIdxHCSecondPRED = idxHCSecondPRED;
    data.fpIdxHCThirdPRED = idxHCThirdPRED;
    data.fpIdxTOFirst = idxTOFirst;
    data.fpIdxTOSecond = idxTOSecond;
    data.fpIdxTOZeroPRED = idxTOZeroPRED;
    data.fpTimeTOZeroPRED = timeTOZeroPRED;
    
else
    data.fpEvents = [];
end

% Determining metadata
mdat = btkGetMetaData(data_raw);
sub_info.Filename = file_c3d;
if isfield(mdat.children,'SUBJECTS')
    sub_info.Name = mdat.children.SUBJECTS.children.NAMES.info.values{1};
    sub_info.MarkerSet = mdat.children.SUBJECTS.children.MARKER_SETS.info.values{1};
else
    disp('Hey now, no name was found in the c3d data, you oughta set a name manually each time but now we take the c3d filenames')
    sub_info.Name = file_c3d(1:end-4);
    sub_info.MarkerSet = file_c3d(1:end-4);
end
if isfield(mdat.children,'PROCESSING')
    proc_fields = fieldnames(mdat.children.PROCESSING.children);
    for i = 1:length(proc_fields)
        sub_info.Processing_Data.(proc_fields{i}) = mdat.children.PROCESSING.children.(proc_fields{i}).info.values;
    end
    if isfield(sub_info.Processing_Data,'Bodymass')
        data.Mass = sub_info.Processing_Data.Bodymass;
    end
    if isfield(sub_info.Processing_Data,'Height')
        data.Height = sub_info.Processing_Data.Height;
    end
end
data.Name = sub_info.Name;
data.sub_info = sub_info;

clear sub_info mdat i proc_fields
end


function data = create_motion_and_grf_files(data, file_c3d, path_c3d, file_osim)

% Initializing scale properties
% Select the start and end frames for the processing
if isempty(data.fpEvents)
    freq = data.marker_data.Info.frequency;
    startFrame = data.marker_data.First_Frame();
    endFrame = data.marker_data.Last_Frame();
else % subtract a few frames corresponding to .03 secs for cmc purposes
    freq = data.marker_data.Info.frequency;
    startFrame = data.fpEvents(1,1);
    if startFrame == 0
        startFrame = data.marker_data.First_Frame();
    end
    
    endFrame = data.fpEvents(1,2)-5;
    %     endFrame = data.fpEvents(1,2)-6;
    
    %     startFrame = 1; endFrame = 499;
end

% Create time matrices to use for scaling data (minus 0.03 for cmc purposes
usedFrames = startFrame:endFrame;
nrows = length(usedFrames);
nmarkers = length(data.marker_data.markerNames);
data.time = (startFrame/freq : 1/freq: endFrame/freq)';

% Create structure framework for exportation of TRC file
% Headers that use XML/ Excel/ Matlab floating properties
upperHeader = 'Frame#\tTime\t';
coordHeader = '\t\t';
floatFormat = '%i\t%2.4f\t';

% Preallocate output matrix
dataTRC = zeros(2+nmarkers*3,nrows);
dataTRC(1:2,:) = [usedFrames; data.time'];
for i = 1:nmarkers
    % Dynamically update headers
    upperHeader = [upperHeader data.marker_data.markerNames{i} '\t\t\t'];
    coordHeader = [coordHeader 'X' num2str(i) '\t' 'Y' num2str(i) '\t'...
        'Z' num2str(i) '\t'];
    floatFormat = [floatFormat '%f\t%f\t%f\t'];
    % NAN check (stolenlol)
    clear m
    m = find(isnan(data.marker_data.Markers.(data.marker_data.markerNames{i})((startFrame:endFrame),1))>0);
    if ~isempty(m)
        clear t d
        disp(['Warning -' data.marker_data.markerNames{i} ' data missing in parts. Frames ' num2str(m(1)) '-'  num2str(m(end))])
        t = data.time;
        t(m) = [];
        d = data.marker_data.Markers.(data.marker_data.markerNames{i})((startFrame:endFrame),:);
        d(m,:) = [];
        data.marker_data.Markers.(data.marker_data.markerNames{i})((startFrame:endFrame),:) = interp1(t,d,time,'linear','extrap');
    end
    dataTRC(i*3:i*3+2,:) = (data.marker_data.Markers.(data.marker_data.markerNames{i})((startFrame:endFrame),:))';
end
clear i m usedFrames

% Extra carriage return to enhance clarity or visibility
upperHeader = [upperHeader '\n'];
coordHeader = [coordHeader '\n'];
floatFormat = [floatFormat '\n'];
disp('Writing trc file...')

%Output marker data to an OpenSim TRC file
file_trc = strrep(file_c3d,'c3d','trc');
data.TRC_Filename = [path_c3d file_trc];

%open the file
fid_1 = fopen([path_c3d file_trc],'w');

% First write the header data
fprintf(fid_1,'PathFileType\t4\t(X/Y/Z)\t %s\n',file_trc);
fprintf(fid_1,'DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n');
fprintf(fid_1,'%d\t%d\t%d\t%d\t%s\t%d\t%d\t%d\n', freq, freq, nrows, nmarkers, data.marker_data.Info.units.ALLMARKERS, freq,startFrame,endFrame);
fprintf(fid_1, upperHeader);
fprintf(fid_1, coordHeader);

% Then write the output marker data
fprintf(fid_1, floatFormat,dataTRC);
fclose(fid_1);
disp('Done.')
clear ans dataTRC floatFormat fid_1 upperHeader coordHeader nrows nmarkers

% Write grf.mot data
if strcmp(file_osim(end-10:end-5),'SCALED')
    % Create time structures of FP data for .mot file
    disp('Writing grf.mot file...')
    ratioFreq = data.fp_data.Info(1).frequency/freq; % assume that all force plates are collected at the same frequency!!!
    fp_time = startFrame/freq:...
        1/data.fp_data.Info(1).frequency:...
        (ratioFreq*endFrame)/data.fp_data.Info(1).frequency;
    clear freq
    
    % initialise force data matrix with the time array and column header
    dataFP = fp_time';
    upperHeaderFP = 'time\t';
    floatFormatFP = '%20.6f\t';
    
    % go through each marker field and re-order from X Y Z to Y Z X and place
    % into data array and add data to the force data matrix --> also need to
    % divide by 1000 to convert to mm from m if necessary and Nmm to Nm
    % these are the conversions usually used in most motion analysis systems
    % and if they are different, just change the scale factor below to p_sc
    % value for the M data to 1. It should however get this from the file.
    
    for i = 1:length(data.fp_data.GRF_data)
        % do some cleaning of the COP before and after contact
        b = find(abs(diff(data.fp_data.GRF_data(i).P(:,3)))>0);
        if ~isempty(b)
            for j = 1:3
                data.fp_data.GRF_data(i).P(1:b(1),j) = data.fp_data.GRF_data(i).P(b(1)+1,j);
                data.fp_data.GRF_data(i).P(b(end):end,j) = data.fp_data.GRF_data(i).P(b(end)-1,j);
            end
        end
        
        % define the period which we are analysing
        K = (ratioFreq*startFrame):1:(ratioFreq*endFrame);
        
        % add the force, COP and moment data for current plate to the force matrix
        dataFP = [dataFP data.fp_data.GRF_data(i).F(K,:) data.fp_data.GRF_data(i).P(K,:) data.fp_data.GRF_data(i).M(K,:)];
        % define the header and formats
        upperHeaderFP = [upperHeaderFP num2str(i) '_ground_force_vx\t' num2str(i) '_ground_force_vy\t' num2str(i) '_ground_force_vz\t'...
            num2str(i) '_ground_force_px\t' num2str(i) '_ground_force_py\t' num2str(i) '_ground_force_pz\t' ...
            num2str(i) '_ground_torque_x\t' num2str(i) '_ground_torque_y\t' num2str(i) '_ground_torque_z\t'];
        floatFormatFP = [floatFormatFP '%20.6f\t%20.6f\t%20.6f\t%20.6f\t%20.6f\t%20.6f\t%20.6f\t%20.6f\t%20.6f\t'];
    end
    clear b i j startFrame endFrame p_sc K ratioFreq
    
    upperHeaderFP = [upperHeaderFP(1:end-2) '\n'];
    floatFormatFP = [floatFormatFP(1:end-2) '\n'];
    
    % assign a value of zero to any NaNs
    dataFP(logical(isnan(dataFP))) = 0;
    
    file_trc = [file_c3d(1:end-4) '_grf.mot'];
    
    data.GRF_Filename = [path_c3d file_trc];
    
    fid_2 = fopen([path_c3d file_trc],'w');
    
    % write the header information
    fprintf(fid_2,'%s\n', file_trc);
    fprintf(fid_2,'version=1\n');
    fprintf(fid_2,'nRows=%d\n',length(fp_time)); % number of datarows
    fprintf(fid_2,'nColumns=%d\n', size(dataFP,2));  % total # of datacolumns
    fprintf(fid_2,'inDegrees=yes\n');
    fprintf(fid_2,'endheader\n');
    fprintf(fid_2,upperHeaderFP);
    clear fp_time upperHeaderFP
    
    % write the data
    fprintf(fid_2,floatFormatFP,dataFP');
    fclose(fid_2);
    disp('Done.')
    clear ans dataFP floatFormatFP fid_2
else
end
end


function [columnHeaders, kinematicData] = read_dot_mot(motFile)
% Open motFile in matlab
fid = fopen(motFile, 'r');

% Preallocate matrix size variables
nRows = 0;
nColumns = 0;

% .mot files give variable data in the header, so we read through the
% header in order to allocate matrix size variables
curLine = fgetl(fid);
while ~startsWith(curLine, 'endheader')
    if startsWith(curLine, 'nRows')
        nRows = str2double(regexp(curLine,'\d*','Match'));
    elseif startsWith(curLine, 'nColumns')
        nColumns = str2double(regexp(curLine,'\d*','Match'));
    end
    curLine = fgetl(fid);
end

% Gain currentLine after the endheader (column headers)
curLine = fgetl(fid);
% ... and extract these column headers
columnHeaders = textscan(curLine,'%s','MultipleDelimsAsOne',1,'Delimiter',sprintf('\t'));
columnHeaders = columnHeaders{:}';
% ... and then extract motion data
kinematicData = fscanf(fid, '%f', [nColumns, nRows])';fclose(fid);
end


% |------ Scaling
function data = create_marker_and_task_set_file(data)

% Allocating roots of the xml file
root = 'MarkerSet';
root2 = 'TaskSet';

% Give a name to the xml file
fileTBD.ATTRIBUTE.name = data.Name;
fileTBD2.ATTRIBUTE.name = data.Name;

% Marker positions
for i = 1:length(data.marker_data.markerNames)
    if (strcmp(char(data.marker_data.markerNames(i)),'LHME') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHME') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LCB') == 0)&& ...
            (strcmp(char(data.marker_data.markerNames(i)),'RCB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'IN') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSTL') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LSTL') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTL') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTL') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTTUB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTTUB') == 0)&& ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTAP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTAP') == 0)&& ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTPD') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTPD') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSL') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RST') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'SAC') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RIC') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LIC') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RGT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LGT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSHT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSHP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LSHT') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LSHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LSHP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTAD') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTAD') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RTPP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LTPP') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RSSH') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LSSH') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'RHB') == 0) && ...
            (strcmp(char(data.marker_data.markerNames(i)),'LHB') == 0)
        
        
        fileTBD.objects.Marker(i).ATTRIBUTE.name = char(data.marker_data.markerNames(i));
        fileTBD2.objects.IKMarkerTask(i).ATTRIBUTE.name = char(data.marker_data.markerNames(i));
    else
        continue
    end
    switch char(data.marker_data.markerNames(i))
        % Shoulder gurdle
        case {'RA', 'RACR'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '-0.00683972 0.413517 0.134403';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LA', 'LACR'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '-0.00683972 0.413517 -0.134403 ';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
            
            % Sternum
        case {'T2'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '-0.091318 0.361676 0';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '4';
        case {'T10'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '-0.0910071 0.151935 0';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '4';
        case {'IJ'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '0.0351111 0.381049 0';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '4';
        case {'SANG', 'SA'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '0.0663815 0.334692 0';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '4';
            
            % Pelvis
        case {'RPSIS'}
            fileTBD.objects.Marker(i).body = 'pelvis';
            fileTBD.objects.Marker(i).location = '-0.155963 0.0462669 0.0435927';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '20';
        case {'LPSIS'}
            fileTBD.objects.Marker(i).body = 'pelvis';
            fileTBD.objects.Marker(i).location = '-0.155963 0.0462669 -0.0435927';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '20';
        case {'RASIS'}
            fileTBD.objects.Marker(i).body = 'pelvis';
            fileTBD.objects.Marker(i).location = '0.0124779 0.0147738 0.121818';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '20';
        case {'LASIS'}
            fileTBD.objects.Marker(i).body = 'pelvis';
            fileTBD.objects.Marker(i).location = '0.0124779 0.0147738 -0.121818';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '20';
            
            %         case {'RTP'}
            %             fileTBD.objects.Marker(i).body = 'femur_r';
            %             fileTBD.objects.Marker(i).location = '-0.00498783 -0.19592 0.0848607';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTT', 'LTL'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '0.0160356 -0.212759 -0.0855404';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTB'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '0.012 -0.206363 -0.090267';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTP'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '-0.00498783 -0.19592 -0.0848607';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RTAP'}
            %             fileTBD.objects.Marker(i).body = 'femur_r';
            %             fileTBD.objects.Marker(i).location = '0.0615444 -0.164605 0.0313692';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTAP'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '0.0615444 -0.164605 -0.0313692';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RTAD'}
            %             fileTBD.objects.Marker(i).body = 'femur_r';
            %             fileTBD.objects.Marker(i).location = '0.0615444 -0.230247 0.0243361';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTAD'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '0.0615444 -0.230247 -0.0243361';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RTPP'}
            %             fileTBD.objects.Marker(i).body = 'femur_r';
            %             fileTBD.objects.Marker(i).location = '-0.0429794 -0.164605 0.0313692';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTPP'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '-0.0429794 -0.164605 -0.0313692';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RTPD'}
            %             fileTBD.objects.Marker(i).body = 'femur_r';
            %             fileTBD.objects.Marker(i).location = '-0.0311155 -0.230247 0.0243361';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTPD'}
            %             fileTBD.objects.Marker(i).body = 'femur_l';
            %             fileTBD.objects.Marker(i).location = '-0.0311155 -0.230247 -0.0243361';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            % Knee/ epicondyles
        case {'RLK', 'RFLE'}
            fileTBD.objects.Marker(i).body = 'femur_r';
            fileTBD.objects.Marker(i).location = '0 -0.395961 0.0520455';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'RMK', 'RFME'}
            fileTBD.objects.Marker(i).body = 'femur_r';
            fileTBD.objects.Marker(i).location = '0.000447072 -0.393705 -0.0521872';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LLK', 'LFLE'}
            fileTBD.objects.Marker(i).body = 'femur_l';
            fileTBD.objects.Marker(i).location = '0 -0.395961 -0.0520455';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LMK', 'LFME'}
            fileTBD.objects.Marker(i).body = 'femur_l';
            fileTBD.objects.Marker(i).location = '0.000447072 -0.393705 0.0521872';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'RHFIB'}
            fileTBD.objects.Marker(i).body = 'tibia_r';
            fileTBD.objects.Marker(i).location = '-0.0024603 -0.0644868 0.0635701';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
        case {'LHFIB'}
            fileTBD.objects.Marker(i).body = 'tibia_l';
            fileTBD.objects.Marker(i).location = '-0.0024603 -0.0644868 -0.0635701';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RTTUB'}
            %             fileTBD.objects.Marker(i).body = 'tibia_r';
            %             fileTBD.objects.Marker(i).location = '0.0548718 -0.0612408 0.00639424';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LTTUB'}
            %             fileTBD.objects.Marker(i).body = 'tibia_l';
            %             fileTBD.objects.Marker(i).location = '0.0548718 -0.0612408 -0.00639424';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            % Shank
            %         case {'RSSH'}
            %             fileTBD.objects.Marker(i).body = 'tibia_r';
            %             fileTBD.objects.Marker(i).location = '0.0328082 -0.271616 -0.00146281';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LSSH'}
            %             fileTBD.objects.Marker(i).body = 'tibia_l';
            %             fileTBD.objects.Marker(i).location = '0.0327492 -0.270295 -0.00667004';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            
            % Talus/ calc
        case {'RHT', 'RCAL'}
            fileTBD.objects.Marker(i).body = 'calcn_r';
            fileTBD.objects.Marker(i).location = '-0.0111553 0.02 0';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '10';
        case {'LHT', 'LCAL'}
            fileTBD.objects.Marker(i).body = 'calcn_l';
            fileTBD.objects.Marker(i).location = '-0.0111553 0.02 0';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '10';
            %         case {'RSTL'}
            %             fileTBD.objects.Marker(i).body = 'calcn_r';
            %             fileTBD.objects.Marker(i).location = '0.0860846 0.0252828 -0.0484846';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LSTL'}
            %             fileTBD.objects.Marker(i).body = 'calcn_l';
            %             fileTBD.objects.Marker(i).location = '0.0860846 0.0252828 0.0484846';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            % Ankle joint/ talus
        case {'RLMAL'}
            fileTBD.objects.Marker(i).body = 'tibia_r';
            fileTBD.objects.Marker(i).location = '-0.00817075 -0.408502 0.0542902';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'RMMAL'}
            fileTBD.objects.Marker(i).body = 'tibia_r';
            fileTBD.objects.Marker(i).location = '0.00884135 -0.3927 -0.0333514';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LLMAL'}
            fileTBD.objects.Marker(i).body = 'tibia_l';
            fileTBD.objects.Marker(i).location = '-0.00817075 -0.408502 -0.0542902';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LMMAL'}
            fileTBD.objects.Marker(i).body = 'tibia_l';
            fileTBD.objects.Marker(i).location = '0.00884135 -0.3927 0.0333514';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '5';
            
            
            %         % Toes
            %         case {'R1M', 'RM1D'}
            %             fileTBD.objects.Marker(i).body = 'toes_r';
            %             fileTBD.objects.Marker(i).location = '0.060078 0.01556429 -0.0187782';
            %             fileTBD.objects.Marker(i).fixed = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
            %         case {'R5M', 'RM5D'}
            %             fileTBD.objects.Marker(i).body = 'toes_r';
            %             fileTBD.objects.Marker(i).location = '0.0350939 0.00856429 0.0510924';
            %             fileTBD.objects.Marker(i).fixed = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
            %         case {'RM2B'}
            %             fileTBD.objects.Marker(i).body = 'calcn_r';
            %             fileTBD.objects.Marker(i).location = '0.155118 0.0256935 -0.00285984';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'L1M', 'LM1D'}
            %             fileTBD.objects.Marker(i).body = 'toes_l';
            %             fileTBD.objects.Marker(i).location = '0.065078 0.01556429 0.0187782';
            %             fileTBD.objects.Marker(i).fixed = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
            %         case {'L5M', 'LM5D'}
            %             fileTBD.objects.Marker(i).body = 'toes_l';
            %             fileTBD.objects.Marker(i).location = '0.060939 0.00856429 -0.0510924';
            %             fileTBD.objects.Marker(i).fixed = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
            %         case {'LM2B'}
            %             fileTBD.objects.Marker(i).body = 'calcn_l';
            %             fileTBD.objects.Marker(i).location = '0.155118 0.0256935 0.00285984';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            % Toes
        case {'R1M', 'RM1D'}
            fileTBD.objects.Marker(i).body = 'toes_r';
            fileTBD.objects.Marker(i).location = '0.00681936 0.0109963 -0.0278487';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '15';
        case {'R5M', 'RM5D'}
            fileTBD.objects.Marker(i).body = 'toes_r';
            fileTBD.objects.Marker(i).location = '-0.0360916 0 0.0452183';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '15';
        case {'RM2B'}
            fileTBD.objects.Marker(i).body = 'calcn_r';
            fileTBD.objects.Marker(i).location = '0.123167 0.0411582 -0.012112';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
        case {'L1M', 'LM1D'}
            fileTBD.objects.Marker(i).body = 'toes_l';
            fileTBD.objects.Marker(i).location = '0.00681936 0.0109963 0.0278487';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '15';
        case {'L5M', 'LM5D'}
            fileTBD.objects.Marker(i).body = 'toes_l';
            fileTBD.objects.Marker(i).location = '-0.0360916 0 -0.0452183';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '15';
        case {'LM2B'}
            fileTBD.objects.Marker(i).body = 'calcn_l';
            fileTBD.objects.Marker(i).location = '0.123167 0.0411582 0.012112';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
            
            % Arms and head (excess)
            %         case {'RHLE'}
            %             fileTBD.objects.Marker(i).body = 'humerus_r';
            %             fileTBD.objects.Marker(i).location = '0.0114493 -0.275937 0.0368024';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'RHLE'}
            fileTBD.objects.Marker(i).body = 'humerus_r';
            fileTBD.objects.Marker(i).location = '0.00832132 -0.279773 -0.0418442';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LHLE'}
            %             fileTBD.objects.Marker(i).body = 'humerus_l';
            %             fileTBD.objects.Marker(i).location = '0.0114493 -0.275937 -0.0368024';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '5';
        case {'LHLE'}
            fileTBD.objects.Marker(i).body = 'humerus_l';
            fileTBD.objects.Marker(i).location = '0.00832132 -0.279773 0.0418442';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
        case {'RUSP'}
            fileTBD.objects.Marker(i).body = 'ulna_r';
            fileTBD.objects.Marker(i).location = '-0.0189607 -0.245498 0.075386';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
        case {'LUSP'}
            fileTBD.objects.Marker(i).body = 'ulna_l';
            fileTBD.objects.Marker(i).location = '-0.0189607 -0.245498 -0.075386';
            fileTBD.objects.Marker(i).fixed = 'true';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'RCB'}
            %             fileTBD.objects.Marker(i).body = 'torso';
            %             fileTBD.objects.Marker(i).location = '0.0825111 0.537301 0.0397761';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %         case {'LCB'}
            %             fileTBD.objects.Marker(i).body = 'torso';
            %             fileTBD.objects.Marker(i).location = '0.0825111 0.537301 -0.0397761';
            %             fileTBD.objects.Marker(i).fixed = 'false';
            %             fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            %             fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%% FALSE OMDAT WE NOG GEEN HEADBAND HEBBEN GEBRUIKT (DE
            %%%%%%%%%%%%%%%%%%%% MARKER ZIT OP EEN HAIRY SPOT).
        case {'IN'}
            fileTBD.objects.Marker(i).body = 'torso';
            fileTBD.objects.Marker(i).location = '-0.0930774 0.550839 0';
            fileTBD.objects.Marker(i).fixed = 'false';
            fileTBD2.objects.IKMarkerTask(i).apply = 'true';
            fileTBD2.objects.IKMarkerTask(i).weight = '1';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
end

% find array elements that have all fields empty:
empty_elems = arrayfun(@(s) all(structfun(@isempty,s)), fileTBD.objects.Marker);
fileTBD.objects.Marker = fileTBD.objects.Marker(~empty_elems);
fileTBD2.objects.IKMarkerTask = fileTBD2.objects.IKMarkerTask(~empty_elems);

% Create the formulated marker -and taskset xml files
fileout = [data.Name '_markerSet.xml'];
fileout2 = [data.Name '_taskSet.xml'];
data.markerSet_Filename = fileout;
data.taskSet_Filename = fileout2;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root, Pref);
xml_write(fileout2, fileTBD2, root2, Pref);
end


function data = create_measurement_scale_file(data)
root = 'MeasurementSet';
fileTBD.ATTRIBUTE.name = data.Name;

measAttNames            = {...
    'pelvis21Y', 'pelvis21X', 'pelvis11Z', ...
    'l_thigh12Y', 'r_thigh12Y', 'l_thigh12XZ', 'r_thigh12XZ', ...
    'l_shanks11Y', 'r_shanks11Y', 'l_shanks11Z', 'r_shanks11Z', ...
    'l_foot12Z', 'r_foot12Z', 'l_foot21X', 'r_foot21X', ...
    'torso21Y', 'torso11X', 'torso11Z' ...
    'l_humerus11Y', 'r_humerus11Y', ...
    'l_farm13Y', 'r_farm13Y'};

if any(strcmp(data.marker_data.markerNames, 'SANG'))
    measMarkerPairs         = {...
        'SANG LASIS', 'SANG RASIS', 'LASIS LPSIS', 'RASIS RPSIS', 'LASIS RASIS', ...
        'LASIS LLK', 'RASIS RLK', 'LLK LMK', 'LLK LMK',...
        'LLK LLMAL', 'RLK RLMAL', 'LMMAL LLMAL', 'RMMAL RLMAL', ...
        'L1M L5M', 'R1M R5M', 'LHT L1M', 'LHT L5M', 'RHT R1M', 'RHT R5M', ...
        'IJ SANG', 'SANG LASIS', 'SANG RASIS', 'T2 IJ', 'LA RA', ...
        'LA LHLE', 'RA RHLE', ...
        'LHLE LUSP', 'RHLE RUSP'};
else
    measMarkerPairs         = {...
        'SA LASIS', 'SA RASIS', 'LASIS LPSIS', 'RASIS RPSIS', 'LASIS RASIS', ...
        'LASIS LFLE', 'RASIS RFLE', 'LFLE LFME', 'RFLE RFME',...
        'LFLE LLMAL', 'RFLE RLMAL', 'LMMAL LLMAL', 'RMMAL RLMAL', ...
        'LM1D LM5D', 'RM1D RM5D', 'LCAL LM1D', 'LCAL LM5D', 'RCAL RM1D', 'RCAL RM5D', ...
        'SA LASIS', 'SA RASIS', 'T2 IJ', 'LACR RACR', ...
        'LACR LHLE', 'RACR RHLE', ...
        'LHLE LUSP', 'RHLE RUSP'};
end

measBodyScaleNames      = {...
    'pelvis', 'pelvis', 'pelvis', ...
    'femur_l', 'patella_l', 'femur_r', 'patella_r', 'femur_l', 'patella_l', 'femur_r', 'patella_r', ...
    'tibia_l', 'tibia_r', 'tibia_l', 'tibia_r', ...
    'calcn_l', 'toes_l', 'calcn_r', 'toes_r', 'calcn_l', 'calcn_r', ...
    'torso', 'torso', 'torso', ...
    'humerus_l', 'humerus_r', ...
    'radius_l', 'ulna_l', 'hand_l', 'radius_r', 'ulna_r', 'hand_r'};


idx = 0;
idx2 = 0;

for i = 1:length(measAttNames)
    fileTBD.objects.Measurement(i).ATTRIBUTE.name = measAttNames{i};
    fileTBD.objects.Measurement(i).apply = 'true';
    fileTBD.objects.Measurement(i).MarkerPairSet.ATTRIBUTE.name = '';
    if isletter(measAttNames{i}(end-2))
        subA = 4;
        subB = 3;
        %         fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end-2) ' ' measAttNames{i}(end-1) ' ' measAttNames{i}(end)];
    elseif isletter(measAttNames{i}(end-1))
        subA = 3;
        subB = 2;
        %         fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end-1) ' ' measAttNames{i}(end)];
    elseif isletter(measAttNames{i}(end))
        subA = 2;
        subB = 1;
        %         fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end)];
    end
    for j = 1:str2double(measAttNames{i}(end-subA))
        fileTBD.objects.Measurement(i).MarkerPairSet.objects.MarkerPair(j).ATTRIBUTE.name = '';
        fileTBD.objects.Measurement(i).MarkerPairSet.objects.MarkerPair(j).markers = measMarkerPairs{idx+j};
    end
    idx = idx + str2double(measAttNames{i}(end-subA));
    for j = 1:str2double(measAttNames{i}(end-subB))
        fileTBD.objects.Measurement(i).BodyScaleSet.ATTRIBUTE.name = '';
        fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).ATTRIBUTE.name = measBodyScaleNames{idx2+j};
        switch subB
            case 3
                fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end-2) ' ' measAttNames{i}(end-1) ' ' measAttNames{i}(end)];
            case 2
                fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end-1) ' ' measAttNames{i}(end)];
            case 1
                fileTBD.objects.Measurement(i).BodyScaleSet.objects.BodyScale(j).axes = [measAttNames{i}(end)];
        end
    end
    idx2 = idx2 + str2double(measAttNames{i}(end-subB));
    
end

fileout = [data.Name '_measurementSet.xml'];
data.measurementSet_Filename = fileout;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root, Pref);

end


function data = setup_scale_file(data, unscaledModel, markerFile, reqFiles)
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% ScaleTool
fileTBD.ScaleTool.ATTRIBUTE.name = sprintf('%s_SCALED', data.Name);
fileTBD.ScaleTool.mass = char(string(data.Mass));
fileTBD.ScaleTool.notes = 'Unassigned';

% ScaleTool -> GenericModelMaker
fileTBD.ScaleTool.GenericModelMaker.ATTRIBUTE.name = '';
fileTBD.ScaleTool.GenericModelMaker.model_file = ['../skeletal_models/' unscaledModel];
fileTBD.ScaleTool.GenericModelMaker.marker_set_file = reqFiles.scaleMarkerSet;

% ScaleTool -> ModelScaler
fileTBD.ScaleTool.ModelScaler.ATTRIBUTE.name = '';
fileTBD.ScaleTool.ModelScaler.apply = 'true';
fileTBD.ScaleTool.ModelScaler.scaling_order = 'measurements manualScale';
fileTBD.ScaleTool.ModelScaler.MeasurementSet.ATTRIBUTE.file = reqFiles.scaleMeasurements;
% fileTBD.ScaleTool.ModelScaler.ScaleSet.ATTRIBUTE.file = reqFiles.scaleScaleSet;
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(1).scales = '.9 1 .9';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(1).segment = 'toes_r';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(1).apply = 'true';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(2).scales = '.9 1 .9';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(2).segment = 'toes_l';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(2).apply = 'true';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(3).scales = '.5 1 1';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(3).segment = 'calcn_r';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(3).apply = 'true';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(4).scales = '.5 1 1';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(4).segment = 'calcn_l';
% fileTBD.ScaleTool.ModelScaler.ScaleSet.objects.Scale(4).apply = 'true';
fileTBD.ScaleTool.ModelScaler.marker_file = ['../experimental_data/' markerFile];
fileTBD.ScaleTool.ModelScaler.time_range = num2str([data.time(1),data.time(end)]);
fileTBD.ScaleTool.ModelScaler.preserve_mass_distribution = 'true';
fileTBD.ScaleTool.ModelScaler.output_scale_file = sprintf('%s_ScaleSet_Applied.xml', data.Name);

% ScaleTool -> MarkerPlacer
fileTBD.ScaleTool.MarkerPlacer.ATTRIBUTE.name = '';
fileTBD.ScaleTool.MarkerPlacer.apply = 'true';
fileTBD.ScaleTool.MarkerPlacer.optimizer_algorithm = 'ipopt';
fileTBD.ScaleTool.MarkerPlacer.IKTaskSet.ATTRIBUTE.file = reqFiles.scaleTasks;
fileTBD.ScaleTool.MarkerPlacer.marker_file = ['../experimental_data/' markerFile];
fileTBD.ScaleTool.MarkerPlacer.coordinate_file = 'Unassigned';
fileTBD.ScaleTool.MarkerPlacer.time_range = num2str([data.time(floor(end/2)),data.time(end)]);
fileTBD.ScaleTool.MarkerPlacer.output_model_file = ['../skeletal_models/' sprintf('%s_SCALED.osim', [data.Name, '_', unscaledModel(1:end-5)])];          % output model with markers
fileTBD.ScaleTool.MarkerPlacer.output_motion_file = sprintf('%s_static_output.mot', data.Name);

% Exporting newly created scale setup file
fileout = [data.Name '_Setup_Scale.xml'];
data.newScaledOsim_Filename = fileTBD.ScaleTool.MarkerPlacer.output_model_file;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root, Pref);

end


function data = setup_scale_file2(data, scaledModel, markerFile, reqFiles)
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% ScaleTool
fileTBD.ScaleTool.ATTRIBUTE.name = sprintf('%s_SCALED', data.Name);
fileTBD.ScaleTool.mass = 0;
fileTBD.ScaleTool.height = -1;
fileTBD.ScaleTool.age = -1;
fileTBD.ScaleTool.notes = 'Unassigned';

% ScaleTool -> GenericModelMaker
fileTBD.ScaleTool.GenericModelMaker.ATTRIBUTE.name = '';
fileTBD.ScaleTool.GenericModelMaker.model_file = ['../skeletal_models/' scaledModel];
fileTBD.ScaleTool.GenericModelMaker.marker_set_file = 'Unassigned';

% ScaleTool -> ModelScaler
fileTBD.ScaleTool.ModelScaler.ATTRIBUTE.name = '';
fileTBD.ScaleTool.ModelScaler.apply = 'true';
% fileTBD.ScaleTool.ModelScaler.scaling_order = 'manualScale';






fileTBD.ScaleTool.ModelScaler.marker_file = 'Unassigned';
fileTBD.ScaleTool.ModelScaler.time_range = '.1 .2';
fileTBD.ScaleTool.ModelScaler.preserve_mass_distribution = 'false';
fileTBD.ScaleTool.ModelScaler.output_model_file = 'Unassigned';
fileTBD.ScaleTool.ModelScaler.output_scale_file = 'Unassigned';

% ScaleTool -> MarkerPlacer
fileTBD.ScaleTool.MarkerPlacer.ATTRIBUTE.name = '';
fileTBD.ScaleTool.MarkerPlacer.apply = 'true';
fileTBD.ScaleTool.MarkerPlacer.IKTaskSet.ATTRIBUTE.file = reqFiles.scaleTasks;
fileTBD.ScaleTool.MarkerPlacer.marker_file = ['../experimental_data/' markerFile];
fileTBD.ScaleTool.MarkerPlacer.coordinate_file = 'Unassigned';
fileTBD.ScaleTool.MarkerPlacer.time_range = num2str([data.time(floor(end/2)),data.time(end)]);
fileTBD.ScaleTool.MarkerPlacer.output_model_file = ['../skeletal_models/' scaledModel];          % output model with markers
fileTBD.ScaleTool.MarkerPlacer.output_motion_file = sprintf('%s_static_output.mot', data.Name);

% Exporting newly created scale setup file
fileout = [data.Name '_Setup_Scale2.xml'];
data.newScaledOsim_Filename = fileTBD.ScaleTool.MarkerPlacer.output_model_file;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root, Pref);

end


% IK
function data = setup_IK_file(data, model_file, marker_file, reqFiles)
% Setup root and InverseKinematicsTool
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% IKTool
fileTBD.InverseKinematicsTool.ATTRIBUTE.name = data.Name;

% ----- Model File
fileTBD.InverseKinematicsTool.model_file = ['../skeletal_models/' model_file];

% ----- IK Task Set
fileTBD.InverseKinematicsTool.IKTaskSet.ATTRIBUTE.file = ['../inverse_kinematics/' reqFiles.IKTasks];

% ----- Marker and coordinate files
fileTBD.InverseKinematicsTool.marker_file = ['../experimental_data/' marker_file];
fileTBD.InverseKinematicsTool.coordinate_file = 'Unassigned';

% ----- Define the time range
fileTBD.InverseKinematicsTool.time_range = num2str([data.time(1),data.time(end)]);

% ----- Name of motion file output
fileTBD.InverseKinematicsTool.report_errors = 'true';
fileTBD.InverseKinematicsTool.output_motion_file = sprintf('%s_ik.mot', data.Name);

% ----- What is the weight off actually satisfying constraints
fileTBD.InverseKinematicsTool.constraint_weight = Inf;
fileTBD.InverseKinematicsTool.accuracy = 1e-05;


% % define results and input directories
% fileTBD.InverseKinematicsTool.results_directory = ResultsDirectory;
% if isempty(InputDirectory)
%     fileTBD.InverseKinematicsTool.input_directory = cd;
% else fileTBD.InverseKinematicsTool.input_directory = InputDirectory;
% end

% Exporting newly created IK setup file
fileout = [data.Name '_Setup_InverseKinematics.xml'];
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);
data.IK_Filename = fileTBD.InverseKinematicsTool.output_motion_file;

end


% RRA
function data = create_external_loads_file(data, firstStep)
% Setup root and ExternalForceCalcFile
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% External Loads
[~, filename, ~] = fileparts(data.TRC_Filename);
fileTBD.ExternalLoads.ATTRIBUTE.name = 'externalloads';

% ----- ExternalForces
nCpoints = 2;
if firstStep == 'r'
    namesEF = {'Right_GRF', 'Left_GRF'};
    namesAB = {'calcn_r', 'calcn_l'};
else
    namesEF = {'Left_GRF', 'Right_GRF'};
    namesAB = {'calcn_l', 'calcn_r'};
end
for i = 1:nCpoints
    fileTBD.ExternalLoads.objects.ExternalForce(i).ATTRIBUTE.name = namesEF{i};
    fileTBD.ExternalLoads.objects.ExternalForce(i).applied_to_body = namesAB{i};
    fileTBD.ExternalLoads.objects.ExternalForce(i).force_expressed_in_body = 'ground';
    fileTBD.ExternalLoads.objects.ExternalForce(i).point_expressed_in_body = 'ground';
    fileTBD.ExternalLoads.objects.ExternalForce(i).force_identifier = [num2str(i) '_ground_force_v'];
    fileTBD.ExternalLoads.objects.ExternalForce(i).point_identifier = [num2str(i) '_ground_force_p'];
    fileTBD.ExternalLoads.objects.ExternalForce(i).torque_identifier = [num2str(i) '_ground_torque_'];
    [~, x, y] = fileparts(data.GRF_Filename);
    fileTBD.ExternalLoads.datafile=['../experimental_data/' [x y]];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fileTBD.ExternalLoads.external_loads_model_kinematics_file= ['../inverse_kinematics/' data.IK_Filename];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

OutputFile = [x '.xml'];
data.externalLoads_Filename = OutputFile;
Pref.StructItem = false;
xml_write(OutputFile, fileTBD, root, Pref);

end


function data = create_RRA_task_file(data, dirModel)
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';
fileTBD.CMC_TaskSet.ATTRIBUTE.name = 'rraTasks';

% ... Defaults
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).ATTRIBUTE.name = 'default';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).on = 'true';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).wrt_body = '-1';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).express_body = '-1';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).active = 'true false false';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).weight = '1 1 1';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).kp = '100';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).kv = '20';
fileTBD.CMC_TaskSet.defaults.CMC_Joint(1).ka = '1 1 1';

% Get modeldata
import org.opensim.modeling.*
model = Model(dirModel);
coordSet = model.getCoordinateSet();
coords = cell(coordSet.getSize(),1);

% Specific weights
mat_names = ["pelvis_ty", "pelvis_tx", "pelvis_tz", ...
    "pelvis_tilt", "pelvis_list", "pelvis_rotation", ...
    "hip", "lumbar", "ankle", "subtalar"];
mat_weights   = [5, 5, 5, ...
    25, 25, 25, ...
    15, 5, 15, 5];

for i = 0:coordSet.getSize()-1
    
    % Whats the current name
    coName = coordSet.get(i);
    
    % Generic filler
    if ~coName.get_locked
        coStrName = char(coName);
        if ~strcmp('beta',coStrName(end-3:end))
            fileTBD.CMC_TaskSet.objects.CMC_Joint(i+1).ATTRIBUTE.name = coStrName;
            fileTBD.CMC_TaskSet.objects.CMC_Joint(i+1).coordinate = coStrName;
            fileTBD.CMC_TaskSet.objects.CMC_Joint(i+1).weight = '1'; % Special weights can be added later
        end
        
        % Change the special tracking weights
        idx_special = startsWith(coStrName, mat_names);
        if any(idx_special)
            for j = 1:length(mat_names)
                if contains(coStrName, mat_names(j))
                    fileTBD.CMC_TaskSet.objects.CMC_Joint(i+1).weight = mat_weights(j);
                end
            end
        end
    end
end; clear i
fileTBD.CMC_TaskSet.objects.CMC_Joint = fileTBD.CMC_TaskSet.objects.CMC_Joint(all(~cellfun(@isempty,struct2cell(fileTBD.CMC_TaskSet.objects.CMC_Joint))));

% Export actuator file to given directory
fileout = [data.Name '_RRA_Tasks.xml'];
data.RRA_Tasks_Filename = fileout;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);
end


function data = create_RRA_actuator_file(data,coms,coords, dirModel)
% Setup .xml root
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';
fileTBD.ForceSet.ATTRIBUTE.name = 'Lowerbody_resids';

fileTBD.ForceSet.defaults.PointActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.PointActuator.max_force = '10000';
fileTBD.ForceSet.defaults.PointActuator.min_force = '-10000';
fileTBD.ForceSet.defaults.PointActuator.optimal_force = '3';
fileTBD.ForceSet.defaults.PointActuator.point = '0 0 0';
fileTBD.ForceSet.defaults.PointActuator.direction = '1 0 0';
fileTBD.ForceSet.defaults.TorqueActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.TorqueActuator.max_force = '1000';
fileTBD.ForceSet.defaults.TorqueActuator.min_force = '-1000';
fileTBD.ForceSet.defaults.TorqueActuator.optimal_force = '2';
fileTBD.ForceSet.defaults.TorqueActuator.axis = '1 0 0';
fileTBD.ForceSet.defaults.CoordinateActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.CoordinateActuator.max_force = '1000';
fileTBD.ForceSet.defaults.CoordinateActuator.min_force = '-1000';
fileTBD.ForceSet.defaults.CoordinateActuator.optimal_force = '300';



for i = 1:6
    if i == 1 || i == 2 || i == 3
        if i == 1
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FX';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '1 0 0';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '5';
        elseif i == 2
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FY';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '0 1 0';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '6';
        elseif i == 3
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FZ';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '0 0 1';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '5';
        end
        fileTBD.ForceSet.objects.PointActuator(i).body = 'pelvis';
        fileTBD.ForceSet.objects.PointActuator(i).point = num2str(coms(2,:),8);
        fileTBD.ForceSet.objects.PointActuator(i).point_is_global = 'false';
        fileTBD.ForceSet.objects.PointActuator(i).force_is_global = 'true';
        %         fileTBD.ForceSet.objects.PointActuator(i).min_force = '-10000';
        %         fileTBD.ForceSet.objects.PointActuator(i).max_force = '10000';
    end
    if i == 4 || i == 5 || i == 6
        if i == 4
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MX';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '1 0 0';
            
        elseif i == 5
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MY';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '0 1 0';
            
        elseif i == 6
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MZ';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '0 0 1';
        end
        fileTBD.ForceSet.objects.TorqueActuator(i-3).bodyA = 'pelvis';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).bodyB = 'ground';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).optimal_force = '5';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).min_control = '-Inf';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).max_control = 'Inf';
        %         fileTBD.ForceSet.objects.TorqueActuator(i-3).min_force = '-1000';
        %         fileTBD.ForceSet.objects.TorqueActuator(i-3).max_force = '1000';
    end
end; clear i

% Get modeldata
import org.opensim.modeling.*
model = Model(dirModel);
coordSet = model.getCoordinateSet();
coords = cell(coordSet.getSize(),1);

% Specific weights
mat_names = ["hip_flexion", "ankle", "knee", "arm", "elbow", "pro_", "wrist"];
mat_weights   = [500, 500, 350, 150, 150, 150, 150];

% Insert coordinates that are used in the .osim model. Maybe automatically
% load them from .osim file
for i = 0:coordSet.getSize()-1
    
    % Whats the current name
    coName = coordSet.get(i);
    
    % Generic filler
    if ~coName.get_locked
        coStrName = char(coName);
        if ~strcmp('beta',coStrName(end-3:end))
            if ~startsWith(coStrName, "pelvis")
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).ATTRIBUTE.name = [coStrName '_reserve'];
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).coordinate = coStrName;
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).optimal_force = '300';
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).min_control = '-Inf';
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).max_control = 'Inf';
            end
        end
        
        % Change the special tracking weights
        idx_special = startsWith(coStrName, mat_names);
        if any(idx_special)
            for j = 1:length(mat_names)
                if contains(coStrName, mat_names(j))
                    fileTBD.ForceSet.objects.CoordinateActuator(i+1).optimal_force = mat_weights(j);
                end
            end
        end
    end
end; clear i
fileTBD.ForceSet.objects.CoordinateActuator = fileTBD.ForceSet.objects.CoordinateActuator(all(~cellfun(@isempty,struct2cell(fileTBD.ForceSet.objects.CoordinateActuator))));




% Insert coordinates that are used in the .osim model. Maybe automatically
% load them from .osim file
%
% for i = 7:length(coords)
%     fileTBD.ForceSet.objects.CoordinateActuator(i-6).ATTRIBUTE.name = coords{i};
%     fileTBD.ForceSet.objects.CoordinateActuator(i-6).coordinate = coords{i};
%     if startsWith(coords{i},'hip_flexion')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '300';
%     elseif startsWith(coords{i},'hip_adduction')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'hip_rotation')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '100';
%     elseif startsWith(coords{i},'knee_angle')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '300';
%     elseif startsWith(coords{i},'ankle_angle')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '300';
%     elseif startsWith(coords{i},'subtalar_angle')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '100';
%     elseif startsWith(coords{i},'mtp_angle')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '100';
%     elseif startsWith(coords{i},'lumbar_extension')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'lumbar_bending')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'lumbar_rotation')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'arm_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'arm_add')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'arm_rot')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'pro_sup')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     elseif startsWith(coords{i},'elbow_flex')
%         fileTBD.ForceSet.objects.CoordinateActuator(i-6).optimal_force = '200';
%     end
%     fileTBD.ForceSet.objects.CoordinateActuator(i-6).min_control = '-Inf';
%     fileTBD.ForceSet.objects.CoordinateActuator(i-6).max_control = 'Inf';
% end; clear i

% Export actuator file to given directory
fileout = [data.Name '_RRA_Actuators.xml'];
data.RRA_Actuator_Filename = fileout;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);
end


function data = setup_RRA_file(data, model_file, file_grfmot, reqFiles)
% Setup root and InverseKinematicsTool
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% RRA Tool
fileTBD.RRATool.ATTRIBUTE.name = data.Name;

% ----- Model File
fileTBD.RRATool.model_file = ['../skeletal_models/' model_file];

% ----- Replace force set
fileTBD.RRATool.replace_force_set = 'false';

%  ----- Force set files
fileTBD.RRATool.force_set_files = reqFiles.RRAactuatorFile;

% ----- Outpout directory and precision
fileTBD.RRATool.results_directory = ['./results'];
fileTBD.RRATool.output_precision = 8;

% ----- RRA timeframe
fileTBD.RRATool.initial_time = char(string(data.time(1)));
fileTBD.RRATool.final_time = char(string(data.time(end)));

% ----- COMPUTE EQUILIBRIUM WHUT?
fileTBD.RRATool.solve_for_equilibrium_for_auxiliary_states = 'false';

% ----- Integrator properties
fileTBD.RRATool.maximum_number_of_integrator_steps = 20000;
fileTBD.RRATool.maximum_integrator_step_size = 1;
fileTBD.RRATool.minimum_integrator_step_size = 1e-08;
fileTBD.RRATool.integrator_error_tolerance = 1e-05;

% ----- Input Files (see documentation of opensim TK)
fileTBD.RRATool.external_loads_file = reqFiles.RRAgrfFile;
fileTBD.RRATool.desired_kinematics_file = ['../inverse_kinematics/' data.IK_Filename];

% ----- Desired RRA tracking tasks and constraints
fileTBD.RRATool.task_set_file = reqFiles.RRAtaskFile;
fileTBD.RRATool.constraints_file = reqFiles.RRAconstraintFile;
% fileTBD.RRATool.rra_controls_file = RRAControlsFile;

% ----- Defining filter properties
fileTBD.RRATool.lowpass_cutoff_frequency = 10;

% ------ Defining optimizer properties
fileTBD.RRATool.optimizer_algorithm = 'ipopt';
fileTBD.RRATool.optimizer_derivative_dx = .0001;
fileTBD.RRATool.optimizer_convergence_criterion = 1e-005;

% ----- ASets
fileTBD.RRATool.AnalysisSet.objects.BodyKinematics.ATTRIBUTE.name = 'BodyKinematics';
fileTBD.RRATool.AnalysisSet.objects.BodyKinematics.on = 'true';
fileTBD.RRATool.AnalysisSet.objects.Kinematics.step_interval = 10;
fileTBD.RRATool.AnalysisSet.objects.Kinematics.in_degrees = 'true';

% ----- Alter model by changing COM?
fileTBD.RRATool.adjust_com_to_reduce_residuals = 'true';
fileTBD.RRATool.adjusted_com_body = 'torso';

% ----- Time used for calculating residuals
fileTBD.RRATool.initial_time_for_com_adjustment  = -1;
fileTBD.RRATool.final_time_for_com_adjustment  = -1;
fileTBD.RRATool.cmc_time_window = .001;
fileTBD.RRATool.use_fast_optimization_target = 'false';
% Exporting newly created IK setup file
fileTBD.RRATool.output_model_file = ['../skeletal_models/' sprintf('%s_RAD.osim', data.Name)];
fileTBD.RRATool.use_verbose_printing = 'false';
fileout = [data.Name '_Setup_ReduceResiduals.xml'];
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);

end


function data = setup_RRA_file2(data, model_file, file_grfmot, reqFiles)
% Setup root and InverseKinematicsTool
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% RRA Tool
fileTBD.RRATool.ATTRIBUTE.name = data.Name;

% ----- Model File
fileTBD.RRATool.model_file = ['../skeletal_models/' model_file];

% ----- Replace force set
fileTBD.RRATool.replace_force_set = 'false';

%  ----- Force set files
fileTBD.RRATool.force_set_files = reqFiles.RRAactuatorFile;

% ----- Outpout directory and precision
fileTBD.RRATool.results_directory = ['./results'];
fileTBD.RRATool.output_precision = 8;

% ----- RRA timeframe
fileTBD.RRATool.initial_time = char(string(data.time(1)));
fileTBD.RRATool.final_time = char(string(data.time(end)));

% ----- COMPUTE EQUILIBRIUM WHUT?
fileTBD.RRATool.solve_for_equilibrium_for_auxiliary_states = 'true';

% ----- Integrator properties
fileTBD.RRATool.maximum_number_of_integrator_steps = 20000;
fileTBD.RRATool.maximum_integrator_step_size = 1;
fileTBD.RRATool.minimum_integrator_step_size = 1e-08;
fileTBD.RRATool.integrator_error_tolerance = 1e-05;

% ----- Input Files (see documentation of opensim TK)
fileTBD.RRATool.external_loads_file = reqFiles.RRAgrfFile;
fileTBD.RRATool.desired_kinematics_file = ['../inverse_kinematics/' data.IK_Filename];

% ----- Desired RRA tracking tasks and constraints
fileTBD.RRATool.task_set_file = reqFiles.RRAtaskFile2;
% fileTBD.RRATool.constraints_file = RRAConstraintsFile;
% fileTBD.RRATool.rra_controls_file = RRAControlsFile;

% ----- Defining filter properties
fileTBD.RRATool.lowpass_cutoff_frequency = 6;

% ------ Defining optimizer properties
fileTBD.RRATool.optimizer_algorithm = 'ipopt';
fileTBD.RRATool.optimizer_derivative_dx = .0001;
fileTBD.RRATool.optimizer_convergence_criterion = 1e-005;

% ----- Alter model by changing COM?
fileTBD.RRATool.adjust_com_to_reduce_residuals = 'true';
fileTBD.RRATool.adjusted_com_body = 'torso';

% ----- ASets
fileTBD.RRATool.AnalysisSet.objects.BodyKinematics.ATTRIBUTE.name = 'BodyKinematics';
fileTBD.RRATool.AnalysisSet.objects.BodyKinematics.on = 'true';
fileTBD.RRATool.AnalysisSet.objects.Kinematics.step_interval = 10;
fileTBD.RRATool.AnalysisSet.objects.Kinematics.in_degrees = 'true';

% ----- Time used for calculating residuals
fileTBD.RRATool.initial_time_for_com_adjustment  = -1;
fileTBD.RRATool.final_time_for_com_adjustment  = -1;
fileTBD.RRATool.cmc_time_window = .001;

% Exporting newly created IK setup file
fileTBD.RRATool.output_model_file = ['../skeletal_models/' sprintf('%s_RAD2.osim', data.Name)];
fileTBD.RRATool.use_verbose_printing = 'false';
fileout = [data.Name '_Setup_ReduceResiduals2.xml'];
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);

end


function osimModel_rraMassChanges = set_model_masses(osimModel, massChange)
currTotalMass = get_mass_of_model(osimModel);
suggestedNewTotalMass = currTotalMass + massChange;
massScaleFactor = suggestedNewTotalMass/currTotalMass;

allBodies = osimModel.getBodySet();
for i = 0:allBodies.getSize()-1
    currBodyMass = allBodies.get(i).getMass();
    newBodyMass = currBodyMass*massScaleFactor;
    allBodies.get(i).setMass(newBodyMass);
end
osimModel_rraMassChanges = osimModel;
end


function totalMass = get_mass_of_model(osimModel)
totalMass = 0;
allBodies = osimModel.getBodySet();
for i=0:allBodies.getSize()-1
    curBody = allBodies.get(i);
    totalMass = totalMass + curBody.getMass();
end
end


% DC
function OsimModel = add_controllers(OsimModel)
%Use this code to add Controllers to a given OpenSim model.
% Input argument
% OsimModel: an input OpenSim model
% Output argument
% OsimModel: a output OpenSim model

import org.opensim.modeling.*
muscleController= PrescribedController();
muscleController.setActuators(OsimModel.updActuators());
numActuators = OsimModel.getActuators().getSize();
for i=1:numActuators
    muscleController.prescribeControlForActuator(i-1,Constant(1));
end
OsimModel.addController(muscleController)
end


function OsimModel = add_hc_spheres(Model_dir, locL, locR, rads, yG, logicsCase, d2, s2)

% Initiate the model provided by Model_dir
import org.opensim.modeling.*
OsimModel = Model(Model_dir);

% Allocate variables of the initial spheres
stiffness           = 1e+6;
if d2~=0
    dissipation = d2;
else
    dissipation         = repmat(1.5,length(rads),1);
end
if s2~=0
    stiffness = s2;
else
    stiffness         = repmat(1e+6,length(rads),1);
end
sFric               = .8;
dFric               = .8;
vFric               = 0;
TransitionVelocity  = 0.11;
% .. + ground properties
OrientationG        = Vec3(0,0,-pi/2);
if yG ~=0
    LocationG           = Vec3(0,yG,0);
else
    LocationG           = Vec3(0, .01,0);
end

Ground              = ContactHalfSpace();
Ground.setName('ground')
Ground.setBodyName('ground')
Ground.setOrientation(OrientationG)
Ground.setLocation(LocationG)
OsimModel.addContactGeometry(Ground)

if ~(size(locL,2) == 1)
    locL = locL'; locL = locL(:); locL = locL';
    locR = locR'; locR = locR(:); locR = locR';
end

% Transform sphere locations to usable Vec3 variables
for j = 1:length(rads)
    eval(['locLV3(' num2str(j) ') = Vec3(locL(' num2str((j-1)*3 + 1) '), locL(' num2str((j-1)*3 + 2) '), locL(' num2str((j-1)*3 + 3) '));']);
    eval(['locRV3(' num2str(j) ') = Vec3(locR(' num2str((j-1)*3 + 1) '), locR(' num2str((j-1)*3 + 2) '), locR(' num2str((j-1)*3 + 3) '));']);
end; clear j
% for i = 1:size(locL, 1)
%     eval(['locLV3(' num2str(i) ') = Vec3(locL(' num2str(i) ',1), locL(' num2str(i) ',2), locL(' num2str(i) ',3));']);
%     eval(['locRV3(' num2str(i) ') = Vec3(locR(' num2str(i) ',1), locR(' num2str(i) ',2), locR(' num2str(i) ',3));']);
% end

for j = 1:length(locLV3)
    
    %     if j == 2 || j == 3
    %         stiffness = 1e+6;
    %     end
    
    % First of all, create spheres r
    sr = HuntCrossleyForce();
    sr.addGeometry('ground')
    sr.setStiffness(stiffness(j))
    sr.setDissipation(dissipation(j))
    sr.setStaticFriction(.8)
    sr.setDynamicFriction(.8)
    sr.setViscousFriction(0)
    sr.setTransitionVelocity(.11);
    sr.addGeometry(['right_' num2str(j)])
    sr.setName(['Foot_Ground_R' num2str(j)])
    OsimModel.addForce(sr)
    % ..
    sl = HuntCrossleyForce();
    sl.addGeometry('ground')
    sl.setStiffness(stiffness(j))
    sl.setDissipation(dissipation(j))
    sl.setStaticFriction(.8)
    sl.setDynamicFriction(.8)
    sl.setViscousFriction(0)
    sl.setTransitionVelocity(.11);
    sl.addGeometry(['left_' num2str(j)])
    sl.setName(['Foot_Ground_L' num2str(j)])
    OsimModel.addForce(sl)
    
    %
    Radius      = rads(j);
    LocationR   = locRV3(j); LocationL   = locLV3(j);
    
    % Addin'
    if logicsCase == 1
        
        
        %         if j == 4
        %             srr = ContactSphere(Radius,LocationR,OsimModel.getBodySet.get('toes_r'),['right_' num2str(j) '']);
        %             sll = ContactSphere(Radius,LocationL,OsimModel.getBodySet.get('toes_l'),['left_' num2str(j) '']);
        %
        %         elseif j == 6
        %             srr = ContactSphere(Radius,LocationR,OsimModel.getBodySet.get('toes_r'),['right_' num2str(j) '']);
        %             sll = ContactSphere(Radius,LocationL,OsimModel.getBodySet.get('toes_l'),['left_' num2str(j) '']);
        %
        %         else
        srr = ContactSphere(Radius,LocationR,OsimModel.getBodySet.get('calcn_r'),['right_' num2str(j) '']);
        sll = ContactSphere(Radius,LocationL,OsimModel.getBodySet.get('calcn_l'),['left_' num2str(j) '']);
        
        %         end
        
        
    elseif logicsCase == 2
        srr = ContactSphere(Radius,LocationR,OsimModel.getBodySet.get('calcn_r'),['right_' num2str(j) '']);
        sll = ContactSphere(Radius,LocationL,OsimModel.getBodySet.get('calcn_l'),['left_' num2str(j) '']);
    else
        srr = ContactSphere(Radius,LocationR,OsimModel.getBodySet.get('calcn_r'),['right_' num2str(j) '']);
        sll = ContactSphere(Radius,LocationL,OsimModel.getBodySet.get('calcn_l'),['left_' num2str(j) '']);
    end
    OsimModel.addContactGeometry(srr)
    OsimModel.addContactGeometry(sll)
    
    
end; clear j
end


function [OsimModel, locL, locR, rads] = add_temporary_markers(OsimModel, scaleSetStruct, logicsCase)

% Assign marker locations
if ~isempty(scaleSetStruct)
    
    % If we use our own model, we have scaled it thus we have a scaleSet
    scaleSetToes = str2num(scaleSetStruct.Children(2).Children(2).Children(16).Children(2).Children.Data);
    scaleSetCalc = str2num(scaleSetStruct.Children(2).Children(2).Children(14).Children(2).Children.Data);
    modelyZerodiff = 0.00150881;
    yCalc = 0;
    %     yCalc = -0.010;
    yCalctoToe = (yCalc * scaleSetCalc(2) + modelyZerodiff) / scaleSetToes(2);
    
    % For all footmodel cases, enter the locations of the markers where the
    % spheres will be placed
    if logicsCase == 1
        %
        %         L1 = [0.0190115788407966 yCalc 0.00382630379623308];
        %         R1 = [0.0190115788407966 yCalc -0.00382630379623308];
        %         L2 = [0.180386399942063 yCalc 0.028713422052654];
        %         R2 = [0.180386399942063 yCalc -0.028713422052654];
        %         L3 = [0.160001170607051 yCalc -0.0516362473449566];
        %         R3 = [0.160001170607051 yCalc 0.0516362473449566];
        %         L4 = [0.06 yCalctoToe 0.0187603084619177];
        %         R4 = [0.06 yCalctoToe -0.0187603084619177];
        %         L5 = [0.0862346661991635 yCalc -0.0263641606741698];
        %         R5 = [0.0862346661991635 yCalc 0.0263641606741698];
        %         L6 = [0.04 yCalctoToe -0.0618569567549652];
        %         R6 = [0.04 yCalctoToe 0.0618569567549652];
        
        LocationR1=Vec3(0.00190115788407966,-0.021859,-0.00382630379623308);
        LocationR2=Vec3(0.148386399942063,-0.021859,-0.028713422052654);
        LocationR3=Vec3(0.133001170607051,-0.021859,0.0516362473449566);
        LocationR4=Vec3(0.06,-0.0214476,-0.0187603084619177);
        LocationR5=Vec3(0.0662346661991635,-0.021859,0.0263641606741698);
        LocationR6=Vec3(0.045,-0.0214476,0.0618569567549652);
        LocationL1=Vec3(0.00190115788407966,-0.021859,0.00382630379623308);
        LocationL2=Vec3(0.148386399942063,-0.021859,0.028713422052654);
        LocationL3=Vec3(0.133001170607051,-0.021859,-0.0516362473449566);
        LocationL4=Vec3(0.06,-0.0214476,0.0187603084619177);
        LocationL5=Vec3(0.0662346661991635,-0.021859,-0.0263641606741698);
        LocationL6=Vec3(0.045,-0.0214476,-0.0618569567549652);
        
        
        
        
        L1 = [0.02 yCalc 0];
        R1 = [0.02 yCalc 0];
        L2 = [0.1475 yCalc 0.02];
        R2 = [0.1475 yCalc -0.02];
        L3 = [0.1375 yCalc -0.04];
        R3 = [0.1375 yCalc 0.04];
        L4 = [0.22 yCalc 0.018];
        R4 = [0.22 yCalc -0.018];
        L5 = [0.06 yCalc -0.025];
        R5 = [0.06 yCalc 0.025];
        L6 = [0.20 yCalc -0.041];
        R6 = [0.20 yCalc 0.041];
        locL = [L1; L2; L3; L4; L5; L6];
        locR = [R1; R2; R3; R4; R5; R6];
    elseif logicsCase == 2
        %         L1 = [0.019 yCalc 0];
        %         R1 = [0.019 yCalc 0];
        %         L2 = [0.16 yCalc 0.01];
        %         R2 = [0.16 yCalc -0.01];
        %         L3 = [0.16 yCalc -0.04];
        %         R3 = [0.16 yCalc 0.04];
        %         L4 = [0.18 yCalc 0.01];
        %         R4 = [0.18 yCalc -0.01];
        %         L5 = [0.07 yCalc -0.025];
        %         R5 = [0.07 yCalc 0.025];
        %         L6 = [-0.01 yCalctoToe -0.04];
        %         R6 = [-0.01 yCalctoToe 0.04];
        
        L1 = [0.025 yCalc 0] ;
        R1 = [0.025 yCalc 0];
        
        L2 = [0.21 yCalc 0.0275] ;
        R2 = [0.21 yCalc -0.0275] ;
        L3 = [0.14 yCalc -0.06] ;
        R3 = [0.14 yCalc 0.06] ;
        L4 = [0.08 yCalc -0.015];
        R4 = [0.08 yCalc 0.015];
        %         L5 = [0.12 yCalc 0.025];
        %         R5 = [0.12 yCalc -0.025];
        
        locL = [L1; L2; L3; L4];%: L6; L7; L8];
        locR = [R1; R2; R3; R4];%; R6; R7; R8];
    elseif logicsCase == 3
        L1 = [0.01 yCalc 0] ;
        R1 = [0.01 yCalc 0];
        
        L2 = [0.18 yCalc 0.03] ;
        R2 = [0.18 yCalc -0.03] ;
        %         L3 = [0.16 yCalc -0.04] ;
        %         R3 = [0.16 yCalc 0.04] ;
        
        locL = [L1; L2];
        locR = [R1; R2];
    end
    
else
    
    if logicsCase == 1
        L1 = [0.00190115788407966,-0.021859,0.00382630379623308] ;
        R1 = [0.00190115788407966,-0.021859,-0.00382630379623308];
        L2 = [0.148386399942063,-0.021859,0.028713422052654] ;
        R2 = [0.148386399942063 -0.021859 -0.028713422052654] ;
        L3 = [0.133001170607051,-0.021859,-0.0516362473449566] ;
        R3 = [0.133001170607051,-0.021859,0.0516362473449566] ;
        L4 = [0.06,-0.0214476,0.0187603084619177];
        R4 = [0.06,-0.0214476,-0.0187603084619177] ;
        L5 = [0.0662346661991635 -0.021859 -0.0263641606741698];
        R5 = [0.0662346661991635 -0.021859 0.0263641606741698];
        L6 = [0.045,-0.0214476,-0.0618569567549652] ;
        R6 = [0.045,-0.0214476,0.0618569567549652] ;
        locL = [L1; L2; L3; L4; L5; L6];
        locR = [R1; R2; R3; R4; R5; R6];
        
    elseif logicsCase == 2
        L1 = [0.00190116 -0.0104916 0.0125028];
        R1 = [0.00190116 -0.0104916 -0.0125028];
        L2 = [0.00190116 -0.0104916 -0.0038263];
        R2 = [0.00190116 -0.0104916 0.0038263];
        L3 = [0.0352295 -0.0519821 -0.0122341];
        R3 = [0.0352295 -0.0519821 0.0122341];
        L4 = [-0.0307522 -0.0057042 -0.0360427];
        R4 = [-0.0307522 -0.0057042 0.0360427];
        L5 = [0.00012177 -0.00570324 0.0239847];
        R5 = [0.00012177 -0.00570324 -0.0239847];
        L6 = [0.0304433 -0.00570494 0.00928034];
        R6 = [0.0304433 -0.00570494 -0.00928034];
        L7 = [0.0227419 -0.00453964 -0.025943];
        R7 = [0.0227419 -0.00453964 0.025943];
        locL = [L1; L2; L3; L4; L5; L6; L7];
        locR = [R1; R2; R3; R4; R5; R6; R7];
    elseif logicsCase == 3
        L1 = [0.012695 -0.0084318 0.0109041];
        R1 = [0.012695 -0.0084318 -0.0109041];
        %         L2 = [0.109758 -0.0089653 -0.0223917] ;
        %         R2 = [0.109758 -0.0089653 0.0223917] ;
        L2 = [0.0147591 -0.00597921 0.00269796];
        R2 = [0.0147591 -0.00597921 -0.00269796];
        locL = [L1; L2];%; L3];
        locR = [R1; R2];%; R3];
    end
end

% Allocate markerset variable in order to assign new markers that pose as
% sphere locations
import org.opensim.modeling.*
cMarkerSet      = OsimModel.getMarkerSet;

% Add the markers to the markerset on the specific bodies
if logicsCase == 1
    % Amount of spheres
    nSpheres    = size(locR, 1);
    rads        = repmat(.03, nSpheres, 1);
    %     rads(end) = .001;
    for i=1:length(locR)
        dLocationR = locR(i,:);
        %         if i == 4
        %             cMarkerSet.addMarker(['right_' num2str(i) ''], dLocationR, OsimModel.getBodySet().get("toes_r"));
        %
        %         elseif i ==6
        %             cMarkerSet.addMarker(['right_' num2str(i) ''], dLocationR, OsimModel.getBodySet().get("toes_r"));
        %
        %         else
        cMarkerSet.addMarker(['right_' num2str(i) ''], dLocationR, OsimModel.getBodySet().get("calcn_r"));
        %         end
    end
    for i=1:length(locL)
        dLocationL = locL(i,:);
        
        %         if i == 4
        %             cMarkerSet.addMarker(['left_' num2str(i) ''], dLocationL, OsimModel.getBodySet().get("toes_l"));
        %
        %         elseif i ==6
        %             cMarkerSet.addMarker(['left_' num2str(i) ''], dLocationL, OsimModel.getBodySet().get("toes_l"));
        %
        %         else
        cMarkerSet.addMarker(['left_' num2str(i) ''], dLocationL, OsimModel.getBodySet().get("calcn_l"));
        %         end
    end
elseif logicsCase == 2
    
    % Amount of spheres
    nSpheres    = size(locR, 1);
    rads        = repmat(.035, nSpheres, 1);
    for i=1:nSpheres
        
        dLocationR = locR(i,:);
        dLocationL = locL(i,:);
        cMarkerSet.addMarker(['right_' num2str(i) ''], dLocationR, OsimModel.getBodySet().get("calcn_r"));
        cMarkerSet.addMarker(['left_' num2str(i) ''], dLocationL, OsimModel.getBodySet().get("calcn_l"));
    end
elseif logicsCase == 3
    % Amount of spheres
    nSpheres    = size(locR, 1);
    rads        = repmat(.03, nSpheres, 1);
    for i=1:nSpheres
        
        dLocationR = locR(i,:);
        dLocationL = locL(i,:);
        
        cMarkerSet.addMarker(['right_' num2str(i) ''], dLocationR, OsimModel.getBodySet().get("calcn_r"));
        cMarkerSet.addMarker(['left_' num2str(i) ''], dLocationL, OsimModel.getBodySet().get("calcn_l"));
        
    end
end
end


function data = createAnalyzeActuatorfile(data,coms,coords, dirModel)
% Setup .xml root
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';
fileTBD.ForceSet.ATTRIBUTE.name = 'Lowerbody_resids';

fileTBD.ForceSet.defaults.PointActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.PointActuator.max_force = '10000';
fileTBD.ForceSet.defaults.PointActuator.min_force = '-10000';
fileTBD.ForceSet.defaults.PointActuator.optimal_force = '5';
fileTBD.ForceSet.defaults.PointActuator.point = '0 0 0';
fileTBD.ForceSet.defaults.PointActuator.direction = '1 0 0';
fileTBD.ForceSet.defaults.TorqueActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.TorqueActuator.max_force = '1000';
fileTBD.ForceSet.defaults.TorqueActuator.min_force = '-1000';
fileTBD.ForceSet.defaults.TorqueActuator.optimal_force = '5';
fileTBD.ForceSet.defaults.TorqueActuator.axis = '1 0 0';
fileTBD.ForceSet.defaults.CoordinateActuator.ATTRIBUTE.name = 'default';
fileTBD.ForceSet.defaults.CoordinateActuator.max_force = '1000';
fileTBD.ForceSet.defaults.CoordinateActuator.min_force = '-1000';
fileTBD.ForceSet.defaults.CoordinateActuator.optimal_force = '1000';



for i = 1:6
    if i == 1 || i == 2 || i == 3
        if i == 1
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FX';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '1 0 0';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '5';
        elseif i == 2
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FY';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '0 1 0';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '5';
        elseif i == 3
            fileTBD.ForceSet.objects.PointActuator(i).ATTRIBUTE.name = 'FZ';
            fileTBD.ForceSet.objects.PointActuator(i).direction = '0 0 1';
            fileTBD.ForceSet.objects.PointActuator(i).optimal_force = '5';
        end
        fileTBD.ForceSet.objects.PointActuator(i).body = 'pelvis';
        fileTBD.ForceSet.objects.PointActuator(i).point = num2str(coms(2,:),8);
        fileTBD.ForceSet.objects.PointActuator(i).point_is_global = 'false';
        fileTBD.ForceSet.objects.PointActuator(i).force_is_global = 'true';
        %         fileTBD.ForceSet.objects.PointActuator(i).min_force = '-10000';
        %         fileTBD.ForceSet.objects.PointActuator(i).max_force = '10000';
    end
    if i == 4 || i == 5 || i == 6
        if i == 4
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MX';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '1 0 0';
            
        elseif i == 5
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MY';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '0 1 0';
            
        elseif i == 6
            fileTBD.ForceSet.objects.TorqueActuator(i-3).ATTRIBUTE.name = 'MZ';
            fileTBD.ForceSet.objects.TorqueActuator(i-3).axis = '0 0 1';
        end
        fileTBD.ForceSet.objects.TorqueActuator(i-3).bodyA = 'pelvis';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).bodyB = 'ground';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).optimal_force = '5';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).min_control = '-inf';
        fileTBD.ForceSet.objects.TorqueActuator(i-3).max_control = 'inf';
        %         fileTBD.ForceSet.objects.TorqueActuator(i-3).min_force = '-1';
        %         fileTBD.ForceSet.objects.TorqueActuator(i-3).max_force = '15';
    end
end; clear i

% Get modeldata
import org.opensim.modeling.*
model = Model(dirModel);
coordSet = model.getCoordinateSet();
coords = cell(coordSet.getSize(),1);

% Specific weights
mat_names = ["hip"];%, "ankle", "knee", "arm", "elbow", "pro_", "wrist"];
mat_weights   = [1000];%, 500, 350, 150, 150, 150, 150];

% Insert coordinates that are used in the .osim model. Maybe automatically
% load them from .osim file
for i = 0:coordSet.getSize()-1
    
    % Whats the current name
    coName = coordSet.get(i);
    
    % Generic filler
    if ~coName.get_locked
        coStrName = char(coName);
        if ~strcmp('beta',coStrName(end-3:end))
            if ~startsWith(coStrName, "pelvis")
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).ATTRIBUTE.name = [coStrName '_reserve'];
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).coordinate = coStrName;
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).optimal_force = '1000';
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).min_control = '-Inf';
                fileTBD.ForceSet.objects.CoordinateActuator(i+1).max_control = 'Inf';
            end
        end
        
        % Change the special tracking weights
        idx_special = startsWith(coStrName, mat_names);
        if any(idx_special)
            for j = 1:length(mat_names)
                if contains(coStrName, mat_names(j))
                    fileTBD.ForceSet.objects.CoordinateActuator(i+1).optimal_force = mat_weights(j);
                end
            end
        end
    end
end; clear i
fileTBD.ForceSet.objects.CoordinateActuator = fileTBD.ForceSet.objects.CoordinateActuator(all(~cellfun(@isempty,struct2cell(fileTBD.ForceSet.objects.CoordinateActuator))));



% Export actuator file to given directory
fileout = [data.Name '_RRA_Actuators.xml'];
data.fwd_Actuator_Filename = fileout;
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);
end


function data = setupAnalysisFile(data, model_file, states, controls, reqFiles,t)
% Setup root and InverseKinematicsTool
root = 'OpenSimDocument';
fileTBD.ATTRIBUTE.Version = '30000';

% FWD Tool
fileTBD.AnalyzeTool.ATTRIBUTE.name = [data.Name '_' controls(1:6)];

% ----- Model File
fileTBD.AnalyzeTool.model_file = ['../skeletal_models/' model_file];

% ----- Replace force set
fileTBD.AnalyzeTool.replace_force_set = 'false';

%  ----- Force set files
fileTBD.AnalyzeTool.force_set_files = reqFiles.FWDactuatorFile;
% fileTBD.AnalyzeTool.external_loads_file = grf;

% ----- Outpout directory and precision
fileTBD.AnalyzeTool.results_directory = ['./DCResults'];
fileTBD.AnalyzeTool.output_precision = 8;

% ----- RRA timeframe
% fileTBD.AnalyzeTool.initial_time = char(string(data.time(1)));
% fileTBD.AnalyzeTool.final_time = char(string(data.time(end)));

fileTBD.AnalyzeTool.initial_time = char(string(t(1)));
fileTBD.AnalyzeTool.final_time = char(string(t(end)));
% ----- COMPUTE EQUILIBRIUM WHUT?
fileTBD.AnalyzeTool.solve_for_equilibrium_for_auxiliary_states = 'false';

% ----- Integrator properties
% fileTBD.AnalyzeTool.maximum_number_of_integrator_steps = 20000;
% fileTBD.AnalyzeTool.maximum_integrator_step_size = 1;
% fileTBD.AnalyzeTool.minimum_integrator_step_size = 1e-08;
% fileTBD.AnalyzeTool.integrator_error_tolerance = 1e-05;

% A-sets
fileTBD.AnalyzeTool.AnalysisSet.objects.Actuation.ATTRIBUTE.name = 'Actuation';
fileTBD.AnalyzeTool.AnalysisSet.objects.Actuation.on = 'true';
fileTBD.AnalyzeTool.AnalysisSet.objects.Actuation.step_interval = 1;
fileTBD.AnalyzeTool.AnalysisSet.objects.Actuation.in_degrees = 'true';
% A-sets
fileTBD.AnalyzeTool.AnalysisSet.objects.BodyKinematics.ATTRIBUTE.name = 'BodyKinematics';
fileTBD.AnalyzeTool.AnalysisSet.objects.BodyKinematics.on = 'true';
fileTBD.AnalyzeTool.AnalysisSet.objects.BodyKinematics.step_interval = 1;
fileTBD.AnalyzeTool.AnalysisSet.objects.BodyKinematics.in_degrees = 'true';

% ----- Input Files (see documentation of opensim TK)
fileTBD.AnalyzeTool.states_file = states;
fileTBD.AnalyzeTool.ControllerSet.objects.ATTRIBUTE.name = 'Controllers';
fileTBD.AnalyzeTool.ControllerSet.objects.ControlSetController.enable_controller = 'true';
fileTBD.AnalyzeTool.ControllerSet.objects.ControlSetController.controls_file = controls;

% fileTBD.AnalyzeTool.Output.prefix = [data.Name '_' controls(1:6)];

% ----- Desired RRA tracking tasks and constraints
% fileTBD.AnalyzeTool.task_set_file = reqFiles.RRAtaskFile;

% ----- Defining filter properties
% fileTBD.AnalyzeTool.lowpass_cutoff_frequency = 6;


% Exporting newly created IK setup file
fileTBD.AnalyzeTool.use_verbose_printing = 'false';
fileout = [data.Name '_Setup_Analysis.xml'];
Pref.StructItem = false;
xml_write(fileout, fileTBD, root,Pref);
% data.fwd_Actuator_Filename =
end


function PredGRF = CalcPredGRF(contactloads, nNodes, nFGContacts)
if size(contactloads,1) == nNodes
    contactloads = contactloads';
end
%         nFGContacts = 12;
contactforces = contactloads([1:6:nFGContacts/2*6, ...
    2:6:nFGContacts/2*6, ...
    3:6:nFGContacts/2*6,...
    nFGContacts/2*6+1:6:nFGContacts*6, ...
    nFGContacts/2*6+2:6:nFGContacts*6, ...
    nFGContacts/2*6+3:6:nFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
%     contactforces = contactforces';
PredGRF = [sum(contactforces(1:nFGContacts/2,:)); ...
    sum(contactforces(nFGContacts/2+1:nFGContacts,:)); ...
    sum(contactforces(nFGContacts+1:3*nFGContacts/2,:)); ...%Total RFx RFy RFz
    sum(contactforces(3*nFGContacts/2+1:2*nFGContacts,:)); ...
    sum(contactforces(2*nFGContacts+1:5*nFGContacts/2,:)); ...
    sum(contactforces(5*nFGContacts/2+1:3*nFGContacts,:))];%Total RFx RFy RFz
end