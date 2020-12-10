function [savedname,actuatorModel] = parametrize_states_and_controls ...
    (OsimModel, path_model, file_motion, file_controls, nNodes, reqTimesDis)

%% Initiate workspace
import org.opensim.modeling.*

%% Load OpenSim model parameters
% Prep model
state = OsimModel.initSystem();
% actuatorModel = char(OsimModel.getForceSet.get(0).getConcreteClassName);

% ... state variables + names
setStateVars = OsimModel.getStateVariableNames();
numStates = setStateVars.getSize;
namesStateVars = strings(numStates,1);
for i = 0:setStateVars.getSize()-1
    namesStateVars{i+1} = char(setStateVars.get(i));
end; clear i

% ... actuators + names
setActuators = OsimModel.getActuators();
numActuators	= setActuators.getSize;
namesActuators = strings(numActuators,1);
for i = 0:setActuators.getSize()-1
    namesActuators{i+1} = char(setActuators.get(i));
end; clear i

% ... coordinates + names
setCoordinates = OsimModel.getCoordinateSet();
numCoordinates	= setCoordinates.getSize;
namesCoordinates = strings(numCoordinates,1);
for i = 0:setCoordinates.getSize()-1
    namesCoordinates{i+1} = char(setCoordinates.get(i));
end; clear i
indexCoordinates = 0:numCoordinates-1;

% % ... muscles + names
% setMuscles = OsimModel.getMuscles();
% numMuscles	= setMuscles.getSize;
% namesMuscles = strings(numMuscles,1);
% for i = 0:setMuscles.getSize()-1
%     namesMuscles{i+1} = char(setMuscles.get(i));
% end; clear i

% ... reserves (torques) + names
% numResids	= numActuators - numMuscles;
% idxResids = ~ismember(namesActuators, namesMuscles);
% namesResids = namesActuators(idxResids,:);
% zclear idxResids

% ... checking for locked joints/ states/ coordinates
namesLockedItems = {};
namesBetaItems = {};
indexDOFLocked = [];
indexDOFbeta = [];
for i = indexCoordinates
    if setCoordinates.get(i).get_locked
        namesLockedItems{end+1} = namesCoordinates{i+1};
        namesLockedItems{end+1} = [namesLockedItems{end}, '_u'];
        indexDOFLocked(end+1)=i;
        indexDOFLocked(end+1)=indexDOFLocked(end) + numCoordinates;
    elseif endsWith(char(setCoordinates.get(i)), 'beta')
        namesBetaItems{end+1} = namesCoordinates{i+1};
        namesBetaItems{end+1} = [namesBetaItems{end}, '_u'];
        indexDOFbeta(end+1)=i;
        indexDOFbeta(end+1)=indexDOFbeta(end) + numCoordinates;
    end
end; clear i 

% ... removing locked items from state vars
namesLockedItems = string(namesLockedItems)';
idxStateVarsFree = ~ismember(namesStateVars, namesLockedItems);
% idxActuators = ~ismember(namesActuators, namesLockedItems);
namesStateVarsFree = namesStateVars(idxStateVarsFree,:);
% namesActuatorsFree = namesActuators(idxActuators, :);
numStateVarsFree = length(namesStateVarsFree);
clear idxStateVarsFree
indexStateVarsFreeOS = find(ismember(namesStateVars, namesStateVarsFree))' - 1;

% ... removing locked items from coordinates
idxCoordinatesFree = ~ismember(namesCoordinates, namesLockedItems);
namesCoordinatesFree = namesStateVars(idxCoordinatesFree,:);
numCoordinatesFree = length(namesCoordinatesFree);
clear idxCoordinatesFree
indexCoordinatesFreeOS = find(ismember(namesCoordinates, namesCoordinatesFree))' - 1;
indexPosition = 1:numCoordinatesFree;
indexVelocity = numCoordinatesFree + 1 : numCoordinatesFree*2;
indexPositionOS = indexCoordinatesFreeOS;
indexVelocityOS = indexCoordinatesFreeOS + numCoordinates;


%% Discreditize state and control variables
% Select states (ff kort gemaakt voor ease)
% [fileStates,pathStates] = uigetfile('*states.sto','Select a states file for discretization :');
% fileStates = [subName '_states.sto'];
% fileStates = file_motion;
% pathStates = 'C:\Users\lsboe\Documents\NiksWerkt\CMC\walk\Results_CMC\';
% [fileFiberLength,pathFiberLength] = uigetfile('*FiberLength.sto','Select a states file for discretization :');
% fileFiberLength = 'motion_capture_walk_MuscleAnalysis_FiberLength.sto';
% pathFiberLength = 'C:\Users\lsboe\Documents\NiksWerkt\CMC\walk\Results_CMC\';
% [fileControls,pathControls] = uigetfile('*controls.sto','Select a controls file for discretization:');
% fileControls = [subName '_controls.sto'];
% pathControls = 'C:\Users\lsboe\Documents\NiksWerkt\CMC\walk\Results_CMC\';


% Create states variable for given reqTimes values
states          = readMOT(file_motion);
timeVector      = states.data(:,1);
if strcmp(file_motion, 'Subj1PW_CMC_states.sto')
    
    corTVals = 1:length(timeVector);
    matrixStates    = states.data(corTVals,:);
    timeVector      = timeVector(corTVals,:);
    t0 = .5; tf = 1;
elseif strcmp(file_motion, 'subject01_walk1_states.sto')
    corTVals = 1:length(timeVector);
    matrixStates    = states.data(corTVals,:);
    timeVector      = timeVector(corTVals,:);
    t0 = timeVector(1); tf = timeVector(end);
else
    
    corTStart       = find(timeVector - reqTimesDis(1) <= .00001, 1, 'last' );
    corTEnd         = find(timeVector - reqTimesDis(end) <= .00001, 1, 'last' );
    corTVals        = corTStart:1:corTEnd;
    matrixStates    = states.data(corTVals,:);
    timeVector      = timeVector(corTVals,:);
    t0 = timeVector(1); tf = timeVector(end);

end

    


% ... and remove locked coordinates from the state matrix
matrixStatesFree = matrixStates(:, [1, indexStateVarsFreeOS+2]);
% Create activation variable
% indexActivation = find(~cellfun(@isempty,strfind(states.labels,'activation'))); 
% namesActivation = string(states.labels(indexActivation))';
% indexActivationOS = indexActivation - 2; % In the osim model
% clear indexActivation
% indexActivation = find(ismember(namesStateVarsFree, namesActivation))';
% matrixActivation = matrixStates(:, indexActivationOS + 2);
% numActivationVars = size(matrixActivation,2);
% matrixActivation = [timeVector, matrixActivation];

% Create fiber length variable
% indexFiberLength = find(~cellfun(@isempty,strfind(states.labels,'fiber_length')));
% namesFiberLength = string(states.labels(indexFiberLength))';
% indexFiberLengthOS = indexFiberLength - 2; % In the osim model
% clear indexFiberLength
% indexFiberLength = find(ismember(namesStateVarsFree, namesFiberLength))';
% matrixFiberLength = matrixStates(:, indexFiberLengthOS + 2);
% numFiberLengthVars = size(matrixFiberLength,2);
% matrixFiberLength = [timeVector, matrixFiberLength];
% clear states

% matrixStatesFree = matrixStatesFree(:,[1, indexPosition+1, indexVelocity+1, indexActivation+1, indexFiberLength+1]);  


% Create controls variable
controls = readMOT(file_controls);
indexControlsOS = (1:length(controls.labels)-1)-1; % In the osim model
namesControls = string(controls.labels(1,2:end))';
matrixControls = controls.data(corTVals,:);
% %%
% idxControls = ~ismember(namesControls, namesLockedItems);
% namesControls = namesControls(idxControls);
% labelsControls = controls.labels([logical(1); idxControls]);
% matrixControls = controls.data(corTVals,[logical(1); idxControls]);
% %%
reserve_index = find(~cellfun(@isempty,strfind(controls.labels,'reserve'))); reserve_index = sort(reserve_index);
% indexControlsOS(:, reserve_index-1) = [];
% matrixControls(:, reserve_index) = [];
% namesControls(reserve_index-1) = [];
numControlVars = size(matrixControls,2) - 1;
clear controls

% Remove dupe times
samet = find(diff(timeVector)==0);%find rows with duplicated time
timeVector(samet)=[];%remove rows with duplicated time
matrixStatesFree(samet,:)=[];
% matrixActivation(samet,:)=[];
% matrixFiberLength(samet,:)=[];
matrixControls(samet,:)=[];
clear samet

% Select time frame to simulate and create discreditized state/ control/
% fiber matrices
% fprintf('-->CMC simulation time: t=%f to t=%f \n',timeVector(1),timeVector(end));
% t0 = input('Please input the starting simulation time within the above CMC time, t0: ');
% tf = input('Please input the final simulation time within the above CMC time, tf: ');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t0 = timeVector(1); tf = timeVector(end);
% % t0 = .525; tf = 1.085;
% t0 = .5; tf = 1;

timeVector = linspace(t0,tf,nNodes);
matrixStatesFreeResampled   = [timeVector; spline(matrixStatesFree(:,1)', matrixStatesFree(:,2:end)', timeVector)];
matrixControlsResampled     = [timeVector; pchip(matrixControls(:,1)', matrixControls(:,2:end)', timeVector)];

% Reorganize matrices and remove time columns
% Generelized coordinates
matrixStatesFreeResampled = matrixStatesFreeResampled';
matrixStatesFreeResampled = matrixStatesFreeResampled(:,2:end);
matrixPositionResampled = matrixStatesFreeResampled(:,1:numCoordinatesFree);
matrixVelocityResampled = matrixStatesFreeResampled(:,numCoordinatesFree+1:numCoordinatesFree*2);
% ... activation
% matrixActivationResampled = matrixActivationResampled';
% matrixActivationResampled = matrixActivationResampled(:,2:end);
% matrixActivationResampled(matrixActivationResampled == 1) = 1-1e-6;% Chagne U=1 to U=1-1e-6 to avoid hitting the upper bound of excitation
% ... fiber length
% matrixFiberLengthResampled = matrixFiberLengthResampled';
% matrixFiberLengthResampled = matrixFiberLengthResampled(:,2:end);
% ... controls
matrixControlsResampled = matrixControlsResampled';
matrixControlsResampled = matrixControlsResampled(:,2:end);
matrixControlsResampled(matrixControlsResampled == 1) = 1-1e-6;% Chagne U=1 to U=1-1e-6 to avoid hitting the upper bound of excitation

% -- Create indiceMatrix and labels for plotting purpose
indexPosition = 1:numCoordinatesFree;
indexVelocity = numCoordinatesFree + 1 : numCoordinatesFree*2;
namesPosition = namesStateVarsFree(indexPosition);
namesVelocity = namesStateVarsFree(indexVelocity);
indexControls = indexControlsOS + 1 + numStateVarsFree;
indexStateVarsFree = 1:numStateVarsFree;

% -- Output the sampling results
matrixStatesControls = [matrixStatesFreeResampled, matrixControlsResampled];
numStateVarsControls = size(matrixStatesControls,2);
indexStateVarsControls = 1:numStateVarsControls;
namesStateVarsControls = [namesStateVarsFree; namesControls];

savedname = ['Sample' num2str(nNodes) 'nodesfor',file_motion,'.mat'];
% indexDOFLocked is hier weggehaald, maybe terugzetten? + numDOFLocked
% save([path_model, '/', savedname],'matrixStatesControls','timeVector','nNodes', ...
%     'numStateVarsFree', 'numMuscles', 'numActuators', 'numResids', 'numCoordinatesFree', 'numStateVarsControls', ...
%     'indexStateVarsFreeOS', 'indexActivationOS', 'indexFiberLengthOS', 'indexPositionOS', 'indexVelocityOS', 'indexControlsOS', ...
%     'indexStateVarsFree', 'indexActivation', 'indexFiberLength', 'indexPosition', 'indexVelocity', 'indexControls', 'indexStateVarsControls', ...
%     'namesStateVarsFree', 'namesActivation', 'namesFiberLength', 'namesPosition', 'namesVelocity', 'namesControls', 'namesStateVarsControls', ...
%     'namesResids', 'namesBetaItems', 'indexDOFLocked', 'indexDOFbeta'); % actuatorModel removed 5-9
save([path_model, '/', savedname],'matrixStatesControls','timeVector','nNodes', ...
    'numStateVarsFree', 'numActuators', 'numCoordinatesFree', 'numStateVarsControls', ...
    'indexStateVarsFreeOS', 'indexPositionOS', 'indexVelocityOS', 'indexControlsOS', ...
    'indexStateVarsFree', 'indexPosition', 'indexVelocity', 'indexControls', 'indexStateVarsControls', ...
    'namesStateVarsFree', 'namesPosition', 'namesVelocity', 'namesControls', 'namesStateVarsControls', ...
    'namesBetaItems', 'indexDOFLocked', 'indexDOFbeta'); % actuatorModel removed 5-9
% fprintf('------------------------------------\n| Summary of the discretization process\n| Num of nodes: %d\n| Num of muscles: %d\n| Num of independent DOFs: %d\n| Num of states: %d\n| Num of (fixed) MTP jts: %d\n-----------------------------------\n',...
%     nNodes,numCoordinatesFree,numStateVarsFree, length(namesLockedItems)/2)
