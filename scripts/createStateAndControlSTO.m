function createStateAndControlSTO(osimModel, datSVCfmincon, nNodes, nPos, nCon, ...
    idxPos, idxVel, idxCon, sto_states_fileName, sto_contrl_fileName, tVec, idxFixed, nameControl)

% Create model instance
import org.opensim.modeling.*
if ischar(osimModel)
    osimModel = Model(osimModel);
end

%% States
% Acces opensim manager in order to create STO files
myManager           = Manager(osimModel);
myStorage           = myManager.getStateStorage();
defStates           = osimModel.initSystem();
coordSet            = osimModel.updCoordinateSet();
numOfCoordinates    = osimModel.getNumCoordinates();
labels              = org.opensim.modeling.ArrayStr(); labels.append('time');
labels.append(osimModel.getStateVariableNames());
myStorage.setColumnLabels(labels);

% Create state matrices + locked joints
datSV = datSVCfmincon(:,[idxPos, idxVel]);
datSC = datSVCfmincon(:,idxCon);
idxFixed = vec2mat(idxFixed,2);%First column is for generalized coords and the second column is for generalzed speeds
idxFixed = idxFixed(:);
prscrbd_Joints      = zeros(size(datSV,1),length(idxFixed)/2);
prscrbd_Joints_Dot	= zeros(size(datSV,1),length(idxFixed)/2);
idxFixed(2:end)=idxFixed(2:end)-[1:(length(idxFixed)-1)]';
datSV = insertrows(datSV',[prscrbd_Joints,prscrbd_Joints_Dot]',idxFixed');%update with the prescribed qs and dqs
datSV=datSV';

for tIndex = 1:nNodes
    myArray = ArrayDouble();
    for state = 1:2*(numOfCoordinates)
        myArray.append(datSV(tIndex,state));
    end
    myStorage.append(tVec(tIndex),myArray);
end

displayName = sto_states_fileName(1:end-4);
myStorage.setName(displayName);
myStorage.print(sto_states_fileName);


%% Controls
% Acces opensim manager in order to create STO files
myManager           = Manager(osimModel);
defStates           = osimModel.initSystem();
coordSet            = osimModel.updCoordinateSet();
numOfCoordinates    = osimModel.getNumCoordinates();
myStorage2          = myManager.getStateStorage();
labels2             = org.opensim.modeling.ArrayStr(); labels2.append('time');
idxConNot              = zeros(1,nCon);
for i = 0:nCon-1 % for the pelvis actuators (Fx,Fy ... Mz)
    
    curCon = osimModel.getControllerSet.get(0).get_actuator_list(i);
    labels2.append(curCon);
    
    if ~any(strcmp(nameControl, char(curCon)))
        idxConNot(i) = 1;
    end
    
end
% CC = bwconncomp(idxConNot);
% nConNot = length(CC.PixelIdxList{1,1});
% datNot = zeros(nNodes,nConNot);
myStorage2.setColumnLabels(labels2);
% datSC = insertrows(datSC',CC.PixelIdxList{1,1}(1))';%update with the prescribed qs and dqs


for tIndex = 1:nNodes
    myArray2 = ArrayDouble();
    for con = 1:nCon
        myArray2.append(datSC(tIndex,con));
    end
    myStorage2.append(tVec(tIndex),myArray2);
end

displayName2 = sto_contrl_fileName(1:end-4);
myStorage2.setName(displayName2);
myStorage2.print(sto_contrl_fileName);


end