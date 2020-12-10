function saveAsSTO(osimModel,Opt_xandu, mapTarget, nNodes, nIndDOF, nFib, nIndMuscles, nCon, ...
    idxPos, idxVel, idxMusc, idxFib, idxCon, theFileName,tVec, FixedDofs)

import org.opensim.modeling.*
if ischar(osimModel)
    osimModel = Model(osimModel);
end
myManager = Manager(osimModel);
myStorage = myManager.getStateStorage();
defStates = osimModel.initSystem();
coordSet = osimModel.updCoordinateSet();
numOfCoordinates = osimModel.getNumCoordinates();
labels = org.opensim.modeling.ArrayStr();
labels.append('time');
labels.append(["1_ground_force_vx"]);
labels.append(["1_ground_force_vy"]);
labels.append(["1_ground_force_vz"]);
labels.append(["1_ground_torque_x"]);
labels.append(["1_ground_torque_y"]);
labels.append(["1_ground_torque_z"]);
labels.append(["2_ground_force_vx"]);
labels.append(["2_ground_force_vy"]);
labels.append(["2_ground_force_vz"]);
labels.append(["2_ground_torque_x"]);
labels.append(["2_ground_torque_y"]);
labels.append(["2_ground_torque_z"]);

myStorage.setColumnLabels(labels);

%% Prepare ans Map
% NS = 2*nIndDOF + nFib + nIndMuscles;
% NPPN = NS+nCon;
% % Xs = Opt_xandu(:,1:NS).*repmat(diff(mapTarget(1:NS,:),[],2)',nNodes,1)+repmat(mapTarget(1:NS,1)',nNodes,1);
% if ~isempty(mapTarget)
%     Opt_xandu(:,1:NPPN) = Opt_xandu(:,1:NPPN).*(mapTarget(nNodes+1:2*nNodes,1:NPPN)-mapTarget(1:nNodes,:))+mapTarget(1:nNodes,:);
% end
% Xs = Opt_xandu(:,1:NS);
% x1 = Xs(:,idxPos);
% x2 = Xs(:,idxVel);
% x3 = Xs(:,idxMusc);
% x4 = Xs(:,idxFib);
% x3and4 = zeros(size(x4,1),size(x3,2)+size(x4,2));
% x3and4(:, idxMusc - 2 * nIndDOF) = x3;
% x3and4(:, idxFib - 2 * nIndDOF) = x4;

prmtrsMtrx =Opt_xandu;

%% add prescribed joints
% PrescribedStates = vec2mat(FixedDofs,2);%First column is for generalized coords and the second column is for generalzed speeds
% PrescribedStates = PrescribedStates(:);
% 
% prscrbd_Joints      = NaN(size(prmtrsMtrx,1),length(PrescribedStates)/2);
% prscrbd_Joints_Dot	= NaN(size(prmtrsMtrx,1),length(PrescribedStates)/2);
% for tIndex = 1:nNodes
%     defStates.setTime((tVec(tIndex)-tVec(1))/(tVec(end)-tVec(1))*range(tVec)+tVec(1));
%     coordSet.get(12).setValue(defStates,0);
%     for coord = 1:length(PrescribedStates)/2
%         prscrbd_Joints(tIndex,coord)     = coordSet.get(PrescribedStates(coord)).getValue(defStates);
%         prscrbd_Joints_Dot(tIndex,coord) = coordSet.get(PrescribedStates(coord)).getSpeedValue(defStates);
%     end
% end
% PrescribedStates(2:end)=PrescribedStates(2:end)-[1:(length(PrescribedStates)-1)]';
% prmtrsMtrx = insertrows(prmtrsMtrx',[prscrbd_Joints,prscrbd_Joints_Dot]',PrescribedStates');%update with the prescribed qs and dqs
% prmtrsMtrx=prmtrsMtrx';
%% 
for tIndex = 1:nNodes
    myArray = ArrayDouble();
    for state = 1:12
        myArray.append(prmtrsMtrx(tIndex,state));
    end
    myStorage.append(tVec(tIndex),myArray);
end

displayName = theFileName(1:end-4);
myStorage.setName(displayName);
myStorage.print(theFileName);
end