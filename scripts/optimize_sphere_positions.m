function [funcVal] = optimize_sphere_positions(pars, ...
    TargetDvs, fpDataProcessed, residsPelvisZero,...
    t, t0, tf, h, nNodes, nPPN, ...
    nPos, numFGContacts, nSVF, nSpheres, ...
    idxSVFOS, idxActuatorSphere, ...
    locL, firstStep)

% Allocate prediction-variables (predicted F and predicted GRF)
predFuncVal     = zeros(nNodes, nSVF);
grs             = zeros(nNodes, nSpheres *12);

% Allocate design variables in usable matrices
matrixStatesControls    = TargetDvs;

% .. get DVs from pars variable
locLPlanar              = pars(1:nSpheres*3); % Z for heel and XYZ forefoot
stiff                   = pars(nSpheres*3+1:nSpheres*4);
rads                    = pars(nSpheres*4+1:nSpheres*5);
% .. and create matrix of u and q to actuate the model
matrixStatesControlsSP  = [matrixStatesControls, ...
                        repmat(locLPlanar, 1, nNodes)', ...
                        repmat(stiff, 1, nNodes)', ...
                        repmat(rads, 1, nNodes)'];

% Calculate initial predictions (X) and GRFs
for n = 1:nNodes
    [derivs, gr] = mex_sphere_optimization(...
        matrixStatesControlsSP(n,:), locL, idxSVFOS, ...
        nPos, idxActuatorSphere, nSpheres, nPPN, size(matrixStatesControls,2),...
        (t(n) - t0)/(tf - t0), t0, tf);
    predFuncVal(n,:) = derivs;
    grs(n,:) = gr;
end; clear n
idxzero = fpDataProcessed <= 5;
grfs = CalcPredGRF(grs, nNodes, nSpheres*2)';

f2 = sum(sum((sqrt((grfs(:,[2,5]) - fpDataProcessed(:,[2,5])).^2))));
% -- Use Trapezoidal Method to calculate the defect errors (dimensional
z =  (diff(matrixStatesControls(:, 1:nSVF), [], 1) - h/2 * (predFuncVal(1:end - 1, :) + predFuncVal(2:end, :)))';

% Creating the function value output for this function
wDef        = 1;

% All the weighted values of the function output
valDefect       = wDef * z(:);
funcVal         = (.1*f2) + (valDefect'*valDefect);


function PredGRF = CalcPredGRF(contactloads, nNodes, numFGContacts)
if size(contactloads,1) == nNodes
    contactloads = contactloads';
end
%         numFGContacts = 12;
contactforces = contactloads([1:6:numFGContacts/2*6, ...
    2:6:numFGContacts/2*6, ...
    3:6:numFGContacts/2*6,...
    numFGContacts/2*6+1:6:numFGContacts*6, ...
    numFGContacts/2*6+2:6:numFGContacts*6, ...
    numFGContacts/2*6+3:6:numFGContacts*6],:);%Only 6 contact forces are needed [RFx;RFy;RFz;LFx;LFy;LFz] for each contact pair
%     contactforces = contactforces';
if numFGContacts == 2
    PredGRF = contactforces;
else
    PredGRF = [sum(contactforces(1:numFGContacts/2,:)); ...
        sum(contactforces(numFGContacts/2+1:numFGContacts,:)); ...
        sum(contactforces(numFGContacts+1:3*numFGContacts/2,:)); ...%Total RFx RFy RFz
        sum(contactforces(3*numFGContacts/2+1:2*numFGContacts,:)); ...
        sum(contactforces(2*numFGContacts+1:5*numFGContacts/2,:)); ...
        sum(contactforces(5*numFGContacts/2+1:3*numFGContacts,:))];%Total RFx RFy RFz
end
end
end


