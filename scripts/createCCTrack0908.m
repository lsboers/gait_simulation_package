function [Cost, Const] = createCCTrack0907(designVars, ...
    initialX, deltaX, ...
    desiredStates, powerRRA, residsPelvisZero, fpDataProcessed, max_z, ...
    t, t0, tf, h, nNodes, nPPN, nSpheres, ...
    nPos, nCon, nPR, numFGContacts, nSVF, ...
    idxPos, idxVel, idxCon, idxSVFOS, firstStep, namePos, nameControl)

% Assign global tags on vars
matrixStatesControls        = [];
predFuncVal                 = [];
grs                         = [];
LastX                       = [];
deltaALL                    = [];
Wc                          = [];
% clear mex
% clear anozerFast
% clear anozerFast.mexw64

% Cost&Constraint function-calls
Cost    = @objTrack;
Const   = @constTrack;

%% Functions
    function [funcVal, gradObj]         = objTrack(designVars)
        
        % Allocate prediction-variables
        predFuncVal             = zeros(nNodes, nSVF);
        deltaALL                = deltaX*abs(initialX);
        
        % Allocate design variables in usable matrices
        matrixStatesControls    = vec2mat(designVars(1:nNodes*nPPN), nPPN);
        maxTargetPos            = repmat( max( abs( desiredStates'), [], 2), 1, nNodes)';

        % Calculate initial predictions (X) and GRFs
        for n = 1:nNodes
            [derivs, gr] = mex_calculate_state_derivatives(...
                matrixStatesControls(n,:), idxSVFOS, ...
                nPos, (nCon+nPR), nSpheres, ...
                (t(n) - t0)/(tf - t0), t0, tf);
            predFuncVal(n,:) = derivs;
            grs(n,:) = gr;
        end; clear n
        grfs = CalcPredGRF(grs, nNodes, nSpheres*2)';
        
        % Functional
        to_minimize3 = (matrixStatesControls(:, end-5:end)./maxTargetPos(:, end-5:end)).^2;
        gt0 = to_minimize3>0;
        pos_area = trapz(to_minimize3(gt0));
        neg_area = trapz(to_minimize3(~gt0));
        total_area2 = pos_area + sqrt(neg_area^2);
        funcVal   = total_area2;
        
        if nargout > 1
            
            % Calculating gradient of cost
            gradObj         = zeros(length(designVars), 1);
            
            for numParameter = 1:length(gradObj)
                
                % Allocate all index variables
                numParState     = numParameter;
                node            = floor((numParState -  1)/nPPN) + 1;
                i               = mod(numParState -  1, nPPN) + 1 ;
                
                % allocate stuff
                parmat = matrixStatesControls;
                marmat = matrixStatesControls;
                parmat(node,i) = parmat(node,i) + deltaALL(numParameter)*maxTargetPos(node,i);
                marmat(node,i) = marmat(node,i) - deltaALL(numParameter)*maxTargetPos(node,i);

                to_minimize3 = (parmat(:, end-5:end)./maxTargetPos(:, end-5:end)).^2;
                gt0 = to_minimize3>0;
                pos_area = trapz(to_minimize3(gt0));
                neg_area = trapz(to_minimize3(~gt0));
                total_areaP = pos_area + sqrt(neg_area^2);
                
                to_minimize3 = (marmat(:, end-5:end)./maxTargetPos(:, end-5:end)).^2;
                gt0 = to_minimize3>0;
                pos_area = trapz(to_minimize3(gt0));
                neg_area = trapz(to_minimize3(~gt0));
                total_areaM = pos_area + sqrt(neg_area^2);
                
                % resids
                errP = total_areaP;
                errM = total_areaM;
                gradObj(numParameter) = ((errP-errM)/(2*(deltaALL(numParameter)*maxTargetPos(node,i))));
                
            end
            
            clear gradObjQandG gradObjA
        end
        
    end


    
    function [C, Ceq, GradC, GradCeq]   = constTrack(designVars)
        
        if ~isequal(designVars,LastX)
            objTrack(designVars);
        end
        LastX = designVars;

        z =  ((diff(matrixStatesControls(:, 1:nSVF), [], 1) - h/2 * (predFuncVal(1:end - 1, :) + predFuncVal(2:end, :)))./max_z).^2';

        % origineel .5
        Wc = 10;
        Ceq = z(:);
        C=[];
        if nargout > 2
            GradCeq = parralelTrapezoidCentralCeq; GradCeq = Wc * GradCeq;
            GradC=[];
        end
    end

        
        

%% Subroutines
    function gradObjCeq = parralelTrapezoidCentralCeq
        
        % Preallocate gradient bla
        gradObjCeq(length(deltaALL), nSVF * (nNodes -  1)) = 0;
        matrixSubStatesLength = deltaALL(:);
        delta = deltaALL(:);

       
        parfor numParameter = 1:length(gradObjCeq)
            
            % Allocate all index variables
            node            = floor((numParameter -  1)/nPPN) + 1;
            i               = mod(numParameter -  1, nPPN) + 1 ;
            
            % allocate stuff
            parmat = matrixStatesControls;
            marmat = matrixStatesControls;
            parmat(node,i) = parmat(node,i) + deltaALL(numParameter);
            marmat(node,i) = marmat(node,i) - deltaALL(numParameter);
            predicted_f_P = predFuncVal;
            predicted_f_M = predFuncVal;
            for n = node
                [derivs, ~] = mex_calculate_state_derivatives(...
                    parmat(n,:), idxSVFOS, ...
                    nPos, (nCon), nSpheres, ...
                    (t(n) - t0)/(tf - t0), t0, tf);
                predicted_f_P(n,:) = derivs;
            end
            for n = node
                [derivs, ~] = mex_calculate_state_derivatives(...
                    marmat(n,:), idxSVFOS, ...
                    nPos, (nCon), nSpheres, ...
                    (t(n) - t0)/(tf - t0), t0, tf);
                predicted_f_M(n,:) = derivs;
            end
            
            zp =  ((diff(parmat(:, 1:nSVF), [], 1) - h/2 * (predicted_f_P(1:end - 1, :) + predicted_f_P(2:end, :)))./max_z).^2';
            zm =  ((diff(marmat(:, 1:nSVF), [], 1) - h/2 * (predicted_f_M(1:end - 1, :) + predicted_f_M(2:end, :)))./max_z).^2';
            
            gradObjCeq(numParameter,:) = ((zp(:)-zm(:))/(2*deltaALL(numParameter)))';
           
        end

        gradObjCeq(abs(gradObjCeq)<1e-7)=0;
        gradObjCeq = sparse(gradObjCeq);
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

end


