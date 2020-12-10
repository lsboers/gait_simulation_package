%% Preparing command window, workspace and backgrounds
clearvars; close all; clc;
import org.opensim.modeling.*

% Consider adding paths add the beginning of the runall file, in a similar
% fashion as to how the configureOpensim matlab script performs this task
path_btk        = 'C:\btk-0.2.1_Win7_MatlabR2009b_64bit\BTK\share\btk-0.2\Wrapping\Matlab\btk';
path_scripts    = 'C:\Users\lsboe\Documents\gait_simulation_package\Scripts';
addpath(path_btk, path_scripts)
clear path_btk path_scripts

% Configure path settings (assuming the Run button was pressed)
path_cur        = fileparts(mfilename('fullpath'));
cd([path_cur,'/..']);
path_main       = cd;
clear path_cur

% Quickly get some information on the used model
path_osim   = [path_main '\skeletal_models'];
file_osim   = 'normalP03_RAD2.osim';
cd(path_osim);
current_osim_model = Model(file_osim);
state = current_osim_model.initSystem();
% ... coordinates + names
setCoordinates = current_osim_model.getCoordinateSet();
numCoordinates	= setCoordinates.getSize;
namesCoordinates = strings(numCoordinates,1);
namesLockedItems = {};
for i = 0:setCoordinates.getSize()-1
    namesCoordinates{i+1} = char(setCoordinates.get(i));
    if setCoordinates.get(i).get_locked
        namesLockedItems{end+1} = namesCoordinates{i+1};
        namesLockedItems{end+1} = [namesLockedItems{end}, '_u'];
    end
end; clear i
indexCoordinates = 0:numCoordinates-1;
idxCoordinatesFree = ~ismember(namesCoordinates, namesLockedItems);
labelsKin = namesCoordinates(idxCoordinatesFree,:);
clear idxCoordinatesFree indexCoordinates namesCoordinates namesLockedItems numCoordinates

%% Extract ALL data for ALL subjects
% Collect fsolve data
% paths and filenamesearchers
path_data   = [path_main '\direct_collocation_files'];
path_grfmot = [path_main '\experimental_data'];
cd(path_data);
ground_RO = dir('fsolveOptGroundTracking*.*sto');
data_RO    = dir('fsolveOptStatesTracking*.*mat');
ground_TSI   = dir('fminconoptground_*.*sto');
data_TSI      = dir('fminconoptStates_*.*mat');

% subjects
num_subjects   = length(data_TSI);
neworder = [];
for i = 1:num_subjects
    neworder(i) = str2double(data_TSI(i).name(18:20));
    if isnan(neworder(i))
        neworder(i) = str2double(data_TSI(i).name(18:19));
    end
    
end
[~,neworder] = sort(neworder,'descend');

% store data into temporarily new variables
matFsolvecell = struct2cell(data_RO); matFMincell = struct2cell(data_TSI);
groundFsolvec = struct2cell(ground_RO); groundFminc = struct2cell(ground_TSI);
data_RO = cell2struct(matFsolvecell(1,neworder),'name');
data_TSI = cell2struct(matFMincell(1,neworder),'name');
ground_RO = cell2struct(groundFsolvec(1,neworder),'name');
ground_TSI = cell2struct(groundFminc(1,neworder),'name');

% mask for double subjects and recreate matrices
unique_mask = true(size(data_TSI));
for ii  = 1:length(data_TSI)-1
    for jj = ii+1:length(data_TSI)
        if isequal(data_TSI(ii).name(1:44),data_TSI(jj).name(1:44))
            unique_mask(ii) = false;
            break;
        end
    end
end
unique_mask([6]) = 0;
data_RO = data_RO(unique_mask);
data_TSI = data_TSI(unique_mask);
ground_RO = ground_RO(unique_mask);
ground_TSI = ground_TSI(unique_mask);
num_subjects = length(data_RO);

current_osim_bw      = zeros(num_subjects,1);
nPos        = 31;
nVel        = 31;
nCon        = 25;


%% Collecting all data (GRF, Kinematics, Power, COM)
if ~exist('datamat.mat','file')
    for i = 1:num_subjects
        
        % --- Subject specifics
        idx_name_subject = strfind(data_RO(i).name, 'normal');
        name_current_subject = data_RO(i).name(idx_name_subject:end-9);
        file_osim = [name_current_subject '_RAD2_MARKED.osim'];
        current_osim_model = Model(file_osim);
        current_osim_bw = getMassOfModel(current_osim_model);
        nNodes = str2double(data_TSI(i).name(18:20));
        if isnan(nNodes)
            nNodes = str2double(data_TSI(i).name(18:19));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % ------------------------- LOADING DATA -----------------------------%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % POWER
        % ... exp
        path_power_EXP  = [path_main '\RRA\results\' name_current_subject '_Actuation_Power.sto'];
        dataPowerTBD    = readMOT(path_power_EXP);
        data_power_EXP  = dataPowerTBD.data;
        data_power_EXP(diff(data_power_EXP(:,1))==0,:) = [];
        labels_power    = dataPowerTBD.labels;
        % ... RO
        path_power_RO   = [path_main '\direct_collocation_files\DCResults\' name_current_subject '_Fsolve_Actuation_Power.sto'];
        dataPowerTBD    = readMOT(path_power_RO);
        data_power_RO   = dataPowerTBD.data;
        time_two        = linspace(data_power_RO(1,1),data_power_RO(end,1),nNodes);
        data_power_EXP  = resamp3(data_power_EXP', time_two)';
        % ... TSI
        path_power_TSI  = [path_main '\direct_collocation_files\DCResults\' name_current_subject '_Fminco_Actuation_Power.sto'];
        dataPowerTBD    = readMOT(path_power_TSI);
        data_power_TSI  = dataPowerTBD.data;
        
        % GRFs
        % ... exp
        dataGrfTBD      = readMOT([path_grfmot '\' name_current_subject '_grf.mot']);
        data_grf_EXP      = dataGrfTBD.data;
        labels_grf       = dataGrfTBD.labels;
        % ... Select both vx (both feet) kinetic data points in the grf.mot file
        time_grf         = time_two;
        diffG = mean(diff(time_grf));
        data_grf_EXP      = data_grf_EXP(:,[1,find(contains(labels_grf, '_v'))]);
        data_grf_EXP      = resamp3(data_grf_EXP', time_grf)';
        data_grf_EXP(:,1) = [];
        data_grf_EXP = resample(data_grf_EXP(:,:),101,100,5,20); data_grf_EXP = data_grf_EXP(1:nNodes,:);
        % ... fslv and fmin
        data_grf_RO   = readMOT(ground_RO(i).name);
        data_grf_RO = data_grf_RO.data(:,[2:4 8:10]);
        data_grf_TSI     = readMOT(ground_TSI(i).name);
        data_grf_TSI = data_grf_TSI.data(:,[2:4 8:10]);
        % .. filter data over 25 Hz (4th order)
        Fc = 25;
        Fs = 110/2; 
        [b,a]=butter(2,Fc/Fs); % symmetric smoothing filter @ fc hz.
        data_grf_EXP=filtfilt(b,a,data_grf_EXP);
        data_grf_RO=filtfilt(b,a,data_grf_RO); % net 4th order lowpass
        data_grf_TSI=filtfilt(b,a,data_grf_TSI); % net 4th order lowpass
        clear dataGrfTBD
               
        % KINEMATICS
        % .. exp and fslv
        dataKinTBD      = load(data_RO(i).name);
        data_kin_EXP      = [dataKinTBD.datSVC];
        data_kin_RO   = [dataKinTBD.datSVC_F];
        % .. fmin
        dataKinTBD      = load(data_TSI(i).name);
        data_kin_TSI     = [dataKinTBD.datSVC_F];
        clear dataKinTBD
        data_kin_EXP = data_kin_EXP(:,1:87);
        data_kin_RO = data_kin_RO(:,1:87);
        data_kin_TSI = data_kin_TSI(:,1:87);

        % remove first & final 2 indexes, mostly faulty
        rudimentary_nodes = [1];
%         [1,end-1:end]
        data_grf_EXP(rudimentary_nodes,:) = [];
        data_grf_RO(rudimentary_nodes,:) = [];
        data_grf_TSI(rudimentary_nodes,:) = [];
        data_kin_EXP(rudimentary_nodes,:) = [];
        data_kin_RO(rudimentary_nodes,:) = [];
        data_kin_TSI(rudimentary_nodes,:) = [];
        data_power_EXP(rudimentary_nodes,:) = [];
        data_power_RO(rudimentary_nodes,:) = [];
        data_power_TSI(rudimentary_nodes,:) = [];
        time_two(rudimentary_nodes) = [];
        time_grf(rudimentary_nodes) = [];
        nNodes = nNodes - length(rudimentary_nodes);
      
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % ------------------------- Correcting data --------------------------%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % during experiments, the first leg touching the FP differs between
        % subjects -> here we corrrect data so that we ALWAYS have the
        % right leg as the first leg that gives data in the data variables
        % (e.g. dataKin & dataPower
        if data_grf_RO(20,2) <= 100
            
            % allocte first step
            first_step{i} = 'l';
            
            % correct grfs (in exp we always have the correct (right) leg 
            % first)
            data_grf_RO = data_grf_RO(:,[4:6,1:3]);
            data_grf_TSI = data_grf_TSI(:,[4:6,1:3]);

            % correct kinematics
            % .. instances of left-and right
            left_index = [endsWith(labelsKin, '_l'); endsWith(labelsKin, '_l'); endsWith(labels_power(2:end-6), '_l')'];
            right_index = [endsWith(labelsKin, '_r'); endsWith(labelsKin, '_r'); endsWith(labels_power(2:end-6), '_r')'];
            new_kinematics_var = data_kin_EXP(end-(nNodes-1):end,:); old_kinematics_var = data_kin_EXP(end-(nNodes-1):end,:);
            new_kinematics_var(:,left_index) = old_kinematics_var(:, right_index);
            new_kinematics_var(:,right_index) = old_kinematics_var(:, left_index);
            data_kin_EXP(end-(nNodes-1):end,:) = new_kinematics_var;
            new_kinematics_var = data_kin_RO(end-(nNodes-1):end,:); old_kinematics_var = data_kin_RO(end-(nNodes-1):end,:);
            new_kinematics_var(:,left_index) = old_kinematics_var(:, right_index);
            new_kinematics_var(:,right_index) = old_kinematics_var(:, left_index);
            data_kin_RO(end-(nNodes-1):end,:) = new_kinematics_var;
            new_kinematics_var = data_kin_TSI(end-(nNodes-1):end,:); old_kinematics_var = data_kin_TSI(end-(nNodes-1):end,:);
            new_kinematics_var(:,left_index) = old_kinematics_var(:, right_index);
            new_kinematics_var(:,right_index) = old_kinematics_var(:, left_index);
            data_kin_TSI(end-(nNodes-1):end,:) = new_kinematics_var;
            % .. instances where there is no left right
            nright_index = not(endsWith(labelsKin, '_r')); nleft_index = not(endsWith(labelsKin, '_l'));
            das_index = nright_index == nleft_index;
            max_kinval_exp = max(data_kin_EXP(:,das_index));
            max_kinval_ro = max(data_kin_RO(:,das_index));
            max_kinval_tsin = max(data_kin_TSI(:,das_index));
            min_kinval_exp = min(data_kin_EXP(:,das_index));
            min_kinval_ro = min(data_kin_RO(:,das_index));
            min_kinval_tsi = min(data_kin_TSI(:,das_index));
            mid_kinval_exp = max_kinval_exp - .5*(max_kinval_exp-min_kinval_exp);
            mid_kinval_ro = max_kinval_ro - .5*(max_kinval_ro-min_kinval_ro);
            mid_kinval_tsi = max_kinval_tsin - .5*(max_kinval_tsin-min_kinval_tsi);
            data_kin_EXP(:,das_index) = data_kin_EXP(:,das_index) - 2*(data_kin_EXP(:,das_index)-mid_kinval_exp);
            data_kin_RO(:,das_index) = data_kin_RO(:,das_index) - 2*(data_kin_RO(:,das_index)-mid_kinval_ro);
            data_kin_TSI(:,das_index) = data_kin_TSI(:,das_index) - 2*(data_kin_TSI(:,das_index)-mid_kinval_tsi);

            % correct powers
            left_index = [endsWith(labels_power, '_l_reserve')];
            right_index = [endsWith(labels_power, '_r_reserve')];
            new_powers_var = data_power_EXP(end-(nNodes-1):end,:); old_powers_var = data_power_EXP(end-(nNodes-1):end,:);
            new_powers_var(:,left_index) = old_powers_var(:, right_index);
            new_powers_var(:,right_index) = old_powers_var(:, left_index);
            data_power_EXP(end-(nNodes-1):end,:) = new_powers_var;
            new_powers_var = data_power_RO(end-(nNodes-1):end,:); old_powers_var = data_power_RO(end-(nNodes-1):end,:);
            new_powers_var(:,left_index) = old_powers_var(:, right_index);
            new_powers_var(:,right_index) = old_powers_var(:, left_index);
            data_power_RO(end-(nNodes-1):end,:) = new_powers_var;
            new_powers_var = data_power_TSI(end-(nNodes-1):end,:); old_powers_var = data_power_TSI(end-(nNodes-1):end,:);
            new_powers_var(:,left_index) = old_powers_var(:, right_index);
            new_powers_var(:,right_index) = old_powers_var(:, left_index);
            data_power_TSI(end-(nNodes-1):end,:) = new_powers_var;
        else
            % allocte first step
            first_step{i} = 'r';
            
        end; clear new_kinematics_var old_kinematics_var ...
            new_powers_var old_powers_var ...
            max_kinval_exp max_kinval_ro max_kinval_tsi ...
            min_kinval_exp min_kinval_ro min_kinval_tsi ...
            mid_kinval_exp mid_kinval_ro mid_kinval_tsi ...
        
        
        
        % Extrapolate powers to get more accurate power/work readings
        time_EX = pchip(time_two,time_two,time_two(1):(time_two(end)-time_two(1))/(length(time_two)*9.999):time_two(end));
        time_grf_EX = pchip(time_grf,time_grf,time_grf(1):(time_grf(end)-time_grf(1))/(length(time_grf)*9.999):time_grf(end));
        clear grf_exp_EX grf_ro_EX grf_tsi_EX ...
            power_exp_EX power_ro_EX power_tsi_EX ...
            kin_exp_EX kin_ro_EX kin_tsi_EX
        
        for j = 1:size(data_kin_EXP,2)
            kin_exp_EX(:,j) = pchip(time_two,data_kin_EXP(:,j),time_EX);
            kin_ro_EX(:,j) = pchip(time_two,data_kin_RO(:,j),time_EX);
            kin_tsi_EX(:,j) = pchip(time_two,data_kin_TSI(:,j),time_EX);
        end; clear j
%         
        for j = 1:6
            grf_exp_EX(:,j) = pchip(time_grf,data_grf_EXP(:,j),time_grf_EX);
            grf_ro_EX(:,j) = pchip(time_grf,data_grf_RO(:,j),time_grf_EX);
            grf_tsi_EX(:,j) = pchip(time_grf,data_grf_TSI(:,j),time_grf_EX);
        end; clear j

        for j = 1:nCon+6+1
            power_exp_EX(:,j) = pchip(time_two,data_power_EXP(:,j),time_EX);
            power_ro_EX(:,j) = pchip(time_two,data_power_RO(:,j),time_EX);
            power_tsi_EX(:,j) = pchip(time_two,data_power_TSI(:,j),time_EX);
        end; clear j
        
        % save duration of pre-emptive po
        [~,idxPeakAnkleExp]= max(power_exp_EX(:,6));
        time_po_two = find(power_exp_EX(idxPeakAnkleExp:end,6) <=eps,1)+idxPeakAnkleExp;
        idx_step_detect = find(grf_exp_EX(:,5)>=100,1)-10;
        idxAdd = idx_step_detect:time_po_two;
        preper = length(idxAdd)/(length(power_exp_EX)+length(idxAdd))*100;
        datatracking.preper(i).exp = preper;

        % Timing of contralateral contact for deciding where we assume
        % symmetry
        idx_CLAC_ro = nNodes*10-5;
        idx_CLAC_tsi = nNodes*10-5;
        if grf_ro_EX(idx_CLAC_ro,2) >= 150
            idx_CLAC_ro = idx_CLAC_ro-find(grf_ro_EX(idx_CLAC_ro:-1:1,2)<=150, 1, 'first')-20;
        elseif grf_ro_EX(idx_CLAC_ro,2) >= 100
            idx_CLAC_ro = idx_CLAC_ro-find(grf_ro_EX(idx_CLAC_ro:-1:1,2)<=100, 1, 'first')-15;
        elseif grf_ro_EX(idx_CLAC_ro,2) >= 50
            idx_CLAC_ro = idx_CLAC_ro-find(grf_ro_EX(idx_CLAC_ro:-1:1,2)<=50, 1, 'first')-10;
        else
            idx_CLAC_ro = idx_CLAC_ro - 5;
        end
        if grf_tsi_EX(idx_CLAC_tsi,2) >= 150
            idx_CLAC_tsi = idx_CLAC_tsi-find(grf_tsi_EX(idx_CLAC_tsi:-1:1,2)<=150, 1, 'first')-20;
        elseif grf_tsi_EX(idx_CLAC_tsi,2) >= 100
            idx_CLAC_tsi = idx_CLAC_tsi-find(grf_tsi_EX(idx_CLAC_tsi:-1:1,2)<=100, 1, 'first')-15;
        elseif grf_tsi_EX(idx_CLAC_tsi,2) >= 50
            idx_CLAC_tsi = idx_CLAC_tsi-find(grf_tsi_EX(idx_CLAC_tsi:-1:1,2)<=50, 1, 'first')-10;
        else
            idx_CLAC_tsi = idx_CLAC_tsi - 5;
        end
        idx_CLAC_exp = idx_CLAC_ro;
        idx_CLIC_exp = find(grf_exp_EX(:,5)>=100,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        idx_CLIC_ro = idx_CLIC_exp;
        idx_CLIC_tsi = idx_CLIC_exp;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        idx_CLIC_exp = idx_CLIC_exp - find(grf_exp_EX(idx_CLIC_exp:-1:1,5)<=25, 1, 'first');
        idx_CLIC_ro = idx_CLIC_exp;
        idx_CLIC_tsi = idx_CLIC_exp;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        grf_ro_EX(1:idx_CLIC_ro,4:6) = 0;
        grf_tsi_EX(1:idx_CLIC_tsi,4:6) = 0;
        
        
        % ...select and preallocate data from CLIC to ILIC (which is the end)
        time_exp_EX_raw = time_EX(idx_CLIC_exp:idx_CLAC_exp)';
        time_ro_EX_raw = time_EX(idx_CLIC_ro:idx_CLAC_ro)';
        time_tsi_EX_raw = time_EX(idx_CLIC_tsi:idx_CLAC_tsi)';
%         time_exp_EX_raw = [time_exp_EX_raw-range(time_exp_EX_raw)-mean(diff(time_exp_EX_raw));time_exp_EX_raw];
%         time_ro_EX_raw = [time_ro_EX_raw-range(time_ro_EX_raw)-mean(diff(time_ro_EX_raw));time_ro_EX_raw];
%         time_tsi_EX_raw = [time_tsi_EX_raw-range(time_tsi_EX_raw)-mean(diff(time_tsi_EX_raw));time_tsi_EX_raw];
%         
        kin_exp_EX_raw = kin_exp_EX(idx_CLIC_exp:idx_CLAC_exp,:);
        kin_ro_EX_raw = kin_ro_EX(idx_CLIC_ro:idx_CLAC_ro,:);
        kin_tsi_EX_raw = kin_tsi_EX(idx_CLIC_tsi:idx_CLAC_tsi,:);
        grf_exp_EX_raw = grf_exp_EX(idx_CLIC_exp:idx_CLAC_exp,:);
        grf_ro_EX_raw = grf_ro_EX(idx_CLIC_ro:idx_CLAC_ro,:);
        grf_tsi_EX_raw = grf_tsi_EX(idx_CLIC_tsi:idx_CLAC_tsi,:);
        power_exp_EX_raw = power_exp_EX(idx_CLIC_exp:idx_CLAC_exp,:);
        power_ro_EX_raw = power_ro_EX(idx_CLIC_ro:idx_CLAC_ro,:);
        power_tsi_EX_raw = power_tsi_EX(idx_CLIC_tsi:idx_CLAC_tsi,:);
      
        % POWER SYMMETRY
        allpowers = {'power_exp_EX'; 'power_ro_EX'; 'power_tsi_EX'};
        allstarts = [idx_CLIC_exp, idx_CLIC_ro, idx_CLIC_tsi];
        allends = [idx_CLAC_exp, idx_CLAC_ro, idx_CLAC_tsi];
        for mode = 1:3
            
            % Select
            datset = eval(allpowers{mode});
            start = allstarts(mode);
            fin = allends(mode);
            
            % current values
            current_power = datset(start:fin,:);
            current_index = start;
            ptest(mode).mode = zeros(1000,32);
            for pi = 2:32
                current_index = start;
                current_joint_power = current_power(:,pi);
                current_time = time_EX(start:fin)';
                cur_pow_test = current_joint_power(1)-current_joint_power(end);
                it = 0;
                if any(pi-[20:32] == 0)
                    ntime = pchip(current_time, current_time, linspace(current_time(1),current_time(end),1000));
                    npowr = pchip(current_time, current_joint_power, ntime);
                    ptest(mode).mode(:,pi) = npowr; 
                    continue
                end
                while abs(current_joint_power(1)-current_joint_power(end)) >= .01*range(current_joint_power)
                    if it == 25
                        break
                    end
                    it = it + 1;
                    gradient_power = diff(current_joint_power);
                    gradient_power1 = gradient_power(1);
                    if (current_joint_power(1)-current_joint_power(end)) <=0 && gradient_power1 > 0
                        current_index = current_index + 1;
                        current_joint_power = datset(current_index:fin,pi);
                        current_time = time_EX(current_index:fin)';
                    elseif (current_joint_power(1)-current_joint_power(end)) > 0 && gradient_power1 > 0
                        current_index = current_index - 1;
                        current_joint_power = datset(current_index:fin,pi);
                        current_time = time_EX(current_index:fin)';
                    elseif (current_joint_power(1)-current_joint_power(end)) <= 0 && gradient_power1 < 0
                        current_index = current_index - 1;
                        current_joint_power = datset(current_index:fin,pi);
                        current_time = time_EX(current_index:fin)';
                    else
                        current_index = current_index + 1;
                        current_joint_power = datset(current_index:fin,pi);  
                        current_time = time_EX(current_index:fin)';
                    end
                end
%                 disp(current_index)
                ntime = pchip(current_time, current_time, linspace(current_time(1),current_time(end),1000));
                npowr = pchip(current_time, current_joint_power, ntime);
                ptest(mode).mode(:,pi) = npowr; 
            end
            ptest(mode).mode(:,1) = ntime;
        end
        
        power_exp_EX_raw = ptest(1).mode;
        power_ro_EX_raw = ptest(2).mode;
        power_tsi_EX_raw = ptest(3).mode;
        
%         time_exp_EX_raw = power_exp_EX_raw(:,1);
%         time_ro_EX_raw = power_ro_EX_raw(:,1);
%         time_tsi_EX_raw = power_tsi_EX_raw(:,1);
        
        % preallocate mirrored data space
        time_exp_EX_mir = [time_exp_EX_raw;time_exp_EX_raw];
        time_ro_EX_mir = [time_ro_EX_raw;time_ro_EX_raw];
        time_tsi_EX_mir = [time_tsi_EX_raw;time_tsi_EX_raw];
        kin_exp_EX_mir = [kin_exp_EX_raw;kin_exp_EX_raw];
        kin_ro_EX_mir = [kin_ro_EX_raw;kin_ro_EX_raw];
        kin_tsi_EX_mir = [kin_tsi_EX_raw;kin_tsi_EX_raw];
        grf_exp_EX_mir = [grf_exp_EX_raw;grf_exp_EX_raw];
        grf_ro_EX_mir = [grf_ro_EX_raw;grf_ro_EX_raw];
        grf_tsi_EX_mir = [grf_tsi_EX_raw;grf_tsi_EX_raw];
        power_exp_EX_mir = [power_exp_EX_raw;power_exp_EX_raw];
        power_ro_EX_mir = [power_ro_EX_raw;power_ro_EX_raw];
        power_tsi_EX_mir = [power_tsi_EX_raw;power_tsi_EX_raw];
        
        % correct powers
        left_index = [endsWith(labels_power, '_l_reserve')];
        right_index = [endsWith(labels_power, '_r_reserve')];
        new_powers_var = power_exp_EX_mir(1:length(power_exp_EX_raw),:); 
        old_powers_var = power_exp_EX_mir(1:length(power_exp_EX_raw),:);
        new_powers_var(:,left_index) = old_powers_var(:, right_index);
        new_powers_var(:,right_index) = old_powers_var(:, left_index);
        power_exp_EX_mir(1:length(power_exp_EX_raw),:) = new_powers_var;
        new_powers_var = power_ro_EX_mir(1:length(power_ro_EX_raw),:); 
        old_powers_var = power_ro_EX_mir(1:length(power_ro_EX_raw),:);
        new_powers_var(:,left_index) = old_powers_var(:, right_index);
        new_powers_var(:,right_index) = old_powers_var(:, left_index);
        power_ro_EX_mir(1:length(power_ro_EX_raw),:) = new_powers_var;
        new_powers_var = power_tsi_EX_raw(1:length(power_tsi_EX_raw),:); 
        old_powers_var = power_tsi_EX_raw(1:length(power_tsi_EX_raw),:);
        new_powers_var(:,left_index) = old_powers_var(:, right_index);
        new_powers_var(:,right_index) = old_powers_var(:, left_index);
        power_tsi_EX_mir(1:length(power_tsi_EX_raw),:) = new_powers_var;
        % ... rough copy pastes -> lets change that
        power_exp_EX_mir(length(power_exp_EX_mir)/2-30:length(power_exp_EX_mir)/2+30,:) = nan;
        power_ro_EX_mir(length(power_ro_EX_mir)/2-30:length(power_ro_EX_mir)/2+30,:) = nan;
        power_tsi_EX_mir(length(power_tsi_EX_mir)/2-30:length(power_tsi_EX_mir)/2+30,:) = nan;
        [power_exp_EX_mir,~] = fillmissing(power_exp_EX_mir,'spline');
        [power_ro_EX_mir,~] = fillmissing(power_ro_EX_mir,'spline');
        [power_tsi_EX_mir,~] = fillmissing(power_tsi_EX_mir,'spline');
        
        
        % grf
        grf_exp_EX_mir(1:length(grf_exp_EX_raw),1:3) = grf_exp_EX_raw(:,4:6);
        grf_ro_EX_mir(1:length(grf_ro_EX_raw),1:3) = grf_ro_EX_raw(:,4:6);
        grf_tsi_EX_mir(1:length(grf_tsi_EX_raw),1:3) = grf_tsi_EX_raw(:,4:6);
        grf_exp_EX_mir(1:length(grf_exp_EX_raw),4:6) = grf_exp_EX_raw(:,1:3);
        grf_ro_EX_mir(1:length(grf_ro_EX_raw),4:6) = grf_ro_EX_raw(:,1:3);
        grf_tsi_EX_mir(1:length(grf_tsi_EX_raw),4:6) = grf_tsi_EX_raw(:,1:3);
        % ... rough copy pastes -> lets change that
        grf_exp_EX_mir(length(grf_exp_EX_mir)/2-15:length(grf_exp_EX_mir)/2+15,:) = nan;
        grf_ro_EX_mir(length(grf_ro_EX_mir)/2-15:length(grf_ro_EX_mir)/2+15,:) = nan;
        grf_tsi_EX_mir(length(grf_tsi_EX_mir)/2-15:length(grf_tsi_EX_mir)/2+15,:) = nan;
        [grf_exp_EX_mir,~] = fillmissing(grf_exp_EX_mir,'spline');
        [grf_ro_EX_mir,~] = fillmissing(grf_ro_EX_mir,'spline');
        [grf_tsi_EX_mir,~] = fillmissing(grf_tsi_EX_mir,'spline');
        time_exp_grf = [time_exp_EX_raw-range(time_exp_EX_raw)-mean(diff(time_exp_EX_raw));time_exp_EX_raw];
        time_ro_grf = [time_ro_EX_raw-range(time_ro_EX_raw)-mean(diff(time_ro_EX_raw));time_ro_EX_raw];
        time_tsi_grf = [time_tsi_EX_raw-range(time_tsi_EX_raw)-mean(diff(time_tsi_EX_raw));time_tsi_EX_raw];
        time_exp_grf_EX = pchip(time_exp_grf,time_exp_grf,linspace(time_exp_grf(1),time_exp_grf(end),2000));
        time_ro_grf_EX = pchip(time_ro_grf,time_ro_grf,linspace(time_ro_grf(1),time_ro_grf(end),2000));
        time_tsi_grf_EX = pchip(time_tsi_grf,time_tsi_grf,linspace(time_tsi_grf(1),time_tsi_grf(end),2000));
        clear grf_exp_EX grf_ro_EX grf_tsi_EX
        for j = 1:6
            grf_exp_EX(:,j) = pchip(time_exp_grf,grf_exp_EX_mir(:,j),time_exp_grf_EX);
            grf_ro_EX(:,j) = pchip(time_ro_grf,grf_ro_EX_mir(:,j),time_ro_grf_EX);
            grf_tsi_EX(:,j) = pchip(time_tsi_grf,grf_tsi_EX_mir(:,j),time_tsi_grf_EX);
        end; clear j
        
        
        % Add assumed/ predicted values to kinematic matrices
        dataKinExp = circshift([nan(length(idxAdd),87);kin_exp_EX],1);
        dataKinRO = circshift([nan(length(idxAdd),87);kin_ro_EX],1);
        dataKinTSI = circshift([nan(length(idxAdd),87);kin_tsi_EX],1);
        [kin_exp_EX_mir,~] = fillmissing(dataKinExp,'pchip');
        [kin_ro_EX_mir,~] = fillmissing(dataKinRO,'pchip');
        [kin_tsi_EX_mir,~] = fillmissing(dataKinTSI,'pchip');
        new_time1 = time_EX(1)-length(idxAdd)*mean(diff(time_EX));
        time_exp_kin = [linspace(new_time1,time_EX(1)-mean(diff(time_EX)),length(idxAdd)),time_EX];
        time_ro_kin = time_exp_kin;
        time_tsi_kin = time_exp_kin;
        time_exp_kin_EX = pchip(time_exp_kin,time_exp_kin,linspace(time_exp_kin(1),time_exp_kin(end),2000));
        time_ro_kin_EX = pchip(time_ro_kin,time_ro_kin,linspace(time_ro_kin(1),time_ro_kin(end),2000));
        time_tsi_kin_EX = pchip(time_tsi_kin,time_tsi_kin,linspace(time_tsi_kin(1),time_tsi_kin(end),2000));
        clear kin_exp_EX kin_ro_EX kin_tsi_EX
        for j = 1:size(kin_exp_EX_mir,2)
            kin_exp_EX(:,j) = pchip(time_exp_kin,kin_exp_EX_mir(:,j),time_exp_kin_EX);
            kin_ro_EX(:,j) = pchip(time_ro_kin,kin_ro_EX_mir(:,j),time_ro_kin_EX);
            kin_tsi_EX(:,j) = pchip(time_tsi_kin,kin_tsi_EX_mir(:,j),time_tsi_kin_EX);
        end; clear j
%         
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % ----------------- Calculating outcome variables --------------------%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Calculate ankle kinetics during push-off phase
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % here, we still use the regular calculated power curved (EX)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        idxAnk = find(startsWith(labels_power, 'ankle_angle'));
        powerAnkleEXP = power_exp_EX_mir(:,idxAnk);
        powerAnkleRO = power_ro_EX_mir(:,idxAnk);
        powerAnkleTSI = power_tsi_EX_mir(:,idxAnk);
        % ... detection of push-off phase indexes
        [~,idxPeakAnkleExp]= max(powerAnkleEXP(:,1)); [~,idxPeakAnkleExp2]= max(powerAnkleEXP(:,2)); 
        [~,idxPeakAnkleRO]= max(powerAnkleRO(:,1)); [~,idxPeakAnkleRO2]= max(powerAnkleRO(:,2)); 
        [~,idxPeakAnkleTSI]= max(powerAnkleTSI(:,1)); [~,idxPeakAnkleTSI2]= max(powerAnkleTSI(:,2));

        tpo1 = find(powerAnkleEXP(1:idxPeakAnkleExp,1) <= eps,1,'last'); time_po_two = find(powerAnkleEXP(idxPeakAnkleExp:end,1) <=eps,1,'first');
        tpo2 = find(powerAnkleEXP(1:idxPeakAnkleExp2,2) <= eps,1,'last'); time_po_two2 = find(powerAnkleEXP(idxPeakAnkleExp2:end,2) <=eps,1,'first');
        if isempty(tpo2)
            tpo2 = 1;
        end
        idxPOExp2 = [(tpo1(end):1:idxPeakAnkleExp+time_po_two(1)-1)] ;
        idxPOExp1 = [(1:idxPeakAnkleExp2+time_po_two2(1)-1)] ;
        tpo1 = find(powerAnkleRO(1:idxPeakAnkleRO) <= eps,1,'last'); time_po_two = find(powerAnkleRO(idxPeakAnkleRO:end) <=eps,1,'first');
        tpo2 = find(powerAnkleRO(1:idxPeakAnkleRO2,2) <= eps,1,'last'); time_po_two2 = find(powerAnkleRO(idxPeakAnkleRO2:end,2) <=eps,1,'first');
        if isempty(tpo2)
            tpo2 = 1;
        end
        idxPOFsl2 = [(tpo1(end):1:idxPeakAnkleRO+time_po_two(1)-1)] ;
        idxPOFsl1 = [(1:idxPeakAnkleRO2+time_po_two2(1)-1)] ;
        tpo1 = find(powerAnkleTSI(1:idxPeakAnkleTSI) <= eps,1,'last'); time_po_two = find(powerAnkleTSI(idxPeakAnkleTSI:end) <=eps,1,'first');
        tpo2 = find(powerAnkleTSI(1:idxPeakAnkleTSI2,2) <= eps,1,'last'); time_po_two2 = find(powerAnkleTSI(idxPeakAnkleTSI2:end,2) <=eps,1,'first');
        if isempty(tpo2)
            tpo2 = 1;
        end
        idxPOFmc2 = [(tpo1(end):1:idxPeakAnkleTSI+time_po_two(1)-1)] ;
        idxPOFmc1 = [(1:idxPeakAnkleTSI2+time_po_two2(1)-1)] ;

        totSumTBD = sum(power_exp_EX_mir,2);
        totIdxTBD2 = idxPOExp2;
        totIdxTBD1 = idxPOExp1;
        idx_diff_rb2 = totIdxTBD2(end)+250;
        idx_diff_rb1 = totIdxTBD1(end)+250;
        idxElLast = find(totSumTBD(idx_diff_rb2:end) <= 0, 1, 'first') + idx_diff_rb2;
        idxRbLast = find(totSumTBD(idx_diff_rb1:end) <= 0, 1, 'first') + idx_diff_rb1;
        idxRBExp2 = [idx_diff_rb2:idxElLast];
        idxRBExp1 = [idx_diff_rb1:idxRbLast];

        totSumTBD = sum(power_ro_EX_mir,2);
        totIdxTBD2 = idxPOFsl2;
        totIdxTBD1 = idxPOFsl1;
        idx_diff_rb2 = totIdxTBD2(end)+250;
        idx_diff_rb1 = totIdxTBD1(end)+250;
        idxElLast = find(totSumTBD(idx_diff_rb2:end) <= 0, 1, 'first') + idx_diff_rb2;
        idxRbLast = find(totSumTBD(idx_diff_rb1:end) <= 0, 1, 'first') + idx_diff_rb1;
        idxRBFsl2 = [idx_diff_rb2:idxElLast];
        idxRBFsl1 = [idx_diff_rb1:idxRbLast];

        totSumTBD = sum(power_tsi_EX_mir,2);
        totIdxTBD2 = idxPOFmc2;
        totIdxTBD1 = idxPOFmc1;
        idx_diff_rb2 = totIdxTBD2(end)+250;
        idx_diff_rb1 = totIdxTBD1(end)+250;
        idxElLast = find(totSumTBD(idx_diff_rb2:end) <= 0, 1, 'first') + idx_diff_rb2;
        idxRbLast = find(totSumTBD(idx_diff_rb1:end) <= 0, 1, 'first') + idx_diff_rb1;
        idxRBFmc2 = [idx_diff_rb2:idxElLast];
        idxRBFmc1 = [idx_diff_rb1:idxRbLast];

        
        % i think that we can now say; rename mir data to just null
        time_exp_EX = time_exp_EX_mir;
        time_ro_EX = time_ro_EX_mir;
        time_tsi_EX = time_tsi_EX_mir;
        
        time_exp_POW = power_exp_EX_raw(:,1);
        time_ro_POW = power_ro_EX_raw(:,1);
        time_tsi_POW = power_tsi_EX_raw(:,1);
        time_exp_POW = [time_exp_POW-range(time_exp_POW)-mean(diff(time_exp_POW));time_exp_POW];
        time_ro_POW = [time_ro_POW-range(time_ro_POW)-mean(diff(time_ro_POW));time_ro_POW];
        time_tsi_POW = [time_tsi_POW-range(time_tsi_POW)-mean(diff(time_tsi_POW));time_tsi_POW];
        
        % ... calculating power DURING these push-off phases
        powerExpPO1  = power_exp_EX_mir(idxPOExp1,:); tExpPO1 = time_exp_POW(idxPOExp1,1);
        powerFslvPO1  = power_ro_EX_mir(idxPOFsl1,:); tFlsPO1 = time_ro_POW(idxPOFsl1,1);
        powerFminPO1  = power_tsi_EX_mir(idxPOFmc2,:); tFmcPO1 = time_tsi_POW(idxPOFmc1,1);
        powerExpPO2   = power_exp_EX_mir(idxPOExp2,:); tExpPO2 = time_exp_POW(idxPOExp2,1);
        powerFslvPO2  = power_ro_EX_mir(idxPOFsl2,:); tFlsPO2 = time_ro_POW(idxPOFsl2,1);
        powerFminPO2  = power_tsi_EX_mir(idxPOFmc2,:); tFmcPO2 = time_tsi_POW(idxPOFmc2,1);
        
        % ... calculating power DURING these rebound phases
        powerExpRb1   = power_exp_EX_mir(idxRBExp1,:); tExpRb1 = time_exp_POW(idxRBExp1,1);
        powerFslvRb1  = power_ro_EX_mir(idxRBFsl1,:); tFlsRb1 = time_ro_POW(idxRBFsl1,1);
        powerFminRb1  = power_tsi_EX_mir(idxRBFmc1,:); tFmcRb1 = time_tsi_POW(idxRBFmc1,1);
        powerExpRb2   = power_exp_EX_mir(idxRBExp2,:); tExpRb2 = time_exp_POW(idxRBExp2,1);
        powerFslvRb2  = power_ro_EX_mir(idxRBFsl2,:); tFlsRb2 = time_ro_POW(idxRBFsl2,1);
        powerFminRb2  = power_tsi_EX_mir(idxRBFmc2,:); tFmcRb2 = time_tsi_POW(idxRBFmc2,1);

        
        power_exp_EX = power_exp_EX_mir;
        power_ro_EX = power_ro_EX_mir;
        power_tsi_EX = power_tsi_EX_mir;
        
        
        
        % ... TOTAL POWER
        sumJointPowerExp    = sum(power_exp_EX,2);
        sumJointPowerFsolve = sum(power_ro_EX,2);
        sumJointPowerFmin   = sum(power_tsi_EX,2);
        sumJointPowerExp(sumJointPowerExp<0) = 0;
        sumJointPowerFsolve(sumJointPowerFsolve<0) = 0;
        sumJointPowerFmin(sumJointPowerFmin<0) = 0;
        
        
        % ... PO work
        workFslPOTotal1 = trapz(tFlsPO1,grf_ro_EX(idxPOFsl1,5));
        workFmcPOTotal1 = trapz(tFmcPO1,grf_tsi_EX(idxPOFmc1,5));
        grfFslPOipsiTotal = workFslPOTotal1;
        grfFmcPOipsiTotal = workFmcPOTotal1;
        grfFslTotal = trapz(time_ro_POW,grf_ro_EX(:,5));
        grfFmcTotal = trapz(time_tsi_POW,grf_tsi_EX(:,5));

        
        
        
        % ... TOTAL WORK
        % cheat work
        workExpTotal = trapz(time_exp_POW,sumJointPowerExp(1:end));
        workFslTotal = trapz(time_ro_POW,sumJointPowerFsolve(1:end));
        workFmcTotal = trapz(time_tsi_POW,sumJointPowerFmin(1:end));
        
        % ... PO work
        workExpPOTotal1 = trapz(tExpPO1,sumJointPowerExp(idxPOExp1));
        workFslPOTotal1 = trapz(tFlsPO1,sumJointPowerFsolve(idxPOFsl1));
        workFmcPOTotal1 = trapz(tFmcPO1,sumJointPowerFmin(idxPOFmc1));
        workExpPOTotal2 = trapz(tExpPO2,sumJointPowerExp(idxPOExp2));
        workFslPOTotal2 = trapz(tFlsPO2,sumJointPowerFsolve(idxPOFsl2));
        workFmcPOTotal2 = trapz(tFmcPO2,sumJointPowerFmin(idxPOFmc2));
        workExpPOTotal = workExpPOTotal1+workExpPOTotal2;
        workFslPOTotal = workFslPOTotal1+workFslPOTotal2;
        workFmcPOTotal = workFmcPOTotal1+workFmcPOTotal2;
        
        % ... RB work
        workExpRbTotal1 = trapz(tExpRb1,sumJointPowerExp(idxRBExp1));
        workFslRbTotal1 = trapz(tFlsRb1,sumJointPowerFsolve(idxRBFsl1));
        workFmcRbTotal1 = trapz(tFmcRb1,sumJointPowerFmin(idxRBFmc1));
        workExpRbTotal2 = trapz(tExpRb2,sumJointPowerExp(idxRBExp2));
        workFslRbTotal2 = trapz(tFlsRb2,sumJointPowerFsolve(idxRBFsl2));
        workFmcRbTotal2 = trapz(tFmcRb2,sumJointPowerFmin(idxRBFmc2));
        workExpRbTotal = workExpRbTotal1+workExpRbTotal2;
        workFslRbTotal = workFslRbTotal1+workFslRbTotal2;
        workFmcRbTotal = workFmcRbTotal1+workFmcRbTotal2;

        
        % ... JOINT POWER
        sideJointPowerExp = power_exp_EX; sideJointPowerExp(:,2) = sum(sideJointPowerExp(:,2:4),2); sideJointPowerExp(:,[3,4]) = 0;
        sideJointPowerFsolve = power_ro_EX; sideJointPowerFsolve(:,2) = sum(sideJointPowerFsolve(:,2:4),2); sideJointPowerFsolve(:,[3,4]) = 0;
        sideJointPowerFmin = power_tsi_EX; sideJointPowerFmin(:,2) = sum(sideJointPowerFmin(:,2:4),2); sideJointPowerFmin(:,[3,4]) = 0;
        sideJointPowerExp(:,8) = sum(sideJointPowerExp(:,8:10),2); sideJointPowerExp(:,[9,10]) = 0;
        sideJointPowerFsolve(:,8) = sum(sideJointPowerFsolve(:,8:10),2); sideJointPowerFsolve(:,[9,10]) = 0;
        sideJointPowerFmin(:,8) = sum(sideJointPowerFmin(:,8:10),2); sideJointPowerFmin(:,[9,10]) = 0;
        sideJointPowerExp(:,14) = sum(sideJointPowerExp(:,14:16),2); sideJointPowerExp(:,[15,16]) = 0;
        sideJointPowerFsolve(:,14) = sum(sideJointPowerFsolve(:,14:16),2); sideJointPowerFsolve(:,[15,16]) = 0;
        sideJointPowerFmin(:,14) = sum(sideJointPowerFmin(:,14:16),2); sideJointPowerFmin(:,[15,16]) = 0;
        sideJointPowerExp(:,17) = sum(sideJointPowerExp(:,17:end-6),2); sideJointPowerExp(:,[18:end-6]) = 0;
        sideJointPowerFsolve(:,17) = sum(sideJointPowerFsolve(:,17:end-6),2); sideJointPowerFsolve(:,[18:end-6]) = 0;
        sideJointPowerFmin(:,17) = sum(sideJointPowerFmin(:,17:end-6),2); sideJointPowerFmin(:,[18:end-6]) = 0;
        sideJointPowerExp(:,end-5) = sum(sideJointPowerExp(:,end-5:end),2); sideJointPowerExp(:,[end-4:end]) = 0;
        sideJointPowerFsolve(:,end-5) = sum(sideJointPowerFsolve(:,end-5:end),2); sideJointPowerFsolve(:,[end-4:end]) = 0;
        sideJointPowerFmin(:,end-5) = sum(sideJointPowerFmin(:,end-5:end),2); sideJointPowerFmin(:,[end-4:end]) = 0;
        sideJointPowerExp(sideJointPowerExp<0) = 0;
        sideJointPowerFsolve(sideJointPowerFsolve<0) = 0;
        sideJointPowerFmin(sideJointPowerFmin<0) = 0;

        
        % ... JOINT WORK
        sideworkExpJoint = trapz(time_exp_POW,sideJointPowerExp);
        sideworkFslJoint = trapz(time_ro_POW,sideJointPowerFsolve);
        sideworkFmcJoint = trapz(time_tsi_POW,sideJointPowerFmin);

        sideworkExpPOJoint1 = trapz(tExpPO1,sideJointPowerExp(idxPOExp1,:));
        sideworkFslPOJoint1 = trapz(tFlsPO1,sideJointPowerFsolve(idxPOFsl1,:));
        sideworkFmcPOJoint1 = trapz(tFmcPO1,sideJointPowerFmin(idxPOFmc1,:));
        sideworkExpPOJoint2 = trapz(tExpPO2,sideJointPowerExp(idxPOExp2,:));
        sideworkFslPOJoint2 = trapz(tFlsPO2,sideJointPowerFsolve(idxPOFsl2,:));
        sideworkFmcPOJoint2 = trapz(tFmcPO2,sideJointPowerFmin(idxPOFmc2,:));
        sideworkExpPOJoint = sideworkExpPOJoint1+sideworkExpPOJoint2;
        sideworkFslPOJoint = sideworkFslPOJoint1+sideworkFslPOJoint2;
        sideworkFmcPOJoint = sideworkFmcPOJoint1+sideworkFmcPOJoint2;
        
        sideworkExpRbJoint1 = trapz(tExpRb1,sideJointPowerExp(idxRBExp1,:));
        sideworkFslRbJoint1 = trapz(tFlsRb1,sideJointPowerFsolve(idxRBFsl1,:));
        sideworkFmcRbJoint1 = trapz(tFmcRb1,sideJointPowerFmin(idxRBFmc1,:));
        sideworkExpRbJoint2 = trapz(tExpRb2,sideJointPowerExp(idxRBExp2,:));
        sideworkFslRbJoint2 = trapz(tFlsRb2,sideJointPowerFsolve(idxRBFsl2,:));
        sideworkFmcRbJoint2 = trapz(tFmcRb2,sideJointPowerFmin(idxRBFmc2,:));
        sideworkExpRbJoint = sideworkExpRbJoint1+sideworkExpRbJoint2;
        sideworkFslRbJoint = sideworkFslRbJoint1+sideworkFslRbJoint2;
        sideworkFmcRbJoint = sideworkFmcRbJoint1+sideworkFmcRbJoint2;
       
   
        % TRUNK AMPLITUDE differences
        idxAnk       = find(startsWith(labels_power, 'ankle_angle'), 1);
        idxTrunk     = find(startsWith(labelsKin, 'lumbar_bending'));
        backExp      = kin_exp_EX(:,idxTrunk);
        backFsolve   = kin_ro_EX(:,idxTrunk);
        backFmin     = kin_tsi_EX(:,idxTrunk);

        [~,idxPeakAnkleExps] = max(power_exp_EX(:,idxAnk));
        [~,idxPeakAnkleFsls] = max(power_ro_EX(:,idxAnk));
        [~,idxPeakAnkleFmcs] = max(power_tsi_EX(:,idxAnk));
                
        ampdifExp = max(backExp) - (max(backExp) - min(backExp))/2;
        ampdifFsl = max(backFsolve) - (max(backFsolve) - min(backFsolve))/2;
        ampdifFmc = max(backFmin) - (max(backFmin) - min(backFmin))/2;

        timingExpPow = 100*(backExp(idxPeakAnkleExps)-ampdifExp)/((max(backExp) - min(backExp))/2);
        timingFslPow = 100*(backFsolve(idxPeakAnkleFsls)-ampdifFsl)/((max(backFsolve) - min(backFsolve))/2);
        timingFmcPow = 100*(backFmin(idxPeakAnkleFmcs)-ampdifFmc)/((max(backFmin) - min(backFmin))/2);

        idxStepExp = find(grf_exp_EX(500:end,5) >= 100,1)-30+500;
        idxStepFsl = find(grf_ro_EX(500:end,5) >= 100,1)-30+500;
        idxStepFmc = find(grf_tsi_EX(500:end,5) >= 100,1)-30+500;

        timingExppk = 100*(backExp(idxStepExp)-ampdifExp)/((max(backExp) - min(backExp))/2);
        timingFslpk = 100*(backFsolve(idxStepFsl)-ampdifFsl)/((max(backFsolve) - min(backFsolve))/2);
        timingFmcpk = 100*(backFmin(idxStepFmc)-ampdifFmc)/((max(backFmin) - min(backFmin))/2);

        
        
        datatracking.time(i).exp = time_exp_kin_EX;
        datatracking.time(i).fsl = time_ro_kin_EX;
        datatracking.time(i).fmc = time_tsi_kin_EX;

        datatracking.kinemat(i).exp = kin_exp_EX;
        datatracking.kinemat(i).fsl = kin_ro_EX;
        datatracking.kinemat(i).fmc = kin_tsi_EX;

        datatracking.bwSubj(i).exp = current_osim_bw;
%         idxStepyo = find(grfSubE(length(idxAdd)+5:end,5) >= 15,1)+length(idxAdd)-1;
        datatracking.timingICandPeak(i).exp = [idxStepFsl, idxPeakAnkleExps];
        datatracking.preper(i).exp = preper;
        datatracking.power(i).exp = power_exp_EX;
        datatracking.power(i).fsl = power_ro_EX;
        datatracking.power(i).fmc = power_tsi_EX;
        
        datatracking.workTot(i).exp = workExpTotal;
        datatracking.workTot(i).fsl = workFslTotal;
        datatracking.workTot(i).fmc = workFmcTotal;

        datatracking.workPO(i).exp = workExpPOTotal;
        datatracking.workPO(i).fsl = workFslPOTotal;
        datatracking.workPO(i).fmc = workFmcPOTotal;

        datatracking.workRb(i).exp = workExpRbTotal;
        datatracking.workRb(i).fsl = workFslRbTotal;
        datatracking.workRb(i).fmc = workFmcRbTotal;

        datatracking.grfPO(i).fsl = grfFslPOipsiTotal;
        datatracking.grftot(i).fsl = grfFslTotal;
        datatracking.grfPO(i).fmc = grfFmcPOipsiTotal;
        datatracking.grftot(i).fmc = grfFmcTotal;

%         grfFslPOipsiTotal = workFslPOTotal1;
%         grfFmcPOipsiTotal = workFmcPOTotal1;
%         grfFslTotal = trapz(time_ro_POW,grf_ro_EX(:,5));
%         grfFmcTotal = trapz(time_tsi_POW,grf_tsi_EX(:,5));
        
        
%         datatracking.JworkTot(i).exp = workExpJoint;
%         datatracking.JworkTot(i).fsl = workFslJoint;
%         datatracking.JworkTot(i).fmc = workFmcJoint;
% 
%         datatracking.JworkPO(i).exp = workExpPOJoint;
%         datatracking.JworkPO(i).fsl = workFslPOJoint;
%         datatracking.JworkPO(i).fmc = workFmcPOJoint;
% %         datatracking.JworkPO(i).fp = workFpPOJoint;
% 
%         datatracking.JworkRb(i).exp = workExpRbJoint;
%         datatracking.JworkRb(i).fsl = workFslRbJoint;
%         datatracking.JworkRb(i).fmc = workFmcRbJoint;
% %         datatracking.JworkRb(i).fp = workFpRbJoint;

        datatracking.sideJworkTot(i).exp = sideworkExpJoint;
        datatracking.sideJworkTot(i).fsl = sideworkFslJoint;
        datatracking.sideJworkTot(i).fmc = sideworkFmcJoint;

        datatracking.sideJworkPO(i).exp = sideworkExpPOJoint;
        datatracking.sideJworkPO(i).fsl = sideworkFslPOJoint;
        datatracking.sideJworkPO(i).fmc = sideworkFmcPOJoint;

        datatracking.sideJworkRb(i).exp = sideworkExpRbJoint;
        datatracking.sideJworkRb(i).fsl = sideworkFslRbJoint;
        datatracking.sideJworkRb(i).fmc = sideworkFmcRbJoint;
    
        datatracking.timingtrunk(i).exp = timingExpPow;
        datatracking.timingtrunk(i).fsl = timingFslPow;
        datatracking.timingtrunk(i).fmc = timingFmcPow;

        datatracking.timingtrunkC(i).exp = timingExppk;
        datatracking.timingtrunkC(i).fsl = timingFslpk;
        datatracking.timingtrunkC(i).fmc = timingFmcpk;

        datatracking.grf(i).exp = grf_exp_EX;
        datatracking.grf(i).fsl = grf_ro_EX;
        datatracking.grf(i).fmc = grf_tsi_EX;

        datatracking.totalPower(i).exp = sumJointPowerExp;
        datatracking.totalPower(i).fsl = sumJointPowerFmin;
        datatracking.totalPower(i).fmc = sumJointPowerFsolve;

        end; clear i
    save('datamat', 'labels_power', 'labelsKin', 'datatracking')
    clear assumptionCutOff meanvalLines
    for i = 1:num_subjects
        
        assumptionCutOff(i) = datatracking.preper(i).exp;
        meanvalLines(:,:,i) = datatracking.timingICandPeak(i).exp;
        current_osim_bw(i) = datatracking.bwSubj(i).exp;
    end
    assumptionCutOff = mean(assumptionCutOff);
    meanvalLines = mean(meanvalLines,3);
    current_osim_bw = current_osim_bw
else
    load('datamat')
    clear assumptionCutOff meanvalLines
    for i = 1:num_subjects
       
        assumptionCutOff(i) = datatracking.preper(i).exp;
        meanvalLines(:,:,i) = datatracking.timingICandPeak(i).exp;
        current_osim_bw(i) = datatracking.bwSubj(i).exp;
    end
    assumptionCutOff = mean(assumptionCutOff);
    meanvalLines = mean(meanvalLines,3);
end
        
return


% CALC RMSE
try
    for i = 1:num_subjects
        % Rotation pqs
        Erp = datatracking.kinemat(i).exp(:,1:3) - datatracking.kinemat(i).fsl(:,1:3);
        SErp = Erp.^2;
        MSErp = mean(SErp);
        RMSErp(i,:) = rad2deg(sqrt(MSErp));
        % Translation pqs
        Etp = datatracking.kinemat(i).exp(:,4:6)*10 - datatracking.kinemat(i).fsl(:,4:6)*10;
        SEtp = Etp.^2;
        MSEtp = mean(SEtp);
        RMSEtp(i,:) = sqrt(MSEtp);
        % Rotation rqs
        Er = datatracking.kinemat(i).exp(:,7:nPos) - datatracking.kinemat(i).fsl(:,7:nPos);
        SEr = Er.^2;
        MSEr = mean(SEr);
        RMSEr(i,:) = rad2deg(sqrt(MSEr));
        % grf x
        Egx = datatracking.grf(i).exp(:,[1,4])./(current_osim_bw(i)*9.81) - datatracking.grf(i).fsl(:,[1,4])./(current_osim_bw(i)*9.81);
        SEgx = Egx.^2;
        MSEgx = mean(SEgx);
        RMSEgx(i,:) = sqrt(MSEgx);
        % grf y
        Egy = datatracking.grf(i).exp(:,[2,5])./(current_osim_bw(i)*9.81) - datatracking.grf(i).fsl(:,[2,5])./(current_osim_bw(i)*9.81);
        SEgy = Egy.^2;
        MSEgy = mean(SEgy);
        RMSEgy(i,:) = sqrt(MSEgy);
        % grf z
        Egz = datatracking.grf(i).exp(:,[3,6])./(current_osim_bw(i)*9.81) - datatracking.grf(i).fsl(:,[3,6])./(current_osim_bw(i)*9.81);
        SEgz = Egz.^2;
        MSEgz = mean(SEgz);
        RMSEgz(i,:) = sqrt(MSEgz);
        % powah
        Ep = datatracking.power(i).exp./(current_osim_bw(i)) - datatracking.power(i).fsl./(current_osim_bw(i));
        SEp = Ep.^2;
        MSEp = mean(SEp);
        RMSEp(i,:) = sqrt(MSEp);
    end; clear i
    RMSErpMean  = mean(RMSErp,2);   RMSErpStd   = mean(std(RMSErp));
    RMSEtpMean  = mean(RMSEtp,2);   RMSEtpStd     = mean(std(RMSEtp));
    RMSErMean   = mean(RMSEr,2);    RMSErStd    = mean(std(RMSEr));
    RMSEgxMean  = mean(RMSEgx,2);   RMSEgxStd   = mean(std(RMSEgx));
    RMSEgyMean  = mean(RMSEgy,2);   RMSEgyStd   = mean(std(RMSEgy));
    RMSEgzMean  = mean(RMSEgz,2);   RMSEgzStd   = mean(std(RMSEgz));
    RMSEpMean   = mean(RMSEp(:,1:end-6),2);    RMSEpStd    = mean(std(RMSEp(:,1:end-6)));
    clear Erp Etp Er Egx Egy Egz Ep...
        SErp SEtp SEr SEgx SEgy SEgz SEp ...
        MSErp MSEtp MSEr MSEgx MSEgy MSEgz MSEp ...
        RMSErp RMSEtp RMSEr RMSEgx RMSEgy RMSEgz RMSEp
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT KINEMATICS
try
    clear meanvalexp meanvalfsl meanvalfmc meanvalfmc
    for i = 1:num_subjects
        meanvalexp(:,:,i) = datatracking.kinemat(i).exp;
        meanvalfsl(:,:,i) = datatracking.kinemat(i).fsl;
        meanvalfmc(:,:,i) = datatracking.kinemat(i).fmc;
        meanvalLines(:,:,i) = datatracking.timingICandPeak(i).exp;
    end
    kinMeanExp = mean(meanvalexp,3);
    kinMeanFsl  = mean(meanvalfsl,3);
    kinMeanFmc  = mean(meanvalfmc,3);
    kinStdExp = sqrt(var(meanvalexp,0,3));
    kinStdFsl = sqrt(var(meanvalfsl,0,3));
    % ... plot mean trajectories of kinematics
    plotLabels  = labelsKin(1:21);
    plotExp     = kinMeanExp(:,1:21);
    plotVars    = kinStdExp(:,1:21);
    plotKins    = kinMeanFsl(:,1:21);
    plotKins2   = kinMeanFmc(:,1:21);
    idxplot     = endsWith(plotLabels,'_l');
    
    
    plotLabels(idxplot)  = [];
    plotKins(:,idxplot)    = [];
    plotKins2(:,idxplot)    = [];
    plotExp(:,idxplot)    = [];
    plotVars(:,idxplot)    = [];
    plotLabels(4:6)  = [];
    plotKins(:,4:6)    = [];
    plotKins2(:,4:6)    = [];
    plotExp(:,4:6)    = [];
    plotVars(:,4:6)    = [];
    
    
    plotLabels = strrep(plotLabels,'_', ' ');
    plotKins(:,startsWith(plotLabels,'subtalar')) = [];
    plotKins2(:,startsWith(plotLabels,'subtalar')) = [];
    plotExp(:,startsWith(plotLabels,'subtalar')) = [];
    plotVars(:,startsWith(plotLabels,'subtalar')) = [];
    plotLabels(startsWith(plotLabels,'subtalar')) = [];
    for i = 1:length(plotLabels)
        if endsWith(plotLabels{i}, 'r')
            plotLabels{i} = plotLabels{i}(1:end-1);
        end
    end
    close all;
    x = (0:100/1998:100.1)';
    for i = 1:size(plotKins,2)+1
        
        if i == size(plotKins,2)+1
            hL = legend({'Variance RRA','Average RO'});
            newPosition = [0.75 0.2 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        subplot(3,4,i)
        y   = rad2deg(plotKins(:,i));
        yexp= rad2deg(plotExp(:,i));
        dy  = rad2deg(plotVars(:,i));
        y2  = rad2deg(plotKins2(:,i));
        hold on;
        fill([x;flipud(x)],[yexp-dy;flipud(yexp+dy)],[.9 .9 .9],'linestyle','none','facealpha', .75);
        line(x,y, 'LineWidth', 1); 
%         plot(x,y2, 'LineWidth', 1);
        
        e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
        
        xlim([x(1) x(end)])
        title(plotLabels(i));
        if i == 1 || i == 5 || i == 9
            ylabel('Degrees [\circ]')
        end
        if any(i-(8:11) == 0)
            xlabel('Gait cycle [%]')
%             text(2,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        end
        
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
    end
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT GRF
try
    clear meanvalexp meanvalfsl meanvalfmc
    for i = 1:num_subjects
        meanvalexp(:,:,i) = datatracking.grf(i).exp./(current_osim_bw(i)*9.81);
        meanvalfsl(:,:,i) = datatracking.grf(i).fsl./(current_osim_bw(i)*9.81);
        meanvalfmc(:,:,i) = datatracking.grf(i).fmc./(current_osim_bw(i)*9.81);
    end
    grfMeanExp  = mean(meanvalexp,3);
    grfMeanFsl  = mean(meanvalfsl,3);
    grfMeanFmc  = mean(meanvalfmc,3);
    grfStdExp   = sqrt(var(meanvalexp,0,3));
    grfStdFsl   = sqrt(var(meanvalfsl,0,3));
    % .. and plot
    close all;
    x = (0:100/1998:100.1)';
    plotLabels = {'fore-aft', 'vertical', 'mediolateral'};
    for i = 1:4
        
        if i == 4
            hL = legend({'Variance RRA', ...
                'Average RO (ipsilateral)', ...
                'Average RO (contralateral)'});
            newPosition = [0.75 0.75 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        subplot(2,4,i); hold on;
        y   = grfMeanExp(:,i);    y2   = grfMeanExp(:,i+3);
        
        dy  = grfStdExp(:,i);    dy2  = grfStdExp(:,i+3);
        
        dyd = grfMeanFsl(:,i);    dyd2 = grfMeanFsl(:,i+3);
        
        dydd = grfMeanFmc(:,i);    dydd2 = grfMeanFmc(:,i+3);
        
        fill([x;flipud(x)],[y-dy;flipud(y+dy)],[.9 .9 .9],'linestyle','none', 'facealpha', .75);
        fill([x;flipud(x)],[y2-dy2;flipud(y2+dy2)],[.9 .9 .9],'linestyle','none', 'HandleVisibility', 'off', 'facealpha', .75);
        
        line(x,dyd,'LineWidth', 1);
        
        plot(x,dyd2,'LineWidth', 1);
        xlim([x(1) x(end)])

%         plot(x,dydd,'LineWidth', 1);
%         
%         plot(x,dydd2,'LineWidth', 1);
        
%         e1 = get(gca, 'ylim');
%         
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         if i == 3
%             e1(2) = .15;
%         end
%         e2 = e1([2,1]);
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         text(2,e1(1)+.05*range(e1),'As', 'FontSize', 8);
%         
        title(plotLabels(i));
        if i == 1
            ylabel('Force [%BW]')
        end
        xlabel('Gait cycle [%]')
        
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
    end
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT POWER
try
    clear meanvalexp meanvalfsl meanvalfmc powerMaxExp
    for i = 1:num_subjects
        meanvalexp(:,:,i) = datatracking.power(i).exp./(current_osim_bw(i));
        meanvalfsl(:,:,i) = datatracking.power(i).fsl./(current_osim_bw(i));
        meanvalfmc(:,:,i) = datatracking.power(i).fmc./(current_osim_bw(i));
    end
    idxTrunk = 20;
    idxLines = mean(meanvalLines,3);
    idxAnk                  = find(startsWith(labels_power, 'ankle_angle'), 1);
    powerMeanExp    = mean(meanvalexp,3);
    powerMeanFsl    = mean(meanvalfsl,3);
    powerMeanFmc    = mean(meanvalfmc,3);
    powerStdExp     = sqrt(var(meanvalexp,0,3));
    powerStdFsl     = sqrt(var(meanvalfsl,0,3));

    % .. and plot
    close all;
    % x = time';
    x = (0:100/1999:100)';
    x = (0:100/999:100)';

    plotLabels = {'Average total mechanical power'};
    idxHip = find(startsWith(labels_power, 'hip'));
    idxKnee= find(startsWith(labels_power, 'knee'));
    idxAnkle= find(startsWith(labels_power, 'ankle'));
    idxSub = find(startsWith(labels_power, 'subtalar'));
    idxTrunk=find(startsWith(labels_power, 'lumbar'));
    idxArmPow = find(startsWith(labels_power, 'arm'),1);
    idxPlv  = find(startsWith(labels_power, 'FX'),1);
    for i = 1:2
        
        if i == 2
            hL = legend({'RRA (residuals incl.)', ...
                'RO'});
            newPosition = [0.5 0.75 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        subplot(2,2,i)
        y   = sum(powerMeanExp,2);
        dy  = sum(powerStdExp,2);
        dyd = sum(powerMeanFsl,2);
        ddyd = sum(powerStdFsl,2);
        powfmc = sum(powerMeanFmc,2);
        hold on;
        plot(x,y,'LineWidth', .5); hold on;
        plot(x,dyd,'LineWidth', 1);
%         plot(x,powfmc,'LineWidth', 1);
        %     plot([0 0], ylim, '-r')
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
        %     e1 = get(gca, 'ylim');
        %     e2 = e1([2,1]);
        %     line([length(idxAdd),length(idxAdd)],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
        %     h=fill([0,length(idxAdd),length(idxAdd), 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
        %     h.FaceAlpha=0.1;
        %     text(5,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        
        
        set(gcf, 'Position', [-1.95384615384615 159.030769230769 1420.80000000000 554.584615384615]);
        title(plotLabels(i));
        if i == 1
            title(plotLabels(i));
            
            ylabel('Power [W/kg]')
        end
        xlabel('Gait cycle [%]')
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         l2 = find(y(ceil(idxLines(2)):end) <= .1,1)+ceil(idxLines(2));
%         h=fill([idxLines(1)-1,63,63, idxLines(1)-1],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         h2=fill([13,28,28, 13],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h2.FaceAlpha=0.1;
%         h3=fill([65,85,85, 65],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h3.FaceAlpha=0.1;
%         text(idxLines(1)+3,e1(1)+1,'PO');
%         text(14.5,e1(1)+1,'Rebound');
%         text(67,e1(1)+1,'CL. Rebound');
%         
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         text(4,e1(1)+1,'As');
        
    end
    
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT JOINT POWER DIST
try
    % Calc positive work distributions total
    clear meanvalexp meanvalfsl powerMaxExp wrks
    for i = 1:num_subjects
        %      try
        %         nt = datatracking.JworkTot(i).time;
        %         nt(2);
        %     catch
        %         nt = linspace(datatracking.time(i).time(1),datatracking.time(i).time(end),150);
        %         datatracking.JworkTot(i).exp = pchip(datatracking.time(i).time, datatracking.JworkTot(i).exp',nt)';
        %         datatracking.JworkTot(i).fsl = pchip(datatracking.time(i).time, datatracking.JworkTot(i).fsl',nt)';
        %         datatracking.JworkTot(i).fmc = pchip(datatracking.time(i).time, datatracking.JworkTot(i).fmc',nt)';
        %         datatracking.JworkTot(i).time = nt;
        %     end
        meanvalexp(:,:,i) = datatracking.sideJworkTot(i).exp./(current_osim_bw(i));
        meanvalfsl(:,:,i) = datatracking.sideJworkTot(i).fsl./(current_osim_bw(i));
    end
    workMeanExp    = mean(meanvalexp(:,:,:),3);
    workMeanFsl    = mean(meanvalfsl(:,:,:),3);
    workStdExp     = sqrt(var(meanvalexp(:,:,:),0,3));
    workStdFsl     = sqrt(var(meanvalfsl(:,:,:),0,3));
    curLab         = labels_power(1:end);
    idxHip = find(startsWith(curLab, 'hip'));
    idxKnee= find(startsWith(curLab, 'knee'));
    idxAnkle= find(startsWith(curLab, 'ankle'));
    idxSub = find(startsWith(curLab, 'subtalar'));
    idxTrunk=find(startsWith(curLab, 'lumbar'));
    idxArmPow = find(startsWith(curLab, 'arm'),1);
    pielabel =  {'Hip', 'Knee', 'Ankle', 'Subtalar', 'LS', 'UE', 'Residual'};
    plotLabels = {'RRA', 'RO'};
    close all;
    for i = 1:3
        
        if i == 3
            
            for j = 1:length(wrks)
                lab{j} = [pielabel{j}, ' ',num2str(wrks(1,j)*100), '% | ', num2str(wrks(2,j)*100), '%']
                if j == length(wrks)
                    lab{j} = [pielabel{j}, ' ',num2str(wrks(1,j)*100), '% | ', num2str(0), '%']
                end
            end
            hL = legend(lab);
            newPosition = [0.475 0.8 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        ax = subplot(1,2,i);
        if i == 1
            curworkarr = workMeanExp;
        elseif i == 2
            curworkarr = workMeanFsl;
        end
        break
        wrk = [sum(curworkarr(idxHip)), sum(curworkarr(idxKnee)), sum(curworkarr(idxAnkle)), ...
            sum(curworkarr(idxSub)), sum(curworkarr(idxTrunk)), sum(curworkarr(idxArmPow:end-6)), ...
            sum(curworkarr(end-5:end))];
        
        wrk = wrk/max(wrk) - .001
        
        if i == 2
            wrk(end) = .0000001;
        end
        numberOfSegments = length(wrk);
        
        hPieComponentHandles = pie(ax, ones(1,numberOfSegments));
        rgbmatrix = [1+(wrk(:) < 0).*wrk(:), 1-(wrk(:) > 0).*wrk(:), 1-abs(wrk(:))];
        
        for k = 1 : numberOfSegments
            % Create a color for this sector of the pie
            pieColorMap = rgbmatrix(k,:);  % Color for this segment.
            % Apply the colors we just generated to the pie chart.
            set(hPieComponentHandles(k*2-1), 'FaceColor', pieColorMap);
            %       set(hPieComponentHandles(k*2), 'String', num2str(wrk(k)), 'FontSize', 24 );
        end
        P = pie(wrk/sum(wrk));
        wrks(i,:) =wrk/sum(wrk);
        title(plotLabels(i));
        set(gcf, 'position', [72.6307692307692 198.169230769231 1145.35384615385 422.400000000000]);
    end
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end

%% Trunk Sway Induction

% PLOT KINEMATICS
try
    clear meanvalexp meanvalfsl meanvalfmc
    for i = 1:num_subjects
        
        meanvalexp(:,:,i) = datatracking.kinemat(i).exp;
        meanvalfsl(:,:,i) = datatracking.kinemat(i).fsl;
        meanvalfmc(:,:,i) = datatracking.kinemat(i).fmc;
        meanvalLines(:,:,i) = datatracking.timingICandPeak(i).exp;
    end
    kinMeanExp = mean(meanvalexp,3);
    kinMeanFsl  = mean(meanvalfsl,3);
    kinMeanFmc  = mean(meanvalfmc,3);
    kinStdExp = sqrt(var(meanvalexp,0,3));
    kinStdFsl = sqrt(var(meanvalfsl,0,3));
    % ... plot mean trajectories of kinematics
    plotLabels  = labelsKin(1:21);
    plotExp     = kinMeanExp(:,1:21);
    plotVars    = kinStdExp(:,1:21);
    plotKins    = kinMeanFsl(:,1:21);
    plotKins2   = kinMeanFmc(:,1:21);
    idxplot     = endsWith(plotLabels,'_l');
    
    
    plotLabels(idxplot)  = [];
    plotKins(:,idxplot)    = [];
    plotKins2(:,idxplot)    = [];
    plotExp(:,idxplot)    = [];
    plotVars(:,idxplot)    = [];
    plotLabels(4:6)  = [];
    plotKins(:,4:6)    = [];
    plotKins2(:,4:6)    = [];
    plotExp(:,4:6)    = [];
    plotVars(:,4:6)    = [];
    
    
    plotLabels = strrep(plotLabels,'_', ' ');
    plotKins(:,startsWith(plotLabels,'subtalar')) = [];
    plotKins2(:,startsWith(plotLabels,'subtalar')) = [];
    plotExp(:,startsWith(plotLabels,'subtalar')) = [];
    plotVars(:,startsWith(plotLabels,'subtalar')) = [];
    plotLabels(startsWith(plotLabels,'subtalar')) = [];
    for i = 1:length(plotLabels)
        if endsWith(plotLabels{i}, 'r')
            plotLabels{i} = plotLabels{i}(1:end-1);
        end
    end
    close all;
    x = (0:100/1999:100)';
    for i = 1:size(plotKins,2)+1
        
        if i == size(plotKins,2)+1
            hL = legend({'Average RO','Average TSI'});
            newPosition = [0.75 0.2 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        subplot(3,4,i)
        y   = rad2deg(plotKins(:,i));
        yexp= rad2deg(plotExp(:,i));
        dy  = rad2deg(plotVars(:,i));
        y2  = rad2deg(plotKins2(:,i));
        hold on;
        %         fill([x;flipud(x)],[yexp-dy;flipud(yexp+dy)],[.9 .9 .9],'linestyle','none','facealpha', .75);
        plot(x,y, 'LineWidth', 1);
        plot(x,y2, 'LineWidth', 1);
        
        e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
        
        xlim([x(1) x(end)])
        title(plotLabels(i));
        if i == 1 || i == 5 || i == 9
            ylabel('Degrees [\circ]')
        end
        if any(i-(8:11) == 0)
            xlabel('Gait cycle [%]')
%             text(2,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        end
        
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
    end
    
    
    idxTrunk = 20;
    % Max trunk amps
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = rad2deg((max(datatracking.kinemat(i).exp(:,idxTrunk)) - min(datatracking.kinemat(i).exp(:,idxTrunk)))/2);
        cvF(:,i) = rad2deg((max(datatracking.kinemat(i).fsl(:,idxTrunk)) - min(datatracking.kinemat(i).fsl(:,idxTrunk)))/2);
        cvP(:,i) = rad2deg((max(datatracking.kinemat(i).fmc(:,idxTrunk)) - min(datatracking.kinemat(i).fmc(:,idxTrunk)))/2);
        
        [~,idxte] = min(datatracking.kinemat(i).exp(:,idxTrunk));
        [~,idxtf] = min(datatracking.kinemat(i).fsl(:,idxTrunk));
        [~,idxtfm] = min(datatracking.kinemat(i).fmc(:,idxTrunk));
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunkAmp.p = p;
    anvDat.trunkAmp.means = means;
    % trunk timing
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = mean(datatracking.timingtrunk(i).exp);
        cvF(:,i) = mean(datatracking.timingtrunk(i).fsl);
        cvP(:,i) = mean(datatracking.timingtrunk(i).fmc);
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunktim.p = p;
    anvDat.trunktim.means = means;
    % trunk timing contact
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = mean(datatracking.timingtrunkC(i).exp);
        cvF(:,i) = mean(datatracking.timingtrunkC(i).fsl);
        cvP(:,i) = mean(datatracking.timingtrunkC(i).fmc);
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunktimC.p = p;
    anvDat.trunktimC.means = means;
    
    
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT GRF
try
    clear meanvalexp meanvalfsl meanvalfmc
    for i = 1:num_subjects
        meanvalexp(:,:,i) = datatracking.grf(i).exp./(current_osim_bw(i)*9.81);
        meanvalfsl(:,:,i) = datatracking.grf(i).fsl./(current_osim_bw(i)*9.81);
        meanvalfmc(:,:,i) = datatracking.grf(i).fmc./(current_osim_bw(i)*9.81);

    end
    grfMeanExp  = mean(meanvalexp,3);
    grfMeanFsl  = mean(meanvalfsl,3);
    grfMeanFmc  = mean(meanvalfmc,3);
    grfStdExp   = sqrt(var(meanvalexp,0,3));
    grfStdFsl   = sqrt(var(meanvalfsl,0,3));
    % .. and plot
    close all;
    x = (0:100/1999:100)';
    plotLabels = {'vertical'};
    for i = 1:2
        
%         if i == 2
%             hL = legend({'Average RO (ipsilateral)', ...
%                 'Average TSI (ipsilateral)'});
%             newPosition = [0.75 0.75 0.1 0.1];
%             newUnits = 'normalized';
%             set(hL,'Position', newPosition,'Units', newUnits);
%             continue
%         end
        
        if i == 2
            hL = legend({'Average RO (ipsilateral)', ...
                'Average TSI (ipsilateral)'});
            newPosition = [0.5 0.75 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        subplot(2,2,i); hold on;
        y   = grfMeanExp(:,2);    y2   = grfMeanExp(:,2);
        
        dy  = grfStdExp(:,2);    dy2  = grfStdExp(:,2);
        
        dyd = grfMeanFsl(:,2);    dyd2 = grfMeanFsl(:,2);
        
        dydd = grfMeanFmc(:,2);    dydd2 = grfMeanFmc(:,2);
        
%         fill([x;flipud(x)],[y-dy;flipud(y+dy)],[.9 .9 .9],'linestyle','none', 'facealpha', .75);
%         fill([x;flipud(x)],[y2-dy2;flipud(y2+dy2)],[.9 .9 .9],'linestyle','none', 'HandleVisibility', 'off', 'facealpha', .75);
        
        plot(x,dyd,'LineWidth', 1);
        
%         plot(x,dyd2,'LineWidth', 1);
        
        plot(x,dydd,'LineWidth', 1);
        
%         plot(x,dydd2,'LineWidth', 1);
        
        e1 = get(gca, 'ylim');
        
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         if i == 3
%             e1(2) = .15;
%         end
%         e2 = e1([2,1]);
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         text(2,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        
        title(plotLabels(i));
        if i == 1
            ylabel('Force [%BW]')
        end
        xlabel('Gait cycle [%]')
        
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
    end
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end
% PLOT POWER
try
    clear meanvalexp meanvalfsl meanvalfmc powerMaxExp
    for i = 1:num_subjects
        meanvalexp(:,:,i) = datatracking.power(i).exp./(current_osim_bw(i));
        meanvalfsl(:,:,i) = datatracking.power(i).fsl./(current_osim_bw(i));
        meanvalfmc(:,:,i) = datatracking.power(i).fmc./(current_osim_bw(i));
    end
    idxTrunk = 20;
    idxLines = mean(meanvalLines,3)/10;
    idxAnk = find(startsWith(labels_power, 'ankle_angle'), 1);
    powerMeanExp = mean(meanvalexp,3);
    powerMeanFsl = mean(meanvalfsl,3);
    powerMeanFmc = mean(meanvalfmc,3);
  

    % .. and plot
    close all;
    % x = time';
    x = (0:100/1999:100)';
    plotLabels = {'Average total mechanical power'};
    idxHip = find(startsWith(labels_power, 'hip'));
    idxKnee= find(startsWith(labels_power, 'knee'));
    idxAnkle= find(startsWith(labels_power, 'ankle'));
    idxSub = find(startsWith(labels_power, 'subtalar'));
    idxTrunk=find(startsWith(labels_power, 'lumbar'));
    idxArmPow = find(startsWith(labels_power, 'arm'),1);
    idxPlv  = find(startsWith(labels_power, 'FX'),1);
    for i = 1:2
        
        if i == 2
            hL = legend({'RO', ...
                'TSI'});
            newPosition = [0.5 0.75 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        subplot(2,2,i)
        y   = sum(powerMeanExp,2);
%         dy  = sum(powerStdExp,2);
        dyd = sum(powerMeanFsl,2);
%         ddyd = sum(powerStdFsl,2);
        powfmc = sum(powerMeanFmc,2);
        hold on;
%         plot(x,y,'LineWidth', .5); hold on;
        plot(x,dyd,'LineWidth', 1);
        plot(x,powfmc,'LineWidth', 1);
        %     plot([0 0], ylim, '-r')
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
        %     e1 = get(gca, 'ylim');
        %     e2 = e1([2,1]);
        %     line([length(idxAdd),length(idxAdd)],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
        %     h=fill([0,length(idxAdd),length(idxAdd), 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
        %     h.FaceAlpha=0.1;
        %     text(5,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        
        
        set(gcf, 'Position', [-1.95384615384615 159.030769230769 1420.80000000000 554.584615384615]);
        title(plotLabels(i));
        if i == 1
            title(plotLabels(i));
            
            ylabel('Power [W/kg]')
        end
        xlabel('Gait cycle [%]')
        
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         l2 = find(y(ceil(idxLines(2)):end) <= .1,1)+ceil(idxLines(2));
%         h=fill([45,63,63, 45],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         h2=fill([13,28,28, 13],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h2.FaceAlpha=0.1;
%         h3=fill([65,85,85, 65],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h3.FaceAlpha=0.1;
%         text(48,e1(1)+1,'PO');
%         text(14.5,e1(1)+1,'Rebound');
%         text(67,e1(1)+1,'CL. Rebound');
%         
        
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         text(4,e1(1)+1,'As');
        
    end
    
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end


% PLOT DIFFERENT JOINTS
try
idxLines = mean(meanvalLines,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calc mean trajectories of power as a percentage of body mass
clear meanvalexp meanvalfsl meanvalfmc
for i = 1:num_subjects
    meanvalexp(:,:,i) = datatracking.power(i).exp./(current_osim_bw(i));
    meanvalfsl(:,:,i) = datatracking.power(i).fsl./(current_osim_bw(i));
    meanvalfmc(:,:,i) = datatracking.power(i).fmc./(current_osim_bw(i));
end
powerMeanExp    = mean(meanvalexp,3);
powerMeanFsl    = mean(meanvalfsl,3);
powerMeanFmc    = mean(meanvalfmc,3);
powerStdExp     = sqrt(var(meanvalexp,0,3));
powerStdFsl     = sqrt(var(meanvalfsl,0,3));
% .. and plot
close all;
    x = (0:100/1999:100)';
plotLabels = {'Total mechanical power during gait cycle'}; 
for i = 1:4
    
    if i == 5
        hL = legend({'Average DC', ...
            'Average trunk-sway'});
        newPosition = [0.8 0.9 0.1 0.1];
        newUnits = 'normalized';
        set(hL,'Position', newPosition,'Units', newUnits);
        continue
    end
    
    if i == 1
        c = [2:4];
        tit = 'Hip';
    elseif i == 2
        c = [5];
        tit = 'Knee';
    elseif i == 3
        c = [6];
        tit = 'Ankle';
    elseif i == 4
        c = [14:16];
        tit = 'Lumbar';
    end
    subplot(4,1,i)
    y   = sum(powerMeanFsl(:,c),2);
    dyd = sum(powerMeanFmc(:,c),2);
    plot(x,y,'LineWidth', .5); hold on;
    plot(x,dyd,'LineWidth', .5);
    title(tit);
    if i == 4 
        xlabel('Gait cycle [%]')
    end
    ylabel('Power [W/kg]')
    ax = gca;
    ax.XGrid = 'off';
    ax.YGrid = 'on';
    e1 = get(gca, 'ylim');
%     e2 = e1([2,1]);
%     l2 = find(y(ceil(idxLines(1)):end) <= .1,1)+ceil(idxLines(1));
%     h=fill([50,62,62, 50],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%     h.FaceAlpha=0.1;
%     h2=fill([13,30.3,30.3, 13],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%     h2.FaceAlpha=0.1;
%     h3=fill([65,85,85, 65],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%     h3.FaceAlpha=0.1;
%     if i == 4
%     
%         text(idxLines(1)+3,e1(1)+.1,'PO');
%         text(14.5,e1(1)+.1,'Rebound');
%         text(67,e1(1)+.1,'CL. Rebound');
% 
%     end
    set(gcf, 'Position', [662.6615   40.8769  610.7077  677.9077]);

%     e1 = get(gca, 'ylim');
%     e2 = e1([2,1]);
%     line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%     h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%     h.FaceAlpha=0.1;
%     if i == 4
%     text(4,e1(1)+.1*range(e1),'As'); 
%     end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end

% GRF MAGNITUDE EACH PHASE TABLE
anmat2 = zeros(2,3);
% grf TOTAL
clear powerMaxExp grMaxFsl grMaxFmc
for i = 1:num_subjects
    grMaxFsl(:,i) = datatracking.grftot(i).fsl./(current_osim_bw(i)*9.81);
    grMaxFmc(:,i) = datatracking.grftot(i).fmc./(current_osim_bw(i)*9.81);
end
[p, ~, stats] = anova1([grMaxFsl',grMaxFmc']);
[~, means] = multcompare(stats);
anmat2(1,:) = [means(:,1)',p];
% grf PO
clear powerMaxExp grMaxFsl grMaxFmc
for i = 1:num_subjects
    grMaxFsl(:,i) = datatracking.grfPO(i).fsl./(current_osim_bw(i)*9.81);
    grMaxFmc(:,i) = datatracking.grfPO(i).fmc./(current_osim_bw(i)*9.81);
end
[p, ~, stats] = anova1([grMaxFsl',grMaxFmc']);
[~, means] = multcompare(stats);
anmat2(2,:) = [means(:,1)',p];


% JOINT WORK ANOVAS EACH PHASEE (BIG TABLE)
try
anmat = zeros(15,3);
% WORK TOTAL
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = datatracking.sideJworkTot(i).exp(2)./(current_osim_bw(i));
    powerMaxFsl(:,i) = datatracking.sideJworkTot(i).fsl(2)./(current_osim_bw(i));
    powerMaxFmc(:,i) = datatracking.sideJworkTot(i).fmc(2)./(current_osim_bw(i));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(1,:) = [means(:,1)',p];
% anvDat.JworkTotH.p = p;
% anvDat.JworkTotH.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = datatracking.sideJworkTot(i).exp(14)./(current_osim_bw(i));
    powerMaxFsl(:,i) = datatracking.sideJworkTot(i).fsl(14)./(current_osim_bw(i));
    powerMaxFmc(:,i) = datatracking.sideJworkTot(i).fmc(14)./(current_osim_bw(i));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(5,:) = [means(:,1)',p];

% anvDat.JworkTotL.p = p;
% anvDat.JworkTotL.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = datatracking.sideJworkTot(i).exp(5)./(current_osim_bw(i));
    powerMaxFsl(:,i) = datatracking.sideJworkTot(i).fsl(5)./(current_osim_bw(i));
    powerMaxFmc(:,i) = datatracking.sideJworkTot(i).fmc(5)./(current_osim_bw(i));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(2,:) = [means(:,1)',p];

% anvDat.JworkTotK.p = p;
% anvDat.JworkTotK.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = datatracking.sideJworkTot(i).exp(6)./(current_osim_bw(i));
    powerMaxFsl(:,i) = datatracking.sideJworkTot(i).fsl(6)./(current_osim_bw(i));
    powerMaxFmc(:,i) = datatracking.sideJworkTot(i).fmc(6)./(current_osim_bw(i));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(3,:) = [means(:,1)',p];

% anvDat.JworkTotA.p = p;
% anvDat.JworkTotA.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = datatracking.sideJworkTot(i).exp(7)./(current_osim_bw(i));
    powerMaxFsl(:,i) = datatracking.sideJworkTot(i).fsl(7)./(current_osim_bw(i));
    powerMaxFmc(:,i) = datatracking.sideJworkTot(i).fmc(7)./(current_osim_bw(i));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(4,:) = [means(:,1)',p];




% WORK PO
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkPO(i).exp(2)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkPO(i).fsl(2)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkPO(i).fmc(2)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(6,:) = [means(:,1)',p];
% anvDat.JworkPOH.p = p;
% anvDat.JworkPOH.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkPO(i).exp(14)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkPO(i).fsl(14)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkPO(i).fmc(14)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(10,:) = [means(:,1)',p];
% anvDat.JworkPOL.p = p;
% anvDat.JworkPOL.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkPO(i).exp(5)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkPO(i).fsl(5)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkPO(i).fmc(5)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(7,:) = [means(:,1)',p];
% anvDat.JworkPOK.p = p;
% anvDat.JworkPOK.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkPO(i).exp(6)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkPO(i).fsl(6)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkPO(i).fmc(6)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(8,:) = [means(:,1)',p];
% anvDat.JworkPOA.p = p;
% anvDat.JworkPOA.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkPO(i).exp(7)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkPO(i).fsl(7)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkPO(i).fmc(7)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(9,:) = [means(:,1)',p];
% anvDat.JworkPOA.p = p;
% anvDat.JworkPOA.means = means;





% WORK Rb
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkRb(i).exp(2)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkRb(i).fsl(2)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkRb(i).fmc(2)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(11,:) = [means(:,1)',p];

% anvDat.JworkRbH.p = p;
% anvDat.JworkRbH.means = means;
% WORK Rb
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkRb(i).exp(14)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkRb(i).fsl(14)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkRb(i).fmc(14)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(15,:) = [means(:,1)',p];

% anvDat.JworkRbL.p = p;
% anvDat.JworkRbL.means = means;
% WORK Rb
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkRb(i).exp(5)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkRb(i).fsl(5)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkRb(i).fmc(5)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(12,:) = [means(:,1)',p];

% anvDat.JworkRbK.p = p;
% anvDat.JworkRbK.means = means;
% WORK Rb
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkRb(i).exp(6)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkRb(i).fsl(6)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkRb(i).fmc(6)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(13,:) = [means(:,1)',p];

% anvDat.JworkRbA.p = p;
% anvDat.JworkRbA.means = means;
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.sideJworkRb(i).exp(7)./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.sideJworkRb(i).fsl(7)./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.sideJworkRb(i).fmc(7)./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anmat(14,:) = [means(:,1)',p];



catch 
end

% TOTAL WORK ANOVAS EACH PHASE
try

% Mean ankle work push-off PO
clear cvE cvF cvP
for i = 1:num_subjects
    cvE(:,i) = mean(datatracking.workPO(i).exp./(current_osim_bw(i)));
    cvF(:,i) = mean(datatracking.workPO(i).fsl./(current_osim_bw(i)));
    cvP(:,i) = mean(datatracking.workPO(i).fmc./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([cvE',cvF',cvP']);
[~, means] = multcompare(stats);
anvDat.work.p = p;
anvDat.work.means = means;


% WORK TOTAL
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.workTot(i).exp./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.workTot(i).fsl./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.workTot(i).fmc./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anvDat.workTot.p = p;
anvDat.workTot.means = means;


% WORK PO
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.workPO(i).exp./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.workPO(i).fsl./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.workPO(i).fmc./(current_osim_bw(i)));
end
[p, atab, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[hr, means] = multcompare(stats);
anvDat.workPO.p = p;
anvDat.workPO.means = means;

% WORK Rb
clear powerMaxExp powerMaxFsl powerMaxFmc
for i = 1:num_subjects
    powerMaxExp(:,i) = max(datatracking.workRb(i).exp./(current_osim_bw(i)));
    powerMaxFsl(:,i) = max(datatracking.workRb(i).fsl./(current_osim_bw(i)));
    powerMaxFmc(:,i) = max(datatracking.workRb(i).fmc./(current_osim_bw(i)));
end
[p, ~, stats] = anova1([powerMaxFsl',powerMaxFmc']);
[~, means] = multcompare(stats);
anvDat.workRb.p = p;
anvDat.workRb.means = means;



catch
end

% BAR CHART
try
% Calc positive work in the bar chart
clear workfsl workfmc workPOfsl workPOfmc workPrefsl workPrefmc workPostfsl ...
    workPostfmc workRatPreFsl workRatPostFsl workRatPreFmc workRatPostFmc
for i = 1:num_subjects
    workfsl(:,:,i) = datatracking.workTot(i).fsl./(current_osim_bw(i));
    workfmc(:,:,i) = datatracking.workTot(i).fmc./(current_osim_bw(i));
    workPOfsl(:,:,i) = datatracking.workPO(i).fsl./(current_osim_bw(i));
    workPOfmc(:,:,i) = datatracking.workPO(i).fmc./(current_osim_bw(i));
    workRbfsl(:,:,i) = datatracking.workRb(i).fsl./(current_osim_bw(i));
    workRbfmc(:,:,i) = datatracking.workRb(i).fmc./(current_osim_bw(i));
end
workMeanFsl    = mean(workfsl,3);
workMeanFmc    = mean(workfmc,3);
workStdFsl     = sqrt(var(workfsl,0,3));
workStdFmc     = sqrt(var(workfmc,0,3));
workPOMeanFsl    = mean(workPOfsl,3);
workPOMeanFmc    = mean(workPOfmc,3);
workRbMeanFsl    = mean(workRbfsl,3);
workRbMeanFmc    = mean(workRbfmc,3);
workPOStdFsl     = sqrt(var(workPOfsl,0,3));
workPOStdFmc     = sqrt(var(workPOfmc,0,3));
workRbStdFsl     = sqrt(var(workRbfsl,0,3));
workRbStdFmc     = sqrt(var(workRbfmc,0,3));
wrk = [workMeanFsl workMeanFmc; ...
    workPOMeanFsl workPOMeanFmc; ...
    workRbMeanFsl workRbMeanFmc];
err = [workStdFsl workStdFmc; ...
    workPOStdFsl workPOStdFmc; ...
    workRbStdFsl workRbStdFmc];
for i = 1
    if i == 1
        num = 3;
        c = 1:num;
        figH = figure;
        axes1 = axes;
        title('Positive work during various conditions'); ylabel('Work [J/kg]');
        hold on
        hBar=bar(double(c),wrk(c,:));         % plot against the underlying value
        errH1 = errorbar(c-0.15,wrk(c,1),err(c,1),'.','Color','black');
        errH2 = errorbar(c+0.15,wrk(c,2),err(c,2),'.','Color','black');
        errH1.LineWidth = 1.5;
        errH2.LineWidth = 1.5;
        %%Set x-ticks
        set(axes1,'Xlim',[0.5 3.5]);
        set(axes1,'XTick',[1 1.5 2 2.5 3 3.5],'XTickLabel',...
            {'Total',' ','PO',' ','Rebound', ''});
        signif = [3];
        
        if ~isempty(signif)
            for kl = 1:length(signif)
            xsig = [hBar(2).XData(signif(kl)) - hBar(2).XOffset, hBar(2).XData(signif(kl)) + hBar(2).XOffset];
            ctr =xsig; ctr = ctr + .25 * [-1 1];
            hold on
            plot(ctr, [1 1]*max(wrk(signif(kl),:))*1.1+max(err(signif(kl),:)), '-k', 'LineWidth',2)
            plot(mean(ctr), 0.05+[1 1]*max(wrk(signif(kl),:))*1.1+max(err(signif(kl),:)), '*k')
            end
        end
        set(gcf, 'position', [551.8923   45.3077  384.7385  587.0769]);
        legend({'RO';'TSI'},'Location','northeast')
    elseif i == 2
        figure
        subplot(1,2,1)
        pie([workRatPremeanFsl workRatPostmeanFsl])
        title('RO')
        subplot(1,2,2)
        pie([workRatPremeanFmc workRatPostmeanFmc])
        title('TSI')
        colormap([0 0.4470 0.7410;      %// red
            0.8500    0.3250    0.0980])
        set(gcf, 'position', [322.2308  178.2308  565.6615  433.4769]);
        hL = legend({'Pre-emptive work [%PO]', 'Push-off into swing work [%PO]'});
        newPosition = [0.48 0.80 0.1 0.1];
        newUnits = 'normalized';
        set(hL,'Position', newPosition,'Units', newUnits);
    end
end
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end    
% PLOT JOINT POWER DIST
try
% Calc positive work distributions total 
clear meanvalexp meanvalfsl powerMaxExp
for i = 1:num_subjects

    meanvalexp(:,:,i) = datatracking.sideJworkTot(i).fmc./(current_osim_bw(i));
    meanvalfsl(:,:,i) = datatracking.sideJworkTot(i).fsl./(current_osim_bw(i));
end
workMeanExp    = mean(meanvalexp(:,:,:),3);
workMeanFsl    = mean(meanvalfsl(:,:,:),3);
workStdExp     = sqrt(var(meanvalexp(:,:,:),0,3));
workStdFsl     = sqrt(var(meanvalfsl(:,:,:),0,3));
curLab         = labels_power(1:end);
idxHip = find(startsWith(curLab, 'hip'));
idxKnee= find(startsWith(curLab, 'knee'));
idxAnkle= find(startsWith(curLab, 'ankle'));
idxSub = find(startsWith(curLab, 'subtalar'));
idxTrunk=find(startsWith(curLab, 'lumbar'));
idxArmPow = find(startsWith(curLab, 'arm'),1);
pielabel =  {'Hip', 'Knee', 'Ankle', 'Subtalar', 'LS', 'UE', 'Residual'};
plotLabels = {'RO', 'TSI'}; 
close all;
rgbmatrix = [1 0.0455396763583960 0.0455396763583960;1 0.782019395369972 0.782019395369972;1 0.504800563998472 0.504800563998472;1 0.935621682607210 0.935621682607210;1 0.956274055493994 0.956274055493994;1 0.991285106250779 0.991285106250779;1 0.999997953636712 0.999997953636712];
for i = 1:3
    
    if i == 3
        
        for j = 1:length(wrks)
            lab{j} = [pielabel{j}, ' ',num2str(wrks(1,j)*100), '% | ', num2str(wrks(2,j)*100), '%']
            if j == length(wrks)
                lab{j} = [pielabel{j}, ' ',num2str(0), '% | ', num2str(0), '%']
            end
        end
        hL = legend(lab);
        newPosition = [0.475 0.8 0.1 0.1];
        newUnits = 'normalized';
        set(hL,'Position', newPosition,'Units', newUnits);
        continue
    end
    
    ax = subplot(1,2,i);
    if i == 2
        curworkarr = workMeanExp;
    elseif i == 1
        curworkarr = workMeanFsl;
    end
    wrk = [sum(curworkarr(idxHip)), sum(curworkarr(idxKnee)), sum(curworkarr(idxAnkle)), ...
        sum(curworkarr(idxSub)), sum(curworkarr(idxTrunk)), sum(curworkarr(idxArmPow:end-6)), ...
        sum(curworkarr(end-5:end))];
    
%     for j = 1:length(wrk)
%         nl = [pielabel
    
    
%     if i == 2
    wrk(end) = .0000001;
%     end
    numberOfSegments = length(wrk);

    hPieComponentHandles = pie(ax, ones(1,numberOfSegments));
%     rgbmatrix = [1+(wrk(:) < 0).*wrk(:), 1-(wrk(:) > 0).*wrk(:), 1-abs(wrk(:))];

    for k = 1 : numberOfSegments
      % Create a color for this sector of the pie
      pieColorMap = rgbmatrix(k,:);  % Color for this segment.
      % Apply the colors we just generated to the pie chart.
      set(hPieComponentHandles(k*2-1), 'FaceColor', pieColorMap);
%       set(hPieComponentHandles(k*2), 'String', num2str(wrk(k)), 'FontSize', 24 );
    end
    if i == 1
        P = pie(wrk/sum(wrk)); 
    else
        P = pie(wrk/sum(wrk), [0 0 0 0 0 0 0]); 
        P(10).String = [P(10).String, '**'];
        P(2).String = [P(2).String, '**'];
    end
%     P = pie(wrk/sum(wrk)); 
    wrks(i,:) =wrk/sum(wrk);
    title(plotLabels(i));
    set(gcf, 'position', [72.6307692307692 198.169230769231 1145.35384615385 422.400000000000]);
end
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end


%% ROM

% PLOT POWER
try
    clear meanvalfmc meanvalfp
    for i = 1:num_subjects
        meanvalfmc(:,:,i) = datatracking.power(i).fmc./(current_osim_bw(i));
        meanvalfp(:,:,i) = datatracking.power(i).fp./(current_osim_bw(i));
    end
    idxTrunk = 20;
    idxLines = mean(meanvalLines,3)/10;
    idxAnk = find(startsWith(labels_power, 'ankle_angle'), 1);
    powerMeanFmc = mean(meanvalfmc,3);
    powerMeanFp = mean(meanvalfp,3);
  

    % .. and plot
    close all;
    % x = time';
    x = (0.1:100/1000:100)';
    plotLabels = {'Total mechanical power'};
    idxHip = find(startsWith(labels_power, 'hip'));
    idxKnee= find(startsWith(labels_power, 'knee'));
    idxAnkle= find(startsWith(labels_power, 'ankle'));
    idxSub = find(startsWith(labels_power, 'subtalar'));
    idxTrunk=find(startsWith(labels_power, 'lumbar'));
    idxArmPow = find(startsWith(labels_power, 'arm'),1);
    idxPlv  = find(startsWith(labels_power, 'FX'),1);
    for i = 1:2
        
        if i == 2
            hL = legend({'RO', ...
                'TSI'});
            newPosition = [0.5 0.75 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        
        subplot(2,2,i)
        y   = sum(powerMeanExp,2);
%         dy  = sum(powerStdExp,2);
        dyd = sum(powerMeanFmc,2);
%         ddyd = sum(powerStdFsl,2);
        powfmc = sum(powerMeanFp,2);
        hold on;
%         plot(x,y,'LineWidth', .5); hold on;
        plot(x,dyd,'LineWidth', 1);
        plot(x,powfmc,'LineWidth', 1);
        %     plot([0 0], ylim, '-r')
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
        %     e1 = get(gca, 'ylim');
        %     e2 = e1([2,1]);
        %     line([length(idxAdd),length(idxAdd)],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
        %     h=fill([0,length(idxAdd),length(idxAdd), 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
        %     h.FaceAlpha=0.1;
        %     text(5,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        
        
        set(gcf, 'Position', [-1.95384615384615 159.030769230769 1420.80000000000 554.584615384615]);
        title(plotLabels(i));
        if i == 1
            title(plotLabels(i));
            
            ylabel('Power [W/kg]')
        end
        xlabel('Gait cycle [%]')
        
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         l2 = find(y(ceil(idxLines(2)):end) <= .1,1)+ceil(idxLines(2));
%         h=fill([45,63,63, 45],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         h2=fill([13,28,28, 13],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h2.FaceAlpha=0.1;
%         h3=fill([65,85,85, 65],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h3.FaceAlpha=0.1;
%         text(48,e1(1)+1,'PO');
%         text(14.5,e1(1)+1,'Rebound');
%         text(67,e1(1)+1,'CL. Rebound');
%         
        
%         e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
%         text(4,e1(1)+1,'As');
        
    end
    
catch
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end


% PLOT KINEMATICS
try
    clear meanvalexp meanvalfsl meanvalfmc meanvalfp
    for i = 1:num_subjects
        
        meanvalfmc(:,:,i) = datatracking.kinemat(i).fmc;
        meanvalfp(:,:,i) = datatracking.kinemat(i).fp;
        meanvalLines(:,:,i) = datatracking.timingICandPeak(i).exp;
    end
    kinMeanFmc  = mean(meanvalfmc,3);
    kinMeanFp  = mean(meanvalfp,3);
    % ... plot mean trajectories of kinematics
    plotLabels  = labelsKin(1:21);
    plotKins    = kinMeanFmc(:,1:21);
    plotKins2   = kinMeanFp(:,1:21);
    idxplot     = endsWith(plotLabels,'_l');
    
    
    plotLabels(idxplot)  = [];
    plotKins(:,idxplot)    = [];
    plotKins2(:,idxplot)    = [];
    plotLabels(4:6)  = [];
    plotKins(:,4:6)    = [];
    plotKins2(:,4:6)    = [];
   
    plotLabels = strrep(plotLabels,'_', ' ');
    plotKins(:,startsWith(plotLabels,'subtalar')) = [];
    plotKins2(:,startsWith(plotLabels,'subtalar')) = [];
    plotExp(:,startsWith(plotLabels,'subtalar')) = [];
    plotVars(:,startsWith(plotLabels,'subtalar')) = [];
    plotLabels(startsWith(plotLabels,'subtalar')) = [];
    for i = 1:length(plotLabels)
        if endsWith(plotLabels{i}, 'r')
            plotLabels{i} = plotLabels{i}(1:end-1);
        end
    end
    close all;
    x = (0.1:100/1000:100)';
    for i = 1:size(plotKins,2)+1
        
        if i == size(plotKins,2)+1
            hL = legend({'Average RO','Average TSI'});
            newPosition = [0.75 0.2 0.1 0.1];
            newUnits = 'normalized';
            set(hL,'Position', newPosition,'Units', newUnits);
            continue
        end
        subplot(3,4,i)
        y   = rad2deg(plotKins(:,i));
        yexp= rad2deg(plotExp(:,i));
        dy  = rad2deg(plotVars(:,i));
        y2  = rad2deg(plotKins2(:,i));
        hold on;
        %         fill([x;flipud(x)],[yexp-dy;flipud(yexp+dy)],[.9 .9 .9],'linestyle','none','facealpha', .75);
        plot(x,y, 'LineWidth', 1);
        plot(x,y2, 'LineWidth', 1);
        
        e1 = get(gca, 'ylim');
%         e2 = e1([2,1]);
%         line([assumptionCutOff,assumptionCutOff],get(gca, 'ylim'),'LineStyle', '--', 'Color','k','LineWidth', 1)
%         h=fill([0,assumptionCutOff,assumptionCutOff, 0],[e1(1),e2(2),e2(1),e1(2)],'black','linestyle','none');
%         h.FaceAlpha=0.1;
        
        xlim([x(1) x(end)])
        title(plotLabels(i));
        if i == 1 || i == 5 || i == 9
            ylabel('Degrees [\circ]')
        end
        if any(i-(8:11) == 0)
            xlabel('Gait cycle [%]')
%             text(2,e1(1)+.05*range(e1),'As', 'FontSize', 8);
        end
        
        ax = gca;
        ax.XGrid = 'off';
        ax.YGrid = 'on';
        
    end
    
    
    idxTrunk = 20;
    % Max trunk amps
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = rad2deg((max(datatracking.kinemat(i).exp(:,idxTrunk)) - min(datatracking.kinemat(i).exp(:,idxTrunk)))/2);
        cvF(:,i) = rad2deg((max(datatracking.kinemat(i).fsl(:,idxTrunk)) - min(datatracking.kinemat(i).fsl(:,idxTrunk)))/2);
        cvP(:,i) = rad2deg((max(datatracking.kinemat(i).fmc(:,idxTrunk)) - min(datatracking.kinemat(i).fmc(:,idxTrunk)))/2);
        
        [~,idxte] = min(datatracking.kinemat(i).exp(:,idxTrunk));
        [~,idxtf] = min(datatracking.kinemat(i).fsl(:,idxTrunk));
        [~,idxtfm] = min(datatracking.kinemat(i).fmc(:,idxTrunk));
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunkAmp.p = p;
    anvDat.trunkAmp.means = means;
    % trunk timing
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = mean(datatracking.timingtrunk(i).exp);
        cvF(:,i) = mean(datatracking.timingtrunk(i).fsl);
        cvP(:,i) = mean(datatracking.timingtrunk(i).fmc);
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunktim.p = p;
    anvDat.trunktim.means = means;
    % trunk timing contact
    clear cvE cvF cvP
    for i = 1:num_subjects
        cvE(:,i) = mean(datatracking.timingtrunkC(i).exp);
        cvF(:,i) = mean(datatracking.timingtrunkC(i).fsl);
        cvP(:,i) = mean(datatracking.timingtrunkC(i).fmc);
    end
    [p, ~, stats] = anova1([cvE',cvF',cvP']);
    [~, means] = multcompare(stats);
    anvDat.trunktimC.p = p;
    anvDat.trunktimC.means = means;
    
    
catch ME
    if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
        msg = ['Dimension mismatch occurred: First argument has ', ...
            num2str(size(A,2)),' columns while second has ', ...
            num2str(size(B,2)),' columns.'];
        causeException = MException('MATLAB:myCode:dimensions',msg);
        ME = addCause(ME,causeException);
    end
    rethrow(ME)
end













function totalMass = getMassOfModel(osimModel)
totalMass = 0;
allBodies = osimModel.getBodySet();
for i=0:allBodies.getSize()-1
    curBody = allBodies.get(i);
    totalMass = totalMass + curBody.getMass();
end
end

function ar=checkArima(y,pp,qq)
    % pp is the maximum for p
    % qq is the maximum for q
    LOGL = zeros(pp+1,qq+1); %Initialize
    PQ = zeros(pp+1,qq+1);
    for p = 1:pp+1
        for q = 1:qq+1
            mod = arima(p-1,0,q-1)
            [fit,~,logL] = estimate(mod,y,'print',false);
            LOGL(p,q) = logL;
            PQ(p,q) = p+q;
        end
    end
    
    LOGL = reshape(LOGL,(pp+1)*(qq+1),1);
    PQ = reshape(PQ,(pp+1)*(qq+1),1);
    [~,bic] = aicbic(LOGL,PQ+1,100);
    ar=reshape(bic,pp+1,qq+1);
    amod = arima(3,0,1);
end


