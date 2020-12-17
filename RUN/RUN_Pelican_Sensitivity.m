clear,clc
% % Sensitivity analysis of Pelican predictions
% 
% % This script prepares for a sensitivity analysis to see how different
% % inputs impact the predictions.
% 
% 
% % Input variables of consideration include CG_x, CG_y, Ixx, Iyy, Izz, RPM
% % This will be done as a hybrid of One-at-a-time (OAT) sensitivity analysis
% % and factored analysis. CG, I and RPM will be looked at serately but
% % within each of those params, a fully factored analysis will be used.
% 
% % Time per run is 184second. Can do 8 cores. 100 cases: approx 40min
% 
% %% Center of gravity
% % Only cg location in the x and y directions are considered
% % The original "Baseline" cg is [-0.0015, 0.0065, 0.0333]
% cg_x = [-20 -15 -10 -8 -3 -1.5 0 1.5 4 10]*0.001;
% cg_y = [-8 -3 0 3 6.5 8 12 16 22 30 35 40 45 50 60 70]*0.001;
% idxCGFACT = fullfact([length(cg_x),length(cg_y)]);
% 
% %% Moment of inertia
% Ixx = 0.025322564860661*[0.5 0.95 1 1.1 1.25 2 5];
% Iyy = 0.028041568149193*[0.5 0.95 1 1.1 1.25 2 5];
% Izz = 0.029055618259337*[0.5 1 2];
% idxIFACT =  fullfact([length(Ixx),length(Iyy),length(Izz)]);
% 
% %% RPM
% RPM1 = [1.17 1.15 1.135 1.12 1.1];
% RPM2 = [1.17 1.15 1.135 1.12 1.1];
% RPM3 = [1.17 1.15 1.135 1.12 1.1];
% RPM4 = [1.17 1.15 1.135 1.12 1.1];
% idxRPMFACT =  fullfact([length(RPM1),length(RPM2),length(RPM3),length(RPM4)]);
% 
% 
% %% RUN CG
% % One time initialization 
% filename = 'AscTec_Pelican';
% load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
% flight_num = 20;
% 
% Euler = flights{1,flight_num}.Euler;
% VEL = flights{1,flight_num}.Vel;
% RPM = 34.676*flights{1,flight_num}.Motors+1333.1;
% POS = flights{1,flight_num}.Pos;
% BODY_RATES = flights{1,flight_num}.pqr;
% begin = 1000;
% fin = 20000;
% datafeq = 100;
% int = 1;
% BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);
% Vel_criteria = 0.09;
% Body_Rates_criteria = 0.12;
% len = (fin-begin)/int + 1;
% 
% FOLDER_ADDRESS = pwd;
% addpath(genpath(FOLDER_ADDRESS))
% 
% % Initialize the variables used for sensitivity analysis
% AVERAGE_ITERATION_CG = NaN(size(idxCGFACT,1),1);
% ITERATION_CG = NaN(size(idxCGFACT,1),len);
% 
% parfor q = 1:size(idxCGFACT,1)
%     tic
%     OVERWRITE = [];
%     OVERWRITE.GEOM.VEH.vecCG = [cg_x(idxCGFACT(q,1)) cg_y(idxCGFACT(q,2))  (152.0153-118.7)*0.001];
%     j = 0;
%     k = 0;
%     cond = false;
%     cond_missed = []; % Condition that causes reset
%     count_iter_num = 0;
%     iter_num  = NaN(len,1);  % Iteration number before it had to reset
%     idxVEL_COND = NaN(len,3);
%     idxBODY_COND = NaN(len,3);
%     OUTP = struct([]);
%  
%     
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     ITERATION_CG(q,:) = iter_num;
%     AVERAGE_ITERATION_CG(q) = nanmean(iter_num);
%     fprintf('Done CG case: %d with average iteration: %f\n',q, AVERAGE_ITERATION_CG(q));
%     toc
% end
% 
% % saving in case of error or issues
% save('CG2_DATA')
% 
% 
% %% RUN Moment of inertia
% AVERAGE_ITERATION_I = NaN(size(idxIFACT,1),1);
% ITERATION_I = NaN(size(idxIFACT,1),len);
% 
% parfor q = 1:size(idxIFACT,1)
%     tic
%     OVERWRITE = [];
%     I =  [Ixx(idxIFACT(q,1)), 0, 0;
%         0, Iyy(idxIFACT(q,2)), 0;
%         0, 0, Izz(idxIFACT(q,3))];
%     phi = 45; % Angle of rotation between reference frames
%     OVERWRITE.GEOM.VEH.I = I;
%     
%     OVERWRITE.GEOM.VEH.I(1,1) = 0.5*(I(1,1)+I(2,2))+0.5*(I(1,1)-I(2,2))*cosd(2*phi)-I(1,2)*sind(2*phi);
%     OVERWRITE.GEOM.VEH.I(2,2) = 0.5*(I(1,1)+I(2,2))-0.5*(I(1,1)-I(2,2))*cosd(2*phi)+I(1,2)*sind(2*phi);
%     OVERWRITE.GEOM.VEH.I(1,2) = 0.5*(I(1,1)-I(2,2))*sind(2*phi)+I(1,2)*cosd(2*phi);
%     OVERWRITE.GEOM.VEH.I(2,1) = OVERWRITE.GEOM.VEH.I(1,2);
%     
%     
%     j = 0;
%     k = 0;
%     cond = false;
%     cond_missed = []; % Condition that causes reset
%     count_iter_num = 0;
%     iter_num  = NaN(len,1);  % Iteration number before it had to reset
%     idxVEL_COND = NaN(len,3);
%     idxBODY_COND = NaN(len,3);
%     OUTP = struct([]);
%  
%     
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     ITERATION_I(q,:) = iter_num;
%     AVERAGE_ITERATION_I(q) = nanmean(iter_num);
%     fprintf('Done inertia case : %d with average iteration: %f\n',q, AVERAGE_ITERATION_I(q));
%     toc
% end
% 
% save('Inertia_DATA')
% 
% 
% %% RUN RPM
% AVERAGE_ITERATION_RPM = NaN(size(idxRPMFACT,1),1);
% ITERATION_RPM = NaN(size(idxRPMFACT,1),len);
% 
% parfor q = 1:size(idxRPMFACT,1)
%     tic
%     OVERWRITE = [];
%     rpm_mulitplier =  [RPM1(idxRPMFACT(q,1)),RPM2(idxRPMFACT(q,2)),RPM3(idxRPMFACT(q,3)),RPM4(idxRPMFACT(q,4))];
%  
%     j = 0;
%     k = 0;
%     cond = false;
%     cond_missed = []; % Condition that causes reset
%     count_iter_num = 0;
%     iter_num  = NaN(len,1);  % Iteration number before it had to reset
%     idxVEL_COND = NaN(len,3);
%     idxBODY_COND = NaN(len,3);
%     OUTP = struct([]);
%  
%     
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = rpm_mulitplier.*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     ITERATION_RPM(q,:) = iter_num;
%     AVERAGE_ITERATION_RPM(q) = nanmean(iter_num);
%     fprintf('Done RPM case : %d with average iteration: %f\n',q, AVERAGE_ITERATION_RPM(q));
%     toc
% end
% 
% save('RPM_DATA')

clear,clc
% Sensitivity analysis of Pelican predictions

% This script prepares for a sensitivity analysis to see how different
% inputs impact the predictions.


% Input variables of consideration include CG_x, CG_y, Ixx, Iyy, Izz, RPM
% This will be done as a hybrid of One-at-a-time (OAT) sensitivity analysis
% and factored analysis. CG, I and RPM will be looked at serately but
% within each of those params, a fully factored analysis will be used.

% Time per run is 184second. Can do 8 cores. 100 cases: approx 40min

%% Center of gravity
% Only cg location in the x and y directions are considered
% The original "Baseline" cg is [-0.0015, 0.0065, 0.0333]

cg_x = (-15:1.5:15)*0.001;
cg_y = (-15:1.5:15)*0.001;
% cg_x = (-1.5:1.5:1.5)*0.001;
% cg_y = (-1.5:1.5:1.5)*0.001;
idxCGFACT = fullfact([length(cg_x),length(cg_y)]);

%% Moment of inertia
Ixx = 0.025322564860661*[0.5 0.95 1 1.1 1.25 2 5];
Iyy = 0.028041568149193*[0.5 0.95 1 1.1 1.25 2 5];
Izz = 0.029055618259337*[0.5 1 2];
idxIFACT =  fullfact([length(Ixx),length(Iyy),length(Izz)]);

%% RPM
RPM_Mulitplier = 4767./[4456 4326 4196 4104];
% [1.0698    1.1019    1.1361    1.1615];

% [0.9 0.95 1 1.05 1.10]
RPM1 = [0.9 0.95 1 1.05 1.10]*RPM_Mulitplier(1);
RPM2 = [0.9 0.95 1 1.05 1.10]*RPM_Mulitplier(2);
RPM3 = [0.9 0.95 1 1.05 1.10]*RPM_Mulitplier(3);
RPM4 = [0.9 0.95 1 1.05 1.10]*RPM_Mulitplier(4);
idxRPMFACT =  fullfact([length(RPM1),length(RPM2),length(RPM3),length(RPM4)]);


%% RUN CG
% One time initialization 
% fcnRUN_DIR to be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()

filename = 'AscTec_Pelican';
load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
flight_num = 23;

Euler = flights{1,flight_num}.Euler;
VEL = flights{1,flight_num}.Vel;
RPM = 34.676*flights{1,flight_num}.Motors+1333.1;
POS = flights{1,flight_num}.Pos;
BODY_RATES = flights{1,flight_num}.pqr;
begin = 1000;
fin = 20000;
datafeq = 100;
int = 1;
BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);
Vel_criteria = 0.09;
Body_Rates_criteria = 0.12;
len = (fin-begin)/int + 1;

% Initialize the variables used for sensitivity analysis
AVERAGE_ITERATION_CG = NaN(size(idxCGFACT,1),1);
ITERATION_CG = NaN(size(idxCGFACT,1),len);
RPM_Hover = [4456 4326 4196 4104];

parfor q = 1:size(idxCGFACT,1)
    tic

    %% Retrieve Input Vehicle Geometry
    [TABLE, GEOM, AIR] = fcnINPUT(filename);

    OVERWRITE = [];
    OVERWRITE.GEOM.VEH.vecCG = [cg_x(idxCGFACT(q,1)) cg_y(idxCGFACT(q,2))  (152.0153-118.7)*0.001];
    try
        RPM_Multiplier = fcnRPMMULTIPLIER(filename,5000,RPM_Hover',OVERWRITE);
    catch
        RPM_Multiplier = 4767./[4456 4326 4196 4104];
    end
    j = 0;
    k = 0;
    cond = false;
    cond_missed = []; % Condition that causes reset
    count_iter_num = 0;
    iter_num  = NaN(len,1);  % Iteration number before it had to reset
    idxVEL_COND = NaN(len,3);
    idxBODY_COND = NaN(len,3);
    OUTP = struct([]);
 
    
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = RPM_Mulitplier'.*[RPM(i+1,1) RPM(i+1,2) RPM(i+1,3) RPM(i+1,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
% %         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1 , OVERWRITE);
%     
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     
    d_d = 0;
    for i = begin:int:fin

%     STATE.RPM = 1.135*[mean(RPM((i-avg_count+1):i,1)) mean(RPM((i-avg_count+1):i,2)) mean(RPM((i-avg_count+1):i,3)) mean(RPM((i-avg_count+1):i,4)) ]; % RPM
% 	STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM.
    STATE = [];
    OUTP = [];
    STATE.FREQ = datafeq/int;
    STATE.accuracy = 3;
    k = 0;
    d_d = d_d+1;
    cond = true;
    count_iter_num = 0;
    while cond
    d = i+begin+k-1;
    
    STATE.RPM = RPM_Multiplier'.*[RPM(d+1,1) RPM(d+1,2) RPM(d+1,3) RPM(d+1,4)]; % RPM
    
    STATE.EULER = Euler(d,:);
    if k == 0
        STATE.VEL = VEL(d-2:d,:); % m/s
        STATE.POS = POS(d-2:d,:);
        STATE.EULER = Euler(d-2:d,:);
        STATE.BODY_RATES = BODY_RATES(d-2:d,:);
        k = 1;
    elseif k == 1
        STATE.VEL = [VEL(d-2:d-1,:);OUTP(k).VEL_NEW']; % m/s
        STATE.POS = [POS(d-2:d-1,:);OUTP(k).POS_NEW'];
        STATE.EULER = [Euler(d-2:d-1,:);OUTP(k).EULER_NEW'];
        STATE.BODY_RATES = [BODY_RATES(d-2:d-1,:);OUTP(k).OMEGA_NEW_B'];
        k = 2;
    elseif k == 2
        STATE.VEL = [VEL(d-2,:);[OUTP(k-1:k).VEL_NEW]']; % m/s
        STATE.POS = [POS(d-2,:);[OUTP(k-1:k).POS_NEW]'];
        STATE.EULER = [Euler(d-2,:);[OUTP(k-1:k).EULER_NEW]'];
        STATE.BODY_RATES = [BODY_RATES(d-2,:);[OUTP(k-1:k).OMEGA_NEW_B]'];
        k = 3;
    else
        STATE.VEL = [OUTP(k-2:k).VEL_NEW]';
        STATE.POS = [OUTP(k-2:k).POS_NEW]';
        STATE.EULER = [OUTP(k-2:k).EULER_NEW]';
        STATE.BODY_RATES = [OUTP(k-2:k).OMEGA_NEW_B]';
        k = k + 1;
    end
    
    [OUTP_temp, ~, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1 , OVERWRITE);
    
    if k == 1
        OUTP = OUTP_temp;
    else
        OUTP(k) = OUTP_temp;
    end
    idxVEL_COND = (abs(VEL(d+1,:)'-OUTP(k).VEL_NEW))>Vel_criteria;
    idxBODY_COND = (abs(BODY_RATE_From_Euler(d+1,:)'-OUTP(k).OMEGA_NEW_B))>Body_Rates_criteria;
    if any(idxVEL_COND) || any(idxBODY_COND)
        cond = false;
        
        iter_num(d_d) = count_iter_num;
%         idxBROKENCOND(i,:) = [idxVEL_COND' idxBODY_COND'];
        
    else
        cond = true;
        count_iter_num = count_iter_num+1;
    end
    end
%     fprintf(strcat(num2str(i),' Complete. Number of successful iterations:',num2str(iter_num(i)) ,'\n'))
    end
    ITERATION_CG(q,:) = iter_num;
    AVERAGE_ITERATION_CG(q) = nanmean(iter_num);
    fprintf('Done CG case: %d with average iteration: %f\n',q, AVERAGE_ITERATION_CG(q));
    toc
end

% saving in case of error or issues
save('CG23_DATA_Symmetric')


% %% RUN Moment of inertia
% AVERAGE_ITERATION_I = NaN(size(idxIFACT,1),1);
% ITERATION_I = NaN(size(idxIFACT,1),len);
% 
% parfor q = 1:size(idxIFACT,1)
%     tic
%     OVERWRITE = [];
%     I =  [Ixx(idxIFACT(q,1)), 0, 0;
%         0, Iyy(idxIFACT(q,2)), 0;
%         0, 0, Izz(idxIFACT(q,3))];
%     phi = 45; % Angle of rotation between reference frames
%     OVERWRITE.GEOM.VEH.I = I;
%     
%     OVERWRITE.GEOM.VEH.I(1,1) = 0.5*(I(1,1)+I(2,2))+0.5*(I(1,1)-I(2,2))*cosd(2*phi)-I(1,2)*sind(2*phi);
%     OVERWRITE.GEOM.VEH.I(2,2) = 0.5*(I(1,1)+I(2,2))-0.5*(I(1,1)-I(2,2))*cosd(2*phi)+I(1,2)*sind(2*phi);
%     OVERWRITE.GEOM.VEH.I(1,2) = 0.5*(I(1,1)-I(2,2))*sind(2*phi)+I(1,2)*cosd(2*phi);
%     OVERWRITE.GEOM.VEH.I(2,1) = OVERWRITE.GEOM.VEH.I(1,2);
%     
%     
%     j = 0;
%     k = 0;
%     cond = false;
%     cond_missed = []; % Condition that causes reset
%     count_iter_num = 0;
%     iter_num  = NaN(len,1);  % Iteration number before it had to reset
%     idxVEL_COND = NaN(len,3);
%     idxBODY_COND = NaN(len,3);
%     OUTP = struct([]);
%  
%     
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = RPM_Mulitplier.*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     ITERATION_I(q,:) = iter_num;
%     AVERAGE_ITERATION_I(q) = nanmean(iter_num);
%     fprintf('Done inertia case : %d with average iteration: %f\n',q, AVERAGE_ITERATION_I(q));
%     toc
% end
% 
% save('Inertia23_DATA')
% 
% 
% %% RUN RPM
% AVERAGE_ITERATION_RPM = NaN(size(idxRPMFACT,1),1);
% ITERATION_RPM = NaN(size(idxRPMFACT,1),len);
% 
% parfor q = 1:size(idxRPMFACT,1)
%     tic
%     OVERWRITE = [];
%     rpm_mulitplier =  [RPM1(idxRPMFACT(q,1)),RPM2(idxRPMFACT(q,2)),RPM3(idxRPMFACT(q,3)),RPM4(idxRPMFACT(q,4))];
%  
%     j = 0;
%     k = 0;
%     cond = false;
%     cond_missed = []; % Condition that causes reset
%     count_iter_num = 0;
%     iter_num  = NaN(len,1);  % Iteration number before it had to reset
%     idxVEL_COND = NaN(len,3);
%     idxBODY_COND = NaN(len,3);
%     OUTP = struct([]);
%  
%     
%     for i = begin:int:fin
%         j = j+1;
%         STATE = [];
%         STATE.accuracy = 3;
%         STATE.FREQ = datafeq/int;
%         STATE.RPM = rpm_mulitplier.*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
%         STATE.EULER = Euler(i,:);
%         
%         if ~cond
%             STATE.VEL = VEL(i-2:i,:); % m/s
%             STATE.POS = POS(i-2:i,:);
%             STATE.EULER = Euler(i-2:i,:);
%             STATE.BODY_RATES = BODY_RATES(i-2:i,:);
%             k = 1;
%         elseif k == 1 && cond
%             STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).OUTP.VEL_NEW']; % m/s
%             STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).OUTP.POS_NEW'];
%             STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).OUTP.EULER_NEW'];
%             STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OUTP.OMEGA_NEW_B'];
%             k = 2;
%         elseif k == 2 && cond
%             STATE.VEL = [VEL(i-2,:);[OUTP(j-2).OUTP.VEL_NEW]';[OUTP(j-1).OUTP.VEL_NEW]']; % m/s
%             STATE.POS = [POS(i-2,:);[OUTP(j-2).OUTP.POS_NEW]';[OUTP(j-1).OUTP.POS_NEW]'];
%             STATE.EULER = [Euler(i-2,:);[OUTP(j-2).OUTP.EULER_NEW]';[OUTP(j-1).OUTP.EULER_NEW]'];
%             STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2).OUTP.OMEGA_NEW_B]';[OUTP(j-1).OUTP.OMEGA_NEW_B]'];
%             k = 3;
%         else
%             STATE.VEL = [[OUTP(j-3).OUTP.VEL_NEW,]';[OUTP(j-2).OUTP.VEL_NEW,]';[OUTP(j-1).OUTP.VEL_NEW,]'];
%             STATE.POS = [[OUTP(j-3).OUTP.POS_NEW,]';[OUTP(j-2).OUTP.POS_NEW,]';[OUTP(j-1).OUTP.POS_NEW,]'];
%             STATE.EULER = [[OUTP(j-3).OUTP.EULER_NEW,]';[OUTP(j-2).OUTP.EULER_NEW,]';[OUTP(j-1).OUTP.EULER_NEW,]'];
%             STATE.BODY_RATES = [[OUTP(j-3).OUTP.OMEGA_NEW_B,]';[OUTP(j-2).OUTP.OMEGA_NEW_B,]';[OUTP(j-1).OUTP.OMEGA_NEW_B,]'];
%         end
%         
%         [OUTP(j).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(filename, STATE, 1,OVERWRITE);
%         
%         idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).OUTP.VEL_NEW))>Vel_criteria;
%         idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
%         if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
%             cond = false;
%             iter_num(j) = count_iter_num;
%             count_iter_num = 0;
%         else
%             cond = true;
%             count_iter_num = count_iter_num+1;
%         end
%     end
%     ITERATION_RPM(q,:) = iter_num;
%     AVERAGE_ITERATION_RPM(q) = nanmean(iter_num);
%     fprintf('Done RPM case : %d with average iteration: %f\n',q, AVERAGE_ITERATION_RPM(q));
%     toc
% end
% 
% save('RPM23_DATA')
% 
