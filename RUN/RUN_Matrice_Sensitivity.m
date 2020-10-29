set(0,'DefaultFigureWindowStyle','docked')
%% Center of gravity
% Only cg location in the x and y directions are considered
% The original "Baseline" cg is [7.119 -10.172 47.397]*0.001
clear,clc

cg_x = (-30:2.5:30)*0.001;
cg_y = (-30:2.5:30)*0.001;
idxCGFACT = fullfact([length(cg_x),length(cg_y)]);

% Ixx = 0.17525014*[0.5 0.75 0.9 0.95 1 1.05 1.1 1.25 1.5 2 3 5];
% Iyy = 0.16151033*[0.5 0.75 0.9 0.95 1 1.05 1.1 1.25 1.5 2 3 5];
% Izz = 0.20452748*[0.5 0.75 1 1.25 2];
% idxCGFACT = fullfact([length(Ixx),length(Iyy),length(Izz)]);


% I_multi = 1;
% GEOM.VEH.I =  [I_multi*0.17525014, 0.00411034, -0.00173288;
%                 0.00411034, I_multi*0.16151033, 0.01333274;
%                 -0.00173288, 0.01333274, 0.20452748];

% Run Matrice 210 RTK dataset
% fcnRUN_DIR to    be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()

% Input filename
filename = 'DJI_Matrice_210_RTK';
% Dataset
load('DATA/Matrice_210_RTK_Dataset/July3_2020_Flight_1.mat','Flight_Data','density','flight_segments')

flight_num = 1;

Euler = Flight_Data(1,flight_num).Euler_Angles;
VEL = Flight_Data(1,flight_num).Velocity;
RPM = [Flight_Data(1,flight_num).RPM1,Flight_Data(1,flight_num).RPM2,Flight_Data(1,flight_num).RPM3,Flight_Data(1,flight_num).RPM4];
Height = Flight_Data(1,flight_num).Height_Above_Takeoff;
POS = [zeros(length(Height),2), Height];
BODY_RATES = Flight_Data(1,flight_num).Body_Rates;
BODY_RATES = diff(Euler)*50;

j = 0;
begin = 1000;
fin = 4000;
maxVal = 4200;
datafeq = 50;
int = 1;
STATE.FREQ = datafeq/int;


Vel_criteria = 0.09;
Body_Rates_criteria = 0.12;
% Body_Rates_criteria = 0.19;
% Body_Rates_criteria = 0.38;
cond = true;
cond_missed = []; % Condition that causes reset
count_iter_num = 0;


len = (fin-begin)/int + 1;
iter_num  = NaN(len,1);  % Iteration number before it had to reset
idxBROKENCOND = NaN(len,6);
idxVEL_COND = NaN(len,3);
idxBODY_COND = NaN(len,3);
avg_count = 5; % How many points to average for moving average of input variables
% RPM_Multiplier = [0.8462,0.8508,0.9785,0.8942];
% RPM_Multiplier = 0.8879;
% Creating OVERWRITE function
% OVERWRITE.AIR.density = density;
% OVERWRITE = [];

%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);
parfor q = 1:size(idxCGFACT,1)
    tic
    OVERWRITE = [];
    OVERWRITE.GEOM.VEH.vecCG = [cg_x(idxCGFACT(q,1)) cg_y(idxCGFACT(q,2))  (47.397)*0.001];
    OVERWRITE.AIR.density = density;
% OVERWRITE.GEOM.VEH.I =  [Ixx(idxCGFACT(q,1)), 0.00411034, -0.00173288;
%                 0.00411034, Iyy(idxCGFACT(q,2)), 0.01333274;
%                 -0.00173288, 0.01333274, Izz(idxCGFACT(q,3))];
    RPM_Hover = [4845.58578567463,4819.17379233759,4190.41476957246,4585.03387007218];
    try
        [RPM_Multiplier] = fcnRPMMULTIPLIER(filename,4000,RPM_Hover',OVERWRITE);
        flagIMAG(q) = 0;
    catch
        RPM_Multiplier = [1 1 1 1]';
        flagIMAG(q) = 1;
    end
%     if ~isreal(RPM_Multiplier)
%         RPM_Multiplier = [1 1 1 1]';
%         flagIMAG(q) = 1;
%     else
%         flagIMAG(q) = 0;
%     end
    
    j = 0;
    k = 0;
    cond = false;
    cond_missed = []; % Condition that causes reset
    count_iter_num = 0;
    iter_num  = NaN(len,1);  % Iteration number before it had to reset
    idxVEL_COND = NaN(len,3);
    idxBODY_COND = NaN(len,3);
    OUTP = struct([]);
    
    
    for i = begin:int:fin
        j = j+1;
        STATE = [];
        STATE.accuracy = 3;
        STATE.FREQ = datafeq/int;
        k = 0;
        cond = true;
        count_iter_num = 0;
        while cond
            d = i+k;
            
            STATE.RPM = RPM_Multiplier'.*[RPM(d,1) RPM(d,2) RPM(d,3) RPM(d,4)]; % RPM
            
            STATE.EULER = Euler(d,:);
            if k == 0
            STATE.VEL = VEL(i-2:i,:); % m/s
            STATE.POS = POS(i-2:i,:);
            STATE.EULER = Euler(i-2:i,:);
            STATE.BODY_RATES = BODY_RATES(i-2:i,:);
            k = 1;
        elseif k == 1 && cond
            k = 2;
            STATE.VEL = [VEL(i-2:i-1,:);OUTP(k-1).OUTP.VEL_NEW']; % m/s
            STATE.POS = [POS(i-2:i-1,:);OUTP(k-1).OUTP.POS_NEW'];
            STATE.EULER = [Euler(i-2:i-1,:);OUTP(k-1).OUTP.EULER_NEW'];
            STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(k-1).OUTP.OMEGA_NEW_B'];
            
        elseif k == 2 && cond
            k = 3;
            STATE.VEL = [VEL(i-2,:);[OUTP(k-2).OUTP.VEL_NEW]';[OUTP(k-1).OUTP.VEL_NEW]']; % m/s
            STATE.POS = [POS(i-2,:);[OUTP(k-2).OUTP.POS_NEW]';[OUTP(k-1).OUTP.POS_NEW]'];
            STATE.EULER = [Euler(i-2,:);[OUTP(k-2).OUTP.EULER_NEW]';[OUTP(k-1).OUTP.EULER_NEW]'];
            STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(k-2).OUTP.OMEGA_NEW_B]';[OUTP(k-1).OUTP.OMEGA_NEW_B]'];
            
            else
            k = k + 1;
            STATE.VEL = [[OUTP(k-3).OUTP.VEL_NEW,]';[OUTP(k-2).OUTP.VEL_NEW,]';[OUTP(k-1).OUTP.VEL_NEW,]'];
            STATE.POS = [[OUTP(k-3).OUTP.POS_NEW,]';[OUTP(k-2).OUTP.POS_NEW,]';[OUTP(k-1).OUTP.POS_NEW,]'];
            STATE.EULER = [[OUTP(k-3).OUTP.EULER_NEW,]';[OUTP(k-2).OUTP.EULER_NEW,]';[OUTP(k-1).OUTP.EULER_NEW,]'];
            STATE.BODY_RATES = [[OUTP(k-3).OUTP.OMEGA_NEW_B,]';[OUTP(k-2).OUTP.OMEGA_NEW_B,]';[OUTP(k-1).OUTP.OMEGA_NEW_B,]'];
            
            end
            
            [OUTP(k).OUTP, ~, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1, OVERWRITE);
            
            idxVEL_COND(k,:) = (abs(VEL(d+1,:)'-OUTP(k).OUTP.VEL_NEW))>Vel_criteria;
            idxBODY_COND(k,:) = (abs(BODY_RATES(d+1,:)'-OUTP(k).OUTP.OMEGA_NEW_B))>Body_Rates_criteria;
            if any(idxVEL_COND(k,:)) || any(idxBODY_COND(k,:)) || d >= maxVal
                cond = false;
                iter_num(j) = count_iter_num;
%                 idxBROKENCOND(j,:) = [idxVEL_COND(k,:) idxBODY_COND(k,:)];
                
            else
                cond = true;
                count_iter_num = count_iter_num+1;
            end        
        end
    end
    AVERAGE_ITERATION_CG(q) = nanmean(iter_num);
    fprintf('Done CG case: %d with average iteration: %f\n',q, AVERAGE_ITERATION_CG(q));
    toc
end
save('CG_Data2')

%% CG Data

[X,Y] = meshgrid(1:length(cg_x),1:length(cg_y));

cg_x_mesh = cg_x(X);
cg_y_mesh = cg_y(Y);

CG_AVG_ITR_mesh = zeros(size(X));
for i = 1:length(cg_x)
    for j = 1:length(cg_y)
        idx = idxCGFACT(:,1)==i & idxCGFACT(:,2) ==j;
        idx2 = X(:,:)==i & Y(:,:) ==j;
        CG_AVG_ITR_mesh(idx2) = AVERAGE_ITERATION_CG(idx);
    end
end

figure(1)
clf(1)
hold on
surf(cg_x_mesh*100,cg_y_mesh*100,CG_AVG_ITR_mesh)
ylabel('Y-Location of CG (cm)')
xlabel('X-Location of CG (cm)')
zlabel('Average Successful Iterations')
% [7.119 -10.172 47.397]*0.001
% scatter3(7.119*0.1, -10.172*.1,5.12,'pk','markerfacecolor','k')
axis square
axis tight
grid on
box on
hold off

