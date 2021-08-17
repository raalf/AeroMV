% Run Pelican Datase
clear,clc
% fcnRUN_DIR to be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()
 
filename = 'AscTec_Pelican';
load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
flight_num = 23;

Euler = flights{1,flight_num}.Euler;
VEL = flights{1,flight_num}.Vel;
% This equation is from the masters thesis of Nguyen Khoi Tran at McGill
% titled: Modeling and Control of a Quadrotor in a Wind Field
RPM = (25+flights{1,flight_num}.Motors*175/200)*43;
% RPM calculated from experiements by Ben
RPM = 34.676*flights{1,flight_num}.Motors+1333.1;
POS = flights{1,flight_num}.Pos;
BODY_RATES = flights{1,flight_num}.pqr;

j = 0;
begin = 1000;
fin = 1100;
% fin = 1005;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

% Calculate body rates by using the Euler angles
BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);

RPM_Multiplier = 4767./[4456 4326 4196 4104]; %from flight 23
% RPM_Mulitplier = 4870./[4456 4326 4196 4104]; %from flight 23
RPM_Hover = [4456 4326 4196 4104]; 

max_iter = 50;  % Iterations per timestep

% Creating OVERWRITE function
OVERWRITE.GEOM.VEH.vecCG = [-1.5 1.5 152.0153-118.7]*0.001;
% OVERWRITE.GEOM.VEH.vecCG = [0 0 152.0153-118.7]*0.001;
% OVERWRITE = [];

try
    RPM_Multiplier = fcnRPMMULTIPLIER(filename,5000,RPM_Hover',OVERWRITE);
catch
    RPM_Multiplier = 4767./[4456 4326 4196 4104];
end

%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);
tic
% parfor i = 1:(fin-begin+1)
for i = begin:int:fin

%     STATE.RPM = 1.135*[mean(RPM((i-avg_count+1):i,1)) mean(RPM((i-avg_count+1):i,2)) mean(RPM((i-avg_count+1):i,3)) mean(RPM((i-avg_count+1):i,4)) ]; % RPM
% 	STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM.
    STATE = [];
    OUTP = [];
    STATE.FREQ = datafeq/int;
    STATE.accuracy = 3;
    k = 0;

%     cond = true;
%     count_iter_num = 0;
    for count_iter_num = 1:max_iter
    d = i+begin+count_iter_num;
    
    STATE.RPM = RPM_Multiplier'.*[RPM(d+1,1) RPM(d+1,2) RPM(d+1,3) RPM(d+1,4)]; % RPM
    
    STATE.EULER = Euler(d,:);
    if count_iter_num == 1
        STATE.VEL = VEL(d-2:d,:); % m/s
        STATE.POS = POS(d-2:d,:);
        STATE.EULER = Euler(d-2:d,:);
        STATE.BODY_RATES = BODY_RATES(d-2:d,:);
    elseif count_iter_num == 2
        STATE.VEL = [VEL(d-2:d-1,:);OUTP(count_iter_num-1).VEL_NEW']; % m/s
        STATE.POS = [POS(d-2:d-1,:);OUTP(count_iter_num-1).POS_NEW'];
        STATE.EULER = [Euler(d-2:d-1,:);OUTP(count_iter_num-1).EULER_NEW'];
        STATE.BODY_RATES = [BODY_RATES(d-2:d-1,:);OUTP(count_iter_num-1).OMEGA_NEW_B'];
    elseif count_iter_num == 3
        STATE.VEL = [VEL(d-2,:);[OUTP(count_iter_num-2:count_iter_num-1).VEL_NEW]']; % m/s
        STATE.POS = [POS(d-2,:);[OUTP(count_iter_num-2:count_iter_num-1).POS_NEW]'];
        STATE.EULER = [Euler(d-2,:);[OUTP(count_iter_num-2:count_iter_num-1).EULER_NEW]'];
    else
        STATE.VEL = [OUTP(count_iter_num-3:count_iter_num-1).VEL_NEW]';
        STATE.POS = [OUTP(count_iter_num-3:count_iter_num-1).POS_NEW]';
        STATE.EULER = [OUTP(count_iter_num-3:count_iter_num-1).EULER_NEW]';
        STATE.BODY_RATES = [OUTP(count_iter_num-3:count_iter_num-1).OMEGA_NEW_B]';
    end
%     tic
    [OUTP_temp, PERF, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1, OVERWRITE);
%     toc
    if count_iter_num == 1
        OUTP = OUTP_temp;
    else
        OUTP(count_iter_num) = OUTP_temp;
    end
    
    if count_iter_num == max_iter
        feildss =["COMP_DRAG_TOTAL","COMP_LIFT_TOTAL","F_r","F_comp","F_B","M_comp","M_r_hub","M_r_total","M_B","PREC_RATE_ROTOR","PREC_VEH_RATE","xi_ddot","OMEGA_NEW","EULER_NEW","POS_NEW","OMEGA_DOT_B"];
%         OUTP = rmfield(OUTP, feildss);
        OUTP_COMPILED(i-begin+1,:) = rmfield(OUTP, feildss);
    end
    end
    fprintf(strcat(num2str(i),' Complete.' ,'\n'))
end
toc
save('DATA','OUTP_COMPILED')
