% Run Pelican Datase
clear,clc
% fcnRUN_DIR to be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()

filename = 'AscTec_Pelican';
load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
% load('DATA/Pelican_Raw_Data/AscTec_Pelican_Flight_Dataset_Original_Motors.mat','flights')

flight_num = 23;

Euler = flights{1,flight_num}.Euler;
% VEL = sqrt(flights{1,flight_num}.Vel(:,1).^2+flights{1,flight_num}.Vel(:,2).^2+flights{1,flight_num}.Vel(:,3).^2);
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
fin = 20000;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

% Calculate body rates by using the Euler angles
BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);

RPM_Multiplier = 4767./[4456 4326 4196 4104]; %from flight 23
% RPM_Mulitplier = 4870./[4456 4326 4196 4104]; %from flight 23
RPM_Hover = [4456 4326 4196 4104];

Vel_criteria = 0.09;
Body_Rates_criteria = 0.12;
% Body_Rates_criteria = 0.19;
% Body_Rates_criteria = 0.38;
cond = true;
cond_missed = []; % Condition that causes reset
count_iter_num = 0;


len = (fin-begin)/int + 1;
iter_num  = NaN(len,1);  % Iteration number before it had to reset

% idxVEL_COND = NaN(len,3);
% idxBODY_COND = NaN(len,3);
avg_count = 5; % How many points to average for moving average of input variables
%
%
% for i = avg_count+1:int:length(VEL)
%     tempPOS(i,:) = mean(POS((i-avg_count+1):i,:));
%     tempVEL(i,:) = mean(VEL((i-avg_count+1):i,:));
%     tempEuler(i,:) = mean(Euler((i-avg_count+1):i,:));
%     tempBODY_RATES(i,:) = mean(BODY_RATES((i-avg_count+1):i,:));
% end

% Creating OVERWRITE function


%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);
KT_ref = 0.0160;
KQ_ref = 0.00202;
KT_sweep = linspace(KT_ref*0.5,KT_ref*1.5,25);
KQ_sweep = linspace(KQ_ref*0.5,KQ_ref*1.5,25);
parfor p = 1:length(KT_sweep)
    OVERWRITE = [];
    OVERWRITE.GEOM.KT = KT_sweep(p);
    DATA_save1 = [];
    for q = 1:length(KQ_sweep)
        iter_num = [];
        idxBROKENCOND = NaN(len,6);
        OVERWRITE.GEOM.KQ = KQ_sweep(q);
        OVERWRITE.GEOM.VEH.vecCG = [-1.5 1.5 152.0153-118.7]*0.001;
        % OVERWRITE.GEOM.VEH.vecCG = [0 0 152.0153-118.7]*0.001;
        % OVERWRITE = [];
        try
            RPM_Multiplier = fcnRPMMULTIPLIER(filename,5000,RPM_Hover',OVERWRITE);
        catch
            RPM_Multiplier = 4767./[4456 4326 4196 4104];
        end
        % parfor i = 1:(fin-begin+1)
        for i = begin:int:fin
            
            %     STATE.RPM = 1.135*[mean(RPM((i-avg_count+1):i,1)) mean(RPM((i-avg_count+1):i,2)) mean(RPM((i-avg_count+1):i,3)) mean(RPM((i-avg_count+1):i,4)) ]; % RPM
            % 	STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM.
            STATE = [];
            OUTP = [];
            STATE.FREQ = datafeq/int;
            STATE.accuracy = 3;
            k = 0;
            
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
                %     tic
                [OUTP_temp, PERF, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE,4, OVERWRITE);
                %     toc
                if k == 1
                    OUTP = OUTP_temp;
                else
                    OUTP(k) = OUTP_temp;
                end
                idxVEL_COND = (abs(VEL(d+1,:)'-OUTP(k).VEL_NEW))>Vel_criteria;
                idxBODY_COND = (abs(BODY_RATE_From_Euler(d+1,:)'-OUTP(k).OMEGA_NEW_B))>Body_Rates_criteria;
                if any(idxVEL_COND) || any(idxBODY_COND)
                    cond = false;
                    
                    iter_num(i-begin+1) = count_iter_num;
                    idxBROKENCOND(i,:) = [idxVEL_COND' idxBODY_COND'];
                    
                else
                    cond = true;
                    count_iter_num = count_iter_num+1;
                end
            end
%             fprintf(strcat(num2str(i),' Complete. Number of successful iterations:',num2str(iter_num(i-begin+1)) ,'\n'))
        end
        fprintf('Done\n')
        DATA_save1(q).iter_num = iter_num;
    end
    DATA_save(p,:) = DATA_save1;
end

save('DATA')
