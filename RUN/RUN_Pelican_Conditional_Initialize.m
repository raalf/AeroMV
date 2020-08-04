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
fin = 1050;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

% Calculate body rates by using the Euler angles
BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);

RPM_Mulitplier = 4767./[4456 4326 4196 4104]; %from flight 23
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
% 
% 
% for i = avg_count+1:int:length(VEL)
%     tempPOS(i,:) = mean(POS((i-avg_count+1):i,:));
%     tempVEL(i,:) = mean(VEL((i-avg_count+1):i,:));
%     tempEuler(i,:) = mean(Euler((i-avg_count+1):i,:));
%     tempBODY_RATES(i,:) = mean(BODY_RATES((i-avg_count+1):i,:));
% end

% Creating OVERWRITE function
OVERWRITE.GEOM.VEH.vecCG = [-1.5 1.5 152.0153-118.7]*0.001;
% OVERWRITE.GEOM.VEH.vecCG = [0 0 152.0153-118.7]*0.001;
% OVERWRITE = [];
FOLDER_ADDRESS = pwd;
addpath(genpath(FOLDER_ADDRESS))

%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);

for i = begin:int:fin
    j = j+1;
%     STATE.RPM = 1.135*[mean(RPM((i-avg_count+1):i,1)) mean(RPM((i-avg_count+1):i,2)) mean(RPM((i-avg_count+1):i,3)) mean(RPM((i-avg_count+1):i,4)) ]; % RPM
% 	STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
    STATE.accuracy = 3;
    k = 0;
    d = 0;
    cond = true;
    count_iter_num = 0;
    while cond
    d = i+k;
    
    STATE.RPM = RPM_Mulitplier.*[RPM(d,1) RPM(d,2) RPM(d,3) RPM(d,4)]; % RPM
    
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
    
    [OUTP(k), PERF, ~, ~, ~, ~] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1, OVERWRITE);
    
    idxVEL_COND(k,:) = (abs(VEL(d+1,:)'-OUTP(k).VEL_NEW))>Vel_criteria;
    idxBODY_COND(k,:) = (abs(BODY_RATE_From_Euler(d+1,:)'-OUTP(k).OMEGA_NEW_B))>Body_Rates_criteria;
    if any(idxVEL_COND(k,:)) || any(idxBODY_COND(k,:))
        cond = false;
        iter_num(j) = count_iter_num;
        idxBROKENCOND(j,:) = [idxVEL_COND(k,:) idxBODY_COND(k,:)];
        
    else
        cond = true;
        count_iter_num = count_iter_num+1;
    end
    end
    fprintf(strcat(num2str(i),' Complete. Number of successful iterations:',num2str(iter_num(j)) ,'\n'))
end

save('DATA')
%% Plotting
figure(1)
clf(1)
hold on
histogram(iter_num)
text(0.8,0.95,strcat('Avg: ',num2str(nanmean(iter_num))),'Units','normalized')
xlabel('Number of Successful Iterations')
ylabel('Number of Occurrence')
title('Successful iterations before conditions were passed')
grid on
box on
axis tight
hold off

figure(2)
clf(2)
hold on
X = categorical({'Vel X-Dir','Vel Y-Dir','Vel Z-Dir','Roll Rate','Pitch Rate','Yaw Rate'});
% sum_cond_missed = cat(2,sum(idxVEL_COND),sum(idxBODY_COND));
bar(X,sum(idxBROKENCOND))
xlabel('Condition Missed')
ylabel('Number of Occurrence')
title('Number of times each condition was missed')
grid on
box on
hold off

figure(3)
clf(3)
hold on
scatter3(POS(begin+int:int:fin+int,1),POS(begin+int:int:fin+int,2),POS(begin+int:int:fin+int,3),'k')
axis equal
xlabel('X-Position')
ylabel('Y-Position')
zlabel('Z-Position')
grid on
grid minor
box on
hold off


figure(4)
clf(4)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,1),'k-','linewidth',2) 
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,2),'r-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,3),'b-','linewidth',2,'markerfacecolor','b')
ylabel('Inertial Position (m)')
xlabel('Time')
yyaxis right
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,iter_num,':','linewidth',2) 
ylabel('Successful Iterations')
legend('Experiment x-dir','Experiment y-dir','Experiment z-dir','Successful Iterations')
title('Position')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

figure(5)
clf(5)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,1),'k-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,2),'r-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,3),'b-','linewidth',2,'markerfacecolor','b')
ylabel('Angle (rad)')
xlabel('Time')
title('Euler Angles')
yyaxis right
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,iter_num,':','linewidth',2) 
ylabel('Successful Iterations')
legend('Experiment \phi','Experiment \theta','Experiment \psi','Successful Iterations')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off


figure(6)
clf(6)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,1),'k-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,2),'r-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,3),'b-','linewidth',2,'markerfacecolor','b')
ylabel('Velocity (m/s)')
xlabel('Time')
title('Velocity')
yyaxis right
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,iter_num,':','linewidth',2) 
ylabel('Successful Iterations')
legend('Experiment x-dir','Experiment y-dir','Experiment z-dir','Successful Iterations')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

figure(7)
clf(7)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,1),'k-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,2),'r-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,3),'b-','linewidth',2,'markerfacecolor','b')
ylabel('Rate (rad/s)')
xlabel('Time')
title('Body Rates')
yyaxis right
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,iter_num,':','linewidth',2) 
ylabel('Successful Iterations')
legend('Experiment $\dot{\phi}$','Experiment $\dot{\theta}$','Experiment $\dot{\psi}$','Successful Iterations','Interpreter','latex')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off


% Height of bar graph would be average number of iterations
% X-axis are bins with max or average body rate

k = 0;
for d = begin:int:fin
    k = k+1;
    max_bodyrate_10(k,:) = max(abs(BODY_RATES(d:(d+10),1:2)));
    max_bodyrate_iter(k,:) = max(abs(BODY_RATES(d:(d+iter_num(k)),1:2)));
    max_bodyaccel_10(k,:) = max(abs((BODY_RATES((d+1):(d+11),1:2)-BODY_RATES(d:(d+10),1:2))/(1/STATE.FREQ)));
    max_bodyaccel_iter(k,:) = max(abs((BODY_RATES((d+1):(d+iter_num(k)+1),1:2)-BODY_RATES(d:d+iter_num(k),1:2))/(1/STATE.FREQ)));
    mean_bodyrate_10(k,:) = mean(abs(BODY_RATES(d:(d+10),1:2)));
    mean_bodyrate_iter(k,:) = mean(abs(BODY_RATES(d:(d+iter_num(k)),1:2)));
    mean_bodyaccel_10(k,:) = mean(abs((BODY_RATES((d+1):(d+11),1:2)-BODY_RATES(d:(d+10),1:2))/(1/STATE.FREQ)));
    mean_bodyaccel_iter(k,:) = mean(abs((BODY_RATES((d+1):(d+iter_num(k)+1),1:2)-BODY_RATES(d:d+iter_num(k),1:2))/(1/STATE.FREQ)));
end

varoi = mean_bodyaccel_iter;
num_bin = 20;
varoi = varoi(:,1).*idxBROKENCOND(:,4) + varoi(:,2).*idxBROKENCOND(:,5);
[a,b] = discretize(nonzeros(varoi),num_bin);

for q = 1:num_bin+1
    avg_iter(q) = mean(iter_num(a == q));
end

figure(8)
clf(8)
hold on
bar(b,avg_iter,'FaceColor',[0.8500 0.3250 0.0980])
ylabel('Average Successful Iterations')
% xlabel('Mean Body Rate (Over Successful Data Points)')
xlabel('Mean Acceleration Rate (Over Successful Data Points)')
title('Flight 23')
grid on 
grid minor
box on
axis tight
hold off
