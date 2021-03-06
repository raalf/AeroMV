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
begin = 5000;
fin = 5050;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

% Calculate body rates by using the Euler angles
BODY_RATE_From_Euler = (Euler(2:end,:)-Euler(1:end-1,:))/(1/datafeq);

RPM_Mulitplier = 4767./[4456 4326 4196 4104]; %from flight 23
% RPM_Mulitplier = 4870./[4456 4326 4196 4104]; %from flight 23
Vel_criteria = 0.09;
Body_Rates_criteria = 0.12;
Vel_criteria = 1;
Body_Rates_criteria = 1;
cond = false;
cond_missed = []; % Condition that causes reset
count_iter_num = 0;


len = (fin-begin)/int + 1;
iter_num  = NaN(len,1);  % Iteration number before it had to reset
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
% OVERWRITE = [];


%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);

for i = begin:int:fin
    j = j+1;
    STATE.accuracy = 3;
%     STATE.RPM = 1.135*[mean(RPM((i-avg_count+1):i,1)) mean(RPM((i-avg_count+1):i,2)) mean(RPM((i-avg_count+1):i,3)) mean(RPM((i-avg_count+1):i,4)) ]; % RPM
% 	STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
    STATE.RPM = RPM_Mulitplier.*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
    
    STATE.EULER = Euler(i,:);
    
%     if ~cond
%         STATE.VEL = VEL(i,:); % m/s
%         STATE.POS = POS(i,:);!g
%         STATE.EULER = Euler(i,:);
% %         STATE.BODY_RATES = BODY_RATES(i,:);
%         STATE.BODY_RATES = BODY_RATE_From_Euler(i,:);
%     else
%         STATE.VEL = OUTP(j-1).VEL_NEW';
%         STATE.POS = OUTP(j-1).POS_NEW';
%         STATE.EULER = OUTP(j-1).EULER_NEW';
%         STATE.BODY_RATES = OUTP(j-1).OMEGA_NEW_B';
%     end
 
    if ~cond
        STATE.VEL = VEL(i-2:i,:); % m/s
        STATE.POS = POS(i-2:i,:);
        STATE.EULER = Euler(i-2:i,:);
        STATE.BODY_RATES = BODY_RATES(i-2:i,:);
        k = 1;
    elseif k == 1 && cond
        STATE.VEL = [VEL(i-2:i-1,:);OUTP(j-1).VEL_NEW']; % m/s
        STATE.POS = [POS(i-2:i-1,:);OUTP(j-1).POS_NEW'];
        STATE.EULER = [Euler(i-2:i-1,:);OUTP(j-1).EULER_NEW'];
        STATE.BODY_RATES = [BODY_RATES(i-2:i-1,:);OUTP(j-1).OMEGA_NEW_B'];
        k = 2;
    elseif k == 2 && cond
        STATE.VEL = [VEL(i-2,:);[OUTP(j-2:j-1).VEL_NEW]']; % m/s
        STATE.POS = [POS(i-2,:);[OUTP(j-2:j-1).POS_NEW]'];
        STATE.EULER = [Euler(i-2,:);[OUTP(j-2:j-1).EULER_NEW]'];
        STATE.BODY_RATES = [BODY_RATES(i-2,:);[OUTP(j-2:j-1).OMEGA_NEW_B]'];
        k = 3;
    else
        STATE.VEL = [OUTP(j-3:j-1).VEL_NEW]';
        STATE.POS = [OUTP(j-3:j-1).POS_NEW]';
        STATE.EULER = [OUTP(j-3:j-1).EULER_NEW]';
        STATE.BODY_RATES = [OUTP(j-3:j-1).OMEGA_NEW_B]';
    end
    
    
    tic
    [OUTP(j), PERF, ~, ~, ~, STATE_OUT(j)] = fcnMAIN(TABLE, GEOM, AIR, STATE, 1, OVERWRITE);
    toc
    % Looking at different reference frames
    R = fcnEUL2R(STATE.EULER,4,0);  
    Flighttest_body_rates_i(i+1,:) = (R\BODY_RATE_From_Euler(i+1,:)')';
    R = [1 0 0]'/OUTP(j).VEL_NEW;
    options.epsilon = 1e-20;
    r = vrrotvec2mat(vrrotvec(OUTP(j).VEL_NEW./norm(OUTP(j).VEL_NEW),[1 0 0]',options),options);
    Flightpath_Body_rates(i+1,:)  = (r*Flighttest_body_rates_i(i+1,:)')';
    Flightpath_Body_rates_predicted(i+1,:) = (r*OUTP(j).OMEGA_NEW)';
    Flightpath_Vel_predicted(i+1,:) = (r*OUTP(j).VEL_NEW)';
    
    % Comparing conditions
    idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).VEL_NEW))>Vel_criteria;
    idxBODY_COND(j,:) = (abs(BODY_RATE_From_Euler(i+1,:)'-OUTP(j).OMEGA_NEW_B))>Body_Rates_criteria;
    if any(idxVEL_COND(j,:)) || any(idxBODY_COND(j,:))
        cond = false;
        iter_num(j) = count_iter_num;
        count_iter_num = 0;
    else
        cond = true;
        count_iter_num = count_iter_num+1;
    end
    fprintf(strcat(num2str(i),' Complete\n'))
end

save('DATA')
%% Plotting
tempPOS = [OUTP.POS_NEW];
figure(1)
clf(1)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempPOS(1,:),'k--o','linewidth',2)
 
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempPOS(2,:),'r--s','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,POS(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempPOS(3,:),'b--d','linewidth',2)
legend('Experiment x-dir','Predicted x-dir','Experiment y-dir','Predicted y-dir','Experiment z-dir','Predicted z-dir')
ylabel('Inertial Position (m)')
xlabel('Time')
title('Position')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off


tempEULER = [OUTP.EULER_NEW];
figure(2)
clf(2)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempEULER(1,:),'k--o','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempEULER(2,:),'r--s','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Euler(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempEULER(3,:),'b--d','linewidth',2)
legend('Experiment \phi','Predicted \phi','Experiment \theta','Predicted \theta','Experiment \psi','Predicted \psi')
ylabel('Angle (rad)')
xlabel('Time')
title('Euler Angles')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

tempVEL = [OUTP.VEL_NEW];
figure(3)
clf(3)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempVEL(1,:),'k--o','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempVEL(2,:),'r--s','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,VEL(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempVEL(3,:),'b--d','linewidth',2)
legend('Experiment x-dir','Predicted x-dir','Experiment y-dir','Predicted y-dir','Experiment z-dir','Predicted z-dir')
ylabel('Velocity (m/s)')
xlabel('Time')
title('Velocity')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off


tempRATES = [OUTP.OMEGA_NEW_B];

% tempRATES = [OUTP.OMEGA_NEW];
figure(4)
clf(4)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempRATES(1,:),'k--o','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempRATES(2,:),'r--s','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,BODY_RATES(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,tempRATES(3,:),'b--d','linewidth',2)
legend('Experiment $\dot{\phi}$','Predicted $\dot{\phi}$','Experiment $\dot{\theta}$','Predicted $\dot{\theta}$','Experiment $\dot{\psi}$','Predicted $\dot{\psi}$', 'Interpreter','latex')
ylabel('Rate (rad/s)')
xlabel('Time')
title('Body Rates')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

% Print Errors to command window for reference
fprintf('Max Positional Difference (x,y,z): (%f, %f, %f)\n',max(abs(POS(begin+int:int:fin+int,:)-tempPOS')))
fprintf('Max Euler Angle Difference (phi,theta,psi): (%f, %f, %f)\n',max(abs(Euler(begin+int:int:fin+int,:)-tempEULER')))
fprintf('Max Velocity Difference (x,y,z): (%f, %f, %f)\n',max(abs(VEL(begin+int:int:fin+int,:)-tempVEL')))
fprintf('Max Body Rate Difference (phi,theta,psi): (%f, %f, %f)\n',max(abs(BODY_RATES(begin+int:int:fin+int,:)-tempRATES')))


figure(5)
clf(5)
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

figure(6)
clf(6)
hold on
X = categorical({'Vel X-Dir','Vel Y-Dir','Vel Z-Dir','Roll Rate','Pitch Rate','Yaw Rate'});
sum_cond_missed = cat(2,sum(idxVEL_COND),sum(idxBODY_COND));
bar(X,sum_cond_missed)
xlabel('Condition Missed')
ylabel('Number of Occurrence')
title('Number of times each condition was missed')
grid on
box on
hold off


figure(7)
clf(7)
hold on
scatter3(POS(begin+int:int:fin+int,1),POS(begin+int:int:fin+int,2),POS(begin+int:int:fin+int,3),'k')
scatter3(tempPOS(1,:),tempPOS(2,:),tempPOS(3,:),'r')
axis equal
xlabel('X-Position')
ylabel('Y-Position')
zlabel('Z-Position')
grid on
grid minor
box on
hold off

figure(8)
clf(8)
hold on
% plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Vel_predicted(begin+int:int:fin+int,1),'k--o','linewidth',2)

% plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Vel_predicted(begin+int:int:fin+int,2),'r--s','linewidth',2)

% plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Vel_predicted(begin+int:int:fin+int,3),'b--d','linewidth',2)
legend('Predicted x-dir','Predicted y-dir','Predicted z-dir', 'Interpreter','latex')
ylabel('Velocity (m/s)')
xlabel('Time')
title('Velocity')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

figure(9)
clf(9)
hold on
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,1),'k*-','linewidth',2)
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates_predicted(begin+int:int:fin+int,1),'k--o','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,2),'rs-','linewidth',2,'markerfacecolor','r')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates_predicted(begin+int:int:fin+int,2),'r--s','linewidth',2)

plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates(begin+int:int:fin+int,3),'bd-','linewidth',2,'markerfacecolor','b')
plot(begin/datafeq:1/STATE.FREQ:fin/datafeq,Flightpath_Body_rates_predicted(begin+int:int:fin+int,3),'b--d','linewidth',2)
legend('Experiment $\dot{\phi}$','Predicted $\dot{\phi}$','Experiment $\dot{\theta}$','Predicted $\dot{\theta}$','Experiment $\dot{\psi}$','Predicted $\dot{\psi}$', 'Interpreter','latex')
ylabel('Rate (rad/s)')
xlabel('Time')
title('Body Rates')
xlim([begin/datafeq fin/datafeq])
box on
grid on
grid minor
hold off

