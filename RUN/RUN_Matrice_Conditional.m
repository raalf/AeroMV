% Run Matrice 210 RTK dataset
clear,clc
% fcnRUN_DIR to be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()

% Input filename
filename = 'DJI_Matrice_210_RTK';
% Dataset
load('DATA/Matrice_210_RTK_Dataset/July3_2020_Flight_1.mat','Flight_Data','density','flight_segments')


flight_num = 6;

Euler = Flight_Data(1,flight_num).Euler_Angles;
VEL = Flight_Data(1,flight_num).Velocity;
RPM = [Flight_Data(1,flight_num).RPM1,Flight_Data(1,flight_num).RPM2,Flight_Data(1,flight_num).RPM3,Flight_Data(1,flight_num).RPM4];
Height = Flight_Data(1,flight_num).Height_Above_Takeoff;
POS = [zeros(length(Height),2), Height];
BODY_RATES = Flight_Data(1,flight_num).Body_Rates;

BODY_RATES = diff(Euler)*50;
j = 0;
begin = 5000;
fin = 5030;
datafeq = 50;
int = 1;
STATE.FREQ = datafeq/int;


Vel_criteria = 0.09;
Body_Rates_criteria = 0.12;
% Body_Rates_criteria = 0.19;
% Body_Rates_criteria = 0.38;
cond = false;
cond_missed = []; % Condition that causes reset
count_iter_num = 0;

len = (fin-begin)/int + 1;
iter_num  = NaN(len,1);  % Iteration number before it had to reset
idxBROKENCOND = NaN(len,6);
idxVEL_COND = NaN(len,3);
idxBODY_COND = NaN(len,3);
avg_count = 5; % How many points to average for moving average of input variables

% Creating OVERWRITE function
OVERWRITE.AIR.density = density;
% OVERWRITE = [];

%% Retrieve Input Vehicle Geometry
[TABLE, GEOM, AIR] = fcnINPUT(filename);



for i = begin:int:fin
    j = j+1;
    STATE.accuracy = 1;
    STATE.RPM = 0.9*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM
    
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
    
    idxVEL_COND(j,:) = (abs(VEL(i+1,:)'-OUTP(j).VEL_NEW))>Vel_criteria;
    idxBODY_COND(j,:) = (abs(BODY_RATES(i+1,:)'-OUTP(j).OMEGA_NEW_B))>Body_Rates_criteria;
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

