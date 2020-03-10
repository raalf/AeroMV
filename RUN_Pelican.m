% Run Pelican Datase
clear,clc
filename = 'AscTec_Pelican';
load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
flight_num = 1;


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
k = 1;
begin = 1000;
fin = 1010;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

for i = begin:int:fin
    j = j+1;
    STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM

    STATE.EULER = Euler(i,:);
    if k == 10 || i == begin
        STATE.VEL = VEL(i,:); % m/s
        STATE.POS = POS(i,:);
        STATE.EULER = Euler(i,:);
        STATE.BODY_RATES = BODY_RATES(i,:);
        k = 1;
    else
        STATE.VEL = OUTP(j-1).VEL_NEW';
        STATE.POS = OUTP(j-1).POS_NEW';
        STATE.EULER = OUTP(j-1).EULER_NEW';
        STATE.BODY_RATES = OUTP(j-1).OMEGA_NEW_B';
%         STATE.BODY_RATES = BODY_RATES(i,:);
        k = k+1;
    end
    
    tic
    [OUTP(j), PERF, TABLE, GEOM, AIR, STATE_OUT(j)] = fcnMAIN(filename, STATE, 1);
    toc
    fprintf(strcat(num2str(i),' Complete\n'))
end

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
legend('Experiment \dot{\phi}','Predicted \dot{\phi}','Experiment \dot{\theta}','Predicted \dot{\theta}','Experiment \dot{\psi}','Predicted $\dot{\psi}',  'Interpreter','latex')
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
