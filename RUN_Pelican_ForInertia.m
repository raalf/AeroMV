% Run Pelican Database to calculate moment of inertias from experiment
% The intention with this is to determine if the moment of inertias from
% being used are reasonable. 

% Code is 2 parts:
%   Part 1 - Calculate the equations needed to solve for moment of inertia
%            from the equations of motion. This is done in symbolic form.
%
%   Part 2 - Solve these equation using the predicted moment and the 
%            measure flight test body rates.
%
%       NOTE: This make a bold decision that the prediction is perfect.
%       This makes AeroMV NOT a soley predictive program and should be used
%       only for testing purposes.
%
% D.F.B. in Barrie ON, Canada, Apr. 2020

clear,clc

%% Solve for moment of inertia equations

syms w_x w_y w_z % Body rates
syms M_x M_y M_z % Moments (N.m)
syms Ixx Iyy Izz % Moment of inertias
syms wd_x wd_y wd_z % Rotational accelertions

% Equation of motion expanded from vector form to 3 eqn with 3 unknowns
%      Notes: - The cross produce has already been executed manually
%             - This assumes that the moment of inertias are symmetric as
%             there are only 3 equations (ie. only Ixx, Iyy, Izz)

eqn1 = (w_y*w_z*(Iyy - Izz)+M_x)/wd_x == Ixx;
eqn2 = (w_z*w_x*(Izz - Ixx)+M_y)/wd_y == Iyy;
eqn3 = (w_x*w_y*(Ixx - Iyy)+M_z)/wd_z == Izz;

% Solve these equations
S = solve([eqn1 eqn2 eqn3],[Ixx Iyy Izz],'ReturnConditions',true);

% These are the resulting equations
% Ixx = (M_x*wd_y*wd_z + M_y*w_y*w_z*wd_z - M_z*w_y*w_z*wd_y + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
% Iyy = (M_y*wd_x*wd_z - M_x*w_x*w_z*wd_z + M_z*w_x*w_z*wd_x + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
% Izz = (M_z*wd_x*wd_y + M_x*w_x*w_y*wd_y - M_y*w_x*w_y*wd_x + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
%  NOTE: These equations were tested by solving for know Inertias and gave
%  the correct values 

%% Run AEROMV
clear Ixx Iyy Izz
filename = 'AscTec_Pelican';
load('DATA/Pelican_Dataset/AscTec_Pelican_Flight_Dataset.mat','flights')
flight_num = 40;


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
fin = 35000;
datafeq = 100;
int = 1;
STATE.FREQ = datafeq/int;

for i = begin:int:fin
    j = j+1;
    STATE.RPM = 1.135*[RPM(i,1) RPM(i,2) RPM(i,3) RPM(i,4)]; % RPM

    STATE.EULER = Euler(i,:);
    if k == 1 || i == begin
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
    
    %  -------- Calculate moment of inertias --------
    % Set moment components as M_B that is calculated from AeroMV
    M_x = OUTP(j).M_B(1); M_y = OUTP(j).M_B(2); M_z = OUTP(j).M_B(3);
    % Set omega as the body_rates from the experiements
    w_x = BODY_RATES(i,1); w_y = BODY_RATES(i,2); w_z = BODY_RATES(i,3);
    % Calculate rotation acceleration from the flight data using the
    % frequency of the data. Assuming first order, linear (as does AeroMV)
    a = (BODY_RATES(i+1,:)- BODY_RATES(i,:)).*(STATE.FREQ);
    wd_x =a(1); wd_y = a(2); wd_z = a(3);
    
    % Calculate the moment of inertias at the given data point
    Ixx(j) = (M_x*wd_y*wd_z + M_y*w_y*w_z*wd_z - M_z*w_y*w_z*wd_y + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
    Iyy(j) = (M_y*wd_x*wd_z - M_x*w_x*w_z*wd_z + M_z*w_x*w_z*wd_x + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
    Izz(j) = (M_z*wd_x*wd_y + M_x*w_x*w_y*wd_y - M_y*w_x*w_y*wd_x + M_x*w_x^2*w_y*w_z + M_y*w_x*w_y^2*w_z + M_z*w_x*w_y*w_z^2)/(wd_x*w_x^2*w_y*w_z + wd_y*w_x*w_y^2*w_z + wd_z*w_x*w_y*w_z^2 + wd_x*wd_y*wd_z);
end

% Calculate the mean moment of inertia, cutting off any strange data points
% The cutoff criteria is set to 1 but this could be modified with
% justification
cutoff = 1; 
I_mean = [mean(Ixx(Ixx<cutoff)) mean(Iyy(Iyy<cutoff)) mean(Izz(Izz<cutoff))];
save('InertiaData_Flight40')
