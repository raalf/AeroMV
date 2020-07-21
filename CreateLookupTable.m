%Creates rotor lookup table
% This program created the lookup tables in the correct format for
% fcnLOOKUP. Currently setup up to run BEMT.
%
% D.F.B. in Braunschweig Germany, Feb. 2020
clear,clc

%% Input variables
GEOM.ROTOR.strNAME = 'Matrice_210_RTK';
AIR.density = 1.225; % Density
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity

aoa = -90:15:90; % Angle of attack range
seqJ = 0:0.05:1; % Advance ratio range (J = V/nD not mu)
rpm = 1000:1000:7000; % RPM range
D = 0.4318; % Rotor diameter


%% Create lookup table
% Create meshgrid with angle of attack, advance ratio and rpm
[Angle,J,RPM] = meshgrid(aoa,seqJ,rpm);

% Pre-allocated the rotor coefficients
CT = zeros(length(seqJ),length(aoa),length(rpm));
CP = zeros(length(seqJ),length(aoa),length(rpm));
CQ = zeros(length(seqJ),length(aoa),length(rpm));
CNx = zeros(length(seqJ),length(aoa),length(rpm));
CNy = zeros(length(seqJ),length(aoa),length(rpm));
CMx = zeros(length(seqJ),length(aoa),length(rpm));
CMy = zeros(length(seqJ),length(aoa),length(rpm));


% Iterate through angle of attack, advance ratio and rpm
for i = 1:length(aoa)
    for k = 1:length(seqJ)
        parfor m = 1:length(rpm) % Run in parallel. Switch delete 'par' to not run parallel
            
            % Create STATE structure
            STATE = [];
            STATE.AOA_R = Angle(k,i,m);
            STATE.VEL_ROTOR_MAG = J(k,i,m)*(RPM(k,i,m)/60)*D;
            STATE.RPM = RPM(k,i,m);
            
            % Create PERF structure
            PERF.TEMP = [];
            try PERF = rmfield(PERF,'ROTOR'); end % Delete PERF.ROTOR if exists
            
            % *********RUN BEMT*********
            PERF = fcnRUNBEMT(GEOM, AIR, PERF, STATE);
            
            % Save rotor performance results
            CT(k,i,m) = PERF.ROTOR.CT;
            CP(k,i,m) = PERF.ROTOR.CP;
            CQ(k,i,m) = PERF.ROTOR.CQ;
            CNx(k,i,m) = PERF.ROTOR.CNx;
            CNy(k,i,m) = PERF.ROTOR.CNy;
            CMx(k,i,m) = PERF.ROTOR.CMx;
            CMy(k,i,m) = PERF.ROTOR.CMy;

            % Print status
            fprintf("Complete: rpm: %f \t J: %f \t aoa: %f\n",STATE.RPM,J(k,i,m),STATE.AOA_R);
            
        end
    end
end

% Save to data to TABLES based on rotor name
save(strcat("TABLES/",GEOM.ROTOR.strNAME,"_BEMTDATA"),'CT','CP','CQ','CNx','CNy','CMx','CMy','RPM','J','Angle');
