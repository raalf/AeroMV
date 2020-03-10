%Create lookup table
clear,clc
GEOM.ROTOR.strNAME = 'APC_10x_4_7_Updated';
AIR.density = 1.225; % Density
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity

aoa = -90:15:90;
seqJ = 0:0.05:1;
rpm = 1000:1000:7000;

aoa = [0 5 10 15 30 90];
seqJ = 0:0.05:1.1;
rpm = 5000;
% aoa = 5; 
% seqJ = 0.6;
% rpm = 5000;

D = 0.254;
%D = 0.2794;

[Angle,J,RPM] = meshgrid(aoa,seqJ,rpm);

CT = zeros(length(seqJ),length(aoa),length(rpm));
CP = zeros(length(seqJ),length(aoa),length(rpm));
CQ = zeros(length(seqJ),length(aoa),length(rpm));
CNx = zeros(length(seqJ),length(aoa),length(rpm));
CNy = zeros(length(seqJ),length(aoa),length(rpm));
CMx = zeros(length(seqJ),length(aoa),length(rpm));
CMy = zeros(length(seqJ),length(aoa),length(rpm));
% RPM = zeros(length(aoa),length(seqJ),length(rpm));
% Angle = zeros(length(aoa),length(seqJ),length(rpm));
% J = zeros(length(aoa),length(seqJ),length(rpm));

for i = 1:length(aoa)
    for k = 1:length(seqJ)
        for m = 1:length(rpm)
            STATE = [];
            STATE.AOA_R = Angle(k,i,m);
            STATE.VEL_ROTOR_MAG = J(k,i,m)*(RPM(k,i,m)/60)*D;
            STATE.RPM = RPM(k,i,m);
            
            
            PERF.TEMP = [];
            try PERF = rmfield(PERF,'ROTOR'); end
            PERF = fcnRUNBEMT(GEOM, AIR, PERF, STATE);
            
            CT(k,i,m) = PERF.ROTOR.CT;
            CP(k,i,m) = PERF.ROTOR.CP;
            CQ(k,i,m) = PERF.ROTOR.CQ;
            CNx(k,i,m) = PERF.ROTOR.CNx;
            CNy(k,i,m) = PERF.ROTOR.CNy;
            CMx(k,i,m) = PERF.ROTOR.CMx;
            CMy(k,i,m) = PERF.ROTOR.CMy;

            fprintf("Complete: rpm: %f \t J: %f \t aoa: %f\n",STATE.RPM,J(k,i,m),STATE.AOA_R);
            
        end
    end
end

%save(strcat("TABLES/",GEOM.ROTOR.strNAME,"_BEMTDATA"),'CT','CP','CQ','CNx','CNy','CMx','CMy','RPM','J','Angle');
