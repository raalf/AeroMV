% Run fcnRUNBEMT_VELDIST for validation purposes
clear,clc
dir = matlab.desktop.editor.getActiveFilename;
cd(dir(1:end-17))
fcnRUN_DIR()

% Values to iterate through:
mu = [0.0464 0.1392];
roll_rate = -1:0.5:1;
AOA_R_sweep = [0 5 15];
matLOCATION_sweep = [0 0 0; 0 0.24 0];

rotor_radius = 0.2286;

% % Creat GEOM structure
GEOM.ROTOR.strNAME = 'Tmotor_inv';
% GEOM.ROTOR.strNAME = 'APC_10x_4_7_Updated';

GEOM.ROTOR.matROT = -1;

% Air structure
AIR.density = 1.225;
AIR.kinvisc = 1.46E-5;

% State structure
STATE.RPM = 4500;

count = 0;
PERF.Temp = [];
for i = 1:size(matLOCATION_sweep,1)
    GEOM.ROTOR.matLOCATION = matLOCATION_sweep(i,:);
    for j = 1:length(AOA_R_sweep)
        STATE.AOA_R = AOA_R_sweep(j);
        for k = 1:length(mu)
            STATE.VEL_ROTOR_MAG = mu(k)*(STATE.RPM/60*2*pi)*rotor_radius;
            
            for m = 1:length(roll_rate)
                STATE.BODY_RATES = [roll_rate(m) 0 0];
                
                TEMP = fcnRUNBEMT_VELDIST(GEOM, AIR, PERF, STATE);
                count = count + 1;

                DATA_temp.RUN_NUM = count;
                DATA_temp.YAxis_offset = GEOM.ROTOR.matLOCATION(2);
                DATA_temp.TPP_Angle = STATE.AOA_R;
                DATA_temp.Velocity = STATE.VEL_ROTOR_MAG;
                DATA_temp.RPM = STATE.RPM;
                DATA_temp.Roll_Rate = STATE.BODY_RATES(1);
                DATA_temp.mu = mu(k);
                DATA_temp.density = AIR.density;
                DATA_temp.kinvisc = AIR.kinvisc;
                DATA_temp.RotorRadius = rotor_radius;
                DATA_temp.CT = TEMP.ROTOR.CT;
                DATA_temp.CP = TEMP.ROTOR.CP;
                DATA_temp.CQ = TEMP.ROTOR.CQ;
                DATA_temp.CMx = TEMP.ROTOR.CMx;
                DATA_temp.CMy = TEMP.ROTOR.CMy;
                DATA_temp.CFx = TEMP.ROTOR.CNx;
                DATA_temp.CFy = TEMP.ROTOR.CNy;
                DATA_temp.RAW_BEMT_RESULTS = TEMP.ROTOR;
            
                DATA(:,count) = DATA_temp;
                
            end
        end
    end
end

