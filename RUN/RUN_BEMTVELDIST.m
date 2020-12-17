% Run fcnRUNBEMT_VELDIST for validation purposes
clear,clc
dir = matlab.desktop.editor.getActiveFilename;
cd(dir(1:end-17))
fcnRUN_DIR()

% Values to iterate through:
mu =  [0.069622 0.13924 0.20886];
roll_rate = -1:0.2:1;

rotor_radius = 0.2286;

% % Creat GEOM structure
GEOM.ROTOR.strNAME = 'Tmotor';
% GEOM.ROTOR.strNAME = 'APC_10x_4_7_Updated';
GEOM.ROTOR.matLOCATION = [0 0 0];
GEOM.ROTOR.matROT = -1;

% Air structure
AIR.density = 1.225;
AIR.kinvisc = 1.46E-5;

% State structure
STATE.RPM = 3000;
STATE.AOA_R = 0;

PERF.Temp = [];
for i = 1:length(mu)
    STATE.VEL_ROTOR_MAG = mu(i)*(STATE.RPM/60*2*pi)*rotor_radius;
    
    for j = 1:length(roll_rate)
        STATE.BODY_RATES = [roll_rate(j) 0 0];
        
       	TEMP = fcnRUNBEMT_VELDIST(GEOM, AIR, PERF, STATE);
        DATA(i,j) = TEMP.ROTOR;
    end
end 
