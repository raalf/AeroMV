function PERF = fcnRUNVAPSINGLE(GEOM,AIR,PERF,STATE)
%fcnRUNVAP sets up and run VAP3p5 for each rotor individually. Timestep
%   size, flagRELAX and max time are hard coded in.


% Hardcoded inputs. Maybe change to input file in the future
timestepsperrev = 20; % Number of timesteps per revolution
maxtime = 40; % Max time
relax = false; % relaxation on or off (true/false) 

% Change directory to VAP3p5 AERO folder
FOLDER_ADDRESS = pwd;
cd(strcat(FOLDER_ADDRESS,'/AERO/VAP3p5'))

% Iterate through each rotor
for i = 1:length(STATE.RPM)
    % Hardcoded variables
    VAP_IN = [];
    VAP_IN.RELAX = relax;
    VAP_IN.valMAXTIME = maxtime;
    VAP_IN.valSTARTFORCES = floor(maxtime-timestepsperrev);
    
    % Overwrite variables from AeroMV case
    VAP_IN.valDENSITY = AIR.density; % Air density
    VAP_IN.valKINV = AIR.kinvisc;% kinematic viscosity
    VAP_IN.vecVEHVINF = STATE.VEL_ROTOR_MAG(i); % freestream velocity of interest
    VAP_IN.vecROTORRPM = STATE.RPM(i); % RPM of rotor of interest
    VAP_IN.vecVEHALPHA = -1*STATE.AOA_R(i); % Angle of attack of rotor. Must put in a -ve angle to convert from rotor reference frame to aircraft reference frame
    VAP_IN.valDELTIME = 1/((STATE.RPM(i)/60)*timestepsperrev); %Calculate deltime
    
    fprintf('Running Rotor #%d in VAP3.5\n',i);
    
    % Run VAP3.5
    % This is a modified version of the github: https://github.com/raalf/VAP3
    OUTP = fcnVAP_MAIN(strcat('inputs/',GEOM.ROTOR.strNAME,'.vap'), VAP_IN);
    
    % Convert outputs to variables used in AeroMV
    PERF.ROTOR(i).T = OUTP.vecCT_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Nx = OUTP.vecCFx_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Ny = OUTP.vecCFy_AVG*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^4)*AIR.density;
    PERF.ROTOR(i).Q =  (OUTP.vecCP_AVG/(2*pi*STATE.RPM(i)/60)).*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).P = OUTP.vecCP_AVG.*((STATE.RPM(i)/60).^3).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).Mx = OUTP.vecCMx_AVG.*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
    PERF.ROTOR(i).My = OUTP.vecCMy_AVG.*((STATE.RPM(i)/60).^2).*((GEOM.ROTOR.vecDIAM(i)).^5)*AIR.density;
end

fprintf('------ Done Running VAP3.5 Cases ------\n\n')

% Go back to AeroMV main folder
cd(FOLDER_ADDRESS)
