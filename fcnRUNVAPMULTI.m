function PERF = fcnRUNVAPMULTI(GEOM,AIR,PERF,STATE)
%fcnRUNVAPMULTI



% Hardcoded inputs. Maybe change to input file in the future
timestepsperrev = 20; % Number of timesteps per revolution
maxtime = 160; % Max time
relax = false; % relaxation on or off (true/false)


% Change directory to VAP3p5 AERO folder
FOLDER_ADDRESS = pwd;
cd(strcat(FOLDER_ADDRESS,'/AERO/VAP3p5'))



% Read xml input file
INPUT_STRUCT = fcnXML2STRUCT(strcat('inputs/',GEOM.ROTOR.strNAME,'.vap'));

% Check if input is only 1 rotor or if the full vehicle is already modded
if length(INPUT_STRUCT.VAP.vehicle.rotor) == 1
    % If there is only 1 rotor, copy the one rotor to the number of rotors
    % to be simualted
    INPUT_STRUCT.VAP.vehicle.rotor = repmat({INPUT_STRUCT.VAP.vehicle.rotor},1,length(STATE.RPM));
end

% Iterate through each rotor and adjust the rotors hub locations, rpms,
% rotation direction and local axis

for i = 1:length(STATE.RPM)
    INPUT_STRUCT.VAP.vehicle.rotor{i}.rpm.Text = num2str(STATE.RPM(i));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_x_hub.Text = num2str(GEOM.ROTOR.matLOCATION(i,1));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_y_hub.Text = num2str(GEOM.ROTOR.matLOCATION(i,2));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_z_hub.Text = num2str(GEOM.ROTOR.matLOCATION(i,3));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_x_axis.Text = num2str(GEOM.ROTOR.matNORMALS(i,1));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_y_axis.Text = num2str(GEOM.ROTOR.matNORMALS(i,2));
    INPUT_STRUCT.VAP.vehicle.rotor{i}.veh_z_axis.Text = num2str(GEOM.ROTOR.matNORMALS(i,3));
    % 1 = CW, -1 = CCW
    if GEOM.ROTOR.matROT(i) == 1
        tempROT = 'CW';
    elseif GEOM.ROTOR.matROT(i) == -1
        tempROT = 'CCW';
    end
    INPUT_STRUCT.VAP.vehicle.rotor{i}.rotation_direction.Text = tempROT;
    
end

% Create new XML
MultirotorXML = fcnSTRUCT2XML(INPUT_STRUCT);

% Write new xml to a tempMULTI input file
tempFILENAME = strcat('inputs/',GEOM.ROTOR.strNAME,'_tempMULTI','.vap');
fid = fopen(tempFILENAME,'w'); % Overwrite if neccessary
fprintf(fid,'%s',MultirotorXML); % Write xml generated above to the file
fclose(fid); % Close file


% Hardcoded variables
VAP_IN = [];
VAP_IN.RELAX = relax;
VAP_IN.valMAXTIME = maxtime;
VAP_IN.valSTARTFORCES = floor(maxtime-timestepsperrev);


% Overwrite variables from AeroMV case
VAP_IN.valDENSITY = AIR.density; % Air density
VAP_IN.valKINV = AIR.kinvisc;% kinematic viscosity
VAP_IN.vecVEHVINF = STATE.VEL_MAG+10; % freestream velocity of interest
VAP_IN.vecVEHALPHA = rad2deg(atan(STATE.VEL(end,1)/STATE.VEL(end,3))); % Angle of attack of rotor. Must put in a -ve angle to convert from rotor reference frame to aircraft reference frame
VAP_IN.vecVEHBETA = rad2deg(atan(STATE.VEL(end,2)/STATE.VEL(end,3)));
VAP_IN.valDELTIME = 1/((min(STATE.RPM)/60)*timestepsperrev); %Calculate deltime


% Run VAP3.5
% This is a modified version of the github: https://github.com/raalf/VAP3
OUTP = fcnVAP_MAIN(tempFILENAME, VAP_IN);

% Convert outputs to variables used in AeroMV
PERF.ROTOR.T = OUTP.vecCT_AVG.*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^4)*AIR.density;
PERF.ROTOR.Nx = OUTP.vecCFx_AVG.*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^4)*AIR.density;
PERF.ROTOR.Ny = OUTP.vecCFy_AVG.*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^4)*AIR.density;
PERF.ROTOR.Q =  (OUTP.vecCP_AVG./(2*pi*STATE.RPM/60)).*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^5)*AIR.density;
PERF.ROTOR.P = OUTP.vecCP_AVG.*((STATE.RPM/60).^3).*((GEOM.ROTOR.vecDIAM').^5)*AIR.density;
PERF.ROTOR.Mx = OUTP.vecCMx_AVG.*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^5)*AIR.density;
PERF.ROTOR.My = OUTP.vecCMy_AVG.*((STATE.RPM/60).^2).*((GEOM.ROTOR.vecDIAM').^5)*AIR.density;

fprintf('------ Done Running VAP3.5 Cases ------\n\n')

% Go back to AeroMV main folder
cd(FOLDER_ADDRESS)
