function [STATE] = fcnSTATES2AOA(STATE, GEOM, vecINDUCEDVEL)
% This function converts the states to the angles needed for the
% aerodynamic models.
%
% INPUTS:
%   vecINDUCEDVEL           - Optional input. Induced velocities
%								experience by a rotor (such as those)
%								due to a wake model. If not included
%								an input, value is assumed to be 0.
%
% OUTPUTS:
% Updated STATE structure with the following additions:
%   STATE.VEL_MAG           - Magnitude of total vehicle velocity
%   STATE.VEL_B             - Ttoal vehicle velocity in body frame
%   STATE.BETA              - Yaw angle (used to assign Fx and Fy dir)
%   STATE.VEL_ROTOR         - Velocity experienced by rotor
%   STATE.VEL_ROTOR_MAG     - Magnitude of velocity experienced by rotor
%   STATE.AOA_R             - TPP angle of a rotor using local velocity
%   STATE.AOA               - TPP angle of rotor based on vehicle
%                             orientation and freestream velocity only


%% Check for vecINDUCEDVEL, if it doesnt exist set it to 0
if ~exist('vecINDUCEDVEL','var')
   vecINDUCEDVEL = 0; 
end

%% Check if there were rotor normals input 
% If the rotor normals were not input, assume [0 0 1] 
if fcnCOMPCHECK(GEOM.ROTOR, 'matNORMALS')
    matROTNORMALS = GEOM.ROTOR.matNORMALS;
else
    matROTNORMALS = repmat([0 0 1], size(GEOM.ROTOR.matLOCATION,1),1);
end

%% Calculate/check general variables
% Calculate vehicle velocity magnitude
STATE.VEL_MAG = sqrt(STATE.VEL(end,1).^2+STATE.VEL(end,2).^2+STATE.VEL(end,3).^2);

% Removed this because it is largely for debugging
% % If rotor angle of attack was input, ignore this function
% if fcnCOMPCHECK(STATE, 'AOA')
%     return
% end

% Get rotation matrix from Euler angles
R = fcnEUL2R(STATE.EULER(end,:),3,0);

%% Calculate Shaft angle
% Dot the velocity with the rotor normals
% Method 1 - Convert the rotor normals into intertial
z_I = (R*matROTNORMALS')';
alpha_shaft1 = acos(dot(repmat(STATE.VEL(end,:),size(GEOM.ROTOR.matLOCATION,1),1),z_I,2)./(STATE.VEL_MAG));

% Method 2 - Convert the velocity from intertial frame to the body frame
STATE.VEL_B  = (R'*STATE.VEL(end,:)')';
VEL_B_MAG = sqrt(STATE.VEL_B(1).^2+STATE.VEL_B(2).^2+STATE.VEL_B(3).^2);
alpha_shaft2 = acos(dot(repmat(STATE.VEL_B(end,:),size(GEOM.ROTOR.matLOCATION,1),1),matROTNORMALS,2)./(VEL_B_MAG));

if abs(alpha_shaft1-alpha_shaft2) > 1e-10
    warning('fcnSTATES2AOA - Two Alpha Calculation Technique Not Equal')
end

% Calculate velocity experienced by rotor hub due to vehicle dynamics
STATE.VEL_ROTOR = STATE.BODY_RATES(end,:).*GEOM.ROTOR.matLOCATION + STATE.VEL_B + vecINDUCEDVEL;
STATE.VEL_ROTOR_MAG = sqrt(STATE.VEL_ROTOR(:,1).^2 + STATE.VEL_ROTOR(:,2).^2 + STATE.VEL_ROTOR(:,3).^2);
STATE.AOA_R = acos(dot(STATE.VEL_ROTOR,matROTNORMALS,2)./(STATE.VEL_ROTOR_MAG));

% Calculate x-dir based on rotor normals
xdir = cross(repmat([0 1 0],size(STATE.VEL_ROTOR,1),1),matROTNORMALS);
% Calculate beta angles
STATE.BETA = acos(dot(STATE.VEL_ROTOR,xdir,2)./(STATE.VEL_ROTOR_MAG));

%% Correct for hover conditions
% If the vehicles is perfectly in hover, the angles must be corrects. 
% Note: that this only realistically happens with simulation data and not
% with experimental data
if STATE.VEL_MAG == 0
    alpha_shaft1 = 0;
end
if any(STATE.VEL_ROTOR_MAG == 0)
    idx = STATE.VEL_ROTOR_MAG == 0;
    STATE.AOA_R(idx) = 0;
    STATE.BETA(idx) = 0;
end
%% Convert Shaft angle to TPP angle
STATE.AOA = 90 - (180/pi)*alpha_shaft1;
STATE.AOA_R = 90 - (180/pi)*STATE.AOA_R;

end
