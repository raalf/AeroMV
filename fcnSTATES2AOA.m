function [STATE] = fcnSTATES2AOA(STATE, GEOM)
% This function converts the states to the angles needed for the
% aerodynamic models.
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


%% Calculate/check general variables
% Calculate vehicle velocity magnitude
STATE.VEL_MAG = sqrt(STATE.VEL(end,1).^2+STATE.VEL(end,2).^2+STATE.VEL(end,3).^2);

% If rotor angle of attack was input, ignore this function
if fcnCOMPCHECK(STATE, 'AOA')
    return
end

% Get rotation matrix from Euler angles
R = fcnEUL2R(STATE.EULER(end,:),3,0);

%% Calculate Shaft angle
% Dot the velocity with the z-direction of the vehicle body
% Method 1 - Convert the z-dir of the body frame into intertial
z_I = R*[0 0 1]';
alpha_shaft1 = acos(dot(STATE.VEL(end,:),z_I)./(STATE.VEL_MAG));

% Method 2 - Convert the velocity from intertial frame to the body frame
STATE.VEL_B  = (R'*STATE.VEL(end,:)')';
VEL_B_MAG = sqrt(STATE.VEL_B(1).^2+STATE.VEL_B(2).^2+STATE.VEL_B(3).^2);
alpha_shaft2 = acos(dot(STATE.VEL_B(end,:),[0 0 1])./(VEL_B_MAG));
STATE.BETA = acos(dot(STATE.VEL_B,[1 0 0])./(VEL_B_MAG));

if abs(alpha_shaft1-alpha_shaft2) > 1e-10
    warning('fcnSTATES2AOA - Two Alpha Calculation Technique Not Equal')
end

% Calculate velocity experienced by rotor hub due to vehicle dynamics
STATE.VEL_ROTOR = STATE.BODY_RATES(end,:).*GEOM.ROTOR.matLOCATION + STATE.VEL_B;
STATE.VEL_ROTOR_MAG = sqrt(STATE.VEL_ROTOR(:,1).^2 + STATE.VEL_ROTOR(:,2).^2 + STATE.VEL_ROTOR(:,3).^2);
STATE.AOA_R = acos(dot(STATE.VEL_ROTOR',repmat([0 0 1]',1,size(STATE.VEL_ROTOR,1)))'./(STATE.VEL_ROTOR_MAG));

STATE.BETA = acos(dot(STATE.VEL_ROTOR',repmat([1 0 0]',1,size(STATE.VEL_ROTOR,1)))'./(STATE.VEL_ROTOR_MAG));

%% Convert Shaft angle to TPP angle
STATE.AOA = 90 - (180/pi)*alpha_shaft1;
STATE.AOA_R = 90 - (180/pi)*STATE.AOA_R;


end
