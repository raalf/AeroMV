function [PERF] = fcnLOOKUP(GEOM, AIR, PERF, STATE)
% This function reads the rotor performance from a lookup table and
%   converts from non-dimensional terms to dimensional values.
%
% The data should be located in the TABLES folder and use the rotor name as
% the filename (either GEOM.ROTOR.strNAME or with _BEMTDATA at end)
%
% OUTPUTS:
%   PERF.ROTOR.T      - Thrust (N)
%   PERF.ROTOR.Nx     - Rotor drag force (N)
%   PERF.ROTOR.Ny     - Rotor side force (N)
%   PERF.ROTOR.P      - Rotor power (W)
%   PERF.ROTOR.Q      - Rotor torque (N.m)
%   PERF.ROTOR.Mx     - Rotor rolling moment (N.m)
%   PERF.ROTOR.My     - Rotor pitching moment (N.m)
%
% D.F.B. in Braunschweig Germany, Feb. 2020

% Lookup tables have Angle, J, RPM and force/moments. These can be created
% using the CreateLookupTable.m script.

%% Load in data
try
    % First check for a filename that end with _BEMTDATA
    load(strcat("TABLES/",GEOM.ROTOR.strNAME,"_BEMTDATA"));
    idx = true;
catch
    idx = false;
end
% If file didnt exist try without the end _BEMTDATA
if ~idx
    try
        load(strcat("TABLES/",GEOM.ROTOR.strNAME));
        idx = true;
    catch
        idx = false;
    end
end
% Error message if unable to find lookup table
if ~idx
    error("fcnLOOKUP: Unable to open lookup table file");
end

%% Interpolate values
% Calculate advance ratio for the given case
J_case = STATE.VEL_ROTOR_MAG./((STATE.RPM').*(GEOM.ROTOR.vecDIAM));

% Interpolate coefficient values
CT_Interp = interp3(Angle,J,RPM,CT,STATE.AOA_R,J_case,STATE.RPM');
CP_Interp = interp3(Angle,J,RPM,CP,STATE.AOA_R,J_case,STATE.RPM');
CQ_Interp = interp3(Angle,J,RPM,CQ,STATE.AOA_R,J_case,STATE.RPM');
CNx_Interp = interp3(Angle,J,RPM,CNx,STATE.AOA_R,J_case,STATE.RPM');
CNy_Interp = interp3(Angle,J,RPM,CNy,STATE.AOA_R,J_case,STATE.RPM');
CMx_Interp = interp3(Angle,J,RPM,CMx,STATE.AOA_R,J_case,STATE.RPM');
CMy_Interp = interp3(Angle,J,RPM,CMy,STATE.AOA_R,J_case,STATE.RPM');

%% Dimensionalize variables
% Calculate radius and rotation speed omega
R = GEOM.ROTOR.vecDIAM/2;
omega = 2*pi*(STATE.RPM'/60);

% Convert coefficients to dimensional terms
PERF.ROTOR.T = (CT_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.Nx = (CNx_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.Ny = (CNy_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.P = (CP_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^3))';
PERF.ROTOR.Q = (CQ_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';
PERF.ROTOR.Mx = (CMx_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';
PERF.ROTOR.My = (CMy_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';

end

