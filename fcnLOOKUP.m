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

datatype = 0;
% Interpolate coefficient values
if length(unique(RPM)) == 1 % Made a special case when the data is all at 1 rpm (often happens with experimental data)
    CT_Interp = griddata(Angle,J,CT,STATE.AOA_R,J_case);
    CP_Interp = griddata(Angle,J,CP,STATE.AOA_R,J_case);
    CQ_Interp = griddata(Angle,J,CQ,STATE.AOA_R,J_case);
    CNx_Interp = griddata(Angle,J,CNx,STATE.AOA_R,J_case);
    CNy_Interp = griddata(Angle,J,CNy,STATE.AOA_R,J_case);
    CMx_Interp = griddata(Angle,J,CMx,STATE.AOA_R,J_case);
    CMy_Interp = griddata(Angle,J,CMy,STATE.AOA_R,J_case);
elseif datatype == 2 % Assuming 90 deg for all data
    index = find(Angle(1,:,1) == 90);
    CT = permute(CT(:,index,:),[1,3,2]);
    CP = permute(CP(:,index,:),[1,3,2]);
    CQ = permute(CQ(:,index,:),[1,3,2]);
    CNx = permute(CNx(:,index,:),[1,3,2]);
    CNy = permute(CNy(:,index,:),[1,3,2]);
    CMx = permute(CMx(:,index,:),[1,3,2]);
    CMy = permute(CMy(:,index,:),[1,3,2]);
    RPM = permute(RPM(:,index,:),[1,3,2]);
    J = permute(J(:,index,:),[1,3,2]);
    CT_Interp = griddata(RPM,J,CT,STATE.RPM',J_case);
    CP_Interp = griddata(RPM,J,CP,STATE.RPM',J_case);
    CQ_Interp = griddata(RPM,J,CQ,STATE.RPM',J_case);
    CNx_Interp = griddata(RPM,J,CNx,STATE.RPM',J_case);
    CNy_Interp = griddata(RPM,J,CNy,STATE.RPM',J_case);
    CMx_Interp = griddata(RPM,J,CMx,STATE.RPM',J_case);
    CMy_Interp = griddata(RPM,J,CMy,STATE.RPM',J_case);
else % Complete 3D interpolation (Using interp3 because it is much faster than griddata)
    CT_Interp = interp3(Angle,J,RPM,CT,STATE.AOA_R,J_case,STATE.RPM');
    CP_Interp = interp3(Angle,J,RPM,CP,STATE.AOA_R,J_case,STATE.RPM');
    CQ_Interp = interp3(Angle,J,RPM,CQ,STATE.AOA_R,J_case,STATE.RPM');
    CNx_Interp = interp3(Angle,J,RPM,CNx,STATE.AOA_R,J_case,STATE.RPM');
    CNy_Interp = interp3(Angle,J,RPM,CNy,STATE.AOA_R,J_case,STATE.RPM');
    CMx_Interp = interp3(Angle,J,RPM,CMx,STATE.AOA_R,J_case,STATE.RPM');
    CMy_Interp = interp3(Angle,J,RPM,CMy,STATE.AOA_R,J_case,STATE.RPM');
end
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

