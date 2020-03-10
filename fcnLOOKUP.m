function [PERF] = fcnLOOKUP( GEOM, AIR, PERF, STATE )
% This function reads the rotor performance from a lookup table.

% Lookup tables have Angle, J, RPM and force/moments.
% The force/moments are CT, CP, CNx, CNy, CMx, CMy

idx = false;
try
    load(strcat("TABLES/",GEOM.ROTOR.strNAME,"_BEMTDATA"));
    idx = true;
end
try
    load(strcat("TABLES/",GEOM.ROTOR.strNAME));
    idx = true;
end
if ~idx
    error("fcnLOOKUP: Unable to open lookup table file");
end

J_case = STATE.VEL_ROTOR_MAG./((STATE.RPM').*(GEOM.ROTOR.vecDIAM));
CT_Interp = interp3(Angle,J,RPM,CT,STATE.AOA_R,J_case,STATE.RPM');
CP_Interp = interp3(Angle,J,RPM,CP,STATE.AOA_R,J_case,STATE.RPM');
CQ_Interp = interp3(Angle,J,RPM,CQ,STATE.AOA_R,J_case,STATE.RPM');
CNx_Interp = interp3(Angle,J,RPM,CNx,STATE.AOA_R,J_case,STATE.RPM');
CNy_Interp = interp3(Angle,J,RPM,CNy,STATE.AOA_R,J_case,STATE.RPM');
CMx_Interp = interp3(Angle,J,RPM,CMx,STATE.AOA_R,J_case,STATE.RPM');
CMy_Interp = interp3(Angle,J,RPM,CMy,STATE.AOA_R,J_case,STATE.RPM');

%% Dimensionalize variables
R = GEOM.ROTOR.vecDIAM/2;
omega = 2*pi*(STATE.RPM'/60);
PERF.ROTOR.T = (CT_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.Nx = (CNx_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.Ny = (CNy_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^2))';
PERF.ROTOR.P = (CP_Interp.*(AIR.density).*pi.*(R.^2).*((omega.*R).^3))';
PERF.ROTOR.Q = (CQ_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';
PERF.ROTOR.Mx = (CMx_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';
PERF.ROTOR.My = (CMy_Interp.*(AIR.density).*pi.*(R.^2).*((omega.^2).*(R.^3)))';


end

