function [PERF] = fcnKOMEGA2(GEOM,AIR,PERF,STATE)
%fcnKOMEGA2 uses a simple K-Omega to calculate the thrust of the rotors.
%   This is a very simplified rotor model that can be used as a comparison
%   to the higher-fidelity models.

%%
% Calculate area and rotation speed (omega)
A = pi*(GEOM.ROTOR.vecDIAM/2).^2;
omega = 2*pi*(STATE.RPM'/60);
% VEL = sqrt(STATE.VEL(end,:).^2);

% Convert coefficients to dimensional terms
num_rot = size(GEOM.ROTOR.matLOCATION,1);
PERF.ROTOR.T = (AIR.density*A*GEOM.KT.*(omega.*GEOM.ROTOR.vecDIAM/2).^2)';
PERF.ROTOR.Nx = zeros(1,num_rot);
PERF.ROTOR.Ny = zeros(1,num_rot);
PERF.ROTOR.Q = (AIR.density.*A.*GEOM.KQ.*(omega).^2.*(GEOM.ROTOR.vecDIAM/2).^3)';
PERF.ROTOR.P = omega'.*PERF.ROTOR.Q;
PERF.ROTOR.Mx = zeros(1,num_rot);
PERF.ROTOR.My = zeros(1,num_rot);
end

