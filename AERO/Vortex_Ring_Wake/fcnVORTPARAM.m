function [VORTPARAM] = fcnVORTPARAM(N_b,RPM,V_inf,T,rho,R)
% fcnCREATEGEOM this function calculates the params of the vortex seg



%% Calculate vortex ring parameters
% Calculate circulation
VORTPARAM.circ = 2*T./(rho*R^2*N_b*(RPM/60)*pi);

% Momentum theory
mom = 2*T./(rho.*pi.*(R.^2));

% Z offset
VORTPARAM.z = (1./(2*N_b.*(RPM/60))).*(V_inf(:,3)+sqrt(V_inf(:,3).^2+mom));

% Skew angle
V_xy = sqrt(V_inf(:,1).^2+V_inf(:,2).^2);
VORTPARAM.chi = atan(V_xy./(0.5*V_inf(:,3)+sqrt(V_inf(:,3).^2+mom)));

end

