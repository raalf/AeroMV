function [RPM_Multiplier] = fcnRPMMULTIPLIER(filename,RPM_CT,RPM_Hover,OVERWRITE)
% Calculate the theoretical CG
% The current rotor number for the dji matrice:
% Front left:  2
% Back left:   3
% Back right:  4
% Front right: 1
%
% Assumed rotor number order
%   2   1
%   3   4
%
% Set of equations
% mat*[T1 T2 T3 T4]' = [W 0 0 0]'
% T1+T2+T3+T4 = W   --> Thrust = W
% T2*r2 + T3*r3 = T1*r1 + T4*r4  --> No roll
% T1*r1 + T2*r2 = T3*r3 + T4*r4  --> No pitch
% Q1+Q2+Q3+Q4 = 0   --> No yaw

% For pelican:
% 1 4
% 3 2
% T1*r1 + T3*r3 = T2*r2 + T4*r4  --> No roll
% T1*r1 + T4*r4 = T2*r2 + T3*r3  --> No pitch

% Where Q is:
% Q = CQ*rho*A*Omega^2*R^3
% Omega = sqrt(T/(CT*A))*R
% Q = CQ1*rho*A*(sqrt(T1/(CT1*A))*R)^2*R^3
% Q = CQ1*rho*A*(T1/(CT1*A))*R^2*R^3
% Q = CQ1*rho*(T1/(CT1))*R^5

% Get input data
[~, GEOM, AIR] = fcnINPUT(filename);

if exist('OVERWRITE','var')
    if ~isempty(OVERWRITE)
        [AIR,GEOM,~,~] = fcnOVERWRITE(OVERWRITE,AIR,GEOM,[],[]);
    end
end
%% Load in rotor data
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

%% Calculate distance from rotors to CG
r = GEOM.ROTOR.matLOCATION-GEOM.VEH.vecCG;
r = sqrt(sum(r.^2,2));

%% Calculate Q values
idx = RPM == RPM_CT & J == 0;
CT_hover = CT(idx);
CQ_hover = CQ(idx);
Q = (CQ_hover(1)*AIR.density*(GEOM.ROTOR.vecDIAM/2).^5)./CT_hover(1);
Q = Q.*GEOM.ROTOR.matROT';

%% Solve for thrust values
% Matrice
% mat = [1 1 1 1;
%     -r(1) r(2) r(3) -r(4);
%     r(1) r(2) -r(3) -r(4);
%     Q(1) Q(2) Q(3) Q(4)];

% Pelican:
mat = [1 1 1 1;
    r(1) -r(2) r(3) -r(4);
    r(1) -r(2) -r(3) r(4);
    Q(1) Q(2) Q(3) Q(4)];
% T1*r1 + T3*r3 = T2*r2 + T4*r4  --> No roll
% T1*r1 + T4*r4 = T2*r2 + T3*r3  --> No pitch

rhs = [GEOM.VEH.valMASS*9.81 0 0 0]';

HoverT = mat\rhs;

%% Calculate hover rpm
% omega = (1./(GEOM.ROTOR.vecDIAM/2)).*sqrt(HoverT./(CT_hover(1).*AIR.density.*pi.*(GEOM.ROTOR.vecDIAM./2).^2));
% HoverRPM = omega.*60/(2*pi);

% ***Test case to compare, RPM Multiplier when only assuming 4*T = W
omega = (1./(GEOM.ROTOR.vecDIAM/2)).*sqrt((GEOM.VEH.valMASS*9.81/4)./(CT_hover(1).*AIR.density.*pi.*(GEOM.ROTOR.vecDIAM./2).^2));
HoverRPM = omega.*60/(2*pi);

%% Calculate hover rpm multiplier
if ~isreal(HoverRPM)  
     RPM_Multiplier = [1 1 1 1]';
else
    RPM_Multiplier = HoverRPM./RPM_Hover;
end
