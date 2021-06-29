function [COND, VISC, INPU, VEHI, WAKE, SURF, OUTP, MISC] = fcnVAPINIT(FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP, MISC)
%fcnVAPINIT Run the initialization of the VAP run including discritizing
%geometry into DVEs, calculating D-matrix, inital resulting vector and wing
%coefficients
%
% This is based on the format by M. M. in the VAP3 FLEX branch

%% Discretizing geometry into DVEs
% Adding collective pitch to the propeller/rotor
if ~isempty(COND.vecCOLLECTIVE)
    INPU.matGEOM(:,5,INPU.vecPANELROTOR > 0) = INPU.matGEOM(:,5,INPU.vecPANELROTOR > 0) + repmat(reshape(COND.vecCOLLECTIVE(INPU.vecPANELROTOR(INPU.vecPANELROTOR > 0), 1),1,1,[]),2,1,1);
end
[INPU, COND, MISC, VISC, WAKE, VEHI, SURF, OUTP] = fcnGEOM2DVE(INPU, COND, VISC, VEHI, WAKE, OUTP, SURF, MISC);

%% Advance Ratio
for jj = 1:length(COND.vecROTORRPM)
    MISC.vecROTORJ(jj) = (COND.vecVEHVINF(VEHI.vecROTORVEH(jj))*60)./(abs(COND.vecROTORRPM(jj)).*INPU.vecROTDIAM(jj));
end

%% Add boundary conditions to D-Matrix
[SURF.matD] = fcnDWING(SURF, INPU);

%% Add kinematic conditions to D-Matrix
[SURF.vecK] = fcnSINGFCT(SURF.valNELE, SURF.vecDVESURFACE, SURF.vecDVETIP, SURF.vecDVEHVSPN);
[SURF.matD] = fcnKINCON(SURF.matD, SURF, INPU, FLAG);

%% Preparing to timestep
% Building wing resultant
[vecR] = fcnRWING(0, SURF, WAKE, FLAG);

% Solving for wing coefficients
[SURF.matCOEFF] = fcnSOLVED(SURF.matD, vecR, SURF.valNELE);
SURF.matNPDVE = SURF.matDVE;
