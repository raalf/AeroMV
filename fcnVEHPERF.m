function [PERF, TABLE, GEOM] = fcnVEHPERF(AIR, TABLE, GEOM, STATE)
% This function computes the aerodynamic loads acting on the individual
% vehicle components

%% Compute fuse body forces
GEOM.VEH.idxBODY = fcnCOMPCHECK(GEOM.VEH,'BODY');
if GEOM.VEH.idxBODY
    GEOM.VEH.BODY.valAOA = STATE.AOA; 
    [PERF.BODY.vecCD, PERF.BODY.vecDRAG, PERF.BODY.vecCL, PERF.BODY.vecLIFT, TABLE] ...
        = fcnVEHFORCE(STATE, GEOM.VEH.BODY, AIR, TABLE);
end

%% Compute arm force
GEOM.VEH.idxARM = fcnCOMPCHECK(GEOM.VEH,'ARM');
if GEOM.VEH.idxARM
    GEOM.VEH.ARM.valAOA = STATE.AOA; 
    [PERF.ARM.vecCD, PERF.ARM.vecDRAG, PERF.ARM.vecCL, PERF.ARM.vecLIFT, TABLE] ...
        = fcnVEHFORCE(STATE, GEOM.VEH.ARM, AIR, TABLE);
end

%% Compute leg forces
GEOM.VEH.idxLEG = fcnCOMPCHECK(GEOM.VEH,'LEG');
if GEOM.VEH.idxLEG
    GEOM.VEH.LEG.valAOA = STATE.AOA; 
    [PERF.LEG.vecCD, PERF.LEG.vecDRAG, PERF.LEG.vecCL, PERF.LEG.vecLIFT, TABLE] ...
        = fcnVEHFORCE(STATE.VEL_MAG, GEOM.VEH.LEG, AIR, TABLE);
end

%% Compute payload forces
GEOM.VEH.idxPAYLOAD = fcnCOMPCHECK(GEOM.VEH,'PAYLOAD');
if GEOM.VEH.idxPAYLOAD
    GEOM.VEH.PAYLOAD.valAOA = STATE.AOA; 
    [PERF.PAYLOAD.vecCD, PERF.PAYLOAD.vecDRAG, PERF.PAYLOAD.vecCL, PERF.PAYLOAD.vecLIFT, TABLE] ...
        = fcnVEHFORCE(STATE, GEOM.VEH.PAYLOAD, AIR, TABLE);
end

%% Compute motor forces
GEOM.VEH.idxMOTOR = fcnCOMPCHECK(GEOM.VEH,'MOTOR');
if GEOM.VEH.idxMOTOR
    GEOM.VEH.MOTOR.valAOA = STATE.AOA; 
    [PERF.MOTOR.vecCD, PERF.MOTOR.vecDRAG, PERF.MOTOR.vecCL, PERF.MOTOR.vecLIFT, TABLE] ...
        = fcnVEHFORCE(STATE, GEOM.VEH.MOTOR, AIR, TABLE);
end