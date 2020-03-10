function [OUTP] = fcnFORCES(PERF, GEOM, STATE)
% This function is used to sum the total vehicle forces and moment in the
% body frame.

%% Sum Force in Body Frame
COMP_DRAG_TOTAL = [];
COMP_LIFT_TOTAL = [];
e_L = [];
r = [];

if GEOM.VEH.idxBODY == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.BODY.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.BODY.vecLIFT];
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.BODY,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.BODY)];
end
if GEOM.VEH.idxARM == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.ARM.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.ARM.vecLIFT]; 
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.ARM,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.ARM)];
end
if GEOM.VEH.idxLEG == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.LEG.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.LEG.vecLIFT];  
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.LEG,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.LEG)];
end
if GEOM.VEH.idxPAYLOAD == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.PAYLOAD.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.PAYLOAD.vecLIFT]; 
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.PAYLOAD,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.PAYLOAD)];
end
if GEOM.VEH.idxMOTOR == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.MOTOR.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.MOTOR.vecLIFT];  
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.MOTOR,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.MOTOR)];
end

OUTP.COMP_DRAG_TOTAL = COMP_DRAG_TOTAL;
OUTP.COMP_LIFT_TOTAL = COMP_LIFT_TOTAL;

% Rotor forces
thrust = [PERF.ROTOR.T]';
Fx_R = [PERF.ROTOR.Nx]';
Fy_R = [PERF.ROTOR.Ny]'.*(GEOM.ROTOR.matROT');

Fx = Fx_R.*cos(STATE.BETA)-Fy_R.*sin(STATE.BETA);
Fy = Fx_R.*sin(STATE.BETA)+Fy_R.*cos(STATE.BETA);

OUTP.F_r = [Fx, Fy, thrust];
%OUTP.F_r = [0*Fx, 0*Fy, thrust];


% Calculate rotation matrix
R = fcnEUL2R(STATE.EULER,3,0);

e_D = (R'*[0 0 -1]')';
e_V = (R'*(STATE.VEL_B/STATE.VEL_MAG)')';

OUTP.F_B = sum(OUTP.F_r,1) + sum(OUTP.COMP_DRAG_TOTAL*e_V) + sum(OUTP.COMP_LIFT_TOTAL.*e_L) + e_D*GEOM.VEH.valMASS*9.81;

%% Sum the moments in the body frame
M_comp = cross(r,(OUTP.COMP_LIFT_TOTAL.*e_L + OUTP.COMP_DRAG_TOTAL*e_V));


% Rotor Moments
Q = [PERF.ROTOR.Q]'.*(GEOM.ROTOR.matROT');
Mx_R = [PERF.ROTOR.Mx]'.*(GEOM.ROTOR.matROT');
My_R = [PERF.ROTOR.My]';

Mx = Mx_R.*cos(STATE.BETA)-My_R.*sin(STATE.BETA);
My = Mx_R.*sin(STATE.BETA)+My_R.*cos(STATE.BETA);

OUTP.M_r = [Mx, My, Q];
%OUTP.M_r = [0*Mx, 0*My, Q];

M_rotor = OUTP.M_r + cross(GEOM.ROTOR.matLOCATION,OUTP.F_r);

OUTP.M_B = sum(M_rotor) + sum(M_comp);
