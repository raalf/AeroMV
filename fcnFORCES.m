function [OUTP] = fcnFORCES(PERF, GEOM, STATE, idxVEHPERF)
% This function is used to sum the total vehicle forces and moments in the
% body frame.
%
% OUTPUTS:
%   OUTP.COMP_DRAG_TOTAL    - Drag acting on the components
%   OUTP.COMP_LIFT_TOTAL    - Lift acting on the comonents;
%   OUTP.F_r                - Rotor forces
%   OUTP.F_B                - Total force in body ref
%   OUTP.M_comp             - Moment due to component forces
%   OUTP.M_r                - Rotor moments (not including r x F_r)
%   OUTP.M_B                - Total vehicle moments in body frame


%% Sum Force in Body Frame
% Initialize temp variables
COMP_DRAG_TOTAL = []; 
COMP_LIFT_TOTAL = [];
e_L = [];
r = [];

% Create vectors of the total lift and total drag action on each component
% Check and add body information
if idxVEHPERF ~= 0
if GEOM.VEH.idxBODY == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.BODY.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.BODY.vecLIFT];
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.BODY,STATE.VEL_B)]; % Get lift directions
    r = [r; fcnMOMARM(GEOM.VEH.BODY)]; % Get moment arm
end
% Check and add arm information
if GEOM.VEH.idxARM == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.ARM.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.ARM.vecLIFT]; 
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.ARM,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.ARM)];
end
% Check and add leg information
if GEOM.VEH.idxLEG == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.LEG.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.LEG.vecLIFT];  
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.LEG,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.LEG)];
end
% Check and add payload information
if GEOM.VEH.idxPAYLOAD == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.PAYLOAD.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.PAYLOAD.vecLIFT]; 
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.PAYLOAD,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.PAYLOAD)];
end
% Check and add motor information
if GEOM.VEH.idxMOTOR == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.MOTOR.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.MOTOR.vecLIFT];  
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.MOTOR,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.MOTOR)];
end
% Check and add other component information
if GEOM.VEH.idxOTHER == 1
    COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.OTHER.vecDRAG];
    COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.OTHER.vecLIFT];  
    e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.OTHER,STATE.VEL_B)];
    r = [r; fcnMOMARM(GEOM.VEH.OTHER)];
end

% Save component force data to OUTP
OUTP.COMP_DRAG_TOTAL = COMP_DRAG_TOTAL;
OUTP.COMP_LIFT_TOTAL = COMP_LIFT_TOTAL;
else
   OUTP.COMP_DRAG_TOTAL = [0 0 0];
   OUTP.COMP_LIFT_TOTAL = [0 0 0];
   e_L = [0 0 0];
   r = [0 0 1];
end
% Rotor forces
thrust = [PERF.ROTOR.T]';
Fx_R = [PERF.ROTOR.Nx]';
Fy_R = [PERF.ROTOR.Ny]'.*(GEOM.ROTOR.matROT'); % Change force direction based on rotation direction

% Apply rotation matrix to Fx and Fy based on freestream direction
Fx = Fx_R.*cos(STATE.BETA)-Fy_R.*sin(STATE.BETA);
Fy = Fx_R.*sin(STATE.BETA)+Fy_R.*cos(STATE.BETA);

% Save rotor forces to OUTP in the local rotor reference frame
OUTP.F_r = [Fx, Fy, thrust];
%OUTP.F_r = [0*Fx, 0*Fy, thrust];

% Convert the rotor forces from rotor reference frame to body frame
[OUTP.F_r] = fcnROTREF2BODY(GEOM, OUTP.F_r);

% Calculate rotation matrix
R = fcnEUL2R(STATE.EULER(end,:),3,0);

% Calculate down direction and velocity direction (unit vectors)
e_D = (R'*[0 0 -1]')';
e_V = (R'*(STATE.VEL_B/STATE.VEL_MAG)')';

% Adjusting for any 0 drag or lift values
if isnan(e_V)
    e_V = [0 0 0];
end

OUTP.F_comp = sum(OUTP.COMP_DRAG_TOTAL.*e_V,1) + ...
    sum(OUTP.COMP_LIFT_TOTAL.*e_L,1);
% Calcuate total forces in body reference frame (as a vector)
% rotor forces + component drag + component lift + mg
OUTP.F_B = sum(OUTP.F_r,1) + OUTP.F_comp + e_D*GEOM.VEH.valMASS*9.81;


%% Sum the moments in the body frame
% Calculate moments due to forces on vehicle components
OUTP.M_comp = cross(r,(OUTP.COMP_LIFT_TOTAL.*e_L + OUTP.COMP_DRAG_TOTAL.*e_V));

% Rotor Moments
Q = [PERF.ROTOR.Q]'.*(GEOM.ROTOR.matROT'); % Sign based on rotation direction
Mx_R = [PERF.ROTOR.Mx]'.*(GEOM.ROTOR.matROT'); % Sign based on rotation direction
My_R = [PERF.ROTOR.My]';

% Rotate based on freestream direction
Mx = Mx_R.*cos(STATE.BETA)-My_R.*sin(STATE.BETA);
My = Mx_R.*sin(STATE.BETA)+My_R.*cos(STATE.BETA);

% Save rotor moments to OUTP as a vector in rotor reference frame
OUTP.M_r_hub = [Mx, My, Q];
%OUTP.M_r = [0*Mx, 0*My, Q];

% Convert the rotor moments from rotor reference frame to body frame
[OUTP.M_r_hub] = fcnROTREF2BODY(GEOM, OUTP.M_r_hub);

% Calculate total vehcile moments due to rotors. M_r + r x F_r
OUTP.M_r_total = OUTP.M_r_hub + cross(GEOM.ROTOR.matLOCATION,OUTP.F_r);

% Calculate total moments in the body reference frame
OUTP.M_B = sum(OUTP.M_r_total) + sum(OUTP.M_comp);
