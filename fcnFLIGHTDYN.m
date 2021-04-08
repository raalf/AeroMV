function [OUTP] = fcnFLIGHTDYN(GEOM, STATE, OUTP)
%fcnFLIGHTDYN Used to calculate the vehicle response from the calculate
%   vehcle loads. The results predict the motion in the next time step by
%   calculating the new accelerations based on the predictions of the new
%   motor commands.
%
% The main equations being solves are:
%       m*ddot(xi) =R*(F_B)
%       I_B * dot(omega_B) = -omega_B x I_B*omega_B + M_B
%
% INPUTS:
%   STATE.accuracy      - Order of accuracy for backwards differencing
%                           NOTE: if not input, it is assumed to be = 1
%
% OUTPUTS:
%   OUTP.xi_ddot        - Positional accelerations (global ref)
%   OUTP.VEL_NEW        - New positional velocities (global ref)
%   OUTP.POS_NEW        - New position (global ref)
%   OUTP.OMEGA_DOT_B    - Angular accelerations (body ref)
%   OUTP.OMEGA_NEW_B    - Angular velocities (body ref)
%   OUTP.OMEGA_NEW      - Angular velocities (gloabl ref)
%   OUTP.EULER_NEW      - New Euler angles


% Order of accuracy for backwards differencing
% If not input in STATE structure, assume accuracy = 1
if ~fcnCOMPCHECK(STATE, 'accuracy')
    STATE.accuracy = 3;
end  

% Calculate the precession rate
[OUTP] = fcnPRECESSION(GEOM, STATE, OUTP);

h = 1/(STATE.FREQ); % Grid spacing size, ie delta time

% Calculate rotation matrix
R = fcnEUL2R(STATE.EULER(end,:),3,0);

% Calculate vehicle positional accelerations
OUTP.xi_ddot =  (R*OUTP.F_B'/GEOM.VEH.valMASS);

% Calculate the new velocity and position
OUTP.VEL_NEW = fcnBCKWRDDIFF(h,OUTP.xi_ddot',STATE.VEL,STATE.accuracy)';
OUTP.POS_NEW = fcnBCKWRDDIFF(h,OUTP.VEL_NEW',STATE.POS,STATE.accuracy)';

% Calculate vehicle rotational accelerations
OUTP.OMEGA_DOT_B = GEOM.VEH.I\(OUTP.M_B + cross(-1*STATE.BODY_RATES(end,:)',STATE.BODY_RATES(end,:)*GEOM.VEH.I))';
% OUTP.OMEGA_B = GEOM.VEH.I\(cross(-1*STATE.BODY_RATES',STATE.BODY_RATES*GEOM.VEH.I))';

% Calculate the new angular velocities and euler angles
R = fcnEUL2R(STATE.EULER,4,0);

OUTP.OMEGA_NEW_B = fcnBCKWRDDIFF(h,OUTP.OMEGA_DOT_B',STATE.BODY_RATES,STATE.accuracy)'; % Body frame

% Add Precession rate
% OUTP.OMEGA_NEW_B = sum(OUTP.PREC_VEH_RATE,1)'+OUTP.OMEGA_NEW_B;

OUTP.OMEGA_NEW = R\OUTP.OMEGA_NEW_B; % Inertial frame
OUTP.EULER_NEW = fcnBCKWRDDIFF(h,OUTP.OMEGA_NEW',STATE.EULER,STATE.accuracy)'; % Inertial frame

end