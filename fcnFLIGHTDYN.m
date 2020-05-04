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
%   
% OUTPUTS:
%   OUTP.xi_ddot       - Positional accelerations (global ref)
%   OUTP.VEL_NEW        - New positional velocities (global ref)
%   OUTP.POS_NEW        - New position (global ref)
%   OUTP.OMEGA_DOT_B    - Angular accelerations (body ref)
%   OUTP.OMEGA_NEW_B    - Angular velocities (body ref)
%   OUTP.OMEGA_NEW      - Angular velocities (gloabl ref)
%   OUTP.EULER_NEW      - New Euler angles


% % Calculate rotation matrix
% R = fcnEUL2R(STATE.EULER(end,:),3,0);
% 
% % Calculate vehicle positional accelerations
% OUTP.xi_ddot =  (R*OUTP.F_B'/GEOM.VEH.valMASS);
% 
% % Calculate the new velocity and position
% OUTP.VEL_NEW = OUTP.xi_ddot*(1/(STATE.FREQ))+STATE.VEL'; % Inertial frame
% OUTP.POS_NEW = OUTP.VEL_NEW*(1/(STATE.FREQ))+STATE.POS'; % Inertial frame
% 
% 
% % Calculate vehicle rotational accelerations
% OUTP.OMEGA_DOT_B = GEOM.VEH.I\(OUTP.M_B - cross(-1*STATE.BODY_RATES',STATE.BODY_RATES*GEOM.VEH.I))';
% % OUTP.OMEGA_B = GEOM.VEH.I\(cross(-1*STATE.BODY_RATES',STATE.BODY_RATES*GEOM.VEH.I))';
% 
% % Calculate the new angular velocities and euler angles
% R = fcnEUL2R(STATE.EULER,4,0);
% OUTP.OMEGA_NEW_B = (OUTP.OMEGA_DOT_B*(1/(STATE.FREQ))+STATE.BODY_RATES'); % Body frame
% OUTP.OMEGA_NEW = R\OUTP.OMEGA_NEW_B; % Inertial frame
% OUTP.EULER_NEW = (OUTP.OMEGA_NEW)*(1/(STATE.FREQ))+STATE.EULER'; % Inertial frame


%% Third order accuracy, backward finite difference
h = 1/(STATE.FREQ);

% Calculate rotation matrix
R = fcnEUL2R(STATE.EULER(end,:),3,0);

% f'(x0) = (11/6*f(x0)-3*f(x-1)+3/2*f(x-2)-1/3*f(x-3))/h
% f(x0) = (f'(x0)*h+3*f(x-1)-3/2*f(x-2)+1/3*f(x-3))/(11/6)

OUTP.xi_ddot =  (R*OUTP.F_B'/GEOM.VEH.valMASS);

OUTP.VEL_NEW = (OUTP.xi_ddot*h+3*STATE.VEL(end,:)'-(3/2)*STATE.VEL(end-1,:)'+(1/3)*STATE.VEL(end-2,:)')/(11/6);
OUTP.POS_NEW = (OUTP.VEL_NEW*h+3*STATE.POS(end,:)'-(3/2)*STATE.POS(end-1,:)'+(1/3)*STATE.POS(end-2,:)')/(11/6);

OUTP.OMEGA_DOT_B = GEOM.VEH.I\(OUTP.M_B - cross(-1*STATE.BODY_RATES(end,:)',STATE.BODY_RATES(end,:)*GEOM.VEH.I))';

% Calculate the new angular velocities and euler angles
R = fcnEUL2R(STATE.EULER,4,0);
OUTP.OMEGA_NEW_B = (OUTP.OMEGA_DOT_B*h+3*STATE.BODY_RATES(end,:)'-(3/2)*STATE.BODY_RATES(end-1,:)'+(1/3)*STATE.BODY_RATES(end-2,:)')/(11/6);
OUTP.OMEGA_NEW = R\OUTP.OMEGA_NEW_B; % Inertial frame
OUTP.EULER_NEW = (OUTP.OMEGA_NEW*h+3*STATE.EULER(end,:)'-(3/2)*STATE.EULER(end-1,:)'+(1/3)*STATE.EULER(end-2,:)')/(11/6);



end