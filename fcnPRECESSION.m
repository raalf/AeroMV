function [OUTP] = fcnPRECESSION(GEOM, STATE, OUTP)
%fcnPRECESSION is used to calculate the total vehicle rates due to
%   gyroscopic precesion of the spinning rotors.
%
% INPUTS:
%       GEOM.ROTOR.vecRMASS     - Rotor mass, vec or single value (kg) 
%       GEOM.ROTOR.vecDIAM      - Rotor diameter, vec or single value (m)
%       GEOM.ROTOR.matLOCATION  - Rotor locations relative to CG (m)
%       STATE.RPM               - Rotor rotational speed (RPM)
%       OUTP.M_r                - Rotor hub moments (Nm)
%       OUTP.F_r                - Rotor hub forces (N)
%
% OUTPUTS:
%       OUTP.PREC_RATE          - Precession rate vector for each rotor (Rad/s)
%
%
% D.F.B. in Toronto Canada, AUGUST 2020

% Calculate moment of inertia of the blades. This assumes the rotors to
% have a constant mass along its radius. 
I_r = 1/12*(GEOM.ROTOR.vecRMASS.*(GEOM.ROTOR.vecDIAM.^2));


% Calculate the precession rate (ie the anglular velocity due to precession)
OUTP.PREC_RATE = (OUTP.M_B./(I_r.*(STATE.RPM'.*2.*pi)/60)).*GEOM.ROTOR.matROT';
%******NOTE: NEED TO WORK ON CORRECT DIRECTIONS!

% For reference, this is incorrect because must use the total torque at
% each rotor, which is the same as the total torque acting on the vehicle
% M_rotor = OUTP.M_r + cross(GEOM.ROTOR.matLOCATION,OUTP.F_r); WRONG!!
%OUTP.PREC_RATE = M_rotor./(I_r.*(STATE.RPM'.*2.*pi)/60); WRONG!!

end

