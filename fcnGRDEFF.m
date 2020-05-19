function PERF = fcnGRDEFF(GEOM,PERF,STATE)
%fcnGRDEFF calculated the change in thrust due to ground effect of a 
%   multirotor vehicle.
%   
%       This function is only run if any of the rotors are a  distance of 
%   3 rotor diameters above the ground or lower. The ground effect thrust 
%   ratio is calculated from the results of P. Snachez-Cuevas et. al, 
%   doi: 10.1155/2017/1823056. This funcation also assumes an emperical
%   body lift coefficinet of 2 by default as a better value is unknown.
%
%   INPUTS:
%       GEOM.ROTOR.vecDIAM      - Rotor diameters
%       GEOM.ROTOR.matLOCATION  - Location of rotors
%       PERF.ROTOR.T            - Rotor thrust
%       STATE.POS               - Rotor position (x,y,z)
%
%   OUTPUTS:
%       PERF.ROTOR.T            - The updated thrust after multiplying by
%                                   the ground effect thrust ratio



% If all rotors are more than 3*D above the ground than skip this function
if ~any(3*GEOM.ROTOR.vecDIAM > STATE.POS(end,3))
    return
else
    % Ground effect based on:
    %P. Snachez-Cuevas, G. Heredia, and A. Ollero, "Characterization of the
    % aerodynamic ground effect and its influence in multirotor control,"
    % International Journal of Aerospace Engineering, 2017, doi
    % 10.1155/2017/1823056
    
    Kb = 2; % emperical  body lift coefficient, 2 by default
    z = STATE.POS(end,3); % Z distance from ground
    R = GEOM.ROTOR.vecDIAM./2; % Rotor radius
    d = sqrt(GEOM.ROTOR.matLOCATION(:,1).^2+GEOM.ROTOR.matLOCATION(:,2).^2+GEOM.ROTOR.matLOCATION(:,3).^2); % Distance of each rotor to geometric center
    b =  d*2;% Distance to diagonal rotor
    
    % Calculate thrust ratio
    temp1 = (R./(4.*z)).^2;
    temp2 = (R.^2).*(z./(sqrt((d.^2+4*(z.^2)).^3)));
    temp3 = ((R.^2)./2).*(z./(sqrt((2*(d.^2)+4*(z.^2)).^3)));
    temp4 = 2.*(R.^2).*(z./(sqrt((b.^2+4*(z.^2)).^3)));
    t_ratio = 1./(1-temp1 - temp2 - temp3 - temp4*Kb);
    
    % Apply ground effect thrust ratio
    PERF.ROTOR.T = PERF.ROTOR.T.*t_ratio';
    
end

end

