function PERF = fcnGRDEFF(GEOM,PERF,STATE)
% Ground effect

% If all rotors are more than 3*D above the ground than skip this function
if ~any(3*GEOM.ROTOR.vecDIAM > STATE.POS(end,3))
    return
else
    % Ground effect based on:
    %P. Snachez-Cuevas, G. Heredia, and A. Ollero, "Characterization of the
    % aerodynamic ground effect and its influence in multirotor control,"
    % International Journal of Aerospace Engineering, 2017, dio
    % 10.1155/2017/1823056
    
    Kb = 2; % emperical  body lift coefficient, 2 by default
    z = STATE.POS(end,3);
    R = GEOM.ROTOR.vecDIAM./2;
    d = sqrt(GEOM.ROTOR.matLOCATION(:,1).^2+GEOM.ROTOR.matLOCATION(:,2).^2+GEOM.ROTOR.matLOCATION(:,3).^2); % Distance of each rotor to geometric center
    b =  d*2;% Distance to diagonal rotor
    
    temp1 = (R./(4.*z)).^2;
    temp2 = (R.^2).*(z./(sqrt((d.^2+4*(z.^2)).^3)));
    temp3 = ((R.^2)./2).*(z./(sqrt((2*(d.^2)+4*(z.^2)).^3)));
    temp4 = 2.*(R.^2).*(z./(sqrt((b.^2+4*(z.^2)).^3)));
    t_ratio = 1./(1-temp1 - temp2 - temp3 - temp4*Kb);
    
    
    PERF.ROTOR.T = PERF.ROTOR.T.*t_ratio';
    
end

end

