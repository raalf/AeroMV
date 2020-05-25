function [GEOM] = fcnRELITIVE2CG(GEOM)
%fcnRELCG moves all the vehicle geometry to be relative to the center of
%   gravity. This function only runs when the CG location is not [0 0 0].
%
% INPUTS:
%   GEOM structure which include CG location and all component relative to
%   some geometric point that is not the CG
%
% OUTPUTS:
%   GEOM structure with all component locations relative the the CG
%
% D.F.B. in Toronto Canada, MAY 2020 (COVID-19 lockdown)

% If the CG is at 0,0,0 skip this function
if ~any(GEOM.VEH.vecCG ~= 0)
    return
end

% First update the rotor locations
GEOM.ROTOR.matLOCATION = GEOM.ROTOR.matLOCATION - GEOM.VEH.vecCG;


% Update all the components location using the nexted function fcnMODIFYCG
% with each of the 5 possible component options.
if fcnCOMPCHECK(GEOM.VEH,'BODY')
    [GEOM.VEH.BODY] = fcnMODIFYCG(GEOM.VEH.BODY,GEOM.VEH.vecCG);
end
if fcnCOMPCHECK(GEOM.VEH,'ARM')
    [GEOM.VEH.ARM] = fcnMODIFYCG(GEOM.VEH.ARM,GEOM.VEH.vecCG);
end
if fcnCOMPCHECK(GEOM.VEH,'LEG')
    [GEOM.VEH.LEG] = fcnMODIFYCG(GEOM.VEH.LEG,GEOM.VEH.vecCG);
end
if fcnCOMPCHECK(GEOM.VEH,'PAYLOAD')
    [GEOM.VEH.PAYLOAD] = fcnMODIFYCG(GEOM.VEH.PAYLOAD,GEOM.VEH.vecCG);
end
if fcnCOMPCHECK(GEOM.VEH,'MOTOR')
    [GEOM.VEH.MOTOR] = fcnMODIFYCG(GEOM.VEH.MOTOR,GEOM.VEH.vecCG);
end


    function [infoSTRUCT] = fcnMODIFYCG(infoSTRUCT,vecCG)
        %fcnMODIFYCG is a nested function that substracts vecCG from the
        %variables of infoSTRUCT depending on if it is a Sphere, Ellipsoid
        %or Cylinder.
        %
        % INPUTS:
        %   infoSTRUCT  - Structure that must be updated. For ex.
        %               GEOM.VEH.BODY
        %   vecCG       - Center of gravity [x,y,z], (m)
        %
        % OUTPUTS:
        %   infoSTRUCT  - Updated infoSTRUCT with vecCG substracted
        
        % Check if the component is a sphere or ellipsoid        
        if  strcmpi(infoSTRUCT.strTYPE,'Sphere') || strcmpi(infoSTRUCT.strTYPE,'Ellipsoid')
            % Check if the component is a sphere or ellipsoid  
            infoSTRUCT.vecLOCATION = infoSTRUCT.vecLOCATION - vecCG; 
        
        elseif strcmpi(infoSTRUCT.strTYPE,'Cylinder')
            % Check if the component is a cylinder
            infoSTRUCT.matBEGIN = infoSTRUCT.matBEGIN-vecCG;
            infoSTRUCT.matEND = infoSTRUCT.matEND-vecCG;
        end
    end

end