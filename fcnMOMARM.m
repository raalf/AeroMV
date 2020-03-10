function [r] = fcnMOMARM(INFO)
% This function calculate the moment arm of a component. This assumed that
%   the input is relative to the CG location 
%
% INPUT:
%   INFO    - Strucutre with component info. Usually GEOM.VEH.***
%
% OUTPUT:
%   r       - Moment arm (m)


%% Calculate moment arm
% Check if component is a sphere or ellipsoid. 
if strcmpi(INFO.strTYPE,'Sphere') || strcmpi(INFO.strTYPE,'Ellipsoid')
    % Because the center is calculate with respect to CG, the moment arm 
    %   is simply the component locations
  
    r = INFO.vecLOCATION;
    
elseif strcmpi(INFO.strTYPE,'Cylinder')
    % If the component is a cylinder, assume the force acts a the center 
    %   point of the component. 
    r = (INFO.matEND+INFO.matBEGIN)/2;
end

