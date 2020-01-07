function [r] = fcnMOMARM(INFO)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if strcmpi(INFO.strTYPE,'Sphere') || strcmpi(INFO.strTYPE,'Ellipsoid')
    r = INFO.vecLOCATION;
    
elseif strcmpi(INFO.strTYPE,'Cylinder')
    r = (INFO.matEND+INFO.matBEGIN)/2;
end
end

