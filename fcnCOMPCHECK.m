function idx = fcnCOMPCHECK(structCOMP, strCOMP)
% This function checks if there is a vehcile component specified in input
% but searching through a structure for a non-empty field
%
% INPUTS:
%   structCOMP - The overall strucuture that is to be searched through
%   strCOMP - A string of the strucuture field of interst (usually the
%               component of interest)
% OUTPUT: 
%   idx - A true/false flag, true: if the field exists and is not empty 
%

% Example: 

if any(ismember(fields(structCOMP),strCOMP)) == 1
    if isempty(structCOMP.(strCOMP)) == 0
        idx = true;
    else    
        idx = false;
    end 
else
    idx = false;
end
