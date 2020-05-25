function [structORIGINAL] = fcnUPDATESTRUCTVAR(structNEWVAR,structORIGINAL)
%fcnOVRWRTSTRUCTVAR overwrites the variables in structORIGINAL with the
%   values that are in structNEWVAR. This function will only overwrite the
%   variables in structNEWVAR and will leave the rest of the variables in
%   structORIGINAL the same.
%
%   NOTE: This can only handle a max of 4 nested structures. 
%       i.e: Var.var.var.var is the deepest this function can overwrite
%
% INPUTS:
%   structNEWVAR    - A structure with variables that need to be updated
%   structORIGINAL  - The original structure that must be updated
%
% OUTPUTS;
%   structORIGINAL  - The original structure but with the values from
%                       indicated in structNEWVAR 

names = fieldnames(structNEWVAR);
for i = 1:length(names)
    if ~isstruct(structORIGINAL.(names{i}))
        structORIGINAL.(names{i}) = structNEWVAR.(names{i});
    else
        names2 = fieldnames(structNEWVAR.(names{i}));
        for j = 1:length(names2)
            if ~isstruct(structORIGINAL.(names{i}).(names2{j}))
                structORIGINAL.(names{i}).(names2{j}) = structNEWVAR.(names{i}).(names2{j});
            else
                names3 = fieldnames(structNEWVAR.(names{i}).(names2{j}));
                for k = 1:length(names3)
                    if ~isstruct(structORIGINAL.(names{i}).(names2{j}).(names3{k}))
                        structORIGINAL.(names{i}).(names2{j}).(names3{k}) = structNEWVAR.(names{i}).(names2{j}).(names3{k});
                    else
                        names4 = fieldnames(structNEWVAR.(names{i}).(names2{j}));
                        for m = 1:length(names4)
                            if ~isstruct(structORIGINAL.(names{i}).(names2{j}).(names3{k}.(names4{m})))
                                structORIGINAL.(names{i}).(names2{j}).(names3{k}).(names4{m}) = structNEWVAR.(names{i}).(names2{j}).(names3{k}.(names4{m}));
                            else
                                warning('Structure to OVERWRITE has more than 4 nested structures and cannot be used. Var.var.var.var.var is currently not possible. See fcnOVRWRTSTRUCTVAR')
                            end
                        end
                    end
                end
            end
        end
    end
end

