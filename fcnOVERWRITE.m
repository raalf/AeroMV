function [AIR,GEOM,TABLE,STATE] = fcnOVERWRITE(OVERWRITE,AIR,GEOM,TABLE,STATE)
%fcnOVERWRITE overwrites any variables from the input variables using the
%   values in the OVERWRITE structure. The inputs of the structures of
%   AIR, GEOM, TABLE and STATE can be updated.
%
% INPUTS:
%   OVERWRITE   - A structure with variables the must be changed from the
%           input values. An example would be:
%                   OVERWRITE.GEOM.VEH.vecCG = [0 0 0];
%                   OVERWRITE.GEOM.VEH.ARM.valLENGTH = 0.3;
%                   OVERWRITE.GEOM.ROTOR.valNUMB = 3;
%                   OVERWRITE.AIR.density = 1.5;
%   AIR, GEOM, TABLE, STATE - These are the input variables that can be
%   overwriten by the variables in the OVERWRITE structure.
%
% OUTPUTS:
%   AIR, GEOM, TABLE, STATE structures with the updated OVERWRITE values
%
% D.F.B. in Toronto Canada, MAY 2020 (COVID-19 lockdown)


% Go through each input structure and update them using the nested function
% fcnUPDATESTRUCTVAR
if fcnCOMPCHECK(OVERWRITE, 'GEOM')
    GEOM = fcnUPDATESTRUCTVAR(OVERWRITE.GEOM,GEOM);
end
if fcnCOMPCHECK(OVERWRITE, 'AIR')
    AIR = fcnUPDATESTRUCTVAR(OVERWRITE.AIR,AIR);
end
if fcnCOMPCHECK(OVERWRITE, 'TABLE')
    TABLE = fcnUPDATESTRUCTVAR(OVERWRITE.TABLE,TABLE);
end
if fcnCOMPCHECK(OVERWRITE, 'STATE')
    STATE = fcnUPDATESTRUCTVAR(OVERWRITE.STATE,STATE);
end

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
        
        
        % The general format is:
        %   - read structure field names and iterate through them
        %   - check if fields are structure
        %       - if not a structure, apply the values to original struct
        %       - if it is a structure repeat the above step
        %   - Continue looking deeping into the structure until all values
        %   have been updated or structORIGINAL.var.var.var is reached
        %   and is still another nested structure
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
                                        warning('Structure to OVERWRITE has more than 4 nested structures and cannot be used. Var.var.var.var.var is currently not possible. See fcnOVERWRITE')
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
    end
end