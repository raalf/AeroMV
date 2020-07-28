function [vecBODYFRAME] = fcnROTREF2BODY(GEOM, vecROTFRAME)
%% fcnROTREF2BODY converts vectors from the rotor frame to the body frame
% The equation: Body_ref = R * Rotor_ref is used to convert var between
% rotor and body reference frames. R is calculated by assigned Body_ref as
% the rotor normals in body ref and Rotor_ref as [0 0 1].
%
% INPUTS:
%   GEOM structure used for matNORMALS.
%   vecROTFRAME - The vectors in the rotor frame which must be converted
%                   Size must be: (num_Rotors, 3)
%
% OUTPUTS:
%   vecBODYFRAME - The vecROTFRAME converted to the body reference frame
%
% D.F.B. in Toronto Canada, JULY 2020 (COVID-19 lockdown)


%% Check if there were rotor normals input 
% If the rotor normals are not input, it is assumed that the forces are
% allgined with the body frame and therefore this function is not used
if ~fcnCOMPCHECK(GEOM.ROTOR, 'matNORMALS')
    vecBODYFRAME = vecROTFRAME;
    return
end

%% Iterate through each of the rotors
vecBODYFRAME = zeros(size(GEOM.ROTOR.matLOCATION,1),3); % Pre-allocate
for i = 1:size(GEOM.ROTOR.matLOCATION,1)
    
    % Calculate a rotation matrix
    % This is calculated by aligning the rotor normals in body frame with
    % the normals in references frame. Ie, matNORMALS with [0 0 1]
    % respectively.
    R = GEOM.ROTOR.matNORMALS(i,:)'/[0 0 1]';
    
    % Convert vecROTFRAME to body reference frame
    vecBODYFRAME(i,:) = (R * vecROTFRAME(i,:)')';
end

end

