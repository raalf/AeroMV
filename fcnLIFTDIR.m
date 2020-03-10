function [e_L] = fcnLIFTDIR(INFO,vecVEL)
% This function determines the lift direction of a cylinder
%   If the component is not a cylinder set lift direciton to 0
%
% INPUTS:
%   INFO        - Strucutre with component info. Usually GEOM.VEH.***
%   vecVEL      - Vector of velocity
%
% OUTPUT:
%   e_L         - Unit vector in the direction of lift
%
%
% Example: e_L = fcnLIFTDIR(GEOM.VEH.BODY,STATE.VEL_B)];


%%
% Check if the component is a cylindger
if strcmpi(INFO.strTYPE,'Cylinder')
    numCOMP = size(INFO.matEND,1); % Number of components
    matVEL = repmat(vecVEL,numCOMP,1);
    cyl_dir = INFO.matEND-INFO.matBEGIN; % Get cylinder direction
    
    % Make sure that the cylinders are in the +ve x-dir
    idx = cyl_dir(:,1)>0;
    cyl_dir(idx,:) = INFO.matBEGIN(idx,:)-INFO.matEND(idx,:);
    span_dir = cross(matVEL,cyl_dir);
    % Calculate lift direction
    e_L = cross(matVEL,span_dir);
    % Turn lift direction into a unit vector
    e_L = e_L./sqrt(e_L(:,1).^2+e_L(:,2).^2+e_L(:,3).^2);
    
    % %     % Plot test
    %     figure(1)
    %     clf(1)
    %     hold on
    %     cyl_dir = cyl_dir./sqrt(cyl_dir(:,1).^2+cyl_dir(:,2).^2+cyl_dir(:,3).^2);
    %     span_dir = span_dir./sqrt(span_dir(:,1).^2+span_dir(:,2).^2+span_dir(:,3).^2);
    %     matVEL = matVEL./sqrt(matVEL(:,1).^2+matVEL(:,2).^2+matVEL(:,3).^2);
    %     loc = (INFO.matEND+INFO.matBEGIN)/2;
    %     quiver3(loc(:,1),loc(:,2),loc(:,3),cyl_dir(:,1),cyl_dir(:,2),cyl_dir(:,3),'b');
    %     quiver3(loc(:,1),loc(:,2),loc(:,3),span_dir(:,1),span_dir(:,2),span_dir(:,3),'b');
    %     quiver3(loc(:,1),loc(:,2),loc(:,3),matVEL(:,1),matVEL(:,2),matVEL(:,3),'k');
    %     quiver3(loc(:,1),loc(:,2),loc(:,3),e_L(:,1),e_L(:,2),e_L(:,3),'r');
    %     fcnPLTCOMP(INFO,1)
    %     axis equal
    %     box on
    %     xlabel('x-dir')
    %     ylabel('y-dir')
    %     zlabel('z-dir')
    %     hold off
else
    % Set lift direction to 0 if it is not a cylinder
    e_L = [0 1 0];
end
end

