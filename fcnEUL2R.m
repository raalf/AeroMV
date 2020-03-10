function [R] = fcnEUL2R(Euler, varargin)
%fcnEUL2R Create Rotation matrix using Euler angles. 
%   R = fcnEUL2R(Euler) Creates rotation matrix, R, using Euler angle. 
%   Euler angle order is assumed to be yaw, pitch, roll and angles in rad
%   The rotation matrix converts from the body frame to the inertial frame
%
%   R = fcnEUL2R(Euler,Order,idxDEG) to specify the order and angle units
%
%   Order == 1 when euler angles given as psi, theta, phi (default)
%   Order == 2 when euler angles given as phi, theta, psi
%   Order == 3 when using transformation given by Nima's thesis
%   Order == 4 ANGULAR RATES transformation
%   Order == 5 ANGULAR RATES from Nima's thesis
%   Order == 6 from eul2rotm matlab function (same result as 3)
%
%   idxDEG == 0 when angles are given in rad (default)
%   idxDEG == 1 when angles are given in deg 


% Check for Order or idxDEG
if isempty(varargin)
    Order = 1;
    idxDEG = 0;
elseif numel(varargin) == 1
    Order = varargin{1};
    idxDEG = 0;
elseif numel(varargin) == 2
    Order = varargin{1};
    idxDEG = varargin{2};
end

% Convert from rad to deg
if idxDEG == 1
    Euler = (pi/180).*Euler;
end

phi = Euler(1);
theta = Euler(2);
psi = Euler(3);
if Order == 1 % Apply rotation matrix for psi, theta, phi

    R = [cos(theta)*cos(psi) sin(psi)*sin(theta) -sin(theta);
        cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) ...
        sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) cos(theta)*sin(phi);
        cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
        sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi) cos(theta)*cos(psi)];
    
elseif Order == 2 % Apply rotation matrix for phi, theta, psi 

    R = [cos(phi)*cos(psi)-cos(theta)*sin(phi)*sin(psi) ...
        -cos(psi)*sin(phi)-cos(phi)*cos(theta)*sin(psi) sin(theta)*sin(psi);
        cos(theta)*cos(psi)*sin(phi)+cos(phi)*sin(psi) ...
        cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi) -cos(psi)*sin(theta);
        sin(phi)*sin(theta) cos(phi)*sin(theta) cos(theta)];
    
elseif Order == 3 % From Nima's thesis
    
    R = [cos(theta)*cos(psi) cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi)...
        cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi);
        cos(theta)*sin(psi) sin(psi)*sin(theta)*sin(phi)+cos(phi)*cos(psi)...
        cos(phi)*sin(psi)*sin(theta)-sin(phi)*cos(psi);
        -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
    
elseif Order == 4 % From Pelican dataset documentation for angular rates
    R = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi)*sec(theta) cos(phi)*sec(theta)];

elseif Order == 5 % From Nima's thesis with angular rates
    R = [1 0 -sin(theta);
        0 cos(phi) -sin(phi)*cos(theta);
        0 -sin(phi) cos(phi)*cos(theta)];
    
elseif Order == 6 % From matlab robotics toolbox fcn eul2rotm 
    % Note: Yields identical results to Nima's thesis
    R = [cos(theta)*cos(psi) sin(theta)*sin(phi)*cos(psi)-sin(psi)*cos(phi) ...
        sin(theta)*cos(phi)*cos(psi)+sin(psi)*sin(phi);
        cos(theta)*sin(psi) sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(phi) ...
        sin(theta)*cos(phi)*sin(psi)-cos(psi)*sin(phi);
        -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
end
