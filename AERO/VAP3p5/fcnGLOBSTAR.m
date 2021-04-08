function [xsi] = fcnGLOBSTAR(points, roll, pitch, yaw)
% Transforms a point from global to local reference frame
% Input in RADIANS

% INPUT:
%   points - n x 3 matrix of (x,y,z) points in global coordinates
%   roll - n x 1 vector of roll (nu) angles in radians
%   pitch - n x 1 vector of pitch (eps) angles in radians
%   yaw - n x 1 vector of yaw (psi) angles in radians
% OUTPUT:
%   xsi - n x 3 matrix of "points" in local reference frame

len = length(points(:,1));

cnu = cos(roll);
snu = sin(roll);
ceps = cos(pitch);
seps = sin(pitch);
cpsi = cos(yaw);
spsi = sin(yaw);

xsi = zeros(len,3);
xsi(:,1) = points(:,1).*(cpsi.*ceps) + points(:,2).*(cpsi.*seps.*snu+spsi.*cnu) + points(:,3).*(-cpsi.*seps.*cnu+spsi.*snu);
xsi(:,2) = points(:,1).*(-spsi.*ceps) + points(:,2).*(-spsi.*seps.*snu+cpsi.*cnu) + points(:,3).*(spsi.*seps.*cnu+cpsi.*snu);
xsi(:,3) = points(:,1).*(seps) + points(:,2).*(-ceps.*snu) + points(:,3).*(ceps.*cnu);

end

