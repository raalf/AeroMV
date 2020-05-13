% An example run script for the Vortex Ring Wake model
clear,clc
num_seg = 10;
num_ring = 5;


GEOM.ROTCENTER = [0.5 0 0;0 0.5 0; -0.5 0 0; 0 -0.5 0];
GEOM.N_b = 2; % number of blades
GEOM.R = 0.127;
% Calculate circulation


COND.omega = [4000 4500 4222 7546]';
COND.T = [5 4 2.5 7.5]';
COND.rho = 1.225;
COND.V_inf = [0.0926    0.0454    0.1133;
    0.0942    0.0488    0.1133;
    0.0942    0.0454    0.1133;
    0.0926    0.0488    0.1133];

[matQ,VORTPARAM] = fcnVORTRINGMAIN(num_seg,num_ring,GEOM,COND);