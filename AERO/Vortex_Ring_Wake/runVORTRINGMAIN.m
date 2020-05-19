% An example run script for the Vortex Ring Wake model
clear,clc
num_seg = 100;
num_ring = 5;

GEOM.ROTCENTER = [0.5 0 0;0 0.5 0; -0.5 0 0; 0 -0.5 0];
GEOM.N_b = 2; % number of blades
GEOM.R = 0.127;
% Calculate circulation


COND.RPM = [4000 4500 4222 7546]';
COND.T = [5 4 2.5 7.5]';
COND.rho = 1.225;
COND.V_inf = [0.0926    0.0454    0.1133;
    0.0942    0.0488    0.1133;
    0.0942    0.0454    0.1133;
    0.0926    0.0488    0.1133];

[matQ,VORTPARAM] = fcnVORTRINGMAIN(num_seg,num_ring,GEOM,COND);



%% Comparing to J. Tsaltas' Fast Multirotor Performance Prediction wake model
% folder = pwd;
% 
% cd('C:\Users\Devin\Documents\FMVP_FastMultirotorPerformancePrediction')
% vel_mag = sqrt(COND.V_inf(:,1).^2+COND.V_inf(:,2).^2+COND.V_inf(:,3).^2);
% AOA_R = acosd(dot(COND.V_inf',repmat([0 0 1]',1,size(COND.V_inf,1)))'./(vel_mag));
% 
% vel_mag = permute(vel_mag,[3,2,1]);
% AOA_R = permute(AOA_R,[3,2,1]);
% 
% [vi_int,vi_self,skewRAD,wi] = fcnWIM(COND.rho,4,GEOM.N_b,GEOM.R*2,GEOM.ROTCENTER,permute(COND.T,[3 2 1]),permute(COND.RPM,[3 2 1]),AOA_R,vel_mag);
% cd(folder)