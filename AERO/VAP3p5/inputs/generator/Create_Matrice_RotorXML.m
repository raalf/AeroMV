% Creating xml for Matrice rotor
clear,clc

rotor_radius = 0.2286; % Radius (m)
airfoil_name = 'MatriceAirfoil';
output_filename = 'Matrice_210_RTK_Rotor';
%% Read illinois geom data
filename = 'APC_E9x4.5_geom.txt';
load('Matrice_210_RTK');
r_R = Matrice_210_RTK.r_R;
c_R = Matrice_210_RTK.c_R;
beta = Matrice_210_RTK.Beta;

%% Leading edge distribution (x values for each r/R)
% Create from the grabit.m file on a top down photo of the rotor
% le = (-c_R(1)*rotor_radius/2)*(ones(size(r_R,1),1));
% le_R = Matrice_210_RTK.MidChordLine-c_R/2;
le = -1*Matrice_210_RTK.LE;

%% INTERPOLATE for r/R values
% le_R = interp1(tempLE(:,2),tempLE(:,1),r_R,'linear','extrap');
figure(1)
clf(1)
hold on
% plot(tempLE(:,2),tempLE(:,1),'-k*')
plot(r_R,le/rotor_radius,'--dr')
plot(r_R,le/rotor_radius-c_R,'--db')
ylabel('x-dir (m)')
xlabel('y-dir (m)')
title('Top View of Rotor')
axis equal
grid on
box on
hold off

%% Create input
fcnXMLPANEL(-1*le,r_R*rotor_radius,(zeros(size(r_R,1),1)),c_R*rotor_radius,beta,airfoil_name,output_filename,1);
