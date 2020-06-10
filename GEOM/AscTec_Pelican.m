%% Quadcopter (AscTec_Pelican)
% This file contains all the input information for athe AscTec Pelican
% quadrotor   

%% Rotor Information
% Rotor name
% GEOM.ROTOR.strNAME = 'APC_10x4_7_old';
% GEOM.ROTOR.strNAME = 'APC_10x4_7';
% GEOM.ROTOR.strNAME = 'APC_10x4_7_NewFoil';
GEOM.ROTOR.strNAME = 'APC_10x_4_7_Updated';

% GEOM.ROTOR.strNAME = 'APC_10x_4_7_Experiment';

% Vehicle CG Location
theta = 45;
xy = [cosd(theta) -sind(theta);sind(theta) cosd(theta)]*([12.9 -17.9]');
% GEOM.VEH.vecCG = [xy(1) xy(2) 152.0153-118.7]*0.001;
% GEOM.VEH.vecCG = [0 0 152.0153-118.7]*0.001;
GEOM.VEH.vecCG = [-1.5 6.5 152.0153-118.7]*0.001;
% Location of rotor hubs relative to CG
GEOM.ROTOR.matLOCATION = [89.3868 192.014 -89.3868;
                          89.3868 192.014 203.6868;
                          -203.6868 192.014 203.6868;
                          -203.6868 192.014 -89.3868]*0.001;
GEOM.ROTOR.matROT = [1 1 -1 -1]; % 1 = CW, -1 = CCW
                      
GEOM.ROTOR.matLOCATION = GEOM.ROTOR.matLOCATION - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.ROTOR.matLOCATION = GEOM.ROTOR.matLOCATION(:,[1 3 2]);
idx = [4,2,1,3]';
GEOM.ROTOR.matLOCATION = GEOM.ROTOR.matLOCATION(idx,:);
idx2 = [2,1,4,3]';
GEOM.ROTOR.matLOCATION = GEOM.ROTOR.matLOCATION(idx2,:);
% Rotor Diameter *FOR EACH ROTOR
GEOM.ROTOR.vecDIAM = [0.254 0.254 0.254 0.254]';
%GEOM.ROTOR.vecDIAM = [0.2794 0.2794 0.2794 0.2794]';

% Number of blades on a rotor
GEOM.ROTOR.valNUMB = 2;

%% Vehicle information
GEOM.VEH.valMASS = 1.867; % Kg
% GEOM.VEH.valMASS = 1.4; % Kg
% Moment of inertia
% GEOM.VEH.I =  [0.0031887, -0.0000038, -0.0000881;
%                  -0.0000038, 0.0032245, 0.0000846;
%                  -0.0000881, 0.0000846, 0.0013857]; % Inertias from CAD
GEOM.VEH.I =  [0.025322564860661, 0, 0;
                0, 0.028041568149193, 0;
                0, 0, 0.029055618259337]; % From Ben's experiements
% GEOM.VEH.I =  [0.25322564860661, 0, 0; 
%                 0, 0.28041568149193, 0;
%                 0, 0, 0.03417382];
% GEOM.VEH.I = [0.02065734 -0.00085105 -0.00039387;
%                 -0.00085105 0.02069273 0.00039355;
%                 -0.00039387 0.00039355 0.03417382]; % From Ben's updated CAD
phi = 45; % Angle of rotation between reference frames
I = GEOM.VEH.I;
GEOM.VEH.I(1,1) = 0.5*(I(1,1)+I(2,2))+0.5*(I(1,1)-I(2,2))*cosd(2*phi)-I(1,2)*sind(2*phi);
GEOM.VEH.I(2,2) = 0.5*(I(1,1)+I(2,2))-0.5*(I(1,1)-I(2,2))*cosd(2*phi)+I(1,2)*sind(2*phi);
GEOM.VEH.I(1,2) = 0.5*(I(1,1)-I(2,2))*sind(2*phi)+I(1,2)*cosd(2*phi);
GEOM.VEH.I(2,1) = GEOM.VEH.I(1,2);
% GEOM.VEH.I(1,1) = 3.*GEOM.VEH.I(1,1);   
% GEOM.VEH.I(2,2) = 3.*GEOM.VEH.I(2,2);   

 % Arm info
GEOM.VEH.ARM.strTYPE = 'Cylinder';
GEOM.VEH.ARM.valLENGTH = 0.17931; % meters
GEOM.VEH.ARM.valDIAM = 0.0253; % meters
GEOM.VEH.ARM.matBEGIN = [-24.821 154.94 24.821;
                        -24.8214 154.94 89.4789;
                        -89.4789 154.94 89.4789;
                        -89.4789 154.94 24.821]*0.001;
                    
GEOM.VEH.ARM.matBEGIN = GEOM.VEH.ARM.matBEGIN - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.ARM.matBEGIN = GEOM.VEH.ARM.matBEGIN(:,[1 3 2]);
GEOM.VEH.ARM.matBEGIN = GEOM.VEH.ARM.matBEGIN(idx,:);
GEOM.VEH.ARM.matBEGIN = GEOM.VEH.ARM.matBEGIN(idx2,:);

GEOM.VEH.ARM.matEND = [101.9592 154.94 -101.9592;
                        101.9592 154.94 216.2592;
                        -216.2592 154.94 216.2592;
                        -216.2592 154.94 -101.9592]*0.001;
                                    
GEOM.VEH.ARM.matEND = GEOM.VEH.ARM.matEND - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.ARM.matEND = GEOM.VEH.ARM.matEND(:,[1 3 2]);
GEOM.VEH.ARM.matEND = GEOM.VEH.ARM.matEND(idx,:);
GEOM.VEH.ARM.matEND = GEOM.VEH.ARM.matEND(idx2,:);
% Fuselage body
% GEOM.VEH.BODY.strTYPE = 'Sphere';
% TABLE.VEH.strBODYNAME = 'Sphere';
% GEOM.VEH.BODY.valDIAM = 0.115; % meters
% GEOM.VEH.BODY.vecLOCATION = [0 0 0];

GEOM.VEH.BODY.strTYPE = 'Cylinder';
GEOM.VEH.BODY.valLENGTH = 0.17931; % meters
GEOM.VEH.BODY.valDIAM = 142.2406*0.001; % meters
GEOM.VEH.BODY.matBEGIN = [-57.1500 0 57.1500]*0.001;
                    
GEOM.VEH.BODY.matBEGIN = GEOM.VEH.BODY.matBEGIN - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.BODY.matBEGIN = GEOM.VEH.BODY.matBEGIN(:,[1 3 2]);

GEOM.VEH.BODY.matEND = [-57.1500 337.82 57.1500]*0.001;
                                    
GEOM.VEH.BODY.matEND = GEOM.VEH.BODY.matEND - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.BODY.matEND = GEOM.VEH.BODY.matEND(:,[1 3 2]);


% Motor info
GEOM.VEH.MOTOR.strTYPE = 'Cylinder';
GEOM.VEH.MOTOR.valLENGTH = 0.05; % meters
GEOM.VEH.MOTOR.valDIAM = 0.0254; % meters
GEOM.VEH.MOTOR.matBEGIN = [89.3868  154.9400  -89.3868;
                        89.3868  154.9400  203.6868;
                        -203.6868 154.94 203.6868;
                        -203.6868 154.94 -89.3868]*0.001;
                    
GEOM.VEH.MOTOR.matBEGIN = GEOM.VEH.MOTOR.matBEGIN - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.MOTOR.matBEGIN = GEOM.VEH.MOTOR.matBEGIN(:,[1 3 2]);
GEOM.VEH.MOTOR.matBEGIN = GEOM.VEH.MOTOR.matBEGIN(idx,:);
GEOM.VEH.MOTOR.matBEGIN = GEOM.VEH.MOTOR.matBEGIN(idx2,:);

GEOM.VEH.MOTOR.matEND = [89.3868  188.6640  -89.3868;
                        89.3868  188.6640  203.6868;
                        -203.6868 188.6640 203.6868;
                        -203.6868 188.6640 -89.3868]*0.001;
                                    
GEOM.VEH.MOTOR.matEND = GEOM.VEH.MOTOR.matEND - ([-56.0196 150.5562 55.2647]*0.001);
GEOM.VEH.MOTOR.matEND = GEOM.VEH.MOTOR.matEND(:,[1 3 2]);
GEOM.VEH.MOTOR.matEND = GEOM.VEH.MOTOR.matEND(idx,:);
GEOM.VEH.MOTOR.matEND = GEOM.VEH.MOTOR.matEND(idx2,:);

% Leg info
% GEOM.VEH.LEG.strTYPE = 'Cylinder';
% GEOM.VEH.LEG.valDIAM = 0.026;
% GEOM.VEH.LEG.valLENGTH = 0.078;

% Payload info
% GEOM.VEH.PAYLOAD.strTYPE = 'Cylinder';
% TABLE.VEH.strPAYLOADNAME = 'Cylinder';
% GEOM.VEH.PAYLOAD.valDIAM = 0.09;
% GEOM.VEH.PAYLOAD.valLENGTH = 0.165;

%% Air Properties
AIR.density = 1.112; % Density
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity

%% Motor Information
TABLE.valMOTOREFF = 100; % Motor efficiency (%)