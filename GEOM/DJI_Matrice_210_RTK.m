%% Quadcopter (DJI Matrice 210 RTK)
% This file is the input info for the DJI Matrice 210
% All units are in metric


%% Rotor Information
% Rotor name found in 
GEOM.ROTOR.strNAME = 'Matrice_210_RTK_BEMTDATA';

% Location center locations [x,y,z], (meters)
% GEOM.ROTOR.matLOCATION = [mean([209.9119 210.8997]) mean([244.2242 245.6493])  mean([170.4645 158.0862]); % Rotor 1
%                           mean([-250.1931 -251.4234]) mean([220.8841 222.1141])  mean([171.3553 158.9771]); % Rotor 2
%                           mean([-243.7041 -244.9343]) mean([-213.8545 -215.0847])  mean([169.6223 157.244]); % Rotor 3
%                           mean([222.3009 223.2988]) mean([-213.8545 -215.0847])  mean([169.6223 157.244])]*0.001; % Rotor 4
   
GEOM.ROTOR.matLOCATION = [mean([222.3009 223.2988]) mean([-219.1556 -220.5806])  mean([171.0538 158.6755]);
                          mean([209.9119 210.8997]) mean([244.2242 245.6493])  mean([170.4645 158.0862]);
                          mean([-250.1931 -251.4234]) mean([220.8841 222.1141])  mean([171.3553 158.9771]);
                          mean([-243.7041 -244.9343]) mean([-213.8545 -215.0847])  mean([169.6223 157.244])]*0.001;
GEOM.ROTOR.matLOCATION = [0.2228   -0.2199    0.1644
                          0.2228    0.2449    0.1644
                         -0.2443    0.2449    0.1644
                         -0.2443   -0.2199    0.1644];
                     
% GEOM.ROTOR.matLOCATION = [0.26987  -0.227     0.1644
%                           0.26987  0.227     0.1644
%                          -0.1905   0.227     0.1644
%                          -0.1905  -0.227     0.1644];

GEOM.ROTOR.matLOCATION = [0.2228   -0.2199+0.0052    0.1644
                          0.2228    0.2449-0.0052    0.1644
                         -0.2443    0.2449-0.0052    0.1644
                         -0.2443   -0.2199+0.0052    0.1644];
                     
theta = 8;
GEOM.ROTOR.matNORMALS = [-0.5*sind(theta) 0.5*sind(theta) cosd(theta);
                        -0.5*sind(theta) -0.5*sind(theta) cosd(theta);
                        0.5*sind(theta) -0.5*sind(theta) cosd(theta);
                        0.5*sind(theta) 0.5*sind(theta) cosd(theta)];
                        
% Rotation direction for each rotor                  
GEOM.ROTOR.matROT = [-1 1 -1 1]; % 1 = CW, -1 = CCW

% Rotor Diameter for each rotor
GEOM.ROTOR.vecDIAM = [0.4318 0.4318 0.4318 0.4318]';

% Number of blades per rotor (one value is used for all rotors)
GEOM.ROTOR.valNUMB = 2;

% Rotor mass
GEOM.ROTOR.vecRMASS = 0.034; % Kg


%% Vehicle information
% Total vehicle mass
GEOM.VEH.valMASS = 6.7315; % (Kg)

% Vehicle CG Location [x,y,z], (meters)
GEOM.VEH.vecCG = [7.119 -10.172 47.397]*0.001;
GEOM.VEH.vecCG = [mean(GEOM.ROTOR.matLOCATION([1 3],1))*1000+15.24 5.12+7.7 47.397]*0.001;
% CG_X is relative to leg as center, CG_Y is in reference to main body
% centerline
GEOM.VEH.vecCG = [4.96+15.24 5.12+7.7 47.397]*0.001;


% Moment of inertia [Ixx -Ixy -Ixz; -Ixy Iyy, -Iyz, -Ixz, -Iyz, Izz]
I_multi = 1;
GEOM.VEH.I =  [I_multi*0.17525014, 0.00411034, -0.00173288;
                0.00411034, I_multi*0.16151033, 0.01333274;
                -0.00173288, 0.01333274, 0.20452748];


%% Component information
% Component options categories are: ARM, BODY, LEG, MOTOR
% Each component can be approximated by a cylinder or a sphere

% ARM
% Cylinders need .strTYPE, .valLENGTH, .valDIAM, .matBEGIN, .matEND
GEOM.VEH.ARM.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.ARM.valDIAM = 0.025; % Cylinder diameter (meters)

% Each arm location, begin and end [x,y,z], (meters)
GEOM.VEH.ARM.matBEGIN = [209.1574 238.3464 99.983; % Arm 1
                        -86.9833 61.5794 69.5599; % Arm 2
                        -84.3996 -50.6446 67.8268; % Arm 3
                        217.0322 -216.4456 100.5722]*0.001; % Arm 4
GEOM.VEH.ARM.matEND = [95.5585 76.1104 72.1484; % Arm 1
                       -244.536 217.13101 100.8738; % Arm 2
                       -241.9502 -208.1953 99.1408; % Arm 3
                       103.4333 -54.2096 72.7376]*0.001; % Arm 4
                   
% Legs
GEOM.VEH.LEG.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.LEG.valDIAM = 0.018; % Cylinder diameter (meters)

% Each LEG location, begin and end [x,y,z], (meters)
GEOM.VEH.LEG.matBEGIN = [224.71 199.6733 -263.3515;
                            224.71 -178.5158 -257.8621;
                            4.96 196.8921 -254.792;
                            4.96 -177.5886 -255.0089]*0.001; 
GEOM.VEH.LEG.matEND = [-196.79 199.6733 -263.3515;
                        -196.79 -178.5056 -257.862;
                        4.96 88.7362 78.0778;
                        4.96 -69.4326 77.8609]*0.001;                 
% Motor
GEOM.VEH.MOTOR.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.MOTOR.valDIAM = 0.044; % Cylinder diameter (meters)

% Each LEG location, begin and end [x,y,z], (meters)
GEOM.VEH.MOTOR.matBEGIN = [216.735 253.983 85.697;
                            -258.6172 229.30789 86.5585;
                            -252.1281 -222.2784 84.8554
                            229.134 -228.9143 86.2869]*0.001; 
GEOM.VEH.MOTOR.matEND = [210.8977 245.6493 158.0862;
                            -251.4234 222.1141 158.9771;
                            -245.2442, -215.3947, 157.1247;
                            223.2988 -220.5806 158.6755]*0.001;  
                    
                    
% Adding the camera as the payload
GEOM.VEH.PAYLOAD.strTYPE = 'Sphere'; % Component type
GEOM.VEH.PAYLOAD.valDIAM = 0.06; % Sphere diameter (meters)
GEOM.VEH.PAYLOAD.vecLOCATION = [mean([220.0738 207.3815]) mean([96.8405 36.4871]) mean([-67.4016 -127.3381])]*0.001; % Location [x,y,z], (meters)

% Body and battery
GEOM.VEH.BODY.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.BODY.valDIAM = [0.1 0.135]'; % Cylinder diameter (meters)

% Each LEG location, begin and end [x,y,z], (meters)
GEOM.VEH.BODY.matBEGIN = [122.46 7.7 mean([46.5 90]);
                         mean([-110.54 25.46]) 7.6987 -71.5]*0.001; 
GEOM.VEH.BODY.matEND = [-122.54 7.7 mean([90 46.5]);
                        mean([-110.57 25.46])  7.6987 36.5]*0.001; 

% [LEFT GPS, RIGHT GPS, LEFT LEG SENSOR, RIGHT LEG SENSOR, LEFT FOOT SENSOR]
GEOM.VEH.OTHER.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.OTHER.valDIAM = [0.065 0.065 0.02975 0.111 0.0295]'; % Cylinder diameter (meters)

% Each LEG location, begin and end [x,y,z], (meters)
GEOM.VEH.OTHER.matBEGIN = [4.96 197.0104 197.9865
                           4.96 -182.3785 197.9865
                           4.96 mean([176.5874 156.6152]) mean([-88.4228 -94.9122])
                           4.96 mean([-148.9655 -184.5025]) mean([-24.0483 -133.4198])
                           -156.7393 mean([-203.4216 -188.2047]) mean([-247.141 -252.0852])]*0.001; 
GEOM.VEH.OTHER.matEND = [4.96 197.0104 217.9865;
                        4.96 -182.3785 217.9868
                        4.96 mean([136.4152 116.4153]) mean([35.2145 28.7162])
                        4.96 mean([-153.9736  -118.4366]) mean([-143.3393 -33.9677])
                        -20.7393 mean([-203.4216 -188.2047]) mean([-247.141 -252.0852])]*0.001; 

%% Air Properties
AIR.density = 1.112; % Density (kg/m^3)
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity (m^2/s)