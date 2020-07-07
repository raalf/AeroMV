%% Quadcopter (DJI Matrice 200)
% This file is the input info for the DJI Matrice 200
% All units are in metric


%% Rotor Information
% Rotor name found in 
GEOM.ROTOR.strNAME = 'DJI_Matrice_200_BEMTDATA';

% Location center locations [x,y,z], (meters)
GEOM.ROTOR.matLOCATION = [mean([209.9119 210.8997]) mean([244.2242 245.6493])  mean([170.4645 158.0862]); % Rotor 1
                          mean([-250.1931 -251.4234]) mean([220.8841 222.1141])  mean([171.3553 158.9771]); % Rotor 2
                          mean([-243.7041 -244.9343]) mean([-213.8545 -215.0847])  mean([169.6223 157.244]); % Rotor 3
                          mean([222.3009 223.2988]) mean([-213.8545 -215.0847])  mean([169.6223 157.244])]*0.001; % Rotor 4

% Rotation direction for each rotor                  
GEOM.ROTOR.matROT = [-1 1 -1 1]; % 1 = CW, -1 = CCW

% Rotor Diameter for each rotor
GEOM.ROTOR.vecDIAM = [0.4 0.4 0.4 0.4]';

% Number of blades per rotor (one value is used for all rotors)
GEOM.ROTOR.valNUMB = 2;


%% Vehicle information
% Total vehicle mass
GEOM.VEH.valMASS = 3.8; % (Kg)

% Vehicle CG Location [x,y,z], (meters)
GEOM.VEH.vecCG = [0 0 0.1];

% Moment of inertia [Ixx -Ixy -Ixz; -Ixy Iyy, -Iyz, -Ixz, -Iyz, Izz]
GEOM.VEH.I =  [0, 0, 0;
                0, 0, 0;
                0, 0, 0];

%% Component information
% Component options categories are: ARM, BODY, LEG, MOTOR
% Each component can be approximated by a cylinder or a sphere

% ARM
% Cylinders need .strTYPE, .valLENGTH, .valDIAM, .matBEGIN, .matEND
GEOM.VEH.ARM.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.ARM.valLENGTH = 0.17931; % Length of cylinder (meters)
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
GEOM.VEH.LEG.valLENGTH = 0.17931; % Length of cylinder (meters)
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
GEOM.VEH.MOTOR.valLENGTH = 0.17931; % Length of cylinder (meters)
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



%% Air Properties
AIR.density = 1.112; % Density (kg/m^3)
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity (m^2/s)