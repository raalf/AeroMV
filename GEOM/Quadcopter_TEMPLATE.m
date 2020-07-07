%% Quadcopter Template
% This file show a typical input for a quadcopter
% All units are in metric


%% Rotor Information
% Rotor name found in 
GEOM.ROTOR.strNAME = 'Prop_Name';

% Location center locations [x,y,z], (meters)
GEOM.ROTOR.matLOCATION = [89.3868 192.014 -89.3868; % Rotor 1
                          89.3868 192.014 203.6868; % Rotor 2
                          -203.6868 192.014 203.6868; % Rotor 3
                          -203.6868 192.014 -89.3868]; % Rotor 4

% Rotation direction for each rotor                  
GEOM.ROTOR.matROT = [1 1 -1 -1]; % 1 = CW, -1 = CCW

% Rotor Diameter for each rotor
GEOM.ROTOR.vecDIAM = [0.254 0.254 0.254 0.254]';

% Number of blades per rotor (one value is used for all rotors)
GEOM.ROTOR.valNUMB = 2;


%% Vehicle information
% Total vehicle mass
GEOM.VEH.valMASS = 1.867; % (Kg)

% Vehicle CG Location [x,y,z], (meters)
GEOM.VEH.vecCG = [0 0 0.1];

% Moment of inertia [Ixx -Ixy -Ixz; -Ixy Iyy, -Iyz, -Ixz, -Iyz, Izz]
GEOM.VEH.I =  [0.025322564860661, 0, 0;
                0, 0.028041568149193, 0;
                0, 0, 0.029055618259337];

%% Component information
% Component options categories are: ARM, BODY, LEG, MOTOR, PAYLOAD, OTHER
% Each component can be approximated by a cylinder or a sphere

% Example of cylindrical arms
% Cylinders need .strTYPE, .valLENGTH, .valDIAM, .matBEGIN, .matEND
GEOM.VEH.ARM.strTYPE = 'Cylinder'; % Component type ('Cylinder' or 'Sphere')
GEOM.VEH.ARM.valLENGTH = 0.17931; % Length of cylinder (meters)
GEOM.VEH.ARM.valDIAM = 0.0253; % Cylinder diameter (meters)

% Each arm location, begin and end [x,y,z], (meters)
GEOM.VEH.ARM.matBEGIN = [-24.821 154.94 24.821; % Arm 1
                        -24.8214 154.94 89.4789; % Arm 2
                        -89.4789 154.94 89.4789; % Arm 3
                        -89.4789 154.94 24.821]; % Arm 4
GEOM.VEH.ARM.matEND = [101.9592 154.94 -101.9592; % Arm 1
                        101.9592 154.94 216.2592; % Arm 2
                        -216.2592 154.94 216.2592; % Arm 3
                        -216.2592 154.94 -101.9592]; % Arm 4

% Example of a spherical fuselage/body
GEOM.VEH.BODY.strTYPE = 'Sphere'; % Component type
GEOM.VEH.BODY.valDIAM = 0.115; % Sphere diameter (meters)
GEOM.VEH.BODY.vecLOCATION = [0 0 0]; % Location [x,y,z], (meters)



%% Air Properties
AIR.density = 1.112; % Density (kg/m^3)
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity (m^2/s)