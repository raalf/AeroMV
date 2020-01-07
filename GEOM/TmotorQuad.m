%% Quadcopter (T-Motor)
% This file contains all the input information for a quadcopter with
% T-Motor rotors

% NOTE: To not include a wing or a rotor, either comment out the section or
% leave the values empty as: []

%% Rotor Information
% Rotor anem
GEOM.ROTOR.strNAME = 'Tmotor';

% The rotation center (x,y,z) and rotational radius for the rotor hub
GEOM.ROTOR.matROTAORIGIN = [-0.093545 0.07 0; 0.093545 0.07 0;
    -0.093545 -0.07 0; 0.093545 -0.07 0];
GEOM.ROTOR.vecROTARADIUS = [0.07 ,0.07, 0.07, 0.07]';

% Rotor hover origin
GEOM.ROTOR.matHOVERORIGIN = [-0.1635 0.07 0;0.1635 0.07 0;
    -0.1635 -0.07 0; 0.1635 -0.07 0];
GEOM.ROTOR.vecROTARADIUS = abs(GEOM.ROTOR.matHOVERORIGIN(:,1)-GEOM.ROTOR.matROTAORIGIN(:,1));

% Rotor Diameter *FOR EACH ROTOR
GEOM.ROTOR.vecDIAM = [0.4572 0.4572 0.4572 0.4572]';

%% Vehicle information
GEOM.VEH.valWEIGHT = 3.394; % Kg

 % Arm info
GEOM.VEH.ARM.strTYPE = 'Cylinder';
GEOM.VEH.ARM.valLENGTH = 0.28; % meters
GEOM.VEH.ARM.valDIAM = 0.02; % meters

% Fuselage body
GEOM.VEH.BODY.strTYPE = 'Sphere';
TABLE.VEH.strBODYNAME = 'Sphere';
GEOM.VEH.BODY.valDIAM = 0.2; % meters

% Motor info
GEOM.VEH.MOTOR.strTYPE = 'Cylinder';
GEOM.VEH.MOTOR.valLENGTH = 0.03; % meters
GEOM.VEH.MOTOR.valDIAM = 0.04; % meters

% Leg info
GEOM.VEH.LEG.strTYPE = 'Cylinder';
GEOM.VEH.LEG.valDIAM = 0.02;
GEOM.VEH.LEG.valLENGTH = 0.295;

% Payload info
GEOM.VEH.PAYLOAD.strTYPE = 'Cylinder';
TABLE.VEH.strPAYLOADNAME = 'Cylinder';
GEOM.VEH.PAYLOAD.valDIAM = 0.09;
GEOM.VEH.PAYLOAD.valLENGTH = 0.165;

%% Air Properties
AIR.density = 1.225; % Density
AIR.kinvisc = 1.4207E-5; % Kinematic Viscosity

%% Motor Information
TABLE.valMOTOREFF = 100; % Motor efficiency (%)