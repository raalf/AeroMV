function PERF = fcnRUNBEMT(GEOM, AIR, PERF, STATE)
% This function runs the BEMT code for each rotor individually
i = 1;
% Create flow structure
flow.rho = AIR.density; % Air density [kg/m^3]
flow.mu = AIR.density*AIR.kinvisc;	% Dynamic viscosity [kg/m s]
flow.V = STATE.VEL_MAG;	% Freestream velocity [m/s]
flow.inflow_angle = STATE.AOA;  % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90)

% Create oper structure
oper.inflow_type = 1;
oper.azimuth_num = 8;
oper.toggle_vi = 'on';
oper.toggle_visc = 'on';
oper.alpha_zero = -0.03;
oper.a_0 = 2*pi;
oper.toggle_WIM = 'off'; 
oper.gurney = 0;

% Crate rotor structure
rotor.name = GEOM.ROTOR.strNAME;
rotor.num_rotors = 1;
rotor.orientation = 'square';
rotor.armLENGTH = 0;
rotor.roll = 0;
rotor.rp_twist = 0;
rotor.rp_cant = 0;

% Create blade structure
blade.modify_pitch     =   0; % Collectively add/subtract from pitch [deg]
blade.scale_radius     =   1;
load(strcat('AERO\BEMT_Code\Rotors\',GEOM.ROTOR.strNAME,'.mat'));
blade.geometry = eval(char(rotor.name));

% Create wake structure
wake.num_seg = 32; % Number of segments in ring element
wake.num_elements = 50; % Number of ring or helix loop elements
wake.type = 'ring'; % Type of wake elements used (ring or helix)

% Create options structure
options.save_workspace = 'off'; % on/off toggle to save workspace each time BEMT_Carroll is called (i.e. in performance sweeps)
options.saved_workspace_identifier = 'Testing'; % identifier for saved workspace structure, string
options.analysistype = 1;
options.toggle_precompute = 'on';
options.AoAresolution = 0.5; % Database resolution for angle of attack, [deg].
options.REresolution = 10000; % Database resolution for Reynolds number.
options.RErange_max = 300000; % Max Reynolds number range. Min is set @ zero. Keep @ ~ 10^6
options.toggle_coeff_plot = 'off'; % Toggle to generate 3D plots for Re vs AoA vs coeff for newly generated dataset

for i = 1:length(STATE.RPM)
FOLDER_ADDRESS = pwd;
if FOLDER_ADDRESS(end-10:end) == 'BEMT Module'
    addpath(genpath(FOLDER_ADDRESS(1:end-11)))
else
    addpath(genpath(FOLDER_ADDRESS))
    cd(strcat(FOLDER_ADDRESS,'\AERO\BEMT_Code\BEMT Module'))
end

if strcmpi(options.toggle_precompute,'on')
    airfoil_database(blade,options)
    load('airfoil_coeff_database');
    options.airfoil_coeff_database = airfoil_coeff_database;
end
    oper.rpm = STATE.RPM(i);
    flow.V = STATE.VEL_ROTOR_MAG(i); % Freestream velocity [m/s]
    flow.inflow_angle = STATE.AOA_R(i);  % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90)
    PERF.ROTOR(i) = BEMT_Carroll(blade,flow,oper,rotor,wake,options);
cd(FOLDER_ADDRESS)
end
end

