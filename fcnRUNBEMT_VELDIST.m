function PERF = fcnRUNBEMT_VELDIST(GEOM, AIR, PERF, STATE)
% This function runs the BEMT code with a velocity distrubition due to the
% vehicle dynamics
%
% D.F.B. in Toronto, Sept. 2020


% Create flow structure
flow.rho = AIR.density; % Air density [kg/m^3]
flow.mu = AIR.density*AIR.kinvisc;	% Dynamic viscosity [kg/m s]
%flow.V = STATE.VEL_MAG;	% Freestream velocity [m/s]
%flow.inflow_angle = STATE.AOA;  % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90)

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
load(strcat('AERO\BEMT_Code_VelocityDist\Rotors\',GEOM.ROTOR.strNAME,'.mat'));
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
options.AoAresolution = 0.25; % Database resolution for angle of attack, [deg].
options.REresolution = 50000; % Database resolution for Reynolds number.
options.RErange_max = 1000000; % Max Reynolds number range. Min is set @ zero. Keep @ ~ 10^6
options.toggle_coeff_plot = 'off'; % Toggle to generate 3D plots for Re vs AoA vs coeff for newly generated dataset



%% Run BEMT
for i = 1:length(STATE.RPM)
    FOLDER_ADDRESS = pwd;
    if FOLDER_ADDRESS(end-10:end) == 'BEMT Module'
        addpath(genpath(FOLDER_ADDRESS(1:end-11)))
    else
        addpath(genpath(FOLDER_ADDRESS))
        cd(strcat(FOLDER_ADDRESS,'\AERO\BEMT_Code_VelocityDist\BEMT Module'))
    end
    
    if strcmpi(options.toggle_precompute,'on')
        airfoil_database(blade,options)
        load('airfoil_coeff_database');
        options.airfoil_coeff_database = airfoil_coeff_database;
    end
    
    % *****Compute the local velocity due to the vehicles body rates*****
    if oper.azimuth_num==2
        azimuth = [pi/2,3*pi/2];
    else
        azimuth = linspace(0, 2*pi-(2*pi/oper.azimuth_num),oper.azimuth_num);
    end
    stations = height(blade.geometry);
    r = blade.geometry.r_R;
    R = blade.geometry.Radius(stations);
    mid_span = (r(1:end-1).*R+r(2:end).*R)/2;
    mid_span_y = [-1*mid_span,zeros(stations-1,1)];
%     	figure(5)
%         clf(5)
    for j = 1:oper.azimuth_num
        if GEOM.ROTOR.matROT(i) == 1
            % For CW rotor, must mirror the local velocities about the
            % y-axis because BEMT assumes CCW with a freestream velocity in 
            % the -ve x-direction
            R = [cos(azimuth(j)) -sin(azimuth(j)); sin(azimuth(j)) cos(azimuth(j)+pi)];
            points = (R*mid_span_y')';
            points(:,2) = -1*points(:,2); % Take velocities at points mirror about y-axis
            points(:,3) = zeros(stations-1,1);
            local_vel = cross(repmat(STATE.BODY_RATES(end,:),stations-1,1),(GEOM.ROTOR.matLOCATION(i,:)+points));

            local_vel(:,2) = -1*local_vel(:,2); % Mirror the measures local y-velocity
            flow.vel_perp(:,j) = local_vel(:,2)*sin(azimuth(j))+ local_vel(:,1)*cos(azimuth(j));
            flow.vel_z(:,j) = local_vel(:,3);
            
            % Assign these velocities to the correct ordered points. 
            % These are just here for reference plotting
            R = [cos(azimuth(j)) -sin(azimuth(j)); sin(azimuth(j)) cos(azimuth(j))];
            points = (R*mid_span_y')';
            points(:,3) = zeros(stations-1,1);
            
        else
            % If CCW can directly calculate the new velocities
            R = [cos(azimuth(j)) -sin(azimuth(j)); sin(azimuth(j)) cos(azimuth(j))];
            points = (R*mid_span_y')';
            points(:,3) = zeros(stations-1,1);
            local_vel = cross(repmat(STATE.BODY_RATES(end,:),stations-1,1),(GEOM.ROTOR.matLOCATION(i,:)+points));
            
            flow.vel_perp(:,j) = local_vel(:,2)*sin(azimuth(j))+ local_vel(:,1)*cos(azimuth(j));
            flow.vel_z(:,j) = local_vel(:,3);
        end
%                 hold on
%                 plot(points(:,1),points(:,2),'*')
%                 quiver3(points(:,1),points(:,2),points(:,3),local_vel(:,1),local_vel(:,2),local_vel(:,3),'k')
    end
%         axis equal
%         grid on
%         box on
%         xlabel('x-dir')
%         ylabel('y-dir')
%         hold off
    
    
    
    oper.rpm = STATE.RPM(i);
    flow.V = STATE.VEL_ROTOR_MAG(i); % Freestream velocity [m/s]
    flow.inflow_angle = STATE.AOA_R(i);  % Rotor AoA [deg]. Angle between freestream and rotor plane (0 to +/- 90)
    PERF.ROTOR(i) = BEMT_Carroll(blade,flow,oper,rotor,wake,options);
    cd(FOLDER_ADDRESS)
end


