clear,clc
%   ---- Run the vehicle component calculations ----
% This run function can be used to calculate the forces due to the vehicle
% components (not including rotor) based on an input file. The function
% sweeps over angle of attack and beta angles at a given velocity.
%
% INPUTS:
%       filename    - name of file in GEOM/ folder that should be run
%       AoA         - A vector of angles of attack (deg)
%       Beta        - A vecotr of side slip angles (deg)
%       Velocity    - The magnitude of the velocity (m/s)
%
% OUTPUTS:
%       F_comp      - Total vehicle forces in x,y,z 
%                       size = (length(AoA), 3,length(Beta)
%       M_comp      - Total vehicle moments about the CG in x,y,z 
%                       size = (length(AoA),3,length(Beta)
%
% D.F.B. in Toronto Canada, AUGUST 2020


% INPUTS
filename = 'DJI_Matrice_210_RTK'; % Filename input
AoA = -90:90;         % Angle of attack sweep
Beta = -90:90;        % Beta sweep
Velocity = 15;       % Velocity


%% Initial Setup
% fcnRUN_DIR to be able to either run from the RUN folder or the main
% folder if only this file is added to the search path
fcnRUN_DIR()

% Read input file
[TABLE, GEOM, AIR] = fcnINPUT(filename);
STATE.AOA = 5;


%% Iterate through aoa and beta
for j = 1:length(AoA)
    for i = 1:length(Beta)
        STATE.AOA = AoA(j); % This is not used, just for reference
        
        % Calculate velocity vector based on aoa and beta        
        % *** DOUBLE CHECK THIS LINE
        R = fcnEUL2R([0 AoA(j) Beta(i)],3,1);
        STATE.VEL_B = (R\[Velocity 0 0]')';
%         dir = [cosd(AoA(j)) 0 sind(AoA(j))]+[cosd(Beta(i)) sind(Beta(i)) 0];
%         STATE.VEL_B = Velocity*(dir/norm(dir));
%         STATE.VEL_B = Velocity*[(cosd(AoA(j))*cosd(Beta(i))) sind(Beta(i)) sind(AoA(j))];
        
        % Calculate the velocity magnitude
        STATE.VEL_MAG = Velocity;
        
        % Run VEHPERF to get loads acting on each component 
        % This is where the magic happens
        [PERF, TABLE, GEOM] = fcnVEHPERF(AIR, TABLE, GEOM, STATE);
        
        %% Summing all the vehicle component forces
        % Initialize temp variables
        COMP_DRAG_TOTAL = [];
        COMP_LIFT_TOTAL = [];
        e_L = [];
        r = [];
        if GEOM.VEH.idxBODY == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.BODY.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.BODY.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.BODY,STATE.VEL_B)]; % Get lift directions
            r = [r; fcnMOMARM(GEOM.VEH.BODY)]; % Get moment arm
        end
        % Check and add arm information
        if GEOM.VEH.idxARM == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.ARM.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.ARM.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.ARM,STATE.VEL_B)];
            r = [r; fcnMOMARM(GEOM.VEH.ARM)];
        end
        % Check and add leg information
        if GEOM.VEH.idxLEG == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.LEG.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.LEG.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.LEG,STATE.VEL_B)];
            r = [r; fcnMOMARM(GEOM.VEH.LEG)];
        end
        % Check and add payload information
        if GEOM.VEH.idxPAYLOAD == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.PAYLOAD.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.PAYLOAD.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.PAYLOAD,STATE.VEL_B)];
            r = [r; fcnMOMARM(GEOM.VEH.PAYLOAD)];
        end
        % Check and add motor information
        if GEOM.VEH.idxMOTOR == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.MOTOR.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.MOTOR.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.MOTOR,STATE.VEL_B)];
            r = [r; fcnMOMARM(GEOM.VEH.MOTOR)];
        end
        % Check and add other component information
        if GEOM.VEH.idxOTHER == 1
            COMP_DRAG_TOTAL = [COMP_DRAG_TOTAL; PERF.OTHER.vecDRAG];
            COMP_LIFT_TOTAL = [COMP_LIFT_TOTAL; PERF.OTHER.vecLIFT];
            e_L = [e_L;  fcnLIFTDIR(GEOM.VEH.OTHER,STATE.VEL_B)];
            r = [r; fcnMOMARM(GEOM.VEH.OTHER)];
        end
        
        % Save component force data to OUTP
        OUTP.COMP_DRAG_TOTAL = COMP_DRAG_TOTAL;
        OUTP.COMP_LIFT_TOTAL = COMP_LIFT_TOTAL;
        
        % Calculate down direction and velocity direction (unit vectors)
        e_V = ((STATE.VEL_B/STATE.VEL_MAG)')';
        
        %% Final vehicle force and moment vectors
        % Calcuate total forces in body reference frame (as a vector)
        F_comp(j,:,i) = sum(OUTP.COMP_DRAG_TOTAL.*e_V,1) + sum(OUTP.COMP_LIFT_TOTAL.*e_L,1);
        % Calculate moments due to forces on vehicle components
        M_comp(j,:,i) = sum(cross(r,(OUTP.COMP_LIFT_TOTAL.*e_L + OUTP.COMP_DRAG_TOTAL.*e_V)));
        AoA_grid(j,:,i) = [AoA(j) AoA(j) AoA(j)];
        Beta_grid(j,:,i) = [Beta(i) Beta(i) Beta(i)];
    end
end

%% Plot results
len = numel(AoA_grid(:,1,:));

figure(1)
clf(1)
hold on
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(sum(F_comp,2),len,1,1),'k','markerfacecolor','k')
xlabel('Angle of attack (deg)')
ylabel('Side slip angle (deg)')
zlabel('Total force due to components (N)')
box on
grid on

figure(2)
clf(2)
hold on
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(F_comp(:,1,:),len,1,1),'k','markerfacecolor','k')
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(F_comp(:,2,:),len,1,1),'rd','markerfacecolor','r')
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(F_comp(:,3,:),len,1,1),'bs','markerfacecolor','b')
legend('X-Dir (Drag)','Y-Dir (Side Force)','Z-Dir (Lift)')
xlabel('Angle of attack (deg)')
ylabel('Side slip angle (deg)')
zlabel('Force (N)')
box on
grid on


figure(3)
clf(3)
hold on
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(sum(M_comp,2),len,1,1),'k','markerfacecolor','k')
xlabel('Angle of attack (deg)')
ylabel('Side slip angle (deg)')
zlabel('Total moment about CG due to all components (Nm)')
box on
grid on

figure(4)
clf(4)
hold on
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(M_comp(:,1,:),len,1,1),'k','markerfacecolor','k')
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(M_comp(:,2,:),len,1,1),'rd','markerfacecolor','r')
scatter3(reshape(AoA_grid(:,1,:),len,1,1), reshape(Beta_grid(:,1,:),len,1,1), reshape(M_comp(:,3,:),len,1,1),'bs','markerfacecolor','b')
legend('X-Dir (Roll)','Y-Dir (Pitch)','Z-Dir (Yaw)')
xlabel('Angle of attack (deg)')
ylabel('Side slip angle (deg)')
zlabel('Moment about CG (Nm)')
box on
grid on


