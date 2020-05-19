function VORTPARAM = fcnRINGGEOM(VORTPARAM, num_seg, num_ring, R, rotor_centers,rotor_vel)

% Calcualte the segments for the first circle than propogate it in V_inf
% direction


% Calculate number of rotors based on the number of rotor centers
num_rotors = size(rotor_centers,1);

%% Create a segments of a single ring about the origin given number of
% segments and rotor ratius
angles = linspace(0,2*pi,num_seg+1);

x = R.*sin(angles)';
y = R.*cos(angles)';

seg_start = [x(1:end-1), y(1:end-1), zeros(num_seg,1)];
seg_end = [x(2:end), y(2:end), zeros(num_seg,1)];

seg_start = repmat(seg_start,num_rotors*num_ring,1);
seg_end = repmat(seg_end,num_rotors*num_ring,1);

%% Calculate the z location of the rings
z_loc = repmat(-1*VORTPARAM.z',num_seg,1,num_ring);
temp = permute(repmat((1:num_ring)',1,size(z_loc,2),num_seg),[3 2 1]);
z_loc = z_loc.*temp;
% *** This reshape is used consistently throughout rest of this function in
% order to ensure the order of points is consistent 
z_loc = reshape(z_loc,[num_rotors*num_ring*num_seg,1]);


rotor_num = repmat(1:num_rotors,num_seg,1,num_ring);
rotor_num = reshape(rotor_num,[num_rotors*num_ring*num_seg,1]);


seg_start(:,3) = z_loc;
seg_end(:,3) = z_loc;

%% Move the rings based on the rotor centers
for i = 1:num_rotors
    seg_start(rotor_num == i,:) = seg_start(rotor_num == i,:)+rotor_centers(i,:);
    seg_end(rotor_num == i,:) = seg_end(rotor_num == i,:)+rotor_centers(i,:);
    
end

%% Move the rings based on the skew angle and freestream direction

chi = repmat(VORTPARAM.chi',num_seg,1,num_ring);
chi = reshape(chi,[num_rotors*num_ring*num_seg,1]);
%Calculate displancement in x,y plane
displacement = z_loc.*sin(chi);

% Calculate unit velocity vector is x y plane
unit_vel = rotor_vel(:,1:2)./sqrt(rotor_vel(:,1).^2+rotor_vel(:,1).^2);
x_vel_dir = repmat(unit_vel(:,1)',num_seg,1,num_ring);
x_vel_dir = reshape(x_vel_dir,[num_rotors*num_ring*num_seg,1]);
y_vel_dir = repmat(unit_vel(:,2)',num_seg,1,num_ring);
y_vel_dir = reshape(y_vel_dir,[num_rotors*num_ring*num_seg,1]);

seg_start(:,1) = seg_start(:,1)+displacement.*x_vel_dir;
seg_start(:,2) = seg_start(:,2)+displacement.*y_vel_dir;

seg_end(:,1) = seg_end(:,1)+displacement.*x_vel_dir;
seg_end(:,2) = seg_end(:,2)+displacement.*y_vel_dir;

%% Assign start and end points to vortparam structure
VORTPARAM.seg_start = seg_start;
VORTPARAM.seg_end = seg_end;
VORTPARAM.rotor_num = rotor_num;

% Update circulation to correct size
VORTPARAM.matCIRC = VORTPARAM.circ(rotor_num);

end