% Airfoil coefficient lookup function.
% Uses airfoil database format
% Input:  options: Info about database resolution
%         database_coeffs: aero_coeff_database
%         AoA: angle of attack
%         Re: Reynolds number
%         airfoil: airfoil
% Output: [c_l,c_d,c_m]

function [c_l,c_d,c_m] = lookup_aero_coeff_database(options,airfoil_coeff_database,AoA,Re,airfoil)

% Preallocation
c_l = zeros(size(AoA,1),size(AoA,2));
c_d = zeros(size(AoA,1),size(AoA,2));
c_m = zeros(size(AoA,1),size(AoA,2));

    % Round AoA to the nearest AoAresolution from options, for database
    AoA = round(rad2deg(AoA)./options.AoAresolution)*options.AoAresolution;

    % Round Reynolds number to the nearest REresolution from options, for database
    Re = round(Re./options.REresolution)*options.REresolution;
    
    % Find row from options.REresolution, options.AoAresolution and options.airfoil
    idx1 = repmat([airfoil_coeff_database.REresolution]' == options.REresolution,1,size(airfoil,1));
    idx2 = repmat([airfoil_coeff_database.AoAresolution]' == options.AoAresolution,1,size(airfoil,1));
    temp1 = repmat({airfoil_coeff_database.airfoil}',1,size(airfoil,1));
    temp2 = repmat(airfoil',size({airfoil_coeff_database.airfoil},2),1);
    idx3 = strcmp(temp1,temp2);
    [row_index_in_database,~] = find(idx1 & idx2 & idx3);
%     for k = 1:size({airfoil_coeff_database.airfoil},2)
%         Re_database(k,:) = airfoil_coeff_database(k).interp_coeffs.Re;
%         Alpha_database(k,:) = airfoil_coeff_database(row_index_in_database(k)).interp_coeffs.alpha;
%     end
%     
    for i = 1:size(AoA,2)
        for j = 1:size(AoA,1)
            idx1 = airfoil_coeff_database(row_index_in_database(j)).interp_coeffs.Re == Re(j,i);
            idx2 = abs(airfoil_coeff_database(row_index_in_database(j)).interp_coeffs.alpha - AoA(j,i));
            row_in_coeff_array = find(idx1 & idx2 <10^-6);
            
            c_l(j,i) = airfoil_coeff_database(row_index_in_database(j)).interp_coeffs.CL(row_in_coeff_array);
            c_d(j,i) = airfoil_coeff_database(row_index_in_database(j)).interp_coeffs.CD(row_in_coeff_array);
            c_m(j,i) = airfoil_coeff_database(row_index_in_database(j)).interp_coeffs.CM(row_in_coeff_array);
        end
    end
end