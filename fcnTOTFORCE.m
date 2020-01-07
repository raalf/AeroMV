function [Force, Moment] = fcnTOTFORCE(PERF)
% This function totals all forces and moment acting on the vehicle

F_rotor_ind = [[PERF.ROTOR.Nx]' [PERF.ROTOR.Ny]' [PERF.ROTOR.T]'];
F_rotor = sum(F_rotor_ind,1);

M_rotor_ind = [[PERF.ROTOR.Mx]' [PERF.ROTOR.My]' [PERF.ROTOR.Q]'];
M_rotor = sum(M_rotor_ind,1);

F_comp = [];
if fcnCOMPCHECK(PERF, 'BODY')
   F_Body = PERF.BODY.CL;
end
end

