clc
clear
warning off

% filename = 'inputs/X57_Cruise.vap';
% filename = 'inputs/TMotor.vap';
filename = 'inputs/Matrice_210_RTK_Rotor.vap';
% filename = 'inputs/Goland_Wing.vap';
% alpha_seq = -5:10;

% for i = 1:length(alpha_seq)
    
    VAP_IN = [];
    VAP_IN.valMAXTIME = 20;
    VAP_IN.valSTARTFORCES = 20;
    VAP_IN.RELAX = 1;
    VAP_IN.valTIMETRUNC = 5;
    VAP_IN.TRUNCATE = 1;    
    [OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAP_MAIN(filename, VAP_IN);

    load(strcat(MISC.timestep_folder, 'timestep_', num2str(VAP_IN.valMAXTIME ), '.mat'))
    valBEGINTIME = COND.valMAXTIME+1;
    COND.valMAXTIME = VAP_IN.valMAXTIME +5;
    [OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAPTIMESTEP(FLAG, COND, VISC,INPU,VEHI,WAKE,SURF,OUTP,MISC,valBEGINTIME);
    %Comparing new fcnVAP_MAIN setup to old
    VAP_IN.valMAXTIME = 25;
    [OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAP_MAIN_OG(filename, VAP_IN);

    
    
%     CL(i) = OUTP.vecCL(end);
%     CDi(i) = OUTP.vecCDI(end);
    
% end
% save('Sharp_Edge.mat')