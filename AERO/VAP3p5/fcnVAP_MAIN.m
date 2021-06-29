function [OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAP_MAIN(filename, VAP_IN)
%fcnVAP_MAIN run the full VAP program based on the filename and overwrites
%given in VAP_IN. If no inputs are given, this function will automatically
%run the VAP_MAIN.m file.

% If there are no inputs, run VAP_MAIN
if nargin == 0
    VAP_MAIN;
    return
end

% The VAP Main function is now broken into 3 individual functions to make
% it easier to run one portion of the program at a time. For refernce,
% fcnVAP_MAIN_OG.m has the original function as of the date that this
% division was done.

% Read input, initialize variables, set hardcoded flags, viscous check and
% begin command window display
[FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP] = fcnVAPBEGIN(filename, VAP_IN);

% Disritize geometry, calculate D-matrix, initial resultant vector and
% surface coefficients
[COND, VISC, INPU, VEHI, WAKE, SURF, OUTP, MISC, matD]= fcnVAPINIT(FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP);

% Run timestepping procedure follow by output and plotting functions
[OUTP, COND, INPU, FLAG, MISC, SURF, VEHI, VISC, WAKE] = fcnVAPTIMESTEP(FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP,MISC, matD);