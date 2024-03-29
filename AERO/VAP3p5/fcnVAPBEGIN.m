function [FLAG, COND, VISC, INPU, VEHI, WAKE, SURF, OUTP, MISC] = fcnVAPBEGIN(filename, VAP_IN)
%fcnVAPBEGIN Run the beginning portion of the VAP program where input is
%read, flags are set, parameters are initialized, display is setup and
%viscous is checked
%
% This is based on the format by M. M. in the VAP3 FLEX branch
%
% INPUTS are the same typical inputs to VAP_MAIN



warning off

if nargin == 1
    VAP_IN = [];
end
%% Reading in geometry
[FLAG, COND, VISC, INPU, VEHI, SURF] = fcnXMLREAD(filename, VAP_IN);

FLAG.PRINT = 0;
FLAG.OLDPRINT = 1;
FLAG.PLOT = 1;
FLAG.VISCOUS = 0;
FLAG.CIRCPLOT = 0;
FLAG.GIF = 0;
FLAG.PREVIEW = 0;
FLAG.PLOTWAKEVEL = 0;
FLAG.PLOTUINF = 0;
FLAG.VERBOSE = 0;
FLAG.SAVETIMESTEP = 1;
FLAG.HOVERWAKE = 0;
FLAG.NACELLE = 0;
FLAG.GPU = 0;
%     FLAG.TRUNCATE = 1;
%     INPU.valTIMETRUNC = 5;
FLAG.STRUCTURE = 0; % Structure code will not work

% Initializing parameters to null/zero/nan
[WAKE, OUTP, INPU, SURF, MISC] = fcnINITIALIZE(COND, INPU, SURF);
INPU.filename = filename;

if FLAG.OLDPRINT == 1
    disp('============================================================================');
    disp('                  /$$    /$$  /$$$$$$  /$$$$$$$         /$$$$$$     /$$$$$$$')
    disp('+---------------+| $$   | $$ /$$__  $$| $$__  $$       /$$__  $$   | $$____/') ;
    disp('| RYERSON       || $$   | $$| $$  \ $$| $$  \ $$      |__/  \ $$   | $$      ');
    disp('| APPLIED       ||  $$ / $$/| $$$$$$$$| $$$$$$$/         /$$$$$/   | $$$$$$$ ');
    disp('| AERODYNAMICS  | \  $$ $$/ | $$__  $$| $$____/         |___  $$   |_____  $$');
    disp('| LABORATORY OF |  \  $$$/  | $$  | $$| $$             /$$  \ $$    /$$  \ $$');
    disp('| FLIGHT        |   \  $/   | $$  | $$| $$            |  $$$$$$//$$|  $$$$$$/');
    disp('+---------------+    \_/    |__/  |__/|__/             \______/|__/ \______/ ');
    disp('============================================================================');
    disp(' ');
end

% Setting up timestep saving feature
if FLAG.SAVETIMESTEP == 1
    if exist('timesteps/') ~= 7; mkdir('timesteps'); end
    if isfield(VAP_IN,'TimestepName')
        MISC.timestep_folder = strcat('timesteps/',VAP_IN.TimestepName,'/');
    else
        MISC.timestep_folder = ['timesteps/',regexprep(INPU.filename,{'inputs/', '.vap'}, ''), '_(', datestr(now, 'dd_mm_yyyy HH_MM_SS_FFF'),')/'];
    end
    mkdir(MISC.timestep_folder);
end

% Check if the files required by the viscous calculations exist
[FLAG] = fcnVISCOUSFILECHECK(FLAG, VISC);
