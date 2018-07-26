%% Calculs des moments d'inertie
%% Relevé pour le moment Ix
nbPeriodes = 25;
periode1 = (36.782 - 2.935)/nbPeriodes;
periode2 = (35.983 - 2.130)/nbPeriodes;
periode = (periode1 + periode2)/2;
longueur_fil = 0.145; % longueur entre le point d'attache du drone et le support
Ix = calcul_moment_inertie(periode, masse_drone, envergure, longueur_fil);
%% Relevé pour le moment Iy
nbPeriodes = 25;
periode1 = (36.784 - 3.510)/nbPeriodes;
periode2 = (36.562 - 3.283)/nbPeriodes;
periode = (periode1 + periode2)/2;
longueur_fil = 0.145; % longueur entre le point d'attache du drone et le support
Iy = calcul_moment_inertie(periode, masse_drone, envergure, longueur_fil);
%% Relevé pour le moment Iz
nbPeriodes = 20;
periode1 = (32.237 - 2.012)/nbPeriodes;
periode2 = (33.383 - 3.337)/nbPeriodes;
periode3 = (32.168 - 2.359)/nbPeriodes;
periode = (periode1 + periode2 + periode3)/3;
longueur_fil = 0.145; % longueur entre le point d'attache du drone et le support 
Iz = calcul_moment_inertie(periode, masse_drone, longueur_bras, longueur_fil);