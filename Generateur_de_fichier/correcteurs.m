%% Calculs des correcteurs
%% modèles
G_phi = (1/Ix)/p^2;     %modèle du roulis
G_theta = (1/Iy)/p^2;   %modèle du tangage
G_r = (1/Iz)/p;         %modèle de la vitesse du lacet
%% Calculs du correcteur du roulis
[cxn_roll,cxn_1_roll,cxn_2_roll,cyn_1_roll,cyn_2_roll] = calcul_correcteur_PID(wu,wi,Mphase,G_phi,F);

%% Calculs du correcteur du tangage
[cxn_pitch,cxn_1_pitch,cxn_2_pitch,cyn_1_pitch,cyn_2_pitch] = calcul_correcteur_PID(wu,wi,Mphase,G_theta,F);

%% Calculs du correcteur de la vitesse du lacet
[cxn_yaw,cxn_1_yaw,cyn_1_yaw] = calcul_correcteur_PI(wu,wi,G_r,F);

%% Calculs du correcteur des consignes smooth
%% choix des paramètres de la fonction de transfert du 2° ordre
[cxn_smooth,cxn_1_smooth,cxn_2_smooth,cyn_1_smooth,cyn_2_smooth] = calcul_fct_smooth(10,1,F);
