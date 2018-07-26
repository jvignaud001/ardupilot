%% Cr�ation fichier
FileID=fopen(NomFichier,'wt+');
% Effacement du fichier de sortie qui contiendra les coefficients des PID
fprintf(FileID,'');
fclose(FileID);
%% Ecriture des coefficients bruts dans un fichier nomm� CoefficientPID.conf
fileID = fopen(NomFichier,'a');
% Titre
fprintf(fileID,'-----------------------------------------------------------------------------------------------\n');
fprintf(fileID,'Fichier parametre du drone (a mettre dans le repertoire : /home/pi/ardupilot/build/navio2/bin/)\n');
fprintf(fileID,'-----------------------------------------------------------------------------------------------\n');

% nom du fichier LOG
fprintf(fileID,'\nNom du fichier LOG :\n');
fprintf(fileID,'%s\n',nom_fichier);

% rotation maximale des moteurs
fprintf(fileID,'\nRotation max des moteurs\n');
fprintf(fileID,'%.2f\n',rotation_max);

% coefficient de train�e
fprintf(fileID,'\nCoefficient de trainee :\n');
fprintf(fileID,'%.21f\n',coef_trainee);

% coefficient de pouss�e
fprintf(fileID,'\nCoefficient de poussee :\n');
fprintf(fileID,'%.21f\n',coef_poussee);

% coefficient de pouss�e
fprintf(fileID,'\nEnvergure du drone :\n');
fprintf(fileID,'%.4f\n',envergure);

% masse d'arrachage
fprintf(fileID,'\nMasse d`arrachage :\n');
fprintf(fileID,'%.3f\n',masse_arrachage);

% correcteur roulis
fprintf(fileID,'\nSuite recursive pour le correcteur du roulis :\n');
fprintf(fileID,'yn = %f.x(n)+ %f.x(n-1)+ %f.x(n-2)+ %f.y(n-1)+ %f.y(n-2)\n',cxn_roll,cxn_1_roll,cxn_2_roll,cyn_1_roll,cyn_2_roll);
fprintf(fileID,'\n%s\n','Coefficient du roulis : xn, xn-1, xn-2, yn-1, yn-2');
fprintf(fileID,'%.16f\n',cxn_roll,cxn_1_roll,cxn_2_roll,cyn_1_roll,cyn_2_roll);

% correcteur tangage
fprintf(fileID,'\nSuite recursive pour le correcteur du tangage :\n');
fprintf(fileID,'yn = %f.x(n)+ %f.x(n-1)+ %f.x(n-2)+ %f.y(n-1)+ %f.y(n-2)\n',cxn_pitch,cxn_1_pitch,cxn_2_pitch,cyn_1_pitch,cyn_2_pitch);
fprintf(fileID,'\n%s\n','Coefficient du tangage : xn, xn-1, xn-2, yn-1, yn-2');
fprintf(fileID,'%.16f\n',cxn_pitch,cxn_1_pitch,cxn_2_pitch,cyn_1_pitch,cyn_2_pitch);

% correcteur vitesse du lacet
fprintf(fileID,'\nSuite recursive pour le correcteur de la vitesse du lacet :\n');
fprintf(fileID,'yn = %f.x(n)+ %f.x(n-1)+ %f.y(n-1)\n',cxn_yaw,cxn_1_yaw,cyn_1_yaw);
fprintf(fileID,'\n%s\n','Coefficient de la vitesse du lacet : xn, xn-1, yn-1');
fprintf(fileID,'%.16f\n',cxn_yaw,cxn_1_yaw,cyn_1_yaw);

% correcteur des consignes smooth
fprintf(fileID,'\nSuite recursive pour la fonction smooth de la RC :\n');
fprintf(fileID,'yn = %f.x(n)+ %f.x(n-1)+ %f.x(n-2)+ %f.y(n-1)+ %f.y(n-2)\n',cxn_smooth,cxn_1_smooth,cxn_2_smooth,cyn_1_smooth,cyn_2_smooth);
fprintf(fileID,'\n%s\n','Coefficient des consignes smooth : xn, xn-1, xn-2, yn-1, yn-2');
fprintf(fileID,'%.16f\n',cxn_smooth,cxn_1_smooth,cxn_2_smooth,cyn_1_smooth,cyn_2_smooth);

% offset du pwm min
fprintf(fileID,'\nOffset du PWM min (en pourcent) (entier) :\n');
fprintf(fileID,'%d\n',offset_pwm);

% angle maximal de roulis et de tangage
fprintf(fileID,'\nAngle maximal de roulis et de tangage (en °) :\n');
fprintf(fileID,'%f\n',angle_max_roulis_tangage);

% vitesse maximale du lacet
fprintf(fileID,'\nVitesse maximale de lacet (en °/s) :\n');
fprintf(fileID,'%f\n',vitesse_max_lacet);

% pouss�e exponentielle ou lin�aire
fprintf(fileID,'\nPoussee exponentielle si "true" et poussee lineaire sinon :\n');
fprintf(fileID,'%s\n',poussee_exponentielle);

% affichage des bool�ens pour l'affichage du d�buggeur
fprintf(fileID,'\n--------------------------------------------------');
fprintf(fileID,'\nAffichage du debuggeur :\n');
fprintf(fileID,'--------------------------------------------------\n');
fprintf(fileID,'\nAffichage des parametres du drone :\n');
fprintf(fileID,'%s\n',affichage_parametres);
fprintf(fileID,'\nAffichage des consignes :\n');
fprintf(fileID,'%s\n',affichage_consignes);
fprintf(fileID,'\nAffichage des erreurs :\n');
fprintf(fileID,'%s\n',affichage_erreur);
fprintf(fileID,'\nAffichage des donnees du AHRS :\n');
fprintf(fileID,'%s\n',affichage_AHRS);
fprintf(fileID,'\nAffichage des PWM :\n');
fprintf(fileID,'%s\n',affichage_pwm);
fprintf(fileID,'\nAffichage des sorties des PID :\n');
fprintf(fileID,'%s\n',affichage_sorties_PID);
fprintf(fileID,'\nAffichage des commandes :\n');
fprintf(fileID,'%s\n',affichage_commandes);

% Activation du mode "poussee"
fprintf(fileID,'\n--------------------------------------------------');
fprintf(fileID,'\nActivation du mode "poussee" :\n');
fprintf(fileID,'--------------------------------------------------\n');
fprintf(fileID,'%s\n',test_poussee);

fclose(fileID);