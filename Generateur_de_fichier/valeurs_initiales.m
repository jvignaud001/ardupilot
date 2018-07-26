%% initialisation
% Non du fichier a generer
NomFichier='../ParametresDrone.conf';

% Masse du drone
masse_drone = 1.611; % en kg

% tour/min/V : indique sur le moteur
KV = 1150; 
% en V, tension relevee sur la batterie pour les tests
niveau_tension_batterie = 11.57;

% Envergure du drone quadricoptere modele 2
envergure = 0.256 ;
% Longueur de bras du drone modele 2
longueur_bras = 0.375 ;

% frequence d'echantillonnage (400Hz pour se conformer a la boucle rapide de Ardupilot)
F = 400;        %en Hertz 

% correcteurs roll/pitch/yaw
wu = 20;        %rad/s
wi = wu/3;     %rad/s
Mphase = 60;    %degres
offset_pwm = 0; %en % (entier)
angle_max_roulis_tangage = 20; %en °
vitesse_max_lacet = 45; %en °/s

% nom du fichier log
nom_fichier = "IMS_test";

Te=1/F;
% booleen qui permet de rendre la poussee exponentielle si "true", et lineaire sinon
poussee_exponentielle = "false";

% booleen authorisant ou non l'affichage des debuggeur
affichage_parametres = "false"; 
affichage_consignes = "false";
affichage_erreur = "false";
affichage_AHRS = "false";
affichage_pwm = "false";
affichage_sorties_PID = "false";
affichage_commandes = "false";
test_poussee = "false";
