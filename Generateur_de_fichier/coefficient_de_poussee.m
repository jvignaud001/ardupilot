%% Calculs du coefficient de trainee
%% Relev�s expr�rimentaux
temps_experimentaux = [19.750 22.224 24.678 27.193 29.668];
coef_trainee = calcul_coef_trainee(temps_experimentaux, rotation_max, Iz);
%% Calculs du coefficient de poussee
%% r�sultat des exp�riences
p1x =[0 11 25 38 60 73 86]; % en pourcent
p1y =[3999 3550 3070 2540 1745 1170 650]; % en gramme

%% Calculs des abscisses et ordonn�e pour la pouss�e
[rotation_carre1,force_poussee_moteurs1] = calcul_abscisse_ordonnee_poussee(p1x,p1y,rotation_max,4);

%% coefficient de poussee
coef_poussee = calcul_coef_poussee(rotation_carre1, force_poussee_moteurs1);  % moindre au carr� : calcul du coef de poussee

%% Trac� de la force de pouss�e en fonction de la rotation
% calcul th�orique du coefficient de poussee? 
% http://aerotrash.over-blog.com/2015/01/calcul-de-la-poussee-d-une-helice.html
pas = 4.5;  % en pouce
diametre = 10; % en pouce
coef_poussee_bis = calcul_coef_poussee_theo(pas,diametre);

%% masse d'arrachage
masse_arrachage = calcul_masse_arrachage(coef_poussee,rotation_carre1(length(rotation_carre1))); % affichage de la masse d'arrachage maximale que fournie le drone

%% Trac� de la force de pouss�e en fonction de la vitesse au carr�e (les deux courbes)
g = 9.80665; % m/s�
figure, plot(rotation_carre1,force_poussee_moteurs1,'b--o',[0 rotation_carre1(length(rotation_carre1))],[0 coef_poussee*rotation_carre1(length(rotation_carre1))],'m',[0 rotation_carre1(length(rotation_carre1))],[masse_drone*g/4 masse_drone*g/4],'g'),grid on;
legend('courbe premier tableau','courbe moyenne','Pouss�e n�cessaire par moteur pour comptenser le poids du drone');
xlabel('Rotation au carr�e {\Omega}^2 (en rad^2/s^2)'),ylabel('Force de pouss�e (en Newton)');
title('Coefficient de pouss�e calcul� par moteur');