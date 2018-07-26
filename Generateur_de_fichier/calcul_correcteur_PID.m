function [cxn,cxn_1,cxn_2,cyn_1,cyn_2] = calcul_correcteur_PID(omega_u,omega_i,Marge_phase,G,Frequence)
%CALCUL_CORRECTEUR_PID Summary of this function goes here
%   Detailed explanation goes here
F = Frequence;
wu = omega_u;
wi = omega_i;
p = tf('s');
%récupération de la phase et du gain du modèle en wu
[G0,phi0] = bode(G,wu);
%calcul de la phase du correcteur intégral
phii = atan(wu/wi)*180/pi - 90;
%calcul de la phase du correcteur à avance de phase
phim = -180 + Marge_phase - phi0 - phii;

a = (1 + sin(phim*pi/180))/(1 - sin(phim*pi/180));
%pulsation basse du correcteur à avance de phase
wb = wu/sqrt(a);
%pulsation haute du correcteur à avance de phase
wh = wu * sqrt(a);
%gain du correcteur
C0 = 1/(G0*sqrt(a))*(wu/wi)/(sqrt(1+(wu/wi)^2));
%correcteur à avance de phase
Cav = (1 + p/wb)/(1 + p/wh);
%correcteur intégral
Ci = (1 + p/wi)/(p/wi);
%correcteur en temps continu
C = C0 * Cav * Ci;
%correcteur en temps discret
C_discret = c2d(C,1/F,'tustin');
% Récupération du numérateur de la transmittance en Z du correcteur PID
num=C_discret.num{1}; 
% Récupération du dénominateur de la transmittance en Z du correcteur PID
den=C_discret.den{1};
 
cxn = num(1)/den(1);
cxn_1 = num(2)/den(1);
cxn_2 = num(3)/den(1);
cyn_1 = -den(2)/den(1);
cyn_2 = -den(3)/den(1);
end

