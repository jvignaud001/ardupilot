function [cxn,cxn_1,cyn_1] = calcul_correcteur_PI(omega_u,omega_i,G,Frequence)
%CALCUL_CORRECTEUR_PI Summary of this function goes here
%   Detailed explanation goes here
    wu = omega_u;
    wi = omega_i;
    F = Frequence; 
    p = tf('s');
    %récupération de la phase et du gain du modèle en wu
    [G0,phi0] = bode(G,wu);
    %gain du correcteur
    C0 = 1/G0*(wu/wi)/(sqrt(1+(wu/wi)^2));
    %correcteur intégral
    Ci = (1 + p/wi)/(p/wi);
    %correcteur en temps continu
    C = C0 * Ci;
    %correcteur en temps discret
    C_discret = c2d(C,1/F,'tustin');
    % Récupération du numérateur de la transmittance en Z du correcteur PID
    num=C_discret.num{1}; 
    % Récupération du dénominateur de la transmittance en Z du correcteur PID
    den=C_discret.den{1}; 

    cxn = num(1)/den(1);
    cxn_1 = num(2)/den(1);
    cyn_1 = -den(2)/den(1);
end

