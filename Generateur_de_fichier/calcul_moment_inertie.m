function moment_inertie = calcul_moment_inertie(periode, masse, longueur_bras_ou_envergure, longueur_fil_suspendu )
%CALCUL_MOMENT_INERTIE
%   Cette fonction permet de calculer le moment d'inertie du drone. Elle
%   prend en parametre la periode mesuree par les experiences, la masse
%   (globale) du drone, ainsi que l'envergure ou la longueur du bras et la
%   longueur du fil qui permet de suspendre le drone.
    g = 9.80665; % en m/s^2
    m = masse; % en kg
    T = periode; % en s
    L = longueur_bras_ou_envergure + longueur_fil_suspendu; % longueur entre le centre d'inertie du drone et le point de suspension
    moment_inertie = (m*g*L*T^2)/(4*pi^2)-m*L^2;
end

