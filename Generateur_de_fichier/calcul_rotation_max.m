function rotation_maximale = Fct_rotation_max(Kv, tension_batterie)
%CALCUL_ROTATION_MAX 
%   La fonction permet de retourner la rotation maximale a partir duKv
%   inscrit sur les moteurs et le niveau de tension maximale de la batterie
    rotation_maximale = Kv * tension_batterie; % en tour/min
    rotation_maximale = rotation_maximale*2*pi/60;
end

