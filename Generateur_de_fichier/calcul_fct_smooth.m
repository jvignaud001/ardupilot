function [cxn,cxn_1,cxn_2,cyn_1,cyn_2] = calcul_fct_smooth(pulstation_coupure,coef_amortissement,Frequence)
%CALCUL_FCT_SMOOTH
%   Calcul de la suite pour la fonction smooth
    p = tf('s');
    xi = coef_amortissement;
    w0 = pulstation_coupure;
    F = Frequence;
    H = 1/(p^2/w0^2 + 2*xi/w0*p + 1);
    % figure, step(H), title('Reponse d un échelon en temps continu');
    H_discret = c2d(H,1/F,'tustin');
    % figure, step(H_discret),title('Reponse d un échelon en temps discret');
    num=H_discret.num{1}; % Récupération du numérateur de la transmittance en Z du correcteur PID
    den=H_discret.den{1}; % Récupération du dénominateur de la transmittance en Z du correcteur PID
    cxn=num(1)/den(1);
    cxn_1 = num(2)/den(1);
    cxn_2 = num(3)/den(1);
    cyn_1= -den(2)/den(1);
    cyn_2 = -den(3)/den(1);
end

