/*
 * control_ims4.cpp
 *
 *  Created on: Jun 8, 2018
 *      Author: Daniel Monier-Reyes
 *
 *
 * Routines d'initialisation et d'appel pour le mode de vol IMS4
 * Ajout des vérifications des moteurs à l'arrêt et du coefficient multiplicateur de poussée
 * afin d'avoir plus ou moins la même vitesse en vol entre le mode Stabilize et IMS.
 * 
 */

#include "Copter.h"
#include "ims.h"
#include <time.h>

#define LOG_TIME 600

#define OFFSET_PWM 65


// --------------------------------------------------------------------
// Déclaration des variables Globales
// --------------------------------------------------------------------

// PID ROLL : y(n)=32.2486.x(n)-64.1172.x(n-1)+31.8696.x(n-2)+1.8397.y(n-1)-0.83971.y(n-2)
Correcteur_2nd_Ordre_Discret pid_roll4(32.2486,-64.1172,31.8696,1.8397,-0.83971);

// PID PITCH : y(n)=35.9066.x(n)-71.3902.x(n-1)+35.4847.x(n-2)+1.8397.y(n-1)-0.83971.y(n-2)
Correcteur_2nd_Ordre_Discret pid_pitch4(35.9066,-71.3902,35.4847,1.8397,-0.83971);

// PI R : y(n)=0.65646.x(n)-0.654.x(n-1)+1.y(n-1)
Correcteur_1er_Ordre_Discret pi_yaw4(0.65646,-0.654,1);

// ofstream est utilisé pour écrire un fichier CSV nommé IMS4_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
std::ofstream outf4;

// Variable permettant de récupérer les valeurs précédentes des PWM (pas utile : gérer directement par les correcteurs)
//int16_t  w1_pwm_prec = 0,w2_pwm_prec = 0,w3_pwm_prec = 0,w4_pwm_prec = 0;

// ---------------------------------------------------------------------------------------------
// ims4_init - Routine d'initialisation du mode de vol IMS4
// ---------------------------------------------------------------------------------------------
bool Copter::ims4_init(bool ignore_checks)
{
    // Initialisation des Offset AHRS
    offset_ahrs_roll=ahrs.roll;
    offset_ahrs_pitch=ahrs.pitch;
    offset_ahrs_yaw=ahrs.get_gyro().z;

    // Récupération des valeurs min et max pwm pour la rotation des moteurs
    pwm_min=copter.motors.get_pwm_output_min();
    pwm_max=copter.motors.get_pwm_output_max();

    // Code d'origine
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }

    // Code d'origine
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// ---------------------------------------------------------------------------------------------
// ims4_init - Routine d'appel du mode de vol IMS4 à exécuter à 400Hz
// ---------------------------------------------------------------------------------------------
void Copter::ims4_run()
{
    // --------------------------------------------------------------------
    // Déclaration des variables
    // --------------------------------------------------------------------

    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // Consignes
    double target_roll_rad, target_pitch_rad, target_yaw_rate_rad; // Target angles en radians et radians/s
    double target_throttle_newton;                                 // Target poussée en Newton

    // ------------------------------------------------------------------------
    // Programme d'origine permettant de récupérer les consignes
    // ------------------------------------------------------------------------

    // Récupération des consignes roulis et tangage (en centidegrés)
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // Récupération de la consigne en lacet (en centidegrés par seconde)
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // Récupération de la consigne en poussée (valeur entre 0 et 1)
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // ------------------------------------------------------------------------
    // Adaptation des valeurs des consignes Roll, Pitch, Yaw(R), Throttle en radian et radian/s
    // ------------------------------------------------------------------------

    // Conversion des consignes de la radiocommande Roll/Pitch de centi-degrés en radians
    target_roll_rad=double((target_roll*M_PI)/18000);
    target_pitch_rad=double((target_pitch*M_PI)/18000);
    // Conversion des consignes de la radiocommande YAW de centi-degrés par seconde en radians par seconde
    target_yaw_rate_rad=double((target_yaw_rate*M_PI)/18000);
    // Conversion des consignes de la radiocommande Throttle de centi-pourcentage en Newton
    target_throttle_newton=double(pilot_throttle_scaled*2.95*GRAVITY_MSS);

    // ----------------------------------------------------------------------------------------
    // Affichage de la sortie de l'AHRS et des consignes
    // ----------------------------------------------------------------------------------------

    // Affichage des consignes Roll, Pitch, Yaw, Throttle
    //hal.console->printf("Consignes - Roll: %f° Pitch: %f° Yaw: %f°/s Throttle %fN\n",target_roll_rad*180/M_PI,target_pitch_rad*180/M_PI,target_yaw_rate_rad*180/M_PI, target_throttle_newton);

    // Affichage des sorties de l'AHRS
    //hal.console->printf("AHRS - Roll: %f Pitch:%f R:%f\n",ahrs.roll, ahrs.pitch, ahrs.get_gyro().z);

    // ------------------------------------------------------------------------
    // Spécifique Ardupilot - Gestion de l'armement des moteurs
    // ------------------------------------------------------------------------

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // Reset des PIDs pour éviter qu'ils divergent
        pid_roll4.reset();
        pid_pitch4.reset();
        pi_yaw4.reset();

        if (fichier_log_ouvert==true) { // Si le fichier de log est ouvert alors le fermer
            outf4.close();
            fichier_log_ouvert=false;
        }

        return;
    } else // Sinon cela veut dire que les moteurs sont armés et que le drone est prêt à décoller
    {
        if (fichier_log_ouvert==false) { // Si le fichier de log n'est pas encore ouvert alors l'ouvrir

            // Construction de la date et de l'heure pour rendre le nom du fichier de log unique
            time_t rawtime;
            struct tm * timeinfo;
            char buffer [30];
            char log_file_name [80];

            time (&rawtime);
            timeinfo = localtime (&rawtime);

            strftime (buffer,30,"%F--%H-%M-%S",timeinfo);

            /*création du fichier à l'emplacement '/home/pi/ardupilot/build/navio2/bin/' sur le drone
            avec pour nom : "IMS4_CSV_LOG-<année>-<mois>-<jour>--<heure>-<minute>-<seconde>.dat"*/
            strcpy(log_file_name,"/home/pi/ardupilot/build/navio2/bin/IMS4_CSV_LOG-");
            strcat(log_file_name,buffer);
            strcat(log_file_name,".dat");

            outf4.open(log_file_name); // Création d'un fichier de log unique

            // Ecriture d'une entête pour savoir à quoi correspond les données
            outf4 << "AHRS.Roll,AHRS.Pitch,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,Uz,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm" << std::endl;

            fichier_log_ouvert=true;
        }
    }


    // Code d'origine - Réinitialisation du flag d'atterissage
    set_land_complete(false);

    // ------------------------------------------------------------------------
    // Calculs des PIDs avec offsets de compensation (car l'AHRS est calibrée à la main et donc
    // les valeurs des accéléromètres et gyroscopes ne sont jamais à 0 en position de repos)
    // Le x(n) de chaque PID prend la valeur calculée entre parenthèses et qui correspond à l'erreur
    // c'est à dire la valeur de la consigne moins la sortie de l'AHRS pour chaque axe concerné
    // L'opération cycle permet de décaler les valeurs successives de x(n) et y(n) dans le temps
    // ------------------------------------------------------------------------

    // Calcul PID Roll
    pid_roll4.cycle(target_roll_rad-(ahrs.roll-offset_ahrs_roll));
    // Calcul PID Pitch
    pid_pitch4.cycle(target_pitch_rad-(ahrs.pitch-offset_ahrs_pitch));
    // Calcul PID Yaw
    pi_yaw4.cycle(target_yaw_rate_rad-(ahrs.get_gyro().z-offset_ahrs_yaw));

    // Calcul PID Roll - Version sans prise en compte des offset de calibrage de l'AHRS au niveau du calcul de l'erreur
    //pid_roll4.cycle(target_roll_rad-ahrs.roll);
    // Calcul PID Pitch
    //pid_pitch4.cycle(target_pitch_rad-ahrs.pitch);
    // Calcul PID Yaw
    //pi_yaw4.cycle(target_yaw_rate_rad-ahrs.get_gyro().z);

    // Assignation des sorties de PIDs
    u_phi=pid_roll4.getyn();
    u_theta=pid_pitch4.getyn();
    u_r=pi_yaw4.getyn();
    u_z=-target_throttle_newton/(cosf(ahrs.roll-offset_ahrs_roll)*cosf(ahrs.pitch-offset_ahrs_pitch));

    // Calcul de la valeur des commandes
    w1=sqrt(constrain_float((d*u_phi+d*u_theta-b*l*u_r-d*l*u_z)/(b*d*l),0,10000000))/2;
    w2=sqrt(constrain_float(-(d*u_phi+d*u_theta+b*l*u_r+d*l*u_z)/(b*d*l),0,10000000))/2;
    w3=sqrt(constrain_float(-(d*u_phi-d*u_theta-b*l*u_r+d*l*u_z)/(b*d*l),0,10000000))/2;
    w4=sqrt(constrain_float((d*u_phi-d*u_theta+b*l*u_r-d*l*u_z)/(b*d*l),0,10000000))/2;

    // Calcul des valeurs de PWM à envoyer à chaque moteur en fonction de w1, w2, w3, w4 - A commenter pour les tests de poussée
    w1_pwm=(w1/ROTATION_MAX)*(pwm_max-(pwm_min+OFFSET_PWM))+pwm_min+OFFSET_PWM;
    w2_pwm=(w2/ROTATION_MAX)*(pwm_max-(pwm_min+OFFSET_PWM))+pwm_min+OFFSET_PWM;
    w3_pwm=(w3/ROTATION_MAX)*(pwm_max-(pwm_min+OFFSET_PWM))+pwm_min+OFFSET_PWM;
    w4_pwm=(w4/ROTATION_MAX)*(pwm_max-(pwm_min+OFFSET_PWM))+pwm_min+OFFSET_PWM;

    // A décommenter pour les tests de poussée
    //w1_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w2_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w3_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w4_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;

    
    // --------------------------------------------------------------------
    // Test pwm des moteurs
    // --------------------------------------------------------------------

    // Test pour protection des moteurs
    if (w1_pwm > pwm_max)
        w1_pwm = pwm_max;
    if (w2_pwm > pwm_max)
        w2_pwm = pwm_max;
    if (w3_pwm > pwm_max)
        w3_pwm = pwm_max;
    if (w4_pwm > pwm_max)
        w4_pwm = pwm_max;


    // Si arret d'un seul moteur, alors on le fait tourner à la rotation minimale (les autres moteurs seront corrigés par les PIDs)
    if (w1_pwm < pwm_min + OFFSET_PWM  && w2_pwm >= pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM)
        w1_pwm = pwm_min+OFFSET_PWM;

    if (w1_pwm >= pwm_min + OFFSET_PWM  && w2_pwm < pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM)
        w2_pwm = pwm_min+OFFSET_PWM;

    if (w1_pwm >= pwm_min + OFFSET_PWM  && w2_pwm >= pwm_min + OFFSET_PWM && w3_pwm < pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM)
        w3_pwm = pwm_min+OFFSET_PWM;

    if (w1_pwm >= pwm_min + OFFSET_PWM  && w2_pwm >= pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM && w4_pwm < pwm_min + OFFSET_PWM)
        w4_pwm = pwm_min+OFFSET_PWM;
    

    // Si arret de deux moteurs sur quatre, alors on les fait tourner à la rotation minimale (les autres moteurs seront corrigés par les PIDs)
    if (w1_pwm < pwm_min + OFFSET_PWM  && w2_pwm < pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM) { // demande d'un lacet max à droite
        w1_pwm = pwm_min+OFFSET_PWM;
        w2_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 1 et 2 à la valeur pwm_min
        //w3_pwm = w3_pwm_prec;
        //w4_pwm = w4_pwm_prec;    //on force la valeur des pwms encoyées au moteur 3 et 4 à la valeur a l'intant t-1
    }

    if (w3_pwm < pwm_min + OFFSET_PWM  && w4_pwm < pwm_min + OFFSET_PWM && w1_pwm >= pwm_min + OFFSET_PWM && w2_pwm >= pwm_min + OFFSET_PWM) { // demande d'un lacet max à gauche
        w3_pwm = pwm_min+OFFSET_PWM;
        w4_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 3 et 4 à la valeur pwm_min
    }

    if (w1_pwm < pwm_min + OFFSET_PWM  && w3_pwm < pwm_min + OFFSET_PWM && w2_pwm >= pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM) { // demande d'un tangage (nez vers le bas) max
        w1_pwm = pwm_min+OFFSET_PWM;
        w3_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 1 et 3 à la valeur pwm_min
    }

    if (w2_pwm < pwm_min + OFFSET_PWM  && w4_pwm < pwm_min + OFFSET_PWM && w1_pwm >= pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM) { // demande d'un tangage (nez vers le haut) max
        w2_pwm = pwm_min+OFFSET_PWM;
        w4_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 2 et 4 à la valeur pwm_min
    }

    if (w2_pwm < pwm_min + OFFSET_PWM  && w3_pwm < pwm_min + OFFSET_PWM && w1_pwm >= pwm_min + OFFSET_PWM && w4_pwm >= pwm_min + OFFSET_PWM) { // demande d'un roulis à droite max
        w2_pwm = pwm_min+OFFSET_PWM;
        w3_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 2 et 3 à la valeur pwm_min
    }

    if (w1_pwm < pwm_min + OFFSET_PWM  && w4_pwm < pwm_min + OFFSET_PWM && w2_pwm >= pwm_min + OFFSET_PWM && w3_pwm >= pwm_min + OFFSET_PWM) { // demande d'un roulis à gauche max
        w1_pwm = pwm_min+OFFSET_PWM;
        w4_pwm = pwm_min+OFFSET_PWM;       //on force la valeur des pwms encoyées au moteur 1 et 4 à la valeur pwm_min
    }
    
    /* //pas utile : gérer par les correcteurs
    w1_pwm_prec = w1_pwm;
    w2_pwm_prec = w2_pwm;
    w3_pwm_prec = w3_pwm;
    w4_pwm_prec = w4_pwm;*/

    // A mettre dans une zone de debug

    // Affichage des valeurs PMW des moteurs
    //hal.console->printf("PWM - Min: %i Max: %i Actuel:%i w4:%i\n",pwm_min,pwm_max,pwm,w4_pwm);

    // Affichage des sorties des PIDs
    //hal.console->printf("PIDs - UPhi:%f, UTheta:%f, Ur:%f, Uz:%f\n",u_phi,u_theta,u_r,u_z);

    // Affichage des paramètres
    //hal.console->printf("Paramètres - d:%lf, b:%lf, l:%lf\n",d,b,l);

    // Affichage des commandes
    //hal.console->printf("Commandes - w1:%f, w2:%f, w3:%f, w4:%f\n",w1,w2,w3,w4);

    // --------------------------------------------------------------------
    // Ecriture des logs
    // --------------------------------------------------------------------

    // "AHRS.Roll,AHRS.Pitch,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,Uz,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm"

    ecriture_log(&outf4, ahrs.roll-offset_ahrs_roll, ahrs.pitch-offset_ahrs_pitch, ahrs.get_gyro().z-offset_ahrs_yaw, target_roll_rad, target_pitch_rad, target_yaw_rate_rad, target_throttle_newton, u_phi, u_theta, u_r, u_z, w1, w2, w3, w4, w1_pwm, w2_pwm, w3_pwm, w4_pwm);

    // ---------------------------------------------------------------------
    // Gestion Moteurs
    // ---------------------------------------------------------------------

    // output_test : Fait tourner les moteurs à une valeur de PWM spécifiée : void output_test(uint8_t motor_seq, int16_t pwm);
    // motor_seq est le numéro du moteur (va de 1 au nombre de moteurs total)
    // pwm est la valeur pwm envoyée en sortie (normalement situé entre 1000 et 2000)

    // Rotation des moteurs en fonction de la valeur en pwm des commandes
    motors.output_test(w1_index,w1_pwm);
    motors.output_test(w2_index,w2_pwm);
    motors.output_test(w3_index,w3_pwm);
    motors.output_test(w4_index,w4_pwm);

}