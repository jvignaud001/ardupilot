/*
 * control_ims3.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: Daniel Monier-Reyes
 *
 *
 * Routines d'initialisation et d'appel pour le mode de vol IMS3
 * Modification de la ligne 235 : u_z. On prend en compte l'offset d'AHRS
 * On rajoute un offset aux PWM des moteurs afins qu'ils "s'enclenche" plus vite (peu concluant)
 * 
 */

#include "Copter.h"
#include "ims.h"
#include <time.h>

#define LOG_TIME 600

#define OFFSET_PWM 200


// --------------------------------------------------------------------
// Déclaration des variables Globales
// --------------------------------------------------------------------

// PID ROLL : y(n)=32.2486.x(n)-64.1172.x(n-1)+31.8696.x(n-2)+1.8397.y(n-1)-0.83971.y(n-2)
Correcteur_2nd_Ordre_Discret pid_roll3(32.2486,-64.1172,31.8696,1.8397,-0.83971);

// PID PITCH : y(n)=35.9066.x(n)-71.3902.x(n-1)+35.4847.x(n-2)+1.8397.y(n-1)-0.83971.y(n-2)
Correcteur_2nd_Ordre_Discret pid_pitch3(35.9066,-71.3902,35.4847,1.8397,-0.83971);

// PI R : y(n)=0.65646.x(n)-0.654.x(n-1)+1.y(n-1)
Correcteur_1er_Ordre_Discret pi_yaw3(0.65646,-0.654,1);

// ofstream est utilisé pour écrire un fichier CSV nommé IMS3_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
std::ofstream outf3;

// ---------------------------------------------------------------------------------------------
// ims3_init - Routine d'initialisation du mode de vol IMS3
// ---------------------------------------------------------------------------------------------
bool Copter::ims3_init(bool ignore_checks)
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
// ims3_init - Routine d'appel du mode de vol IMS3 à exécuter à 400Hz
// ---------------------------------------------------------------------------------------------
void Copter::ims3_run()
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
    //hal.console->printf("Consignes - Roll: %f Pitch: %f Yaw: %f Throttle %f\n",target_roll_rad,target_pitch_rad,target_yaw_rate_rad, target_throttle_newton);

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
        pid_roll3.reset();
        pid_pitch3.reset();
        pi_yaw3.reset();

        if (fichier_log_ouvert==true) { // Si le fichier de log est ouvert alors le fermer
            outf3.close();
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
            avec pour nom : "IMS3_CSV_LOG-<année>-<mois>-<jour>-<heure>-<minute>-<seconde>.dat"*/
            strcpy(log_file_name,"/home/pi/ardupilot/build/navio2/bin/IMS3_CSV_LOG-");
            strcat(log_file_name,buffer);
            strcat(log_file_name,".dat");

            outf3.open(log_file_name); // Création d'un fichier de log unique

            // Ecriture d'une entête pour savoir à quoi correspond les données
            outf3 << "AHRS.Roll,AHRS.Pitch,AHRS.Yaw,AHRS.P,AHRS.Q,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm,pilot_throttle_scaled" << std::endl;

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
    pid_roll3.cycle(target_roll_rad-ahrs.roll+offset_ahrs_roll);
    // Calcul PID Pitch
    pid_pitch3.cycle(target_pitch_rad-ahrs.pitch+offset_ahrs_pitch);
    // Calcul PID Yaw
    pi_yaw3.cycle(target_yaw_rate_rad-ahrs.get_gyro().z+offset_ahrs_yaw);

    // Calcul PID Roll - Version sans prise en compte des offset de calibrage de l'AHRS au niveau du calcul de l'erreur
    //pid_roll3.cycle(target_roll_rad-ahrs.roll);
    // Calcul PID Pitch
    //pid_pitch3.cycle(target_pitch_rad-ahrs.pitch);
    // Calcul PID Yaw
    //pi_yaw3.cycle(target_yaw_rate_rad-ahrs.get_gyro().z);

    // Assignation des sorties de PIDs
    u_phi=pid_roll3.getyn();
    u_theta=pid_pitch3.getyn();
    u_r=pi_yaw3.getyn();
    u_z=-target_throttle_newton/(cosf(ahrs.roll-offset_ahrs_roll)*cosf(ahrs.pitch-offset_ahrs_pitch));

    // Calcul de la valeur des commandes
    w1=sqrt((d*u_phi+d*u_theta-b*l*u_r-d*l*u_z)/(b*d*l))/2;
    w2=sqrt(-(d*u_phi+d*u_theta+b*l*u_r+d*l*u_z)/(b*d*l))/2;
    w3=sqrt(-(d*u_phi-d*u_theta-b*l*u_r+d*l*u_z)/(b*d*l))/2;
    w4=sqrt((d*u_phi-d*u_theta+b*l*u_r-d*l*u_z)/(b*d*l))/2;

    // Calcul des valeurs de PWM à envoyer à chaque moteur en fonction de w1, w2, w3, w4 - A commenter pour les tests de poussée
    // La valeur pwm est dans un intervalle de valeur 1000 ~ 200
    w1_pwm=(w1/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min+OFFSET_PWM;
    w2_pwm=(w2/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min+OFFSET_PWM;
    w3_pwm=(w3/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min+OFFSET_PWM;
    w4_pwm=(w4/ROTATION_MAX)*(pwm_max-pwm_min)+pwm_min+OFFSET_PWM;

    // Test pour protection des moteurs
    if (w1_pwm > pwm_max)
        w1_pwm = pwm_max;
    if (w2_pwm > pwm_max)
        w2_pwm = pwm_max;
    if (w3_pwm > pwm_max)
        w3_pwm = pwm_max;
    if (w4_pwm > pwm_max)
        w4_pwm = pwm_max;
    
    // A décommenter pour les tests de poussée
    //w1_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w2_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w3_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    //w4_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;

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

    // A mettre dans une fonction
    // "AHRS.Roll,AHRS.Pitch,AHRS.Yaw,AHRS.P,AHRS.Q,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm,pilot_throttle_scaled"

        outf3 << ahrs.roll;
        outf3 << ",";
        outf3 << ahrs.pitch;
        outf3 << ",";
        outf3 << ahrs.yaw;
        outf3 << ",";
        outf3 << ahrs.get_gyro().x;
        outf3 << ",";
        outf3 << ahrs.get_gyro().y;
        outf3 << ",";
        outf3 << ahrs.get_gyro().z;
        outf3 << ",";
        outf3 << target_roll_rad;
        outf3 << ",";
        outf3 << target_pitch_rad;
        outf3 << ",";
        outf3 << target_yaw_rate_rad;
        outf3 << ",";
        outf3 << target_throttle_newton;
        outf3 << ",";
        outf3 << pid_roll3.getyn();
        outf3 << ",";
        outf3 << pid_pitch3.getyn();
        outf3 << ",";
        outf3 << pi_yaw3.getyn();
        outf3 << ",";
        outf3 << w1;
        outf3 << ",";
        outf3 << w2;
        outf3 << ",";
        outf3 << w3;
        outf3 << ",";
        outf3 << w4;
        outf3 << ",";
        outf3 << w1_pwm;
        outf3 << ",";
        outf3 << w2_pwm;
        outf3 << ",";
        outf3 << w3_pwm;
        outf3 << ",";
        outf3 << w4_pwm;
        outf3 << ",";
        outf3 << pilot_throttle_scaled;
        outf3 << std::endl;

    // ---------------------------------------------------------------------
    // Gestion Moteurs
    // ---------------------------------------------------------------------
    
    // output_test - spin a motor at the pwm value specified
    // motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    // pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    // void output_test(uint8_t motor_seq, int16_t pwm);

    // Rotation des moteurs en fonction de la valeur en pwm des commandes
    motors.output_test(w1_index,w1_pwm);
    motors.output_test(w2_index,w2_pwm);
    motors.output_test(w3_index,w3_pwm);
    motors.output_test(w4_index,w4_pwm);

}
