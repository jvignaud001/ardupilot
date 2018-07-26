/*
 * ims.cpp
 *
 * Créé le : Feb 15, 2017
 *      Auteur : Daniel Monier-Reyes
 * 
 * Modifié  le : 9 Juillet 2018
 *          par : Jamy Vignaud
 * 
 * Fichier qui regroupe toutes les définitions des méthodes des classes
 */

#include "Copter.h"
#include "ims.hpp"

/*
 *
 * Correcteur_1er_Ordre_Discret :
 * Fonctions de la classe représentant une equation récurrente du premier ordre
 * 
 */

// Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
Correcteur_1er_Ordre_Discret::Correcteur_1er_Ordre_Discret(double m_cxn,double m_cxn_1,double m_cyn_1)
{
    cxn=m_cxn;
    cxn_1=m_cxn_1;
    cyn_1=m_cyn_1;

    xn=0;
    xn_1=0;
    yn=0;
    yn_1=0;

}

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 1er ordre
double Correcteur_1er_Ordre_Discret::getyn() const
{
    return yn;
}

// Réinitialisation des valeurs de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::reset(void)
{
    xn=0;
    xn_1=0;
    yn=0;
    yn_1=0;
}

// Calcul d'un cycle de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::cycle(double new_xn)
{
    xn=new_xn;

    yn=(cxn*xn)+(cxn_1*xn_1)+(cyn_1*yn_1);

    yn_1=yn;
    xn_1=xn;
}


/*
 *
 * Correcteur_2nd_Ordre_Discret :
 * Fonctions de la classe représentant une equation récurrente du second ordre
 * 
 */

// Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
Correcteur_2nd_Ordre_Discret::Correcteur_2nd_Ordre_Discret(double m_cxn,double m_cxn_1,double m_cxn_2,double m_cyn_1,double m_cyn_2)
{
    cxn=m_cxn;
    cxn_1=m_cxn_1;
    cxn_2=m_cxn_2;
    cyn_1=m_cyn_1;
    cyn_2=m_cyn_2;

    xn=0;
    xn_1=0;
    xn_2=0;
    yn=0;
    yn_1=0;
    yn_2=0;

}

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 2nd ordre
double Correcteur_2nd_Ordre_Discret::getyn() const
{
    return yn;
}

// Réinitialisation des valeurs de l'équation récurrente du 2nd ordre
void Correcteur_2nd_Ordre_Discret::reset(void)
{
    xn=0;
    xn_1=0;
    xn_2=0;
    yn=0;
    yn_1=0;
    yn_2=0;
}

// Calcul d'un cycle de l'équation récurrente du 2nd ordre
void Correcteur_2nd_Ordre_Discret::cycle(double new_xn)
{
    xn=new_xn;

    yn=(cxn*xn)+(cxn_1*xn_1)+(cxn_2*xn_2)+(cyn_1*yn_1)+(cyn_2*yn_2);

    yn_2=yn_1;
    yn_1=yn;
    xn_2=xn_1;
    xn_1=xn;
}



/*
 *
 * Parametres_Drone :
 * Fonction de la classe permettant de récupérer les paramètres du drone comptenu dans un fichier
 * 
 */

// Constructeur de la classe Parametres_Drone en prenant pour argument le nom et l'emplacement du fichier contenant les paramètres du drone
Parametres_Drone::Parametres_Drone(std::string const& nom_fichier)
{
    // On ouvre le fichier en lecture 
    fichier.open(nom_fichier.c_str());  
    // Récupération des valeurs dans le fichier
    set_parameters();                   
}

// Fonction qui permet d'aller à la ligne désirée (mise en argument)
void Parametres_Drone::aller_a_la_ligne(int num_ligne)
{
    // Place le curseur virtuel au début du fichier
    fichier.seekg(1,std::ios::beg); 
    for (int i = 1; i < num_ligne; i++)
    {
        // Fonction qui permet d'ignorer les lignes (en premier argument, cette fonction prend le nombre de caractère à ignorer,
        // ici le nombre max int,et le deuxième argument, il s'agit du dernier caractère à ignorer, ici il s'agit du retour à la ligne)
        fichier.ignore(std::numeric_limits<int>::max(),'\n');    
    }                                                              
}

// Fonction qui permet de sauter le nombre de ligne désirée (demandée en argument)
void Parametres_Drone::aller_a_la_ligne_apres(int num_ligne_apres)
{
    // Place le curseur à sa position relative, sur lui-même
    fichier.seekg(1,std::ios::cur); 
    for (int i = 1; i < num_ligne_apres; i++)
    {
        fichier.ignore(std::numeric_limits<int>::max(),'\n');
    }
}

// Fonction permettant de regrouper les fonctions qui récupérent les données du fichier paramètre (attention à l'odre d'appelle des fonctions)
void Parametres_Drone::set_parameters(void)
{
    set_fichier_log();
    set_rotation_max();
    set_coef_trainee();
    set_coef_poussee();
    set_envergure();
    set_masse_arrachage();
    set_roulis();
    set_tangage();
    set_lacet();
    set_consigne_smooth();
    set_offset_pwm();
    set_angle_max_roulis_tangage();
    set_vitesse_max_lacet();
    set_poussee_exponentielle();
    set_affichage_parametres();
    set_affichage_consignes();
    set_affichage_erreur();
    set_affichage_AHRS();
    set_affichage_pwm();
    set_affichage_sorties_PID();
    set_affichage_commandes();
    set_test_poussee();
    fichier.close();
}

// --------------------------------------------------------------------------------------------------
// Fonctions qui permettent de récupérer et initialiser les attributs de la classe "Parametres_Drone"
// --------------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATTENTION : s'il y a modification du fichier paramètre, il faut changer les lignes de lectures des fonctions qui suivent//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Parametres_Drone::set_fichier_log(void)
{
    aller_a_la_ligne(6); // ligne 6
    fichier >> fichier_log;
}

void Parametres_Drone::set_rotation_max(void)
{
    aller_a_la_ligne_apres(3); // ligne 9
    fichier >> rotation_max;
}

void Parametres_Drone::set_coef_trainee(void)
{
    aller_a_la_ligne_apres(3); // ligne 12
    fichier >> coef_trainee;
}

void Parametres_Drone::set_coef_poussee(void)
{
    aller_a_la_ligne_apres(3); // ligne 15
    fichier >> coef_poussee;
}

void Parametres_Drone::set_envergure(void)
{
    aller_a_la_ligne_apres(3); // ligne 18
    fichier >> envergure;
}

void Parametres_Drone::set_masse_arrachage(void)
{
    aller_a_la_ligne_apres(3); // ligne 21
    fichier >> masse_arrachage;
}

void Parametres_Drone::set_roulis(void)
{
    aller_a_la_ligne_apres(6); // ligne 27
    fichier >> roulis.xn;
    aller_a_la_ligne_apres(1); // ligne 28
    fichier >> roulis.xn_1;
    aller_a_la_ligne_apres(1); // ligne 29
    fichier >> roulis.xn_2;
    aller_a_la_ligne_apres(1); // ligne 30
    fichier >> roulis.yn_1;
    aller_a_la_ligne_apres(1); // ligne 31
    fichier >> roulis.yn_2;
}

void Parametres_Drone::set_tangage(void)
{
    aller_a_la_ligne_apres(6); // ligne 37
    fichier >> tangage.xn;
    aller_a_la_ligne_apres(1); // ligne 38
    fichier >> tangage.xn_1;
    aller_a_la_ligne_apres(1); // ligne 39
    fichier >> tangage.xn_2;
    aller_a_la_ligne_apres(1); // ligne 40
    fichier >> tangage.yn_1;
    aller_a_la_ligne_apres(1); // ligne 41
    fichier >> tangage.yn_2;
}

void Parametres_Drone::set_lacet(void)
{
    aller_a_la_ligne_apres(6); // ligne 47
    fichier >> lacet.xn;
    aller_a_la_ligne_apres(1); // ligne 48
    fichier >> lacet.xn_1;
    aller_a_la_ligne_apres(1); // ligne 49
    fichier >> lacet.yn_1;
    lacet.yn_2 = 0;
    lacet.xn_2 = 0;
}

void Parametres_Drone::set_consigne_smooth(void)
{
    aller_a_la_ligne_apres(6); // ligne 55
    fichier >> consigne_smooth.xn;
    aller_a_la_ligne_apres(1); // ligne 56
    fichier >> consigne_smooth.xn_1;
    aller_a_la_ligne_apres(1); // ligne 57
    fichier >> consigne_smooth.xn_2;
    aller_a_la_ligne_apres(1); // ligne 58
    fichier >> consigne_smooth.yn_1;
    aller_a_la_ligne_apres(1); // ligne 59
    fichier >> consigne_smooth.yn_2;
}

void Parametres_Drone::set_offset_pwm(void)
{
    aller_a_la_ligne_apres(3); // ligne 62
    fichier >> offset_pwm;
}

void Parametres_Drone::set_angle_max_roulis_tangage(void)
{
    aller_a_la_ligne_apres(3); // ligne 65
    fichier >> angle_max_roulis_tangage;
}

void Parametres_Drone::set_vitesse_max_lacet(void)
{
    aller_a_la_ligne_apres(3); // ligne 68
    fichier >> vitesse_max_lacet;
}

void Parametres_Drone::set_poussee_exponentielle(void)
{
    aller_a_la_ligne_apres(3); // ligne 71
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        poussee_exponentielle = true;
    }
    else 
    {
        poussee_exponentielle = false;
    }
}

void Parametres_Drone::set_affichage_parametres(void)
{
    aller_a_la_ligne_apres(7); // ligne 78
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_parametres = true;
    }
    else 
    {
        affichage_parametres = false;
    }
}

void Parametres_Drone::set_affichage_consignes(void)
{
    aller_a_la_ligne_apres(3); // ligne 81
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_consignes = true;
    }
    else 
    {
        affichage_consignes = false;
    }
}

void Parametres_Drone::set_affichage_erreur(void)
{
    aller_a_la_ligne_apres(3); // ligne 84
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_erreur = true;
    }
    else 
    {
        affichage_erreur = false;
    }
}

void Parametres_Drone::set_affichage_AHRS(void)
{
    aller_a_la_ligne_apres(3); // ligne 87
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_AHRS = true;
    }
    else 
    {
        affichage_AHRS = false;
    }
}

void Parametres_Drone::set_affichage_pwm(void)
{
    aller_a_la_ligne_apres(3); // ligne 90
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_pwm = true;
    }
    else 
    {
        affichage_pwm = false;
    }
}

void Parametres_Drone::set_affichage_sorties_PID(void)
{
    aller_a_la_ligne_apres(3); // ligne 93
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_sorties_PID = true;
    }
    else 
    {
        affichage_sorties_PID = false;
    }
}

void Parametres_Drone::set_affichage_commandes(void)
{
    aller_a_la_ligne_apres(3); // ligne 96
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        affichage_commandes = true;
    }
    else 
    {
        affichage_commandes = false;
    }
}

void Parametres_Drone::set_test_poussee(void)
{
    aller_a_la_ligne_apres(5); // ligne 101
    fichier >> texte;
    if ( (texte.compare("true") == 0 || texte.compare("vrai") == 0 ) )
    {
        test_poussee = true;
    }
    else 
    {
        test_poussee = false;
    }
}

// -----------------------------------------------------------------------------------
// fonctions qui permettent de retourner les attributs de la classe "Parametres_Drone"
// -----------------------------------------------------------------------------------

std::string Parametres_Drone::get_fichier_log(void) const
{
    return fichier_log;
}

float Parametres_Drone::get_rotation_max(void) const
{
    return rotation_max;
}

float Parametres_Drone::get_masse_arrachage(void) const
{
    return masse_arrachage;
}

double Parametres_Drone::get_coef_trainee(void) const
{
    return coef_trainee;
}

double Parametres_Drone::get_coef_poussee(void) const
{
    return coef_poussee;
}

double Parametres_Drone::get_envergure(void) const
{
    return envergure;
}

struct Coef_Correcteurs Parametres_Drone::get_roulis(void) const
{
    return roulis;
}

struct Coef_Correcteurs Parametres_Drone::get_tangage(void) const
{
    return tangage;
}

struct Coef_Correcteurs Parametres_Drone::get_lacet(void) const
{
    return lacet;
}

struct Coef_Correcteurs Parametres_Drone::get_consigne_smooth(void) const
{
    return consigne_smooth;
}

int16_t Parametres_Drone::get_offset_pwm(void) const
{
    return offset_pwm;
}

float Parametres_Drone::get_angle_max_roulis_tangage(void) const
{
    return angle_max_roulis_tangage;
}

float Parametres_Drone::get_vitesse_max_lacet(void) const
{
    return vitesse_max_lacet;
}

bool Parametres_Drone::get_poussee_exponentielle(void) const
{
    return poussee_exponentielle;
}

bool Parametres_Drone::get_affichage_parametres(void) const
{
    return affichage_parametres;
}

bool Parametres_Drone::get_affichage_consignes(void) const
{
    return affichage_consignes;
}

bool Parametres_Drone::get_affichage_erreur(void) const
{
    return affichage_erreur;
}

bool Parametres_Drone::get_affichage_AHRS(void) const
{
    return affichage_AHRS;
}

bool Parametres_Drone::get_affichage_pwm(void) const
{
    return affichage_pwm;
}

bool Parametres_Drone::get_affichage_sorties_PID(void) const
{
    return affichage_sorties_PID;
}

bool Parametres_Drone::get_affichage_commandes(void) const
{
    return affichage_commandes;
}

bool Parametres_Drone::get_test_poussee(void) const
{
    return test_poussee;
}



/*
 *
 * Quadri :
 * Fonction de la classe permettant de récupérer les données des capteurs, lire le fichier mis en argument (généré par un script MatLab), et de calculer les PWM
 * 
 */

// Constructeur de la classe Quadri prenant en paramètre le nom et l'emplacement du fichier (généré par un script MatLab) qui contient les paramètres
Quadri::Quadri(std::string const& emplacement_fichier_parametres) : 
// initialisation des classes
params(emplacement_fichier_parametres),
pid_roll(params.get_roulis().xn,params.get_roulis().xn_1,params.get_roulis().xn_2,params.get_roulis().yn_1,params.get_roulis().yn_2),
pid_pitch(params.get_tangage().xn,params.get_tangage().xn_1,params.get_tangage().xn_2,params.get_tangage().yn_1,params.get_tangage().yn_2),
pi_yaw(params.get_lacet().xn,params.get_lacet().xn_1,params.get_lacet().yn_1),
roll_smooth(params.get_consigne_smooth().xn,params.get_consigne_smooth().xn_1,params.get_consigne_smooth().xn_2,params.get_consigne_smooth().yn_1,params.get_consigne_smooth().yn_2),
pitch_smooth(roll_smooth),
yaw_rate_smooth(roll_smooth)
{
    //initialisations des attributs
    nom_programme = params.get_fichier_log();
    fichier_log_ouvert=false;
    u_theta = 0;
    u_phi = 0;
    u_r = 0;
    u_z = 0;
    w1 = 0;
    w2 = 0;
    w3 = 0;
    w4 = 0;
    w1_pwm = 0;
    w2_pwm = 0;
    w3_pwm = 0;
    w4_pwm = 0;
    pwm_min = 0;
    pwm_max = 0;
    offset_pwm = 0;
    target_roll = 0;
    target_pitch = 0;
    target_yaw_rate = 0;
    pilot_throttle_scaled = 0;
    target_roll_rad = 0;
    target_pitch_rad = 0;
    target_yaw_rate_rad = 0;
    target_throttle_newton = 0;
    target_roll_smooth = 0;
    target_pitch_smooth = 0;
    target_yaw_rate_smooth = 0;
    angle_roulis = 0;
    angle_tangage = 0;
    vitesse_angle_lacet = 0;
    debug = params.get_affichage_AHRS() || params.get_affichage_commandes() || params.get_affichage_consignes() || params.get_affichage_erreur() || params.get_affichage_parametres() || params.get_affichage_pwm() || params.get_affichage_sorties_PID() || params.get_test_poussee();
}

// Fonction qui crée et donne un titre au fichier log
void Quadri::titre_log(void)
{
    // Construction de la date et de l'heure pour rendre le nom du fichier de log unique
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [30];
        char log_file_name [80];

        time (&rawtime);
        timeinfo = localtime (&rawtime);

        strftime (buffer,30,"%F--%H-%M-%S",timeinfo);
        //création du fichier à l'emplacement '/home/pi/ardupilot/build/navio2/bin/' sur le drone
        //avec pour nom : "<nom_programme>_CSV_LOG-<année>-<mois>-<jour>--<heure>-<minute>-<seconde>.dat"
        strcpy(log_file_name,"/home/pi/ardupilot/build/navio2/bin/");
        strcat(log_file_name,nom_programme.c_str());
        strcat(log_file_name,"_CSV_LOG-");
        strcat(log_file_name,buffer);
        strcat(log_file_name,".dat");

        fichier_log.open(log_file_name); // Création d'un fichier de log unique

        // Ecriture d'une entête pour savoir à quoi correspond les données
        fichier_log << "AHRS_Roll,AHRS_Pitch,AHRS_R,RC_Roll,RC_Pitch,RC_R,RC_Thrust,Uphi,Utheta,Ur,Uz,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm" << std::endl;
}

// Fonction qui permet d'écrire les paramètres dans le fichier log
void Quadri::ecriture_log(void)
{
    // "AHRS.Roll,AHRS.Pitch,AHRS.R,RC.Roll,RC.Pitch,RC.R,RC.Thrust,Uphi,Utheta,Ur,Uz,w1,w2,w3,w4,w1_pwm,w2_pwm,w3_pwm,w4_pwm"
    fichier_log << angle_roulis;
    fichier_log << ",";
    fichier_log << angle_tangage;
    fichier_log << ",";
    fichier_log << vitesse_angle_lacet;
    fichier_log << ",";
    fichier_log << target_roll_smooth;
    fichier_log << ",";
    fichier_log << target_pitch_smooth;
    fichier_log << ",";
    fichier_log << target_yaw_rate_smooth;
    fichier_log << ",";
    fichier_log << target_throttle_newton;
    fichier_log << ",";
    fichier_log << u_phi;
    fichier_log << ",";
    fichier_log << u_theta;
    fichier_log << ",";
    fichier_log << u_r;
    fichier_log << ",";
    fichier_log << u_z;
    fichier_log << ",";
    fichier_log << w1;
    fichier_log << ",";
    fichier_log << w2;
    fichier_log << ",";
    fichier_log << w3;
    fichier_log << ",";
    fichier_log << w4;
    fichier_log << ",";
    fichier_log << w1_pwm;
    fichier_log << ",";
    fichier_log << w2_pwm;
    fichier_log << ",";
    fichier_log << w3_pwm;
    fichier_log << ",";
    fichier_log << w4_pwm;
    fichier_log << std::endl;
}

// Test pour protection des moteurs (on veut éviter que les moteurs ont un pwm supérieur au pwm maximal)
void Quadri::test_pwm(void)
{
    if (w1_pwm > pwm_max)
        w1_pwm = pwm_max;

    if (w2_pwm > pwm_max)
        w2_pwm = pwm_max;

    if (w3_pwm > pwm_max)
        w3_pwm = pwm_max;

    if (w4_pwm > pwm_max)
        w4_pwm = pwm_max;
}

// ---------------------------------------------------------------
// Fonctions qui récupérent et calculent les valeurs des variables
// ---------------------------------------------------------------

// Calcul des sorties des PIDs 

void Quadri::set_u_phi(void)
{
    // Convertion de la consigne envoyée par la manette (en centidegré) en radian 
    target_roll_rad=double((target_roll*M_PI)/18000.0);

    // Passage de la consigne dans la fonction "Smooth" qui ralenti la consigne si l'amplitude est trop élevée
    roll_smooth.cycle(target_roll_rad);
    target_roll_smooth = roll_smooth.getyn();

    // Passage dans le correcteur PID, ayant pour entrée l'erreur (= la différence en la consigne et la valeur des capteurs)
    pid_roll.cycle(target_roll_smooth-angle_roulis);
    u_phi=pid_roll.getyn();
}

void Quadri::set_u_theta(void)
{
    // Convertion de la consigne envoyée par la manette (en centidegré) en radian 
    target_pitch_rad=double((target_pitch*M_PI)/18000.0);

    // Passage de la consigne dans la fonction "Smooth" qui ralenti la consigne si l'amplitude est trop élevée
    pitch_smooth.cycle(target_pitch_rad);
    target_pitch_smooth = pitch_smooth.getyn();

    // Passage dans le correcteur PID, ayant pour entrée l'erreur (= la différence en la consigne et la valeur des capteurs)
    pid_pitch.cycle(target_pitch_smooth-angle_tangage);
    u_theta=pid_pitch.getyn();
}

void Quadri::set_u_r(void)
{
    // Convertion de la consigne envoyée par la manette (en centidegré par seconde) en radian par seconde
    target_yaw_rate_rad=double((target_yaw_rate*M_PI)/18000.0);

    // Passage de la consigne dans la fonction "Smooth" qui ralenti la consigne si l'amplitude est trop élevée
    yaw_rate_smooth.cycle(target_yaw_rate_rad);
    target_yaw_rate_smooth = yaw_rate_smooth.getyn();

    // Passage dans le correcteur PI, ayant pour entrée l'erreur (= la différence en la consigne et la valeur des capteurs)
    pi_yaw.cycle(target_yaw_rate_smooth-vitesse_angle_lacet);
    u_r=pi_yaw.getyn();
}

void Quadri::set_u_z(void)
{
    // Convertion de la consigne envoyée par la manette (comprise entre 0 et 1) en Newton
    target_throttle_newton=double(pilot_throttle_scaled*params.get_masse_arrachage()*GRAVITY_MSS);

    // On caclule la poussée nécessaire dépendant de l'angle de roulis et de tangage
    u_z=-target_throttle_newton/(cosf(angle_roulis)*cosf(angle_tangage));
}

// Valeurs (angles et vitesse angulaire) lues par les capteurs

void Quadri::set_angle_roulis(double m_angle_roulis)
{
    angle_roulis = m_angle_roulis;
}

void Quadri::set_angle_tangage(double m_angle_tangage)
{
    angle_tangage = m_angle_tangage;
}

void Quadri::set_vitesse_angle_lacet(double m_vitesse_angle_lacet)
{
    vitesse_angle_lacet = m_vitesse_angle_lacet;
}

// Valeurs des PWM min et max obtenues par des fonctions d'ArduPilot

void Quadri::set_pwm_min(double m_pwm_min)
{
    offset_pwm = params.get_offset_pwm()/100*(pwm_max - m_pwm_min);
    pwm_min = m_pwm_min + offset_pwm;
}

void Quadri::set_pwm_max(double m_pwm_max)
{
    pwm_max = m_pwm_max;
}

// Fonction récupérant les consignes de la manette

void Quadri::set_pilot_throttle_scaled(double m_pilot_throttle_scaled)
{
    pilot_throttle_scaled = m_pilot_throttle_scaled;
}

void Quadri::set_target_yaw_rate(float m_target_yaw_rate)
{
    target_yaw_rate = m_target_yaw_rate;
}

// Calculs des PWM envoyés à chaque moteur

void Quadri::set_w1_pwm(void)
{
    if (!params.get_test_poussee()) // si on effectue un vol en mode "normal"
    {
        // Calcule de la rotation des moteurs (en rad/s)
        //constrain_float(variable, valeur_min, valeur_max) 
        //si la valeur de "variable" est inférieure à valeur_min alors la fonction retour : valeur_min
        //si la valeur de "variable" est supérieure à valeur_max alors la fonction retour : valeur_max
        //sinon elle retour la valeur de variable
        // w1=sqrt((d*u_phi+d*u_theta-b*l*u_r-d*l*u_z)/(b*d*l)/2;  
        w1=sqrt(constrain_float((params.get_coef_trainee()*u_phi+params.get_coef_trainee()*u_theta-params.get_coef_poussee()*params.get_envergure()*u_r-params.get_coef_trainee()*params.get_envergure()*u_z)/(params.get_coef_poussee()*params.get_coef_trainee()*params.get_envergure()),0,4*params.get_rotation_max()*params.get_rotation_max()))/2;
        
        // Calcule du PWM à envoyer au moteur 1
        w1_pwm=(w1/params.get_rotation_max())*(pwm_max-pwm_min)+pwm_min;
    }
    else    // si on est en mode "test de poussée"
    {
        w1_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    }
}

void Quadri::set_w2_pwm(void)
{
    if (!params.get_test_poussee()) // si on effectue un vol en mode "normal"
    {
        // Calcule de la rotation des moteurs (en rad/s)
        // w2=sqrt(-(d*u_phi+d*u_theta+b*l*u_r+d*l*u_z)/(b*d*l)/2;   
        w2=sqrt(constrain_float(-(params.get_coef_trainee()*u_phi+params.get_coef_trainee()*u_theta+params.get_coef_poussee()*params.get_envergure()*u_r+params.get_coef_trainee()*params.get_envergure()*u_z)/(params.get_coef_poussee()*params.get_coef_trainee()*params.get_envergure()),0,4*params.get_rotation_max()*params.get_rotation_max()))/2;
        
        // Calcule du PWM à envoyer au moteur 2
        w2_pwm=(w2/params.get_rotation_max())*(pwm_max-pwm_min)+pwm_min;
    }
    else    // si on est en mode "test de poussée" 
    {
        w2_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    }
}

void Quadri::set_w3_pwm(void)
{
    if (!params.get_test_poussee()) // si on effectue un vol en mode "normal"
    {
        // Calcule de la rotation des moteurs (en rad/s)
        // w3=sqrt(-(d*u_phi-d*u_theta-b*l*u_r+d*l*u_z)/(b*d*l)/2;
        w3=sqrt(constrain_float(-(params.get_coef_trainee()*u_phi-params.get_coef_trainee()*u_theta-params.get_coef_poussee()*params.get_envergure()*u_r+params.get_coef_trainee()*params.get_envergure()*u_z)/(params.get_coef_poussee()*params.get_coef_trainee()*params.get_envergure()),0,4*params.get_rotation_max()*params.get_rotation_max()))/2;
        
        // Calcule du PWM à envoyer au moteur 3
        w3_pwm=(w3/params.get_rotation_max())*(pwm_max-pwm_min)+pwm_min;
    }
    else    // si on est en mode "test de poussée" 
    {
        w3_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    }
}

void Quadri::set_w4_pwm(void)
{
    if (!params.get_test_poussee()) // si on effectue un vol en mode "normal"
    {
        // Calcule de la rotation des moteurs (en rad/s)
        // w4=sqrt((d*u_phi-d*u_theta+b*l*u_r-d*l*u_z)/(b*d*l)/2; 
        w4=sqrt(constrain_float((params.get_coef_trainee()*u_phi-params.get_coef_trainee()*u_theta+params.get_coef_poussee()*params.get_envergure()*u_r-params.get_coef_trainee()*params.get_envergure()*u_z)/(params.get_coef_poussee()*params.get_coef_trainee()*params.get_envergure()),0,4*params.get_rotation_max()*params.get_rotation_max()))/2;
        
        // Calcule du PWM à envoyer au moteur 4
        w4_pwm=(w4/params.get_rotation_max())*(pwm_max-pwm_min)+pwm_min;
    }
    else    // si on est en mode "test de poussée" 
    {
        w4_pwm=pilot_throttle_scaled*(pwm_max-pwm_min)+pwm_min;
    }
}

// Fonction regroupant d'autres fonctions afin de ne pas mettre plusieurs lignes dans le fichier "control_ims"

void Quadri::set_pwm_moteurs(void)
{
    // Récupération des sorties des correcteurs
    set_u_phi();
    set_u_theta();
    set_u_r();
    set_u_z();

    // Récupération des pwm à envoyer pour chaque moteurs
    set_w1_pwm();
    set_w2_pwm();
    set_w3_pwm();
    set_w4_pwm();

    // Affichage du debugger/ des informations dans la console
    if (debug)
    {
        debugger();
    }

    // Vérification que les pwm ne dépassent pas la valeur de pwm max
    test_pwm();

    // écriture des valeurs dans le fichier log
    ecriture_log();
}

// Fonction qui test et initialise un fichier log

void Quadri::ouverture_fichier_log(void)
{
    if (fichier_log_ouvert==false) { // Si le fichier de log n'est pas encore ouvert alors l'ouvrir
        // Fonction qui crée et donne un titre au fichier log
        titre_log();
        fichier_log_ouvert=true;
    }
}

// Fonction qui test et ferme le fichier log

void Quadri::fermeture_fichier_log(void)
{
    if (fichier_log_ouvert==true) { // Si le fichier de log est ouvert alors le fermer
        fichier_log.close();
        fichier_log_ouvert=false;
    }
}

// Fonction qui reset tous les correcteurs

void Quadri::reset_PID(void)
{
    pid_roll.reset();
    pid_pitch.reset();
    pi_yaw.reset();
    roll_smooth.reset();
    pitch_smooth.reset();
    yaw_rate_smooth.reset();
}

// ------------------------------------------------------------
// Fonction qui retourne les valeurs des attributs de la classe
// ------------------------------------------------------------

double Quadri::get_w1_pwm(void) const
{
    return w1_pwm;
}

double Quadri::get_w2_pwm(void) const
{
    return w2_pwm;
}

double Quadri::get_w3_pwm(void) const
{
    return w3_pwm;
}

double Quadri::get_w4_pwm(void) const
{
    return w4_pwm;
}

float Quadri::get_angle_max_roulis_tangage(void) const
{
    return params.get_angle_max_roulis_tangage();
}

float Quadri::get_vitesse_max_lacet(void) const
{
    return params.get_vitesse_max_lacet();
}

bool Quadri::get_poussee_exponentielle(void) const
{
    return params.get_poussee_exponentielle();
}

// ----------------------------------------------------------------------------------------
// Debugger qui affiche toutes ses données sur la console ou dans le fichier "startup_log"
// ----------------------------------------------------------------------------------------
void Quadri::debugger(void)
{
    // Affichage des paramètres du drone
    if (params.get_affichage_parametres())
    {
        hal.console->printf("Coefficient de poussée : %f et coefficient de trainée : %f et envergure : %f\n",params.get_coef_poussee(),params.get_coef_trainee(),params.get_envergure());
    }

    // Affichage des consignes Roll, Pitch, Yaw, Throttle
    if (params.get_affichage_consignes())
    {
        hal.console->printf("Consignes - Roll: %f Pitch: %f Yaw: %f Throttle %f\n",target_roll_smooth*180.0/M_PI,target_pitch_smooth*180.0/M_PI,target_yaw_rate_smooth*180.0/M_PI, target_throttle_newton);
    }

    // Affichage de l'erreur, des consignes et des sortie de l'AHRS
    if (params.get_affichage_erreur())
    {
        hal.console->printf("Erreur Roll : %f° = Consigne.Roll - AHRS.Roll = %f° - %f° \n",(target_roll_smooth - angle_roulis)*180.0/M_PI, target_roll_smooth*180.0/M_PI, angle_roulis*180.0/M_PI);
        hal.console->printf("Erreur Pitch : %f° = Consigne.Pitch - AHRS.Pitch = %f° - %f° \n",(target_pitch_smooth - angle_tangage)*180.0/M_PI, target_pitch_smooth*180.0/M_PI, angle_tangage*180.0/M_PI);
        hal.console->printf("Erreur Yaw : %f°/s = Consigne.Yaw - AHRS.Yaw = %f° - %f° \n",(target_yaw_rate_smooth - vitesse_angle_lacet)*180.0/M_PI, target_yaw_rate_smooth*180.0/M_PI, vitesse_angle_lacet*180.0/M_PI);
    }
    
    // Affichage des sorties de l'AHRS
    if (params.get_affichage_AHRS())
    {
        hal.console->printf("AHRS - Roll: %f°, Pitch:%f°, R:%f°/s\n",angle_roulis*180.0/M_PI, angle_tangage*180.0/M_PI, vitesse_angle_lacet*180.0/M_PI);
    }
    

    // Affichage des valeurs PMW des moteurs
    if (params.get_affichage_pwm())
    {
        hal.console->printf("PWM - Min: %i Max: %i Actuel: w1 :%i, w2 :%i, w3 :%i, w4 :%i \n",pwm_min,pwm_max,w1_pwm,w2_pwm,w3_pwm,w4_pwm);
    }
    

    // Affichage des sorties des PIDs
    if (params.get_affichage_sorties_PID())
    {
        hal.console->printf("PIDs - UPhi:%f, UTheta:%f, Ur:%f, Uz:%f\n",u_phi,u_theta,u_r,u_z);
    }

    // Affichage des commandes
    if (params.get_affichage_commandes())
    {
        hal.console->printf("Commandes - w1:%f, w2:%f, w3:%f, w4:%f\n",w1,w2,w3,w4);
    }

    // Affichage des éléments importants pour le test de poussée
    if (params.get_test_poussee())
    {
        hal.console->printf("Test poussée : Consigne poussée en pourcent : %f, PWM min : %d, PWM max : %d et PWM : %d\n", pilot_throttle_scaled*100.0,pwm_max,pwm_min,w1_pwm);

    }
}