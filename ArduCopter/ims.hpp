/*
 * ims.h
 *
 * Créé le : Feb 15, 2017
 *      Auteur : Daniel Monier-Reyes
 * 
 * Modifié  le : 9 Juillet 2018
 *          par : Jamy Vignaud
 * 
 * Header file qui regroupe tous les prototypes des classes et des structures utilisés dans "control_ims.cpp"
 */

// Pour les fonctions d'écriture/lecture de fichiers
#include <fstream>



// -----------------------------------------------------------------------
// Déclaration des structures
// -----------------------------------------------------------------------

// Structure regroupant les différents coefficients des suites récursives
struct Coef_Correcteurs
{
    double xn;
    double xn_1;
    double xn_2;
    double yn_1;
    double yn_2;
};

// -----------------------------------------------------------------------
// Déclaration des classes
// -----------------------------------------------------------------------


/*
 *
 * Classe représentant une equation récurrente du premier ordre
 * 
 */

class Correcteur_1er_Ordre_Discret
{
// -----------------------------------------------------------------------
// Constructeur de la classe
// -----------------------------------------------------------------------
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_1er_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cyn_1=0);

// -----------------------------------------------------------------------
// Méthodes publiques
// -----------------------------------------------------------------------
public:
    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn() const;

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);

// -----------------------------------------------------------------------
// Attributs privées
// -----------------------------------------------------------------------
private:
    double cxn,cxn_1,cyn_1;             // Coefficients de l'équation récurrente
    double xn;                          // Valeur de l'échantillon d'entrée x(n)
    double xn_1;                    	// Valeurs successives des échantillons d'entrée x(n-1)
    double yn;                          // Valeur de la sortie y(n)
    double yn_1;                    	// Valeurs successives de la sortie y(n-1)
};



/*
 *
 * Classe représentant une equation récurrente du second ordre
 * 
 */

class Correcteur_2nd_Ordre_Discret
{
// -----------------------------------------------------------------------
// Constructeur de la classe
// -----------------------------------------------------------------------
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_2nd_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cxn_2=0,double m_cyn_1=0,double m_cyn_2=0);

// -----------------------------------------------------------------------
// Méthodes publiques
// -----------------------------------------------------------------------
public:
    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn() const;

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);


// -----------------------------------------------------------------------
// Attributs privées
// -----------------------------------------------------------------------
private:
    double cxn,cxn_1,cxn_2,cyn_1,cyn_2;  // Coefficients de l'équation récurrente
    double xn;                           // Valeur de l'échantillon d'entrée x(n)
    double xn_1,xn_2;                    // Valeurs successives des échantillons d'entrée x(n-1), x(n-2)
    double yn;                           // Valeur de la sortie y(n)
    double yn_1,yn_2;                    // Valeurs successives de la sortie y(n-1), y(n-2)
};



/*
 *
 * Classe permettant de récupérer les paramètres du drone comptenu dans un fichier
 * 
 */

class Parametres_Drone
{
// -----------------------------------------------------------------------
// Constructeur de la classe
// -----------------------------------------------------------------------
public:
    // Constructeur de la classe prenant en param�tre le nom du fichier contenant les paramètres
    Parametres_Drone(std::string const& nom_fichier);

// -----------------------------------------------------------------------
// Méthodes publiques
// -----------------------------------------------------------------------
public:
    // Fonction permettant de regrouper les fonctions qui récupérent les données du fichier paramètre
    void set_parameters(void);

    // fonctions qui permettent de retourner les attributs de la classe "Parametres_Drone"
    float get_rotation_max(void) const;
    float get_masse_arrachage(void) const;
    double get_coef_trainee(void) const;
    double get_coef_poussee(void) const;
    double get_envergure(void) const;
    struct Coef_Correcteurs get_roulis(void) const;
    struct Coef_Correcteurs get_tangage(void) const;
    struct Coef_Correcteurs get_lacet(void) const;
    struct Coef_Correcteurs get_consigne_smooth(void) const;
    int16_t get_offset_pwm(void) const;
    std::string get_fichier_log(void) const;
    float get_angle_max_roulis_tangage(void) const;
    float get_vitesse_max_lacet(void) const;
    bool get_poussee_exponentielle(void) const;
    bool get_affichage_parametres(void) const;
    bool get_affichage_consignes(void) const;
    bool get_affichage_erreur(void) const;
    bool get_affichage_AHRS(void) const;
    bool get_affichage_pwm(void) const;
    bool get_affichage_sorties_PID(void) const;
    bool get_affichage_commandes(void) const;
    bool get_test_poussee(void) const;

// -----------------------------------------------------------------------
// Méthodes privées
// -----------------------------------------------------------------------
private:
    // fonctions qui permettent de récupérer et initialiser les attributs de la classe "Parametres_Drone"
    void set_fichier_log(void);
    void set_rotation_max(void);
    void set_masse_arrachage(void);
    void set_coef_trainee(void);
    void set_coef_poussee(void);
    void set_envergure(void);
    void set_roulis(void);
    void set_tangage(void);
    void set_lacet(void);
    void set_consigne_smooth(void);
    void set_offset_pwm(void);
    void set_angle_max_roulis_tangage(void);
    void set_vitesse_max_lacet(void);
    void set_poussee_exponentielle(void);
    void set_affichage_parametres(void);
    void set_affichage_consignes(void);
    void set_affichage_erreur(void);
    void set_affichage_AHRS(void);
    void set_affichage_pwm(void);
    void set_affichage_sorties_PID(void);
    void set_affichage_commandes(void);
    void set_test_poussee(void);

    // Fonction qui permet d'aller à la ligne désirée
    void aller_a_la_ligne(int num_ligne);
    // Fonction qui permet de sauter le nombre de ligne désirée
    void aller_a_la_ligne_apres(int num_ligne_apres);

// -----------------------------------------------------------------------
// Attributs privées
// -----------------------------------------------------------------------
private:
    // Offset du PWM minimal envoyée au moteur (en %) 
    int16_t offset_pwm;
    
    // Nom du fichier log
    std::string fichier_log;

    // Rotation maximale
    float rotation_max;

    // Masse que produit le drone lorsque le stick de poussée est à fond
    float masse_arrachage;

    // Envergure du drone (attention à ne pas confondre avec la longueur du bras en configuration x)
    double envergure;

    // Coefficients de trainée et de poussée trouvés par expériences
    double coef_trainee, coef_poussee;

    // Coefficients des suites récursives des correcteurs du roulis, tangage, lacet et des consignes des manettes
    struct Coef_Correcteurs roulis,tangage,lacet,consigne_smooth;

    // Déclaration du fichier qui sera lu et contenant les paramètres
    std::ifstream fichier;

    // Angle maximal que peut faire le drone avec le roulis et le tangage et la vitesse angulaire de son lacet
    float angle_max_roulis_tangage, vitesse_max_lacet;

    // Booléen autorisant la poussée de façon exponentielle en fonction de la position du stick (true = poussee exponentielle)
    bool poussee_exponentielle;

    // Booléen permettant d'autorisé l'affichage de certaines données du debugger
    bool affichage_parametres,affichage_consignes,affichage_erreur,affichage_AHRS,affichage_pwm,affichage_sorties_PID,affichage_commandes;

    // Booléen permettant de mettre le drone en mode "test de poussée" (cela affiche les information importante sur la console debug)
    bool test_poussee;

    // Chaine de caractère permettant de lire les "booléen" du fichier paramètre
    std::string texte;
};



/*
 *
 * Classe représentant le drone
 * 
 */

class Quadri
{
// -----------------------------------------------------------------------
// Constructeur de la classe
// -----------------------------------------------------------------------
public:
    Quadri(std::string const& emplacement_fichier_parametres);


// -----------------------------------------------------------------------
// Méthodes publiques
// -----------------------------------------------------------------------
public:
    // Fonction qui permet de calculer les pwm de tous les moteurs
    void set_pwm_moteurs(void); 

    // Fonction de récupération des valeurs relevées par les capteurs
    void set_angle_roulis(double m_angle_roulis);
    void set_angle_tangage(double m_angle_tangage);
    void set_vitesse_angle_lacet(double m_vitesse_angle_lacet);

    // Fonction de récupération des valeurs pwm min et max
    void set_pwm_min(double m_pwm_min);
    void set_pwm_max(double m_pwm_max);

    // Fonction de récupération des consignes de poussée et de la vitesse du lacet
    void set_pilot_throttle_scaled(double m_pilot_throttle_scaled);
    void set_target_yaw_rate(float m_target_yaw_rate);
    
    // Fonction qui test et initialise un fichier log
    void ouverture_fichier_log(void);

    // Fonction qui test et ferme le fichier log
    void fermeture_fichier_log(void);

    // Fonction qui reset toutes les suites récursives
    void reset_PID(void);

    // Fonction qui retourne les pwm de chaque moteur
    double get_w1_pwm(void) const;
    double get_w2_pwm(void) const;
    double get_w3_pwm(void) const;
    double get_w4_pwm(void) const;

    // Fonction qui retourne l'angle maximal que puisse faire le roulis et le tangage (en °)
    float get_angle_max_roulis_tangage(void) const;

    // Fonction qui retourne la vitesse maximale du lacet (en °/s)
    float get_vitesse_max_lacet(void) const;

    // Booléen authorisant la poussée exponentielle si "true", sinon elle est linéaire
    bool get_poussee_exponentielle(void) const;


// -----------------------------------------------------------------------
// Méthodes privées
// -----------------------------------------------------------------------
private:
    // Fonction de récupération des commandes
    void set_u_phi(void);
    void set_u_theta(void);
    void set_u_r(void);
    void set_u_z(void);

    // Fonction de récupération des valeurs des pwm pour chaque moteurs
    void set_w1_pwm(void);
    void set_w2_pwm(void);
    void set_w3_pwm(void);
    void set_w4_pwm(void);

    // Ecriture du fichier log
    void titre_log(void);
    void ecriture_log(void);

    // Vérification du non-dépassement du PWM max
    void test_pwm(void);

    // Debugger 
    void debugger(void);


// -----------------------------------------------------------------------
// Attributs privées
// -----------------------------------------------------------------------
private:
    // -------------------------------------------------------------------
    // Déclarations des classes
    // -------------------------------------------------------------------

    // PARAMETRES :
    // classe qui récupére les paramètres du drone à partir d'un fichier
    Parametres_Drone params;

    // CORRECTEURS (wu = 25 rad/s) :

    // PID ROLL : yn = 57.108973.x(n) - 112.602070.x(n-1) + 55.500948.x(n-2) + 1.839006.y(n-1) - 0.839006.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_roll;

    // PID PITCH : yn = 41.048072.x(n) - 80.934704.x(n-1) + 39.892275.x(n-2) + 1.839006.y(n-1) - 0.839006.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_pitch;

    // PI R : yn = 0.834204.x(n) - 0.829007.x(n-1) + 1.000000.y(n-1)
    Correcteur_1er_Ordre_Discret pi_yaw;

    // Fonction smooth consigne : yn = 0.000152.x(n) + 0.000305.x(n-1) + 0.000152.x(n-2) + 1.950617.y(n-1) - 0.951227.y(n-2)
    Correcteur_2nd_Ordre_Discret roll_smooth,pitch_smooth,yaw_rate_smooth;


    // -------------------------------------------------------------------
    // Déclarations des variables
    // -------------------------------------------------------------------

    // ofstream est utilisé pour écrire un fichier CSV nommé IMS5_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
    std::ofstream fichier_log;

    // variable vérifiant si le fichier est bien ouvert ou non
    bool fichier_log_ouvert;

    // nom du mode de vol qui apparaitra dans le fichier log
    std::string nom_programme;

    // Sortie des correcteurs
    double u_theta, u_phi, u_r, u_z;

    // Commandes (moteurs en rad/s)
    double w1,w2,w3,w4;

    // Commandes (moteurs en pwm)
    int16_t w1_pwm,w2_pwm,w3_pwm,w4_pwm;

    // Valeurs de pwm min et max pour faire tourner les moteurs
    int16_t pwm_min,pwm_max;

    // Offset du pwm min 
    int16_t offset_pwm;

    // Consigne de vitesse de lacet en centidegré/seconde
    float target_yaw_rate;

    // Consigne de poussée entre 0 et 1
    float pilot_throttle_scaled;

    // Consignes en radians et radians/seconde
    double target_roll_rad, target_pitch_rad, target_yaw_rate_rad; // Target angles en radians et radians/s
    double target_throttle_newton;                                 // Target poussée en Newton
    double target_roll_smooth, target_pitch_smooth, target_yaw_rate_smooth; // Target angles en radians et radians/s en smooth

    // Angles et vitesse récupérer par les capteurs (AHRS)
    double angle_roulis, angle_tangage, vitesse_angle_lacet;

    // Booléen permettant d'activer la fonction debugger
    bool debug;

// -----------------------------------------------------------------------
// Attributs public (à limité au maximum!)
// -----------------------------------------------------------------------    
public:
    // Index des moteurs (nécessaire pour adapter la numérotation des moteurs entre la loi de commande et Arducopter)
    const int16_t w1_index=4;                 // Représentation du drone en configuration en x
    const int16_t w2_index=2;                 //    W1   W3         4   1
    const int16_t w3_index=1;                 //       x      =>      x
    const int16_t w4_index=3;                 //    W4   W2         3   2

    // Consigne en centi-degré du roulis et du tangage
    float target_roll, target_pitch;
};
