%% Createur de fichier pour le drone
%% Description
% Il s'agit d'un script MatLab qui permet de generer un fichier contenant
% tous les parametres du drone. Le drone lira le fichier au debut du
% lancement du mode IMS. 
% De plus ce fichier comporte quelques booleen permettant d'activer le
% debugger.

%% commandes de base
close all, clc, clear all, format long;
p = tf('s');
%% initialisation
valeurs_initiales

%% Calculs de la rotation maximale du moteur
rotation_max = calcul_rotation_max(KV,niveau_tension_batterie); % en rad/s

%% Calculs des moments d'inertie
moment_inertie

%% Calculs du coefficient de trainee
coefficient_de_poussee

%% Calculs des correcteurs
correcteurs

%% Creation du fichier parametre
createur_fichier