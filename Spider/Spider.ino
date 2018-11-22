/* Programme COMPLET issu de P30_T4 mais sur lequel, dans le but de minimiser le code, ont été apportées
   les modifications précisées dans l'historique relatant chaque changement d'occupation. 

>>>> Avant de télécharger ce programme il faut impérativement loger les textes en EEPROM. Dans ce but il
     suffit d'activer l'utilitaire dédié P28_Ecrire_les_textes_en_EEPROM.ino.
>>>> Avant de télécharger ce programme il faut impérativement loger les tables de postures.
     Dans ce but il suffit d'activer l'utilitaire dédié P29_Ecriture_des_tableaux_en_EEPROM_T4.
     
>>>> Attendre que la sonde soit en posture VEILLE "p01*" pour la soulever de son berceau
     et la déposer au sol.

>>>> Pensez à terminer Les manipulation par l'appel de "q*" la commande "Quitter le programme".

UTILISATION DU PROGRAMME : Il suffit par le Moniteur série USB d'invoquer à votre guise
   les programmes développés dans ce démonstrateur ainsi que les commandes effectuées
   sur un caractère actuellement disponibles :
   
   "PNr*" : Programme N effectué r fois avec R = 'a', 'b', 'c', 'd' et 'e'.
   ------------------------------------------------------------------------
   P01 : Fait passer en mode VEILLE.
   P02 : Faire passer la sonde en posture Stable Transversal.
            >>> Mouvements de "p" pas si forme de consigne "PNn* <<<
   P03 : Avancer d'un pas et configurer en position Stable Transversal.
   P04 : Reculer d'un pas et configurer en position Stable Transversal.
   P05 : Tourner à droite d'un pas et configurer en position Stable Transversal.
   P06 : Tourner à gauche d'un pas et configurer en position Stable Transversal.
   P07 : Décaler à droite d'un pas et configurer en position Stable Transversal.
   P08 : Décaler à gauche d'un pas et configurer en position Stable Transversal.
   P09 : Piloter un moteur au potentiomètre.
   P10 : Configuration Hauteur Maximale.
   P11 : Retour de configuration Hauteur Maximale.
   P12 : Configuration Stable LASER.
   P13 : Retour de Conf. Stable LASER vers Stable Transvrsl.
   P14 : Libérer les efforts.
   P15 : Tous les moteurs au neutre opérationnel.
   P16 : Configuration Stable Raisonnable.
   P17 : Retour de Stable raisonnable vers VEILLE.
   P18 : Enregistrement en EEPROM de la posture actuelle.
   P19 : Utiliser l'enregistrement EEPROM de la posture.
   P20 : Lister l'enregistrement EEPROM de la posture.
   P21 : Dépose la sonde.
   P22 : Configuration atterrissage.
   P23 : Configuration décollage.

   *  : "*" seule réitère la dernière commande de mouvement.   
   a* :	Allume les Phares OUI / NON.
   b* :	Bloquer le niveau des Phares.
   c* :	Bascule OUI/NON d'ACR de Configuration sur la ligne USB. (ACR : Accusé de réception.)
   d* :	Début du mode apprentissage. (Si EEPROM effacée avec "y*".)
   e* :	Enregistrer en EEPROM un balayage de télémétrie pour avoir la situation horizontale.
   f* : Figer tous les moteurs.
   g* : Retourne une fois valeurs gyroscopiques sur la ligne USB. (Gyroscope recalé si '=*' armé.
   h* :	Active le balayage LASER en Horizontal. (Bascule.)
   i* :	Informations sur l'état de la sonde.
   j* :	Jalonne : Retourne sur la ligne USB la distance Télémétrique.
   k* :	Fin du mode pilotage des moteurs en manuel. (KILL.)
   l* :	Active le LASER. (Bascule de type OUI / NON.)
   m* : Valeurs_meteorologiques listées sur la ligne USB.
   n* :	Potentiomètre actif ou Neutralisé à 127.
   o* :	Retourne sur la ligne USB le cap de l'Orientation magnétique.
   p* :	Désactive la Torsion et replace en Position au centre.
   q* :	Quitter le programme.
   r* :	Mouvements coordonnés Rapides. (Bascule.)
   s* :	Sommeil OUI / NON.  (Éteindre toutes les LED)
   t* :	Active le positionnement en Torsion d'orientation.
   u* :	Utiliser un programme enregistré en mémoire non volatile.
   v* :	Active le balayage LASER en Vertical. (Bascule.)
   w* :	Restituer le balayage de télémétrie enregistré en EEPROM. (Waveform.)
   x* :	Lister sur la ligne USB le programme enregistré en EEPROM.
   y* :	Effacer un programme en EEPROM pour pouvoir débuter un
	autre apprentissage avec "d*".
   z* :	LAffiche la version du programme sur la ligne série.
   =* : Arme le recalage gyroscopique.
   &* : Liste en continu ou stoppe l'affichage des valeurs gyroscopiques.
   (* : Enregistrement en EEPROM d'un spectre colorimétrique.
   )* : Affichage sur l'écran du P.C. des valeurs du spectre colorimétrique.
   _* : (Souligné et non le moins.) Stabilisation GYROSCOPIQUE.
   
DIFFERENTS BRANCHEMENTS :
 A0 : Tension potentiométrique de commande. (0 à +5Vcc)
 A1 : F.E. (Future Expansion.)
 A2 : Pilotage LED de torsion active.
 A3 : Clignotement de la LED de "boucle de PGM active".
 A4 -> SDA pour la ligne ligne I2C.
 A5 -> SCL pour la ligne ligne I2C.
 A6 : Mesure de la lumière pour le luxmètre.
 A7 : Mesure de la tension d'alimentation des servomoteurs.
 
 D2 : Pilotage du BUZZER. (Optionnel le dialogue série accusant réception.
 D3 : Entrée pour tester le Swich de "bouclier en contact avec le sol".
 D4 : Pilotage de la base du transistor de commutation des masses LEDs.
 D5 : Sortie pour l'éclairage analogique.
 D6 : F.E. (Future Expansion.)
 D7 : Pilotage LED témoin Moteurs figés.
 D8 : Pilotage LED témoin Potensiomètre Neutralisé.
 D9 : Pilotage LED H ou V pointage LASER activé.
 D10 : LED du répétiteur de CAN du potentiomètre.
 D11 : Ligne de dialogue avec le capteur d'humidité DHT11.
 D12 : Trig capteur ultrasons HC-SR04.
 D13 : Echo capteur ultrasons HC-SR04.
 
 Sortie S12 Multiplexeur : Recalage Gyroscope armé.
 Sortie S13 Multiplexeur : LED du mode "Niveaux Lumineux Bloqués".
 Sortie S14 Multiplexeur : LED du mode Enregistrement.
 Sortie S15 Multiplexeur : LED du mode pilotage Manuel des moteurs.
 
                                                             PGM              RAM dynamique 
Programme démonstrateur initial P30_T4 :               28792 Octets (93%)    805 Octets (39%)
//------------------------- A partir d'ici les modifications apportées à P30_T4 --------------------------
Ajouté "* qui affiche la version du PGM.               28778 Octets (93%)    819 Octets (39%)  -14 octets
Curieusement on ajoute un ligne d'instruction et fait baisser la taille du code !!!
"z*" affiche maintenant la version du programme.
Enlevé la fonction qui listait l'EEPROM.               28506 Octets (92%)    817 Octets (39%) -272 octets
Enlevé les tests de programme préalable.               28436 Octets (92%)    793 Octets (38%)  -70 octets
Ajouté la correction automatique du compas de route    28746 Octets (93%)    793 Octets (38%) +310 octets
Ajouté le spectromètre colorimétrique                  28952 Octets (94%)    793 Octets (38%) +206 octets
Enlevé l'appel à "libèrer les efforts" à chaque mouvement car pénalise trop les mvt répétitifs.
Ajouté la stabilisation gyroscopique                   29464 Octets (95%)    806 Octets (39%) +512 octets

// Reste 1256 Octets.

//--------------------- Déclarations pour le matériel ----------------------- */
#include <Wire.h>
#include <I2Cdev.h> // Pour le MPU6050 : Directive obligatoirement placée ici.

#include <avr/eeprom.h> // Bibliothèque intégrée à l'IDE qui gère les entiers ,les réels ...
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "positions.h"


#define Lecture_Adresse 0x41  //"A" en hexadécimal correspond à une commande pour le capteur Magnétique.
//--------------- Déclarations pour le module inertiel MPU6050 --------------
// Inclusion d'une bibliothèque permettant de communiquer plus facilement sur l'interface I2C.
#include <MPU6050_6Axis_MotionApps20.h>
MPU6050 Module_Inertiel;
uint16_t Taille_attendue; // Taille du paquet DMP attendu. (Par défaut : 42 octets.)
uint16_t NB_octets;       // Compte de tous les octets actuellement dans la pile FIFO.
uint8_t PILE_FIFO[64];    // Tampon de stockage de la pile FIFO.
Quaternion Q;           // [w, x, y, z]  Quaternion.
VectorFloat Vecteur_gravitation;    // [x, y, z]     Vecteur de pesanteur.

//--------------- Définition des broches d'Entrées Analogiques --------------
#define Entree_mesuree 0 // Entrée analogique A0 utilisée pour mesurer la tension du potentiomètre.
#define Entree_Luxmetre 6 // Entrée analogique A6 utilisée pour le luxmètre.
#define Entree_Alim_moteurs 7 // Entrée analogique A7 utilisée pour mesurer la tension de puissance.
//--------------- Définition des broches d'Entrées BINAIRES -----------------
#define Bouclier_au_sol 3 // Entrée de détection du blouclier en contact avec le sol.
//------------------ Sorties sur broches "Analogiques" ----------------------
#define LED_Torsion_active 16 // Entrée analogique A2 utilisée comme Sortie Binaire.
#define LED_Loop_active 17 // Entrée analogique A3 utilisée comme Sortie Binaire.
//-------------- Définition des broches de sorties Binaires -----------------
#define BUZZER 2 // Le bruiteur est piloté par la sortie binaire D2.
#define Masse_LEDs 4 // Masse des LEDs pour le mode Sommeil : Sortie binaire D4.
#define LED_Moteurs_OFF 7 // Pilotage LED témoin Moteurs figés.
#define LED_Pot_OFF_a_127 8 // Pilotage LED témoin Potentiomètre Neutralisé.
#define LED_pointage_V_ou_H 9 // Pilotage LED H ou V pointage LASER activé.
#define Broche_Com_DH11 11 // Ligne de dialogue avec le capteur d'humidité DHT11.
#define TX_Pulse_sonard 12 // TX capteur ultrasons sur la sortie D12.
#define Echo 13 // RX capteur ultrasons sur la sortie D12.
//-------------- Broches de sorties Binaires utilisées en PWM ---------------
#define Pilotage_eclairage 5 // Pilotage des deux LEDs phares de la sonde : Sortie binaire D5.
#define LASER 6 // Sortie binaire D6 pour piloter le LASER en PWM.
#define Repetiteur_CAN 10 // Sortie D10 pour piloter la LED répétiteur di CAN.

//------------------------ Constantes du programme --------------------------
#define Version_PGM "Version P30-T5." // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define Calibre_U_moteurs 6.67 // Calibrage du diviseur potentiométrique d'entrée. @@@@@@@@@@@@@@@@@@@@@@@@@@@
#define PGMmax 23 // Nombre maximum de programmes.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define CAN_Inf 4     // Valeur minimale retournée par le CAN du potentiomètre. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                   
#define CAN_Sup 1000  // Valeur maximale retournée par le CAN du potentiomètre. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define Consigne_Mini 168  // Butée - la plus grande de tous les moteurs. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define Consigne_Maxi 424  // Butée + la plus petite de tous les moteurs. @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define NB_MAX  12 // Nombre de servomoteurs branchés.
#define LGR_chaine 7 // Longueur maximale de la chaine de dialogue.

//------------------------- Variables du programme --------------------------
// Note : Les déclarations sont classées par type et par ordre alphabétique croissant.
boolean ACR_configuration_sur_USB; // Affiche la configuration sur la ligne USB : OUI/NON.
boolean Apprentissage; // Mode apprentissage en cours.
boolean Appris; // L'EEPROM contient un programme enregistré.
boolean Bloquer_niveaux; // Bloquer les niveaux du LASER et des Phares.
boolean Horizontal; // Le LASER est orienté horizontalementment. (Consigne potentiométrique.)
boolean Initialiser_le_gyroscope;
boolean LASER_actif; // Allumer ou éteindre le LASER.
boolean Listage_Gyro_en_continu;
boolean Mode_manuel; // Piloter un moteur en manuel.
boolean Moteurs_ON; // Si false tous les moteurs sont figés.
boolean PGM; // Indique si la consigne invoque un Programme.
boolean Phares_actifs; // Allumer ou éteindre les phares.
boolean Posture_LASER;
boolean Potentiometre; // Active (Si true) ou neutralise le potentiomètre.
boolean Rapide; // Les mouvements coordonnés seront rapides.
boolean Sentinelle_detectee;  // Délimiteur de fin détecté.
boolean Sommeil; // Coupe tous les consommateurs d'énergie. (LEDs, LASER.)
boolean Spectre;
boolean Stabilisation_gyroscopique;
boolean Torsion_Active; // Les corrections en torsion sont actives.
boolean Vertical; // Le LASER est orienté verticalement. (Consigne potentiométrique.)
boolean Requette_MPU6050;

char Caractere;
byte Compteur_caracteres;
byte Dernier_PGM_actif; // Contient le dernier programme activé.
byte Distance; // Utilisée également pour la gestion de l'EEPROM.
byte LSB; // Utilisé par le capteur magnétique.
byte MSB; // Utilisé par le capteur magnétique.
byte NB_enregistrements;
byte Niveau_eclairage_LASER; // Pilotage analogique du LASER.
byte Niveau_eclairage_Phares; // Pilotage analogique des deux phares.
byte Nombre_de_pas;
byte num_ERR;
byte Num_Moteur;
byte Num_Sortie;

int  Adresse_du_HMC6352 = 0x42; // Écrire à cette adresse. (Capteur magnétique.)
int  Angle; // Int pour pouvoir faire varier entre -90 et +90 et compatible string;
           // ATTENTION : Angle est également utilisé par l'écriture EEPROM en mode en apprentissage.
int  CAN, Numerisation;
int  Lecture_Echo;
int  Lumiere_ambiante;
int  NOMBRE; // Contiendra la transformée numérique d'une chaine.
int  PTR_Apprentissage; // Pointeur du programme enregistré en EEPROM.
int  PTR_EEPROM; // Pointeur des octets situés en EEPROM.
int  Tangage, Roulis, Lacet, Consigne_stabilisation;

unsigned long Ancienne_valeur_Temps_Ecoule_depuis_RESET;

float CAP_construit; // Utilisé par le capteur magnétique.
float LTR[3];           // Lacet/Tangage/Roulis.
float Calage_gyro_initial;

int Configuration_actuelle[12];
int Configuration_voulue[12];
int Increment[12];

String Chaine_Memorisee; // Tampon mémoire pour la chaine entrante.
String SAV_Consigne; // Tampon mémoire pour la chaine entrante.
//-------------------------- Fin des déclarations ---------------------------

void setup() {
  Serial.begin(115200);
  //------------ Mise en service de la centrale gravitationnelle ------------
  Initialiser_le_gyroscope = true; 
  Listage_Gyro_en_continu = false;
  Serial.flush(); // S'assurer que le tampon de transmission est vide.
  Wire.begin(0x20); //Initialise la liaison I2C avec un "esclave".
  // Initialisation du capteur accélérométrique et gyroscopique :
  Module_Inertiel.initialize();
  // Chargement et configuration du DMP :
  int devStatus = Module_Inertiel.dmpInitialize();
  if (devStatus == 0) {
    // Activation du module DMP6050.
    Module_Inertiel.setDMPEnabled(true); // Activation du DMP.
    // Acquisition de la taille de paquet DMP pour comparaison ultérieure
    Taille_attendue = Module_Inertiel.dmpGetFIFOPacketSize(); 
    // 1 = échec de chargement initial de la mémoire.
    // 2 = échec des mises à jour de configuration du DMP.
  } else {
    Serial.print(devStatus); 
    Espace(); 
    Erreur_N_BIP(33);
  }
  // Test de la connexion
  if (Module_Inertiel.testConnection()) {
    Moteurs_ON = false; 
    Module_Inertiel.resetFIFO();
   // Ci-dessus Reset pour vider le FIFO et s'assurer d'une transmission correcte.
   Affiche_le_vecteur_gravitation(); Moteurs_ON = true; // Affiche les angles initiaux.
  } else {
    Erreur_N_BIP(30); 
  }
  Requette_MPU6050 = false;
  //-------------------------------------------------------------------------
  Adresse_du_HMC6352 = Adresse_du_HMC6352 >> 1; 
  Wire.begin(); //(Capteur magnétique.)
  Dernier_PGM_actif = 1; 
  Potentiometre = true;  
  Rapide = true;
  Phares_actifs = false; 
  ACR_configuration_sur_USB = false; 
  Torsion_Active = false;
  Sommeil = false; 
  Bloquer_niveaux = false; 
  LASER_actif = false; 
  Mode_manuel = false;
  Stabilisation_gyroscopique = false;
  Posture_LASER = false; 
  Annule_Direction_Laser();
  Appris = Lire_un_OCTET_en_EEPROM(552);
  pinMode(Bouclier_au_sol,INPUT); 
  pinMode(Echo,INPUT);
  pinMode(TX_Pulse_sonard,OUTPUT); 
  digitalWrite(TX_Pulse_sonard,LOW); // TX HC-SR04 ultrasons.
  pinMode(LED_Torsion_active,OUTPUT); 
  pinMode(LED_Loop_active,OUTPUT);
  pinMode(LED_Moteurs_OFF,OUTPUT); 
  pinMode(LED_Pot_OFF_a_127,OUTPUT); 
  pinMode(LED_pointage_V_ou_H,OUTPUT);
  pinMode(Masse_LEDs,OUTPUT); 
  digitalWrite(Masse_LEDs,HIGH); // Transistor saturé. (Masse active.)
  Serial.println(); 
  Aff_TXT_EEPROM_et_CRLF(681,4);
  Chaine_Memorisee.reserve(LGR_chaine); // Reserve LGR_chaine octets pour la chaine.
  SAV_Consigne.reserve(LGR_chaine); // Reserve LGR_chaine octets pour la chaine.
  pwm.begin(); // Initialise le module multiplexeur  
  pwm.setPWMFreq(50);  // Fréquence de la PWM : En standard 50Hz pour des servomoteurs.   
  configurer(8); // Impératif pour initialiser la table des positions actuelles. (VEILLE.)
} 
  
void loop() {
  if (Requette_MPU6050) {
    Module_Inertiel.resetFIFO(); // Reset pour vider le FIFO et s'assurer d'une transmission correcte.
    Affiche_le_vecteur_gravitation(); 
    Requette_MPU6050 = false;
  }
  if (Listage_Gyro_en_continu) {
    Module_Inertiel.resetFIFO(); // Reset pour vider le FIFO et s'assurer d'une transmission correcte.
    Affiche_le_vecteur_gravitation();
  }
  //-------------- Traite une consigne arrivée sur USB ---------------
  if (Serial.available()) {
    Attendre_une_chaine(); 
    Serial.println(); 
    Serial.print(Chaine_Memorisee); 
    Aff_TEXTE_EEPROM(667,3);
    Analyseur_syntaxique();
    if (Compteur_caracteres == 2) {
      Traite_PGM_un_caractere(); 
      if (num_ERR == 0) 
        Affiche_ACR();
    }else {
      if (num_ERR == 0) {
        Traite_Programme(); 
        Affiche_ACR();
      } else {
        ERREUR();
      }
    }
    if (ACR_configuration_sur_USB) 
      Affiche_la_config_actuelle();
  }
  //-------------- Traiteles LED témoins d'état de la Sonde ----------
  // Faire clignoter la LED de la boucle de base active :
  if (millis() > Ancienne_valeur_Temps_Ecoule_depuis_RESET) {
    LSB = 100; 
    if (Stabilisation_gyroscopique) 
      LSB = 30; 
    Ancienne_valeur_Temps_Ecoule_depuis_RESET = millis() + LSB; // Réarme pour 0,1S ou 0,03S.
    digitalWrite(LED_Loop_active,!digitalRead(LED_Loop_active)); // Inverser l'état de la LED.
  } 
  //--------------- Fin des séquences de clignottement ---------------
  if ((Vertical) || (Horizontal)) 
    digitalWrite(LED_pointage_V_ou_H,HIGH);
  else 
    digitalWrite(LED_pointage_V_ou_H,LOW);
  if (Potentiometre) 
    digitalWrite(LED_Pot_OFF_a_127,LOW); 
  else 
    digitalWrite(LED_Pot_OFF_a_127,HIGH);
  // Si NON apprentissage en cours gérer la LED moteurs OFF.
  if (Moteurs_ON) 
    digitalWrite(LED_Moteurs_OFF,LOW); 
  else 
    digitalWrite(LED_Moteurs_OFF,HIGH);
  if (Bloquer_niveaux) 
    pwm.setPWM(13,0, 254); 
  else 
    pwm.setPWM(13,0, 0); // Niveaux Bloqués.
  
  if (Torsion_Active) 
    digitalWrite(LED_Torsion_active,HIGH); 
  else 
    digitalWrite(LED_Torsion_active,LOW);
  
  if (Apprentissage) 
    pwm.setPWM(14,0, 254); 
  else 
    pwm.setPWM(14,0, 0); // LED Mode enregistrement.
  
  if (Mode_manuel) 
    pwm.setPWM(15,0, 254); 
  else 
    pwm.setPWM(15,0, 0); // LED Mode Manuel.
  if (Initialiser_le_gyroscope) 
    pwm.setPWM(12,0, 254); 
  else 
    pwm.setPWM(12,0, 0); // LED Mode Manuel. 
  //---------------- Traite les fonctions analogiques ----------------
  CAN = map(analogRead(Entree_mesuree),CAN_Inf,CAN_Sup,6,254);
  analogWrite(Repetiteur_CAN, CAN); // LED du répétiteur analogique.
  if (Potentiometre) 
    Numerisation = CAN; 
  else 
    Numerisation = 127;
  if (!Bloquer_niveaux) {
    Niveau_eclairage_Phares = Numerisation; 
    Niveau_eclairage_LASER = Numerisation;
  } 
  if (Sommeil) 
    digitalWrite(Masse_LEDs,LOW); 
  else 
    digitalWrite(Masse_LEDs,HIGH);
  
  if (Phares_actifs) 
    analogWrite(Pilotage_eclairage, Niveau_eclairage_Phares);
  else 
    analogWrite(Pilotage_eclairage, 0);
  //-------------------- Traite spécifique au LASER ------------------ 
  if ((LASER_actif) && (!Vertical) && (!Horizontal) && (!Torsion_Active))
      analogWrite(LASER, Niveau_eclairage_LASER);  
  
  if (!LASER_actif) {
    analogWrite(LASER, 0); 
    Annule_Direction_Laser();
  }
  if (Horizontal) {
    CAN = map(analogRead(Entree_mesuree),CAN_Inf,CAN_Sup,225,415); 
    Mouvoir(0, CAN);
  }
  if (Vertical) {
    CAN = map(analogRead(Entree_mesuree),CAN_Inf,CAN_Sup,197,400); 
    Mouvoir(1, CAN);
  }
  //----------------- Traite les mouvements de Torsion ---------------
  if (Torsion_Active) {
    CAN = analogRead(Entree_mesuree); 
    if(!Potentiometre) 
      CAN = (CAN_Inf + CAN_Sup) >> 1 ;
    Bouge_pour_un_balayage_en_torsion();
  }
  //-------------------- Piloter_un_moteur_en_manuel -----------------    
  // Num_Sortie est représentatif du moteur piloté.           
  if ((num_ERR == 0) && (Mode_manuel)) {
    if (Potentiometre)
      Numerisation = map(analogRead(Entree_mesuree),CAN_Inf,CAN_Sup,Consigne_Mini,Consigne_Maxi);
    else 
      Numerisation = Configuration_actuelle[Num_Sortie];
    Mouvoir(Num_Sortie, Numerisation);
  }
  //--------------- Gère la stabilisation Gyroscopique ---------------
  if (Stabilisation_gyroscopique) {
    Stabilisation_GYROSCOPIQUE();
  } 
}

//======================== Routines d'aiguillage pour réaliser les consignes ========================
void Traite_PGM_un_caractere() {// Placer un break sur tous les cas y compris le dernier.
  Caractere = Chaine_Memorisee[0]; 
  Apprentissage = false;
  switch (Caractere) {         
    case 'a' : Phares_actifs = !Phares_actifs; break; // Allumer les phares OUI/NON.
    case 'b' : Bloquer_niveaux = !Bloquer_niveaux; break;
    case 'c' : ACR_configuration_sur_USB = !ACR_configuration_sur_USB; break;
    case 'd' : Debuter_un_apprentissage(); break;
    case 'e' : Spectre = true; Balayage_telemetrique(); break;
    case 'f' : Moteurs_ON = !Moteurs_ON; break;
    case 'g' : Requette_MPU6050 = !Requette_MPU6050; break;
    case 'h' : Traite_V_H(false, true); break;
    case 'i' : Etat_actuel_de_la_sonde(); break; // 'i' pour informations.
    case 'j' : Spectre = false; Telemetre_ultrasons(); break; // 'J' pour Jalonner.
    case 'k' : Mode_manuel = false; break; // Fin du pilotage manuel. (Kill.)
    case 'l' : LASER_actif = !LASER_actif; break;
    case 'n' : Potentiometre = !Potentiometre; break; // Neutralise le potentiomètre.
    case 'o' : Indique_orientation_sur_USB(); break; // Module magnétique HMC6352.
    case 'p' : Stoppe_la_Torsion_et_recentre(); break;
    case 'q' : Quitter_le_programme(); break;
    case 'r' : Rapide = !Rapide; break;
    case 's' : Sommeil = !Sommeil; Moteurs_ON = false; break;
    case 't' : Active_la_correction_en_Torsion(); break;
    case 'u' : Utilise_PGM_en_memoire(); break;
    case 'v' : Traite_V_H(true, false); break;
    case 'w' : Visualiser_un_Spectre_telemetrie_sur_USB(); break;
    case 'x' : Liste_PGM_appris_sur_USB(); break;
    case 'y' : Efface_PGM_en_EEPROM(); break;
    case 'z' : Serial.println(); Serial.println(Version_PGM); break;
    case '=' : Initialiser_le_gyroscope = true; break;
    case '&' : Listage_Gyro_en_continu = !Listage_Gyro_en_continu; break;
    case '(' : Spectroscope_Chromatique(); break;
    case ')' : Restituer_le_spectre_colorimetrique(); break;
    case '_' : Stabilisation_gyroscopique = !Stabilisation_gyroscopique; break;
    default : Erreur_N_BIP(6);
  }
}
          
void Traite_Programme() {
  if (Compteur_caracteres == 1) {
    NOMBRE = Dernier_PGM_actif; // Réitère le dernier programme invoqué.
    Chaine_Memorisee = "*a"; // Obligatoirement une chaîne de 2 caractères.
    } 
  if (Compteur_caracteres == 2) 
    Traite_PGM_un_caractere();
  else {
    Annule_Direction_Laser(); 
    Torsion_Active = false;
    if (Chaine_Memorisee[0] == '0') 
      Chaine_Memorisee[1] = 'a'; // Éviter une fausse erreur n° 5.
    Torsion_Active = false; // Tous les programmes neutralisent les déplacements LASER sur la jambe A.
    if (NOMBRE == 9) { // Piloter un moteur en manuel.
      Nombre_de_pas = 1; 
      Num_Sortie = byte (Chaine_Memorisee[1]) - 97;
      if (Num_Sortie > 11) 
        Erreur_N_BIP(18); 
      else 
        Piloter_un_moteur_en_manuel();
    } else {
      Dernier_PGM_actif = NOMBRE; 
      switch (Chaine_Memorisee[1]) {// Répétition d'un déplacement..
        case 'a' : 
          Nombre_de_pas = 1; 
          break;
        case 'b' : 
          Nombre_de_pas = 2; 
          break;
        case 'c' : 
          Nombre_de_pas = 3; 
          break;
        case 'd' : 
          Nombre_de_pas = 5; 
          break;
        case 'e' : 
          Nombre_de_pas = 10; 
          break;
        default : 
          Nombre_de_pas = 1; 
          if (NOMBRE < 10) 
            Erreur_N_BIP(5); 
      }        
      if (Nombre_de_pas > 1) {
        Serial.println(); 
        Aff_TEXTE_EEPROM(703,6); 
        Espace();
        Serial.println(Nombre_de_pas);
      }
      for (Nombre_de_pas; Nombre_de_pas > 0; Nombre_de_pas--) {
        switch (NOMBRE) {
          case  1 : 
            Passer_en_VEILLE(); 
            break;
          case  2 : 
            REVEILLER(); 
            break; // Configuration Stable Travers.
          case  3 : 
            Avancer_un_pas(); 
            break;
          case  4 : 
            Reculer_un_pas(); 
            break;
          case  5 : 
            Tourne_a_droite(); 
            break;
          case  6 : 
            Tourne_a_gauche(); 
            break;
          case  7 : 
            Decaler_lateralement(15); 
            break; // Décale latéralement à Droite.
          case  8 : 
            Decaler_lateralement(16); 
            break; // Décale latéralement à Gauche.
          case 10 : 
            Hauteur_Maximale(); 
            break;
          case 11 : 
            Retour_de_hauteur_maximale(); 
            break;
          case 12 : 
            Configuration_stable_LASER(); 
            break;
          case 13 : 
            Retour_de_configuration_LASER(); 
            break;
          case 14 : 
            Libere_efforts(); 
            break;
          case 15 : 
            configurer(2); 
            break; // Tous les moteurs au neutre opérationnel.
          case 16 : 
            Configuration_Stable_Raisonnable(); 
            break;
          case 17 : 
            Config_Stable_Raisonnable_vers_VEILLE(); 
            break;
          case 18 : 
            Enregistrement_de_la_posture_Actuelle_en_EEPROM(); 
            break;
          case 19 : 
            Utilise_la_posture_sauvegardee_en_EEPROM(); 
            break;             
          case 20 : 
            Affiche_enregistrement_de_posture_sur_EEPROM(); 
            break;
          case 21 : 
            Depose_la_sonde(); 
            break;
          case 22 : 
            configurer(4); 
            break; // Configuration Atterrissage.
          case 23 : 
            Configure_pour_Decollage(); 
            break;
        }
      }
    }
  }
}
                          
//=========================== Routines pour la Stabilisation GYROSCOPIQUE ============================
void Stabilise(boolean Plus, byte Sortie) {
  Consigne_stabilisation = Configuration_actuelle[Sortie];
  if (Plus) {
    Consigne_stabilisation++; 
  } else {
    Consigne_stabilisation--;
  }
  Mouvoir(Sortie,Consigne_stabilisation);
}
  
void Stabilisation_GYROSCOPIQUE() {
  Mesure_les_valeurs_de_gravitation();
  if (Roulis > 0) {
    Stabilise(true, 1); 
    Stabilise(false, 4); 
    Stabilise(false, 7); 
    Stabilise(true, 10);
  }
  if (Roulis < 0) {
    Stabilise(false, 1); 
    Stabilise(true, 4); 
    Stabilise(true, 7); 
    Stabilise(false, 10);
  }
  if (Tangage > 0) {
    Stabilise(true, 1); 
    Stabilise(true, 2); 
    Stabilise(true, 4); 
    Stabilise(true, 5);
    Stabilise(false, 7); 
    Stabilise(false, 8); 
    Stabilise(false, 10); 
    Stabilise(false, 11);
  }
  if (Tangage < 0) {
    Stabilise(false, 1); 
    Stabilise(false, 2); 
    Stabilise(false, 4); 
    Stabilise(false, 5);
    Stabilise(true, 7); 
    Stabilise(true, 8); 
    Stabilise(true, 10); 
    Stabilise(true, 11);
  }
}                          

//========================== Routines pour la spectroscopie chromatique ==============================            
void Spectroscope_Chromatique() {// Lecture_Echo contient la consigne angulaire de positionnement.
  Configuration_stable_LASER();
  PTR_Apprentissage = 603; Lecture_Echo = 211; // Positionne sur "transparent".
  for (byte I=1; I < 8; I++) {
    Mouvoir(0, Lecture_Echo); 
    delay(200);
    Lecture_Echo = Lecture_Echo + 34; // Ajoute un "angle" pour aller à la couleur suivante.   
    Lumiere_ambiante = analogRead(Entree_Luxmetre);
    Ecrire_un_Entier_en_EEPROM(PTR_Apprentissage, Lumiere_ambiante);
  }
  Retour_de_configuration_LASER(); 
  Caractere ='(';
} 

void Restituer_le_spectre_colorimetrique() {
  PTR_EEPROM = 603; 
  Serial.println();
  for (byte I=1; I < 8; I++) {
    Serial.println(Lire_un_Entier_en_EEPROM(PTR_EEPROM)); 
    PTR_EEPROM++; 
    PTR_EEPROM++;
  }
}
            
//================================ Routines pour les consignes "C*" ==================================
void Quitter_le_programme() {
  Moteurs_ON = true; 
  Rapide = true; 
  Passer_en_VEILLE(); 
  delay(1000); 
  Sommeil = true;
  Phares_actifs = false; 
  LASER_actif = false; 
  Torsion_Active = false; 
  Potentiometre = false; 
  Aff_TXT_EEPROM_et_CRLF(883,5); 
  Serial.println();
}

//--------------------------------- Capeur gravitationnel MPU6050 ------------------------------
void Mesure_les_valeurs_de_gravitation() {
  Module_Inertiel.resetFIFO(); // Reset pour vider le FIFO et s'assurer d'une transmission correcte.
  NB_octets = Module_Inertiel.getFIFOCount(); // Acquisition du compte actuel de la pile FIFO.
  // Surveillance d'overflow. (Qui ne devrait jamais se produire sauf si le code est inefficace.)
  if (NB_octets == 1024) {
    Erreur_N_BIP(32);
    Module_Inertiel.resetFIFO();// Reset pour pouvoir continuer proprement. 
  } else { 
    // Attente en principe très courte des données de longueur correcte.
    while (NB_octets < Taille_attendue) {
      NB_octets = Module_Inertiel.getFIFOCount(); 
    }
    // Lecture d'un paquet dans la pile FIFO.
    Module_Inertiel.getFIFOBytes(PILE_FIFO, Taille_attendue);
    // Mise à jour du compte de la pile FIFO dans le cas où il y a plus d'un paquet disponible
    // ce qui permet d'en lire immédiatement plus sans attendre une interruption.
    NB_octets -= Taille_attendue;
    // Détermination des angles d'Euler en degrés :
    Module_Inertiel.dmpGetQuaternion(&Q, PILE_FIFO);
    Module_Inertiel.dmpGetGravity(&Vecteur_gravitation, &Q);
    Module_Inertiel.dmpGetYawPitchRoll(LTR, &Q, &Vecteur_gravitation);
    Tangage = (-LTR[1] * 180/M_PI); 
    Roulis = (LTR[2] * 180/M_PI);  
    Lacet = (LTR[0] * 180/M_PI);  
  } 
}

void Affiche_le_vecteur_gravitation() {// Mesure puis Envoie des données sur la liaison série.
  Mesure_les_valeurs_de_gravitation();
  Serial.println();
  Aff_TEXTE_EEPROM(875,7); 
  Aff_TEXTE_EEPROM(838,3);
  AFFICHE(Tangage); // Tangage.
  Affiche_virgule_egal_espace(); 
  Aff_TEXTE_EEPROM(940,6); 
  Aff_TEXTE_EEPROM(838,3); // Roulis
  AFFICHE(Roulis); 
  Affiche_virgule_egal_espace(); 
  Aff_TEXTE_EEPROM(685,5); 
  Aff_TEXTE_EEPROM(838,3);
  AFFICHE(Lacet); // Lacet.
  if (Initialiser_le_gyroscope) {
    Calage_gyro_initial = Lacet; 
    Initialiser_le_gyroscope = false;
  }
  Espace(); 
  Serial.print("Ref Gyro = "); 
  Serial.print(Calage_gyro_initial,0);
  Espace(); 
  Serial.print("Ecart = "); 
  Serial.print(Calage_gyro_initial - Lacet,0);
}
  
void AFFICHE(float VALEUR) {
  if (VALEUR > 0) {
    Aff_TEXTE_EEPROM(882,1);
  } else {
    Aff_TEXTE_EEPROM(946,1);// Affiche le signe.
  } 
  Serial.print(abs(VALEUR),0);
}
  
void Affiche_virgule_egal_espace() {
  Serial.print(", ");
}
  
//--------------------------------- Capteur magnétique HMC6352 ---------------------------------
void Indique_orientation_sur_USB() {
  Mesure_orientation_magnetique();
  Serial.println(); 
  Aff_TEXTE_EEPROM(1004,14);  
  Serial.print(Numerisation); 
  Serial.println(char (186));
}

void Mesure_orientation_magnetique() {// Saisie de la valeur du cap magnétique. 
  Wire.beginTransmission(Adresse_du_HMC6352);
  Wire.write(Lecture_Adresse); // Commande de lecture.
  Wire.endTransmission();
  delay(6); // le HMC6352 a besoin d'un délai de 6ms quand il reçoit une commande.
  Wire.requestFrom(Adresse_du_HMC6352, 2);
  // Saisie des deux octets de donnée "MSB" et "LSB".
  MSB = Wire.read(); 
  LSB = Wire.read();
  // ========== Traite la donnée et calcule l'orientation. ==========
  // La donnée issue de deux octets est concaténée, "MSB" étant décalé huit fois à gauche.
  CAP_construit = (MSB << 8) + LSB;
  // =========== Correction automatique du compas de route ==========
  // La donnée est fournie en degrés et dixièmes de degrés d'où la division par 10.
  Numerisation = int(CAP_construit / 10); // Mesure en degrés.
  if (Numerisation >= 0 && Numerisation < 86) {
    LSB = 1;     // De 0° à 85°.
  }
  if (Numerisation > 85 && Numerisation < 144) {
    LSB = 2;    // De 86° à 143°.
  }
  if (Numerisation > 143 && Numerisation < 308) {
    LSB = 3;   // De 144° à 307°.
  }
  if (Numerisation > 307 && Numerisation <= 359) {
    LSB = 4;  // De 308° à 359°.
  }
  switch (LSB) {
    case 1 : 
      CAN = map(Numerisation,0,85,+30,-40); 
      Numerisation = Numerisation + CAN; 
      break;
    case 2 : 
      Numerisation = Numerisation - 40; 
      break;
    case 3 : 
      CAN = map(Numerisation,144,307,-40,+28); 
      Numerisation = Numerisation + CAN; 
      break;
    case 4 : 
      Numerisation = Numerisation + 29;
  }
  if (Numerisation > 359) {
    Numerisation = Numerisation -360; 
  }
}

//----------------------------------------------------------------------------------------------   

//--------------------------- Télémétrie à capteur Ultrasons HC-SR04 ---------------------------
void Telemetre_ultrasons() {
  // Déclencher une mesure :
  digitalWrite(TX_Pulse_sonard, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TX_Pulse_sonard, LOW);
  // Analyser le retour sur la ligne de réception :
  Lecture_Echo = pulseIn(Echo, HIGH); 
  Distance = Lecture_Echo / 58;
  if (!Spectre) {
    Affiche_Distance();
  }
}
      
void Affiche_Distance() {
  Serial.println(); 
  Aff_TEXTE_EEPROM(709,11); 
  Serial.print(Distance); 
  Aff_TXT_EEPROM_et_CRLF(664,3);
}
  
//-------------------------- Enregistrement d'un balayage Télémétrique -------------------------
void Balayage_telemetrique() {
  Aller_au_Debut(); 
  Bouge_pour_Enregistrer (); 
  Revenir_au_centre();
  Configurer_stable_transversal(); 
  Caractere = 'e';
}
 
void Bouge_pour_Enregistrer () {
  PTR_Apprentissage = 553;
  for (CAN = CAN_Inf; CAN < CAN_Sup; CAN = CAN + ((CAN_Sup - CAN_Inf) / 64)) {
    Bouge_pour_un_balayage_en_torsion(); 
    Telemetre_ultrasons(); 
    delay(30);
    if (PTR_Apprentissage < 617) {
      Ecrire_un_OCTET_en_EEPROM(PTR_Apprentissage, Distance);
    }
  }
}
      
void Aller_au_Debut() {
  int Centrage = (CAN_Inf + CAN_Sup) / 2;
  for (CAN = Centrage; CAN > 4; CAN = CAN - ((Centrage - 4) / 128)) {
    Bouge_pour_un_balayage_en_torsion();
  }
}

void Revenir_au_centre() {
  int Centrage = (CAN_Inf + CAN_Sup) / 2;
  for (CAN = CAN_Sup; CAN > Centrage; CAN = CAN - ((Centrage - 4) / 128)) {
    Bouge_pour_un_balayage_en_torsion();
  }
}
      
void Visualiser_un_Spectre_telemetrie_sur_USB() {
  Serial.println(); 
  Encadrer_les_informations(); 
  PTR_Apprentissage = 553;
  while (PTR_Apprentissage < 616) {
    Distance = Lire_un_OCTET_en_EEPROM(PTR_Apprentissage); 
    PTR_Apprentissage++;
    Distance = Distance >> 2; 
    Serial.println();
    for (Distance; Distance > 0; Distance--) {
      Serial.print('*');
    }
  }
  Serial.println(); 
  Encadrer_les_informations();
}

//------------------------------- Routines pour le balayage LASER ------------------------------
void Traite_V_H(boolean V, boolean H) {
  if (Posture_LASER) {
    if (LASER_actif) {
      Vertical = V; 
      Horizontal = H;
    } else {
      Erreur_N_BIP(13);
    }
  } else {
    Erreur_N_BIP(14);
  }
}
     
void Annule_Direction_Laser() {
  Vertical = false; 
  Horizontal = false;
}

void Configuration_stable_LASER() {
  Posture_LASER = true;
  // Ci-dessous position stable sur trois pattes.
  for (byte Nb = 1; Nb < 6; Nb++) {
    Avance_Recule_Jambe(true, 'D'); 
    delay(100); 
  }
  Coordonne(9); // Patte A en configuration de pointage.
} 
  
void Retour_de_configuration_LASER() {
  Posture_LASER = false;
  configurer(10); 
  delay(300); // Configuration LASER jambe A ramenée au stable transversal.
  for (byte Nb = 1; Nb < 6; Nb++) {
    Avance_Recule_Jambe(false, 'D'); 
    delay(100); 
  }
}

//--------------------------------- Piloter un moteur en manuel --------------------------------
void Piloter_un_moteur_en_manuel() {// Dans ce mode les moteurs sont limités "au maximum" des 12.
  if (Chaine_Memorisee[0] != '0') {
    Potentiometre = false; 
    Mode_manuel = true; 
    Serial.println();
    switch (Num_Sortie) {
      case   0 : Afficher_Hanche(); Serial.println('A'); break;
      case   1 : Afficher_Genou(); Serial.println('A'); break;
      case   2 : Afficher_Pied(); Serial.println('A'); break;
      case   3 : Afficher_Hanche(); Serial.println('B'); break;
      case   4 : Afficher_Genou(); Serial.println('B'); break;
      case   5 : Afficher_Pied(); Serial.println('B'); break;
      case   6 : Afficher_Hanche(); Serial.println('C'); break;
      case   7 : Afficher_Genou(); Serial.println('C'); break;
      case   8 : Afficher_Pied(); Serial.println('C'); break;
      case   9 : Afficher_Hanche(); Serial.println('D'); break;
      case  10 : Afficher_Genou(); Serial.println('D'); break;
      case  11 : Afficher_Pied(); Serial.println('D');
    }
  } else {
    Erreur_N_BIP(19);
  }
}
  
void Afficher_Hanche() {
  Aff_TEXTE_EEPROM(617,7);
}
void Afficher_Genou() {
  Aff_TEXTE_EEPROM(624,6);
}
void Afficher_Pied() {
  Aff_TEXTE_EEPROM(630,5);
}

//------------------------------- Procédures pour l'apprentissage ------------------------------
void Debuter_un_apprentissage() {
  if (!Appris) {
    Apprentissage = true; 
    Efface_PGM_appris(); // Mémoire EEPROM effacée.
    Serial.println(); 
    Aff_TXT_EEPROM_et_CRLF(690,13);
  } else {
    Erreur_N_BIP(9);
  }
}

void Efface_PGM_en_EEPROM() {
  if ((Phares_actifs) && (ACR_configuration_sur_USB)) 
    Efface_PGM_appris(); 
  else 
    Erreur_N_BIP(10);
} 
  
void Efface_PGM_appris() {// Le programme de 120 octets est logé entre [432 et 551] compris.
  PTR_Apprentissage = 432; 
  while (PTR_Apprentissage < 553) {
    Ecrire_un_OCTET_en_EEPROM(PTR_Apprentissage, 0);
  }
  // Le "0" en 552 représente le booléen Appris remis à faux.
  PTR_Apprentissage = 432; // Replace le pointeur au début de la zone mémoire EEPROM réservée à l'apprentissage.
  NB_enregistrements = 0; 
  Appris = false;
}
   
void  Liste_PGM_appris_sur_USB() {
  if (Appris) { 
    Serial.println();
    for (PTR_Apprentissage = 432; PTR_Apprentissage < 552; PTR_Apprentissage++) {
      if (Lire_un_OCTET_en_EEPROM(PTR_Apprentissage) != 0) {
        Caractere = Lire_un_OCTET_en_EEPROM(PTR_Apprentissage); 
        Serial.print(Caractere);
        if (Caractere == '*') {
          Serial.println();
        }
      }
    }
  } else {
    Erreur_N_BIP(8);
  }
}
  
void Utilise_PGM_en_memoire() {
  PTR_Apprentissage = 432;
  if (Appris) {
  Serial.println(); 
  Serial.println(); 
  Aff_TEXTE_EEPROM(703,6); 
  Aff_TEXTE_EEPROM(923,4);
  while (PTR_Apprentissage < 552) {
    // Saisir une chaîne de quatre caractères dans la mémoire dynamique :
    Chaine_Memorisee = "";
    for (byte i=0; i<4; i++) {
      Chaine_Memorisee = Chaine_Memorisee + char(Lire_un_OCTET_en_EEPROM(PTR_Apprentissage));
      PTR_Apprentissage++;}
      // Affiche la chaîne sur la ligne USB :
      if (Chaine_Memorisee[0] != 0) {
        Serial.println(); 
        Aff_TEXTE_EEPROM(681,4);
        for (byte i=0; i<4; i++) {
          Caractere = Chaine_Memorisee[i]; 
          Serial.print(Caractere);
        }
        // réalise le programme enregistré :    
        Analyseur_syntaxique(); 
        Compteur_caracteres == 4; 
        Traite_Programme();
      }
    } 
    Serial.println(); 
    Aff_TXT_EEPROM_et_CRLF(883,3); 
    Serial.println();
  } else {
    Erreur_N_BIP(8);
  }
}

//-------------------------- Routines pour la correction d'orientation -------------------------
void  Active_la_correction_en_Torsion() {  
  Torsion_Active = true; 
  Annule_Direction_Laser();
}

void Stoppe_la_Torsion_et_recentre() {
  Torsion_Active = false; 
  Configurer_stable_transversal();
}
  
void Bouge_pour_un_balayage_en_torsion() {
  Mouvoir(0, (map(CAN,CAN_Inf,CAN_Sup,325,247)) - 47);  // Hanche de la Jambe A.
  Mouvoir(3, (map(CAN,CAN_Inf,CAN_Sup,345,274)) + 35);  // Hanche de la Jambe B.
  Mouvoir(6, (map(CAN,CAN_Inf,CAN_Sup,304,228)) - 44);  // Hanche de la Jambe C.
  Mouvoir(9, (map(CAN,CAN_Inf,CAN_Sup,344,264)) + 42);  // Hanche de la Jambe D. 
}

//============================== Routines pour les programmes "pNN*" =================================
void Aff_TXT_EEPROM_et_CRLF(int PTR,byte LGR) {
  /*
  Aff_TEXTE_EEPROM(PTR,LGR); 
  Serial.println();
  */
}

void Configure_pour_Decollage() {
  if (Contact_avec_le_sol()) {
    BIP(); 
    Serial.println(); 
    Aff_TXT_EEPROM_et_CRLF(645,19);
    Attendre_une_chaine(); 
    if (Chaine_Memorisee == "o*") {
      Compteur_caracteres = 4; 
      Coordonne(3); 
      configurer(3);
    } else {
      BIP();
    }
  } else {
    Erreur_N_BIP(12);
  }
}
  
//-------------------------- Gère l'enregistrement de posture en EEPROM ------------------------
void Affiche_enregistrement_de_posture_sur_EEPROM() {
  // Sauvegarde la Posture Actuelle dans Configuration_Voulue et charge celle en EEPROM :
  PTR_EEPROM = 408;
  for (byte I = 0; I < 12; I++) {
    Configuration_voulue[I] = Configuration_actuelle[I];
    Configuration_actuelle[I] = Lire_un_Entier_en_EEPROM(PTR_EEPROM); PTR_EEPROM++; PTR_EEPROM++;
  }
  Affiche_la_config_actuelle(); 
  Serial.println();
  // Restituee la Posture actuelle.  
  for (byte I = 0; I < 12; I++) {
    Configuration_actuelle[I] = Configuration_voulue[I];
  }
}
  
void  Enregistrement_de_la_posture_Actuelle_en_EEPROM() {
  if ((Phares_actifs) && (ACR_configuration_sur_USB)) {
    PTR_Apprentissage = 408;
    for (byte I = 0; I < 12; I++) {
      Ecrire_un_Entier_en_EEPROM(PTR_Apprentissage, Configuration_actuelle[I]);
    }
  } else {
    Erreur_N_BIP(20);
  }    
}
       
void Utilise_la_posture_sauvegardee_en_EEPROM() {
  /*
  Coordonne(408); 
  configurer(408);
  */
}
  
void Depose_la_sonde() {
  if (Contact_avec_le_sol()) {
    configurer(2); // Tous les moteurs au neutre opérationnel.
    delay(1000); 
    Passer_en_VEILLE(); 
    delay(1000); 
    REVEILLER();
  } else {
    Erreur_N_BIP(12);
  }
}

void Retour_de_hauteur_maximale() {
  Coordonne(12); 
  REVEILLER();
}
  
void Configuration_Stable_Raisonnable() {
  Coordonne(0); 
  configurer(0); 
  Libere_efforts();
}
  
void Config_Stable_Raisonnable_vers_VEILLE() {
  Coordonne(5); 
  Coordonne(8); 
}
   
void Hauteur_Maximale() {
  Coordonne(12); 
  delay(300); 
  Coordonne(6);
}

void Reculer_un_pas() {
  Recule_paire(false); 
  Recule_paire(true); 
  Coordonne(7); 
  Recule_paire(false); 
  Recule_paire(true); 
  Configurer_stable_transversal();
}

void Avancer_un_pas() { // Anticollision par télémètre à ultrasons.
  Spectre = true;
  Telemetre_ultrasons();
  if (Distance > 8) {
    Avance_paire(false); 
    Avance_paire(true); 
    Coordonne(11);
    Avance_paire(false); 
    Avance_paire(true); 
    Configurer_stable_transversal();
  } else {
    Erreur_N_BIP(15); 
    Affiche_Distance();
  } 
}
  
void Decaler_lateralement(int Posture) {
  Passer_en_VEILLE(); 
  configurer(4); 
  delay(500); 
  Coordonne(Posture);
  REVEILLER(); // En réalité relève la sonde et la stabilise.
} 
  
void Tourne_a_droite() {
  Avance_paire(true); 
  Recule_paire(false); 
  Coordonne(13); // Fait tourner.
  Recule_paire(false); 
  Avance_paire(true); 
  Configurer_stable_transversal(); 
}
   
void Tourne_a_gauche() {
  Avance_paire(false); 
  Recule_paire(true); 
  Coordonne(14); // Fait tourner.
  Avance_paire(false); 
  Recule_paire(true); 
  Configurer_stable_transversal(); 
}

void Lever_Baisser_delai_100(boolean Leve, char Membre) {
  Lever_ou_Baisser_Jambe(Leve, Membre); 
  delay(100);
}

void Coordonne(int Configuration_desiree) {
  // Passe de configuration actuelle en configuration désirée en 8 étapes.
  PTR_EEPROM = Configuration_desiree; Charge_configuration_voulue();
  // Passe de configuration actuelle en configuration désirée en 8 étapes.
  for (byte I = 0; I < 12; I++) {
    Increment[I] = (Configuration_voulue[I] - Configuration_actuelle[I]) >> 3;
  }
  for (byte I = 1; I < 9; I++) {
    for (byte sortie = 0; sortie < 12; sortie++) {
      Bouge_un_membre(sortie, Configuration_actuelle[sortie] + Increment[sortie]);
    }
    if (Rapide) {
      delay(50); 
    } else {
      delay(200);
    }
  } 
}

void Libere_efforts() {
  Lever_Baisser_delai_100(true,'A');
  Lever_Baisser_delai_100(false,'A'); 
  Lever_Baisser_delai_100(true,'C'); 
  Lever_Baisser_delai_100(false,'C'); 
  Lever_Baisser_delai_100(true,'B'); 
  Lever_Baisser_delai_100(false,'B'); 
  Lever_Baisser_delai_100(true,'D'); 
  Lever_Baisser_delai_100(false,'D');
}
  
void Lever_ou_Baisser_Jambe(boolean Lever, char Jambe) {
  Caractere = Jambe; 
  Transpose_char_vers_Sortie();
  if ((Jambe == 'B') || (Jambe == 'D')) {
    Angle = -28;
  } else {
    Angle = 28; 
  }
  if (Lever) {
    Bouge_un_membre(Num_Sortie + 1, Configuration_actuelle[Num_Sortie + 1] + Angle);
  } else {
    Bouge_un_membre(Num_Sortie + 1, Configuration_actuelle[Num_Sortie + 1] - Angle);
  }
  if ((Jambe == 'B') || (Jambe == 'D')) {
    Angle = -47;
  } else {
    Angle = 47; 
  }
  if (Lever) {
    Bouge_un_membre(Num_Sortie + 2, Configuration_actuelle[Num_Sortie + 2] + Angle);
  } else {
    Bouge_un_membre(Num_Sortie + 2, Configuration_actuelle[Num_Sortie + 2] - Angle);
  }
}
  
void Transpose_char_vers_Sortie() {
  switch (Caractere) {
    case'A' : 
      Num_Sortie = 0; 
      break;
    case'B' :  
      Num_Sortie = 3; 
      break;
    case'C' : 
      Num_Sortie = 6; 
      break;
    case'D' : 
      Num_Sortie = 9;
      break;
    } 
}

void Recule_paire(boolean Babord) {
  if (Babord) {
    Avance_Recule_Jambe (false , 'C'); 
    delay(200);
    Avance_Recule_Jambe (false , 'D'); 
    delay(200);
  } else {
    Avance_Recule_Jambe (false , 'A'); 
    delay(200);
    Avance_Recule_Jambe (false , 'B'); 
    delay(200);
  }
} 

void Avance_paire(boolean Babord) {
  if (Babord) {
    Avance_Recule_Jambe (true , 'C'); 
    delay(200);
    Avance_Recule_Jambe (true , 'D'); 
    delay(200);
  } else {
    Avance_Recule_Jambe (true , 'A'); 
    delay(200);
    Avance_Recule_Jambe (true , 'B'); 
    delay(200);
  } 
}
        
void Avance_Recule_Jambe (boolean Avancer, char Jambe) {
  int Pas;
  if (Avancer) {
    Pas = 35; 
    if ((Jambe == 'C') || (Jambe == 'D')) {
      Pas = -35;
    }
  } else {
    Pas = -35; 
    if ((Jambe == 'C') || (Jambe == 'D')) {
      Pas = +35;
    }
  }
  Lever_ou_Baisser_Jambe(true, Jambe); 
  delay(50);
  Bouge_un_membre(Num_Sortie, Configuration_actuelle[Num_Sortie] + Pas); 
  delay(50);
  Lever_ou_Baisser_Jambe(false, Jambe);
}
   
//================================ Diverses procédures de servitude ==================================
void Mouvoir(byte Sortie, int Consigne) {
  if (Moteurs_ON) {
    pwm.setPWM(Sortie,0, Consigne); 
    Configuration_actuelle[Sortie] = Consigne;
  }
}
  
void Charge_configuration_voulue() {
  for (byte I=0; I < 12; I++) {
    Configuration_voulue[I] = Lire_un_Entier_en_EEPROM(PTR_EEPROM); PTR_EEPROM++; PTR_EEPROM++;
  }
}
  
void Encadrer_les_informations() {
  for (byte I=0; I < 30; I++) {
    Aff_TEXTE_EEPROM(946,1); Serial.println();
  }
}
 
float Tension_Alim_des_moteurs() {
  return analogRead(Entree_Alim_moteurs) * Calibre_U_moteurs / 1023;
} 
  
void Etat_actuel_de_la_sonde() {
  Serial.println();
  Encadrer_les_informations();
  Aff_TEXTE_EEPROM(745,18); 
  OUI_NON(Contact_avec_le_sol());
  Aff_TEXTE_EEPROM(779,14); 
  if (Vertical) {
    Aff_TEXTE_EEPROM(822,1); 
  }
  if (Horizontal) {
    Aff_TEXTE_EEPROM(617,1);
  }
  Espace(); 
  OUI_NON(LASER_actif);
  Aff_TEXTE_EEPROM(763,16); 
  OUI_NON(Phares_actifs);
  Aff_TEXTE_EEPROM(804,18); 
  OUI_NON(!Potentiometre);
  Aff_TEXTE_EEPROM(793,15); 
  Aff_TEXTE_EEPROM(966,3); 
  Serial.print(CAN); 
  Aff_TEXTE_EEPROM(669,2);
  Serial.print(analogRead(Entree_mesuree)); 
  Serial.println(")");
  Aff_TEXTE_EEPROM(841,22); 
  OUI_NON(Bloquer_niveaux); 
  Aff_TEXTE_EEPROM(822,19); 
  Serial.println(Numerisation);
  Aff_TEXTE_EEPROM(863,12); 
  Serial.println(Niveau_eclairage_Phares);
  Aff_TEXTE_EEPROM(779,6); 
  Aff_TEXTE_EEPROM(873,2);
  Aff_TEXTE_EEPROM(873,2); 
  Serial.print(Niveau_eclairage_LASER); 
  Serial.println(); 
  Aff_TEXTE_EEPROM(888,10); 
  OUI_NON(Sommeil);
  Aff_TEXTE_EEPROM(898,18);
  Serial.print(Tension_Alim_des_moteurs()); 
  Aff_TXT_EEPROM_et_CRLF(822,1);
  Aff_TEXTE_EEPROM(994,10); 
  OUI_NON(Torsion_Active);
  Aff_TEXTE_EEPROM(906,8); 
  Aff_TEXTE_EEPROM(675,6); 
  OUI_NON(!Moteurs_ON);
  Aff_TEXTE_EEPROM(635,9); 
  OUI_NON(Mode_manuel);
  if (Stabilisation_gyroscopique) {
    Serial.print("STB"); 
  }
  Espace(); 
  Aff_TEXTE_EEPROM(936,4); 
  Espace(); 
  if (Rapide) {
    Aff_TXT_EEPROM_et_CRLF(947,7); 
  } else {
    Aff_TXT_EEPROM_et_CRLF(954,5);
  }
  
  Aff_TEXTE_EEPROM(916,20); 
  Serial.println(Dernier_PGM_actif);
  Encadrer_les_informations();
}
  
void REVEILLER() {
  Coordonne(5); 
  Configurer_stable_transversal(); 
  Libere_efforts();
}

void Configurer_stable_transversal() {
  configurer(5);
}
  
void Passer_en_VEILLE() {
  Coordonne(8);
}
  
void Analyseur_syntaxique() {
  SAV_Consigne = Chaine_Memorisee;
  num_ERR = 0;
  Compteur_caracteres = Chaine_Memorisee.length();
  if (Compteur_caracteres == 3) {
    num_ERR = 3; 
  }
  if (Compteur_caracteres > 4) {
    num_ERR = 4; 
  }
  if (Chaine_Memorisee[Compteur_caracteres - 1]  != '*') {
    num_ERR = 2;
  }
  if ((Compteur_caracteres != 1) && (Compteur_caracteres != 2)) {
    Caractere = Chaine_Memorisee[0];  
    //========================== Analyse pour PROGRAMME ==========================
    // En sortie NOMBRE contiendra le nombre dans la chaine ou 00 si incorrect.
    if (Caractere != 'p') {
      num_ERR = 1;
    }
    Decale_la_chaine(); 
    Extraire_un_nombre();
    if ((NOMBRE == 0) || (NOMBRE > PGMmax)) { 
      num_ERR = 7;
    }
    if((Apprentissage) && (num_ERR == 0)) {
      if (PTR_Apprentissage < 549) {
        Enregistrer_une_consigne(); 
      } else {
        num_ERR = 11;
      }
    }
  }
}
       
void  Enregistrer_une_consigne() {// PTR pointe en permanence le premier emplacement libre.
  byte I = 0; // Index de lecture dans le message de réception.
  Appris = true;
  // Ci-dessous : Mise à jour du booléen en EEPROM sans perturber PTR_Apprentissage.
  Angle = PTR_Apprentissage; Ecrire_un_OCTET_en_EEPROM(552, true); 
  PTR_Apprentissage = Angle;
  Caractere = 'X'; // Pour ne pas sortir immédiatement de la boucle ci-dessous;
  while (Caractere != '*') {// Lire la chaîne de caractères dans la consigne reçue.
    Caractere = SAV_Consigne[I]; I++;
    Ecrire_un_OCTET_en_EEPROM(PTR_Apprentissage, Caractere);
  } // Incrémente PTR_Apprentissage.
  NB_enregistrements++;
  Serial.println(); 
  Serial.print(NB_enregistrements);
  Aff_TEXTE_EEPROM(671,4);
}
        
void Affiche_ACR() {
  Aff_TEXTE_EEPROM(887,1); 
  Aff_TEXTE_EEPROM(924,4); // Affiche "!PGM "
  if (Compteur_caracteres == 2) {
    Serial.print(Caractere);
  } else {
    Serial.print(Dernier_PGM_actif);
  }
  Aff_TEXTE_EEPROM(731, 4);
}
  
void Affiche_la_config_actuelle() {   
  Serial.println(); 
  Aff_TEXTE_EEPROM(959,10);
  for (byte I = 0; I < 12; I++) {
    Serial.print(Configuration_actuelle[I]); 
    Espace();
  }
}
        
void Extraire_un_nombre() {// Extraction jusqu'à un caractère non chiffre.
  // L'extraction commence au début de Chaine_Memorisee.
  NOMBRE = Chaine_Memorisee.toInt();
}
  
void Decale_la_chaine() {
  for (byte I = 0; I < LGR_chaine; I++) {
    Chaine_Memorisee[I] = Chaine_Memorisee[I+1];
  }
}
  
void Espace() {Aff_TEXTE_EEPROM(674,1);}

void OUI_NON(boolean OUI) {
  if (OUI) {
    Aff_TXT_EEPROM_et_CRLF(1018, 3);
  } else {
    Aff_TXT_EEPROM_et_CRLF(1021,3);
  }
}
  
void configurer(int position) {
  for (int arm = 0; arm < 4; arm++) {
    Bouge_un_membre(servos[arm * 3].pin, positions[position][arm].A);
    Bouge_un_membre(servos[arm * 3 + 1].pin, positions[position][arm].B);
    Bouge_un_membre(servos[arm * 3 + 2].pin, positions[position][arm].C);
  }
}
       
byte Lire_un_OCTET_en_EEPROM(int ADRESSE) {
  return eeprom_read_byte((unsigned char *) ADRESSE);
}

void Ecrire_un_OCTET_en_EEPROM(int ADRESSE, byte OCTET) { // Pointe l'octet libre suivant.
  // ATTENTION : PTR_Apprentissage est réservé à cette procédure.
  eeprom_write_byte((unsigned char *) ADRESSE, OCTET); 
  PTR_Apprentissage++;
}
   
int Lire_un_Entier_en_EEPROM(int ADRESSE) {
  return eeprom_read_word((unsigned int *) ADRESSE);
}

void Ecrire_un_Entier_en_EEPROM(int ADRESSE, int MOT16Bits) { // Pointe l'octet libre suivant.
  eeprom_write_word((unsigned int *) ADRESSE, MOT16Bits); PTR_Apprentissage++; PTR_Apprentissage++;
}

void Aff_TEXTE_EEPROM(int PTR, byte Longueur) {
/*  
  for (byte I=0; I<Longueur; I++) {
    Serial.print(char(Lire_un_OCTET_en_EEPROM(PTR))); 
    PTR++;
  }
*/
}
       
void Attendre_une_chaine() {// Cette procédure attend une chaine sur la ligne USB.
  Chaine_Memorisee = ""; // Vider le tampon pour une nouvelle capture.
  Sentinelle_detectee = false;  // Sentinelle non reçue.
  Compteur_caracteres=0;
  while (!Sentinelle_detectee) {
    if (Serial.available()) {
      Caractere = Serial.read(); // Puiser un caractère dans le FIFO.
      // Puis concaténer ce caractère y compris la sentinelle :
      if (Compteur_caracteres < LGR_chaine) {
        Chaine_Memorisee = Chaine_Memorisee + Caractere; 
        Compteur_caracteres++;
      }
      // Si sentinelle détectée, informer la boucle de réception :
      Sentinelle_detectee = (Caractere == '*'); 
    }
  } 
}
        
void Bouge_un_membre(byte Num_Sortie, int Consigne) {
  Mouvoir(Num_Sortie, Consigne); 
  Configuration_actuelle[Num_Sortie] = Consigne;
}
 
boolean Contact_avec_le_sol() {
  return digitalRead(Bouclier_au_sol);
} 
  
void BIP() {
  pinMode(BUZZER, OUTPUT); 
  tone(BUZZER,3000); 
  delay(100); 
  noTone(BUZZER); 
  pinMode(BUZZER, INPUT);
}
 
void ERREUR() {
  BIP(); 
  Aff_TEXTE_EEPROM(734,5); 
  Serial.print(num_ERR); 
  Aff_TXT_EEPROM_et_CRLF(734,1);
}
  
void Erreur_N_BIP(byte Type_erreur) {
  num_ERR = Type_erreur; 
  ERREUR();
}
