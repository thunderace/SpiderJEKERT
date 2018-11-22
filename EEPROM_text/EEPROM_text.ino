/* Écriture des textes en EEPROM pour diminuer la place occupée dans la zone
   des variables dynamiques.
   Plus une chaire est longue, plus elle est "rentable". */

#include <EEPROM.h>

int PTR;
char Caractere;
String TEXTE_a_inscrire_en_EEPROM = ""; // Emplacement pour la chaîne entrante.

void setup() { Serial.begin(115200); 
  TEXTE_a_inscrire_en_EEPROM.reserve(30); } // 30 caractères sont très suffisants.

void loop() {  
  PTR = 0; // PTR pointe la première adresse disponible pour les textes, la fin étant en 1023.    

  // Ci-dessous : Possibilité de "purger une zone avec des #FF.
  // for (int i=300; i < 1024; i++) EEPROM.write(i, 255);
  
  TEXTE_a_inscrire_en_EEPROM = "SONDE dans MOUVEMENTS POSTURESConfiguration"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Changer PGM EEPROM ?Tir"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = " OPTIONS DONNEESEXPLOITERepeter"; Ecrire_Texte_en_EEPROM();   
  TEXTE_a_inscrire_en_EEPROM = "  - DEPLACEMENTS -Couper les   ?Moteurs ON ? "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Mouvements RAPIDES ?Stab. Gyroscope ?CLV"; Ecrire_Texte_en_EEPROM(); 
  TEXTE_a_inscrire_en_EEPROM = "Mode APPRENTISSAGE ?"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Version sur OFF : MOTEURS actifs : Mvt. RAPIDE : "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Sauver la POSTURE ?QUITTERBouclier au sol "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "APPRENTISSAGE : Pointer E11E12E15TBMROJV"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "SAV spectre couleur ?AFFOUINON"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "PHARES    (Num "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Utiliser leTestReveiller Gradateur Phares."; Ecrire_Texte_en_EEPROM();   
  TEXTE_a_inscrire_en_EEPROM = "LASER.Phares & LASER = 128VEILLEStable Raisonnable"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Hauteur MaximaleMoteurs au Neutre OPApprise en EEPROM"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "AtterrissageDecollageDeposer JEKERTStable Transversal"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "ATTENTION : Retour par FIN (BERCEAU !)Rot fois"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Dephasage porteuse.TELEMESUREShiberne : un balayage ?"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "TORSION active ?Pilotage MANUEL ?Moteur : JAMBE : "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "HancheGenouGriffePosture actuelle ?Energie 1 a 254."; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Armer la SECURITE ?Effacer le PGM Lister"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Suppr. dernier code ?RECHARGEditerSOURCE :TFR. "; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "CAP actuel =Tang.     Roulis E162mE21xxxxxx"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Ecart route =AFF. Nav. continu(Afficher)E19E221msge"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = "Activer/Desactiver ?Nb de chainagesT =M = E =magnetique"; Ecrire_Texte_en_EEPROM();
  TEXTE_a_inscrire_en_EEPROM = " UHF ?////OK"; Ecrire_Texte_en_EEPROM();
Lister_EEPROM();
  INFINI: goto INFINI; }
  
void Ecrire_Texte_en_EEPROM(){
  byte Curseur = 0; // Pointer le premier caractère du TEXTE.
  // Ne pas déborder la taille mémoire disponible en EEPROM.
  // N'envoyer que le TEXTE.
  while ((PTR < 1024) && (TEXTE_a_inscrire_en_EEPROM[Curseur] != '\0')) {
     EEPROM.write(PTR, TEXTE_a_inscrire_en_EEPROM[Curseur]);
     Serial.print(TEXTE_a_inscrire_en_EEPROM[Curseur]); Curseur++; PTR++; }
  Serial.println(); }
  
void Lister_EEPROM() {
  PTR = 0;
  Lister_un_BLOC();Lister_un_BLOC();
  Lister_un_BLOC();Lister_un_BLOC(); }
  
void Lister_un_BLOC() {
  Lister_ENTETE();
  byte Compteur_lignes = 0;
  while (Compteur_lignes < 16)
    {Lister_une_LIGNE(); Compteur_lignes++; } }
  
void Lister_une_LIGNE() {   
  byte Compteur_Octets = 0; Serial.print(' ');
  if (PTR<10) {Serial.print('0');};
  if (PTR<100) {Serial.print('0');};
  if (PTR<1000) {Serial.print('0');};
  Serial.print(PTR); Serial.print("  ");  
  while (Compteur_Octets < 16) {
    Lister_OCTET(); Compteur_Octets++; }
  Serial.println(); }
  
void Lister_ENTETE() {
   Serial.println();
   Serial.println(" ADRS  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");}
   
void Lister_OCTET() {
  byte octet = EEPROM.read(PTR); 
  if ((octet > 31) && (octet < 126)) {Serial.print(' '); Serial.print(char(octet));}
  else Serial.print("..");
  Serial.print(' '); PTR++; }
