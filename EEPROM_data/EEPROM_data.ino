/* Écriture des diverses données numériques et en particulier les tableaux de consignes. */

#include <avr/eeprom.h> // Bibliothèque intégrée à l'IDE qui gère les entiers ,les réels ...

byte Compteur; // Compteur pour séparer les tableaux dans l'affichage sur le Moniteur de l'IDE.
int PTR; // Pointeur des octets situés en EEPROM. (Pointe le premier emplacement libre.)
int MAX_Octets_dans_tableaux = 408; // 24 x Nb Tableaux. (Ici 17 configurations.)

void setup() { Serial.begin(115200);}

void loop() {
  // Ci-dessous : Possibilité de "purger une zone avec des #FF.
  // for (int i=408; i < 665; i++) Ecrire_un_Entier_en_EEPROM(i,255);
 
  PTR = 0; // Les données sont rangées au début de l'EEPROM. 

  // 01 ------------- Tableau de configuration STABLE Raisonnable. (000) -------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 305); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 274); Ecrire_un_Entier_en_EEPROM(PTR, 406); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 287); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 285); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 02 ------------- Retour de configuration STABLE Transversale. (024) -------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 305); Ecrire_un_Entier_en_EEPROM(PTR, 315); Ecrire_un_Entier_en_EEPROM(PTR, 421); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 274); Ecrire_un_Entier_en_EEPROM(PTR, 296); Ecrire_un_Entier_en_EEPROM(PTR, 192); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 287); Ecrire_un_Entier_en_EEPROM(PTR, 298); Ecrire_un_Entier_en_EEPROM(PTR, 426); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 285); Ecrire_un_Entier_en_EEPROM(PTR, 291); Ecrire_un_Entier_en_EEPROM(PTR, 199); // Jambe D.
  Afficher_PTR(); 
  // 03 ------------------ Tableau des Neutres opérationnels. (048) -------------------
  Ecrire_un_Entier_en_EEPROM(PTR, 320); Ecrire_un_Entier_en_EEPROM(PTR, 308); Ecrire_un_Entier_en_EEPROM(PTR, 316); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 289); Ecrire_un_Entier_en_EEPROM(PTR, 303); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 262); Ecrire_un_Entier_en_EEPROM(PTR, 292); Ecrire_un_Entier_en_EEPROM(PTR, 310); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 300); Ecrire_un_Entier_en_EEPROM(PTR, 285); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe D.
  Afficher_PTR(); 
  // 04 ------------------ Tableau de Configuration Décollage. (072) ------------------ 
  Ecrire_un_Entier_en_EEPROM(PTR, 211); Ecrire_un_Entier_en_EEPROM(PTR, 152); Ecrire_un_Entier_en_EEPROM(PTR, 315); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 413); Ecrire_un_Entier_en_EEPROM(PTR, 486); Ecrire_un_Entier_en_EEPROM(PTR, 316); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 159); Ecrire_un_Entier_en_EEPROM(PTR, 139); Ecrire_un_Entier_en_EEPROM(PTR, 255); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 411); Ecrire_un_Entier_en_EEPROM(PTR, 463); Ecrire_un_Entier_en_EEPROM(PTR, 325); // Jambe D.
  Afficher_PTR(); 
  // 05 ---------------- Tableau de Configuration Atterrissage. (096) ----------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 303); Ecrire_un_Entier_en_EEPROM(PTR, 490); Ecrire_un_Entier_en_EEPROM(PTR, 154); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 303); Ecrire_un_Entier_en_EEPROM(PTR, 141); Ecrire_un_Entier_en_EEPROM(PTR, 481); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 303); Ecrire_un_Entier_en_EEPROM(PTR, 472); Ecrire_un_Entier_en_EEPROM(PTR, 145); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 303); Ecrire_un_Entier_en_EEPROM(PTR, 134); Ecrire_un_Entier_en_EEPROM(PTR, 501); // Jambe D.
  Afficher_PTR();
  // 06 ------------- Tableau de configuration STABLE Transversale. (120)-------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR(); 
  // 07 -------------- Tableau de configuration Hauteur Maximale. (144)--------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 152); Ecrire_un_Entier_en_EEPROM(PTR, 230); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 483); Ecrire_un_Entier_en_EEPROM(PTR, 391); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 135); Ecrire_un_Entier_en_EEPROM(PTR, 232); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 467); Ecrire_un_Entier_en_EEPROM(PTR, 398); // Jambe D.
  Afficher_PTR();
  // 08 ----------------- Configuration "Avancée" pour Reculer. (168) ----------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 274); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 379); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 187); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 311); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 09 ------------------------- Configuration de VEILLE. (192) ---------------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 315); Ecrire_un_Entier_en_EEPROM(PTR, 421); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 296); Ecrire_un_Entier_en_EEPROM(PTR, 192); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 298); Ecrire_un_Entier_en_EEPROM(PTR, 426); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 291); Ecrire_un_Entier_en_EEPROM(PTR, 199); // Jambe D.
  Afficher_PTR();
  // 10 ---------------------- Configuration pointage LASER. (216) -------------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 312); Ecrire_un_Entier_en_EEPROM(PTR, 314); Ecrire_un_Entier_en_EEPROM(PTR, 135); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 171); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 11 --------------------- Configuration Retour de LASER. (240) -------------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 171); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 12 ------------------ Configuration "Reculée" pour Avancer. (264) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 204); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 309); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 257); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 381); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 13 ------------------ Configuration avant Hauteur Maximale. (288) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 264); Ecrire_un_Entier_en_EEPROM(PTR, 449); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 347); Ecrire_un_Entier_en_EEPROM(PTR, 167); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 262); Ecrire_un_Entier_en_EEPROM(PTR, 464); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 312); Ecrire_un_Entier_en_EEPROM(PTR, 158); // Jambe D.
  Afficher_PTR();
  // 14 ------------------- Configuration pour Tourner à Droite. (312) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 269); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 374); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 252); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 376); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 15 ------------------- Configuration pour Tourner à Gauche. (336) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 209); Ecrire_un_Entier_en_EEPROM(PTR, 213); Ecrire_un_Entier_en_EEPROM(PTR, 307); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 314); Ecrire_un_Entier_en_EEPROM(PTR, 414); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 192); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 319); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 316); Ecrire_un_Entier_en_EEPROM(PTR, 388); Ecrire_un_Entier_en_EEPROM(PTR, 314); // Jambe D.
  Afficher_PTR();
  // 16 ------------------- Configuration pour Décaler à Droite. (360) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 270); Ecrire_un_Entier_en_EEPROM(PTR, 281); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 339); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 215); Ecrire_un_Entier_en_EEPROM(PTR, 439); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 381); Ecrire_un_Entier_en_EEPROM(PTR, 188); // Jambe D.
  Afficher_PTR();
  // 17 ------------------- Configuration pour Décaler à Gauche. (384) ---------------- 
  Ecrire_un_Entier_en_EEPROM(PTR, 239); Ecrire_un_Entier_en_EEPROM(PTR, 230); Ecrire_un_Entier_en_EEPROM(PTR, 430); // Jambe A.
  Ecrire_un_Entier_en_EEPROM(PTR, 344); Ecrire_un_Entier_en_EEPROM(PTR, 392); Ecrire_un_Entier_en_EEPROM(PTR, 190); // Jambe B.
  Ecrire_un_Entier_en_EEPROM(PTR, 222); Ecrire_un_Entier_en_EEPROM(PTR, 258); Ecrire_un_Entier_en_EEPROM(PTR, 287); // Jambe C.
  Ecrire_un_Entier_en_EEPROM(PTR, 346); Ecrire_un_Entier_en_EEPROM(PTR, 334); Ecrire_un_Entier_en_EEPROM(PTR, 346); // Jambe D.
  Afficher_PTR();
  //------------------------------------------------------------- (408) ---------------
  
  Lister_EEPROM(); Lister_les_entiers();
  Infini: goto Infini;}
  
void Ecrire_un_Entier_en_EEPROM(int ADRESSE, int MOT16Bits) { // Pointe l'octet libre suivant.
   eeprom_write_word((unsigned int *) ADRESSE, MOT16Bits); PTR++; PTR++;}
   
int Lire_un_Entier_en_EEPROM(int ADRESSE) {
  return eeprom_read_word((unsigned int *) ADRESSE);}

byte Lire_un_OCTET_en_EEPROM(int ADRESSE) {
  return eeprom_read_byte((unsigned char *) ADRESSE);} 
  
void Afficher_PTR() {Serial.println(); Serial.print("PTR = "); Serial.print(PTR);}

void Lister_les_entiers() {
  PTR = 0; Compteur = 0;
  while (PTR < MAX_Octets_dans_tableaux) {
         Serial.println(Lire_un_Entier_en_EEPROM(PTR)); PTR++; PTR++; Compteur++;
         if (Compteur == 12) {Compteur = 0; Serial.println("-----");}}}   

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
  byte octet = Lire_un_OCTET_en_EEPROM(PTR); if (octet < 10)  Serial.print("0");
  Serial.print(octet,HEX); Serial.print(' '); PTR++; }
