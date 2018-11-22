#include <Arduino.h>
#include "positions.h"



SERVO servos[12] = {  
  {0,  120, 600, 0}, // leg A (front right) 
  {1,  120, 600, 0}, 
  {2,  120, 550, 0}, 
  {4,  120, 540, 0}, // leg B (back right)
  {5,  120, 560, 0},
  {6,  120, 580, 0},
  {8,  120, 590, 0}, // leg C (front left)
  {9,  120, 580, 0},
  {10, 120, 580, 0},
  {12, 120, 580, 0}, // Leg D (back left)
  {13, 200, 540, 0},
  {14, 120, 580, 0}
};


const LEG_POSITION positions[17][4] = {
  // 00 ------------- Tableau de configuration STABLE Raisonnable. (000) -------------- 
  {{305,  213,  307}, {274,  406,  314}, {287,  215,  319}, {285,  388,  314}},
  // 01 ------------- Retour de configuration STABLE Transversale. (024) -------------- 
  {{305,  315,  421}, {274,  296,  192}, {287,  298,  426}, {285,  291,  199}},
  // 02 ------------------ Tableau des Neutres opérationnels. (048) -------------------
  {{320,  308,  316}, {289,  303,  314}, {262,  292,  310}, {300,  285,  307}},
  // 03 ------------------ Tableau de Configuration Décollage. (072) ------------------ 
  {{211,  152,  315}, {413,  486,  316}, {159,  139,  255}, {411,  463,  325}},
  // 04 ---------------- Tableau de Configuration Atterrissage. (096) ----------------- 
  {{303,  490,  154}, {303,  141,  481}, {303,  472,  145}, {303,  134,  501}},
  // 05 ------------- Tableau de configuration STABLE Transversale. (120)-------------- 
  {{239,  213,  307}, {344,  414,  314}, {222,  215,  319}, {346,  388,  314}},
  // 06 -------------- Tableau de configuration Hauteur Maximale. (144)--------------- 
  {{239,  152,  230}, {344,  483,  391}, {222,  135,  232}, {346,  467,  398}},
  // 07 ----------------- Configuration "Avancée" pour Reculer. (168) ----------------- 
  {{274,  213,  307}, {379,  414,  314}, {187,  215,  319}, {311,  388,  314}},
  // 08 ------------------------- Configuration de VEILLE. (192) ---------------------- 
  {{239,  315,  421}, {344,  296,  192}, {222,  298,  426}, {346,  291,  199}},
  // 09 ---------------------- Configuration pointage LASER. (216) -------------------- 
  {{312,  314,  135}, {344,  414,  314}, {222,  215,  319}, {171,  388,  314}},
  // 10 --------------------- Configuration Retour de LASER. (240) -------------------- 
  {{239,  213,  307}, {344,  414,  314}, {222,  215,  319}, {171,  388,  314}},
  // 11 ------------------ Configuration "Reculée" pour Avancer. (264) ---------------- 
  {{204,  213,  307}, {309,  414,  314}, {257,  215,  319}, {381,  388,  314}},
  // 12 ------------------ Configuration avant Hauteur Maximale. (288) ---------------- 
  {{239,  264,  449}, {344,  347,  167}, {222,  262,  464}, {346,  312,  158}},
  // 13 ------------------- Configuration pour Tourner à Droite. (312) ---------------- 
  {{269,  213,  307}, {374,  414,  314}, {252,  215,  319}, {376,  388,  314}},
  // 14 ------------------- Configuration pour Tourner à Gauche. (336) ---------------- 
  {{209,  213,  307}, {314,  414,  314}, {192,  215,  319}, {316,  388,  314}},
  // 15 ------------------- Configuration pour Décaler à Droite. (360) ---------------- 
  {{239,  270,  281}, {344,  344,  339}, {222,  215,  439}, {346,  381,  188}},
  // 16 ------------------- Configuration pour Décaler à Gauche. (384) ---------------- 
  {{239,  230,  430}, {344,  392,  190}, {222,  258,  287}, {346,  334,  346}}
};
