# NUCLEO-L152RE-LSM6DSO (GestureFlow)
## Reconnaissance de mouvement STM32

Système de reconnaissance de 4 mouvements (Statique, Haut-Bas, Avant-Arrière, Cercle) 
utilisant un accéléromètre LSM6DSO et l'IA embarquée NanoEdge AI Studio.

![STM32](https://img.shields.io/badge/STM32-L152RE-blue)
![AI](https://img.shields.io/badge/AI-NanoEdge-green)

## Fonctionnalités principales
✅ Reconnaissance de mouvements en temps réel
✅ Affichage sur écran 7 segments
✅ Contrôle du buzzer et du moteur par l'utilisateur

## Matériel requis
Pour utiliser ce projet, vous devez avoir le matériel suivant :
- STM32 Nucleo-L152RE
- Shield X-NUCLEO-IKS01A3 (LSM6DSO)
- Afficheur 7-segments
- 1 moteur
- 1 buzzer
- 1 potentiomètre,
- 2 boutons

## Installation
1. Cloner le repo
2. Ouvrir avec STM32CubeIDE
3. Compiler et flasher sur la carte

## Utilisation
1. Effectuer les mouvements avec la carte
2. Observer l'affichage sur l'écran 7-segments
3. Utiliser les boutons pour contrôler le moteur et le potentiomètre pour contrôler le buzzer.