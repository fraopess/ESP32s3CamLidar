# Intégration du capteur PMW3901

## Vue d'ensemble

Le code a été modifié pour supporter deux modes de flux optique :
1. **Mode Caméra** : Utilise une caméra (OV2640/OV5640/OV7725) avec calcul Lucas-Kanade
2. **Mode PMW3901** : Utilise le capteur dédié PMW3901 pour le flux optique

## Modifications apportées

### Nouveaux fichiers

1. **pmw3901.h** - Header du driver PMW3901
   - Définitions des registres du capteur
   - API publique pour l'initialisation et la lecture
   - Structure de données pour les mesures de mouvement

2. **pmw3901.c** - Implémentation du driver PMW3901
   - Communication SPI avec le capteur
   - Séquence d'initialisation et configuration
   - Lecture des données de mouvement (delta X, delta Y)
   - Contrôle du LED du capteur

### Modifications du main.c

#### Configuration par #define

Au début du fichier, sélectionnez UN SEUL mode :

```c
// ============================================================================
// SÉLECTION DU MODE DE FLUX OPTIQUE (décommenter UNE SEULE ligne)
// ============================================================================
//#define USE_CAMERA        // Utiliser la caméra pour le flux optique
#define USE_PMW3901       // Utiliser le capteur PMW3901 pour le flux optique
```

#### Configuration des pins SPI pour le PMW3901

```c
#define PMW3901_SPI_HOST   SPI2_HOST
#define PMW3901_PIN_CS     1    // À adapter selon votre câblage
#define PMW3901_PIN_SCK    2    // À adapter selon votre câblage
#define PMW3901_PIN_MOSI   3    // À adapter selon votre câblage
#define PMW3901_PIN_MISO   21   // À adapter selon votre câblage
```

**IMPORTANT** : Modifiez ces valeurs selon votre câblage réel !

## Spécifications du PMW3901

### Interface matérielle

- **Bus** : SPI (Mode 3, 4 MHz)
- **Alimentation** : 3.3V
- **Pins requises** :
  - CS (Chip Select)
  - SCK (SPI Clock)
  - MOSI (SPI Data Out)
  - MISO (SPI Data In)
  - VCC (3.3V)
  - GND

### Caractéristiques du capteur

- **Fréquence d'échantillonnage** : Jusqu'à 121 FPS
- **Champ de vision** : 42° (FOV)
- **Portée de détection** : 80mm à l'infini
- **Résolution de mouvement** : 16-bit signé (delta X, delta Y)
- **Courant** : 9mA (mode actif), 12µA (veille)

### Données disponibles

Le PMW3901 retourne les informations suivantes :
- **delta_x** (int16_t) : Déplacement horizontal en pixels depuis la dernière lecture
- **delta_y** (int16_t) : Déplacement vertical en pixels depuis la dernière lecture
- **squal** (uint8_t) : Qualité du signal (0-255)
- **motion** (bool) : Indicateur de mouvement détecté

## Calcul de la vitesse

La vitesse est calculée en combinant les données du PMW3901 avec le LiDAR :

```c
velocity_x = (delta_x * distance * pixel_size) / (focal_length * dt)
velocity_y = (delta_y * distance * pixel_size) / (focal_length * dt)
```

Où :
- `delta_x`, `delta_y` : Déplacements en pixels du PMW3901
- `distance` : Distance mesurée par le LiDAR (en mètres)
- `pixel_size` : Taille physique du pixel (~0.030mm pour PMW3901)
- `focal_length` : Longueur focale équivalente (~42 pixels)
- `dt` : Temps entre deux mesures (1/fps)

## Tester seulement le LiDAR

Pour tester uniquement le LiDAR sans caméra ni PMW3901 :

1. Commentez les deux lignes :
```c
//#define USE_CAMERA
//#define USE_PMW3901
```

2. Vous devrez modifier le code pour permettre un mode "LiDAR seulement" (non implémenté actuellement)

Ou utilisez le mode PMW3901 sans connecter le capteur (pour debug), mais cela générera des erreurs.

## Brochage suggéré pour ESP32-S3

**PMW3901 sur SPI2** :
- CS   → GPIO 1  (ou autre GPIO disponible)
- SCK  → GPIO 2  (ou autre GPIO disponible)
- MOSI → GPIO 3  (ou autre GPIO disponible)
- MISO → GPIO 21 (ou autre GPIO disponible)
- VCC  → 3.3V
- GND  → GND

**Éviter les conflits** :
- Ne pas utiliser les pins déjà utilisés par la caméra (si USE_CAMERA)
- Ne pas utiliser les pins du UART LiDAR (TX=47, RX=14)
- Vérifier que les pins choisis supportent le SPI

## Notes de calibration

### PMW3901

Les paramètres suivants peuvent nécessiter un ajustement :
- `PMW3901_FOCAL_LENGTH_PX` : Basé sur le FOV de 42°
- `PMW3901_PIXEL_SIZE_MM` : Estimation, peut nécessiter calibration
- `PMW3901_FLOW_SCALE` : Facteur d'échelle (par défaut 1.0)

### Calibration recommandée

1. Placez le système sur un objet en mouvement connu
2. Comparez les vitesses calculées avec les vitesses réelles
3. Ajustez les paramètres en conséquence

## Comparaison Camera vs PMW3901

| Caractéristique | Caméra (OV7725) | PMW3901 |
|----------------|-----------------|---------|
| FPS maximum | ~60-80 FPS | 121 FPS |
| Latence | Moyenne | Faible |
| Consommation CPU | Élevée (Lucas-Kanade) | Faible (calcul intégré) |
| Résolution | 160×120 pixels | 35×35 pixels (interne) |
| Coût CPU | ~40-50% | ~5-10% |
| Qualité | Dépend de l'éclairage | Robuste aux variations |

## Dépannage

### PMW3901 ne s'initialise pas

1. Vérifiez le câblage SPI
2. Vérifiez l'alimentation 3.3V
3. Vérifiez que les pins configurées sont correctes
4. Activez les logs : `esp_log_level_set("PMW3901", ESP_LOG_DEBUG);`

### Données aberrantes

1. Vérifiez la qualité du signal (`squal`)
2. Assurez-vous que la surface observée a du contraste
3. Vérifiez la distance au sol (min 80mm)

### Erreurs de compilation

Si vous obtenez des erreurs avec des macros non définies :
- Assurez-vous qu'un seul mode est activé (USE_CAMERA ou USE_PMW3901)
- Si USE_CAMERA est activé, sélectionnez une caméra (OV2640/OV5640/OV7725)

## Exemple de sortie

```
=================================
ESP32-S3 Optical Flow System
Mode: PMW3901 optical flow sensor
Sensor: PMW3901
FOV: 42 degrees
Max rate: 121 FPS
Pixel size: 0.0300 mm
Focal length: 42.00 px
LiDAR: DTS6012M
=================================
Binary Protocol Mode
Packet: [0xAA 0x55][ts(4)][vx(2)][vy(2)][dist(2)][chk(1)]
Starting transmission...
```

## Ressources

- [Datasheet PMW3901](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [PX4 PMW3901 Driver](https://github.com/PX4/PX4-Autopilot)
- [Bitcraze PMW3901 Library](https://github.com/bitcraze/Bitcraze_PMW3901)
