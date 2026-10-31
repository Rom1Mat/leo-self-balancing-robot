# Télécommande tactile iPhone / iPad

1. Ouvrir `self_balancing_robot/self_balancing_robot.ino` dans Arduino IDE, avec le support ESP32-S3 (Arduino-ESP32 3.x).
2. Dans le gestionnaire de bibliothèques, installer **WebSockets**, auteur **Markus Sattler** (projet Links2004/arduinoWebSockets). Cette bibliothèque est différente d’**ArduinoWebsockets**.
3. Conserver les bibliothèques Adafruit MPU6050 et Adafruit Unified Sensor déjà utilisées.
4. Compiler et téléverser le programme, puis ouvrir dans Safari l’adresse `http://` du robot indiquée dans le moniteur série. L’iPhone et le robot utilisent le même réseau Wi-Fi local.
5. Recharger la page après le téléversement. Le titre indique « Robot · télécommande tactile » et le statut devient « Connecté · WebSocket ».

Le programme complet contient la page web : aucun fichier HTML ou JavaScript supplémentaire n’est à téléverser sur l’ESP32. Conserver aussi `XboxBle.h` à côté du `.ino` et installer NimBLE-Arduino 2.5.1 pour cette version avec manette Xbox. Voir `INSTALLATION-XBOX.md` pour l’appairage et les commandes.

## Choix de la source

Au démarrage, **Xbox est la source active**. Sélectionner **Piloter avec le site** pour activer le joystick, le démarrage et les gestes web, ou **Piloter avec la manette** pour les désactiver côté robot. Ce changement freine et annule les appuis en cours. Le WebSocket conserve les mesures et les arrêts dans les deux modes ; une page ouverte ne bloque donc pas la manette. Le choix est synchronisé entre tous les écrans et revient à Xbox au redémarrage.

Valeurs par défaut : limite des roues **12 000 pas/s**, réglable jusqu’à **16 000 pas/s**, translation **4 800 pas/s**, rotation **2 500 pas/s** différentiels, réglable jusqu’à **4 000 pas/s**. La translation garde sa limite à 60 % du maximum des roues. La rotation manuelle répond plus directement ; les gains d’équilibre restent inchangés.

## Commandes

- **Démarrer moteurs** active l’équilibrage.
- Glisser le joystick vers le haut/bas dose l’avance/recul ; gauche/droite dose la rotation. Une diagonale combine les deux.
- Relâcher le joystick remet les consignes de déplacement à zéro pour freiner en gardant l’équilibrage.
- **STOP déplacement** interrompt aussi la danse et conserve l’équilibrage.
- **Couper moteurs** arrête les impulsions et réinitialise la régulation ; le robot ne tient alors plus seul.
- Les boutons des bras et la danse de 32 secondes restent disponibles. Les réglages PID sont dans « Réglages et diagnostic ».

Les gestes de sélection, d’appui long et de défilement sont bloqués dans la zone du joystick. La page reste défilable ailleurs.

## Communication

Le serveur HTTP utilise le port **80**, le WebSocket le port **81**. Le pilotage est envoyé à environ **25 Hz**, la télémétrie à **10 Hz** en mode Web et **5 Hz** en mode Xbox. L’équilibrage reste sur sa tâche locale à **500 Hz**.

Le robot rejette les anciennes trames. Sans nouvelle commande web pendant **350 ms**, il remet les consignes de déplacement à zéro. Le freinage physique dépend ensuite de la régulation et de la mécanique.

Une brève occupation du tampon réseau laisse l’appui actif : la télécommande attend de pouvoir transmettre la position courante, sans empiler les consignes. Au relâchement, la remise à zéro passe en priorité. Une congestion continue de plus de **200 ms** coupe la liaison. Les acquittements des commandes confirment aussi la liaison et actualisent l’horloge utilisée par la télécommande, même si la télémétrie est retardée. Un appui annulé par le navigateur provoque toujours l’arrêt du déplacement et affiche un message.

Après coupure, retard important, arrêt distant ou reconnexion, il faut relâcher puis reprendre le joystick. Une seule page peut posséder la commande de déplacement. Les ordres STOP et coupure restent accessibles depuis les autres pages.

La danse s’interrompt également si la télécommande qui l’a lancée se déconnecte ou n’envoie plus sa présence pendant 600 ms. Son rayon reste une estimation, basée sur des roues de 90 mm, un entraxe de 160 mm et des moteurs de 200 pas par tour en 1/16.

## Danse progressive

La danse dure toujours 32 secondes : 30 secondes de chorégraphie puis 2 secondes de freinage. Chaque mouvement occupe un temps de 3 secondes : accélération sur 800 ms, plateau de 400 ms, freinage sur 800 ms, puis consigne nulle pendant une seconde avant le mouvement suivant. Cette pause laisse la régulation freiner avant les inversions ; elle ne mesure pas l’arrêt réel des roues.

La pointe de translation passe à 5 200 pas/s (environ 0,46 m/s avec la géométrie déclarée), toujours plafonnée à 60 % de la vitesse maximale réglée. La rotation est limitée à 400 pas/s différentiels. Les bras suivent une oscillation continue autour de 90°, avec une amplitude de 40° et un déplacement progressif.

Quand l’écart à l’angle demandé dépasse 1,5° ou la vitesse de basculement dépasse 12°/s, les consignes de danse diminuent progressivement. Elles deviennent nulles à 5°, à 40°/s ou en cas de saturation du contrôle. Les bras suspendent leur mouvement pendant le rattrapage. Ces seuils sont des valeurs initiales à valider sur le robot ; les gains de stabilité et l’accélération maximale restent inchangés.

Le modèle simplifié de suivi des consignes donne un rayon maximal d’environ 0,54 m ; il ne prédit ni le glissement, ni les pertes de pas, ni une chute. La limite estimée de 0,65 m reste active et ne garantit pas le rayon réel d’un mètre sans encodeurs.

## Vérifications disponibles

- `node check_control.cjs` : régulation, trajectoire commandée et limites.
- `node check_remote_protocol.cjs` : expiration des commandes, compteurs, réarmement et cohérence des sources.
- `node check_remote_browser.cjs` : gestes tactiles dans un navigateur Chromium en format mobile et robot simulé. Ce test utilise les chemins du runtime et d’Edge de cet ordinateur.

Ces vérifications ne remplacent pas la compilation ESP32 ni un essai sur Safari iOS et sur le robot réel.

Pour Xbox, les états sont maintenus jusqu’au relâchement ou à la déconnexion BLE détectée ; voir `INSTALLATION-XBOX.md`.
