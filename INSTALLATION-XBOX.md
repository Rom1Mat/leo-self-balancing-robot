# Pilotage direct avec une manette Xbox One 1708

La manette se connecte en Bluetooth Low Energy directement à l’ESP32-S3. Aucun téléphone ni ordinateur n’est nécessaire pour piloter après l’appairage. Le Wi-Fi et l’interface web restent disponibles pour les réglages et le joystick.

Sans réseau Wi-Fi, le robot poursuit le démarrage après une attente maximale d’environ 10 secondes et lance la recherche Bluetooth.

## 1. Vérifier le firmware de la manette

Le numéro 1708 ne suffit pas à garantir la connexion : les firmwares 3.x et 4.x utilisent le Bluetooth classique, absent de l’ESP32-S3. Utiliser un firmware BLE récent ; la documentation Bluepad32 répertorie la 1708 avec les versions **5.15 et suivantes**.

Sur Windows, brancher la manette en USB, ouvrir **Accessoires Xbox**, sélectionner la manette puis le menu « … » pour consulter sa version et appliquer la mise à jour proposée. Une console Xbox peut aussi effectuer cette mise à jour. Débrancher ensuite le câble USB.

Sources : [compatibilité 1708 et firmwares](https://bluepad32.readthedocs.io/en/latest/supported_gamepads/#xbox-wireless-model-1708-2-buttons), [mise à jour Microsoft](https://support.xbox.com/help/hardware-network/controller/update-xbox-wireless-controller).

## 2. Compiler le programme

- Conserver ensemble **self_balancing_robot.ino** et **XboxBle.h** dans le dossier **self_balancing_robot**. Ouvrir le fichier `.ino` dans Arduino IDE.
- Utiliser le support **esp32 par Espressif Systems 3.x**, carte **ESP32S3 Dev Module**.
- Installer **NimBLE-Arduino**, auteur **h2zero**, version **2.5.1** utilisée pour cette version du code. Ne pas installer à la place ArduinoBLE ou ESP32 BLE Arduino.
- Conserver Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO et WebSockets par Markus Sattler.
- Si le compilateur indique que le programme dépasse la partition, sélectionner **Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)**, avec une mémoire flash d’au moins 4 Mo. Ne pas sélectionner une capacité supérieure à celle de la carte réelle.

Le Bluetooth utilise une tâche séparée sur le cœur 0. L’équilibrage reste sur le cœur 1 et la génération des pas sur le timer. Les gains Kp=18 000, Kd=700 et l’accélération maximale de 200 000 pas/s² sont conservés.

## 3. Appairer

1. Allumer le robot, moteurs arrêtés. Le moniteur série à 115 200 bauds affiche la recherche Xbox.
2. Éteindre la console Xbox à proximité et déconnecter la manette de son téléphone/ordinateur pour qu’ils ne la reconnectent pas.
3. Allumer la manette puis maintenir son petit bouton d’appairage jusqu’au clignotement rapide du bouton Xbox.
4. Attendre « Xbox : connectee » dans le moniteur série. L’adresse d’identité Bluetooth et son type sont sauvegardés après un rapport valide. Les clés BLE restent dans la mémoire NVS de NimBLE. Au démarrage, le robot tente directement cette adresse avant de scanner les annonces, y compris dirigées. Les journaux indiquent le nombre de liens restaurés et le résultat de sauvegarde de l’adresse/type.
5. Relâcher tous les boutons et centrer le stick gauche. Un bouton maintenu pendant la connexion ne démarre jamais le robot.

Si aucune connexion n’aboutit, vérifier d’abord le firmware. Une connexion annoncée sans commande exploitable peut signaler un format HID différent : cette version accepte uniquement le rapport BLE Xbox 1 de 16 octets. La compatibilité radio et le format restent à confirmer avec la manette réelle.

### Si le moniteur reste sur la recherche

Le clignotement rapide indique l’appairage, mais ne prouve pas que le firmware utilise le BLE. Vérifier la version avec **Accessoires Xbox**, par câble USB, et appliquer la mise à jour proposée avant de réessayer.

La version avec diagnostic affiche environ toutes les 3,5 secondes le nombre d’appareils BLE vus, puis jusqu’à huit annonces et toutes les annonces candidates Xbox. L’absence du service HID dans une annonce n’élimine plus une manette identifiée par son nom Xbox ou par son constructeur et son apparence ; le service et l’identification Microsoft restent vérifiés après connexion.

- `scan termine, 0 appareil(s)` : aucune annonce BLE reçue sur ce cycle.
- Des appareils apparaissent, mais aucune ligne `candidat=1` : vérifier le firmware et transmettre les lignes du scan pour analyser le nom et l’identification annoncés.
- `autre adresse que la manette memorisee` : envoyer **xbox-reset** avec une fin de ligne dans le moniteur série, puis relancer l’appairage. Cette commande demande l’arrêt des moteurs et efface uniquement l’adresse Xbox mémorisée et les liens BLE de ce programme.
- Les échecs de connexion, de chiffrement et d’identification sont maintenant affichés séparément.

## Commandes

| Commande | Effet |
|---|---|
| **A**, stick gauche centré | Démarrer l’équilibrage si les moteurs sont arrêtés |
| Stick gauche vers le haut / bas | Avancer / reculer, proportionnellement à l’inclinaison |
| Stick gauche horizontal | Tourner, avec ou sans translation |
| **B** | Couper les moteurs : l’équilibrage s’arrête |
| **Y** | Activer / désactiver le mode balancier des bras |
| Croix haut / bas | Lever / descendre le bras gauche |
| Stick droit haut / bas | Lever / descendre le bras droit |

**LB n’est plus nécessaire.** RT, LT, le stick droit horizontal et X n’ont pas d’action. Recentrer le stick gauche remet les consignes de déplacement à zéro en conservant l’équilibrage.

Les bras se déplacent progressivement, jusqu’aux limites de 0 à 180°, pendant le maintien de leur commande. Au relâchement, chaque bras garde sa position. Ils peuvent bouger séparément, simultanément, pendant les déplacements et aussi moteurs coupés. La vitesse maximale des bras en commande manuelle est de 140°/s ; le stick droit dose cette vitesse. Le sens des deux servos est corrigé selon l’essai réel : les commandes vers le haut diminuent maintenant leur angle pour lever les bras.

Les valeurs par défaut sont maintenant : **12 000 pas/s par roue** pour la limite moteur, **4 800 pas/s** pour la translation et **2 500 pas/s** différentiels pour la rotation. Les curseurs autorisent respectivement 16 000, 8 000 et 4 000 pas/s. Ces valeurs restent réglables sur le site. La translation reste plafonnée à 60 % de la vitesse maximale des roues ; avec la limite par défaut de 12 000 pas/s, le plafond de translation est de 7 200 pas/s et la consigne par défaut de 4 800 pas/s reste en dessous. Les gains d’équilibre et l’accélération maximale restent inchangés. Le courant électrique des moteurs reste inchangé.

La zone neutre de la translation est de 12 %, avec une courbe progressive ; celle de la rotation est de 10 % et celle du stick de bras de 12 %. La rotation suit une progression linéaire après la zone neutre et un filtrage de 80 ms au lieu de 200 ms ; les profils de danse gardent leur filtrage précédent. La danse se lance toujours depuis le site.

## Mode balancier dynamique — bouton Y

Le mode est désactivé au démarrage. Chaque nouvel appui sur **Y** le bascule ; maintenir Y ne provoque pas de bascules répétées. Le site affiche sa sélection sous l’état Xbox. Le choix concerne les bras uniquement : vitesses, accélération et régulation des roues restent inchangées.

En marche avant, les deux mains se projettent vers l’avant ; en marche arrière, elles passent vers l’arrière. L’amplitude dépend du joystick. Une projection transitoire accompagne le démarrage et les inversions, puis un léger mouvement alterné anime les bras pendant le maintien. Au relâchement du joystick, ils reviennent progressivement à 90°. Un second appui sur Y désactive le mode et ramène progressivement les deux bras à 90°.

Le profil utilise une projection de 50°, un accent transitoire de 25° et un balancement alterné de 8° jusqu’à 1,2 Hz. Les cibles sont bornées à ±60° autour de 90°. La vitesse automatique est plafonnée à 200°/s et son accélération à 1 000°/s². Ces paramètres donnent une animation dynamique avec des transitions bornées ; ils ne constituent pas une stabilisation active par les bras. L’effet sur l’équilibre doit être vérifié sur le robot réel.

La croix et le stick droit gardent la priorité pour leur bras respectif pendant leur maintien ; l’autre bras continue le balancier. En relâchant la commande manuelle, le bras rejoint progressivement le mouvement automatique. Les commandes manuelles restent utilisables moteurs arrêtés.

Le balancier automatique est suspendu si les moteurs sont arrêtés, après STOP nécessitant un recentrage, à la déconnexion, en mode Web et pendant la danse. Ces interruptions ne déplacent pas automatiquement les bras vers une nouvelle pose et conservent la sélection Y ; le retour à un pilotage Xbox valide avec moteurs actifs permet la reprise progressive. Un Y maintenu pendant une reconnexion n’active pas le mode. La danse conserve sa propre chorégraphie.

Sur ce montage, la diminution des angles correspond à la projection vers l’avant, comme pour les commandes manuelles vers le haut. Si la géométrie réelle inverse ce mouvement, le paramètre `DYNAMIC_ARM_DIRECTION` dans le `.ino` permet d’inverser les deux projections (`-1` par défaut, `+1` dans l’autre sens).

## Sélection exclusive Web / Xbox

Le robot démarre en **mode Xbox**. Pour utiliser le joystick du site, sélectionner **Piloter avec le site**. Pour revenir à la manette, sélectionner **Piloter avec la manette**. Le choix est commun à tous les écrans et revient à Xbox au redémarrage.

Le changement de source remet les consignes à zéro, interrompt la danse et impose de relâcher les commandes avant de reprendre ; il ne coupe pas l’équilibrage. En mode Xbox, le serveur refuse les ordres web de marche, de démarrage, de bras et de danse, même si un ancien onglet continue de les envoyer. En mode Web, il ignore les commandes Xbox de marche, de démarrage et de bras. Les réglages des gains et des vitesses restent disponibles dans les deux modes.

Le WebSocket reste ouvert pour les mesures, l’état de la manette et les arrêts. Le simple fait d’ouvrir le site ne prend pas la main. STOP/coupure du site et B sur la manette restent prioritaires dans les deux modes. Après téléversement, recharger la page pour afficher les deux boutons.

La ligne Xbox du site indique la connexion, l’âge du dernier changement reçu et si la manette est prête ou aux commandes. Cet âge peut augmenter pendant un maintien : il ne signifie plus à lui seul une perte de liaison. Après STOP, chute, déconnexion ou changement de source, recentrer le stick gauche avant de reprendre.

Le délai de **350 ms** reste applicable aux commandes web et au défaut de service de la boucle principale. Pour Xbox, le dernier état reçu reste actif tant que la session BLE est connectée ; le rapport de retour au centre remet les consignes à zéro. Les bras suivent le même principe, jusqu’à leurs limites.

## Corrections de reconnexion et de réactivité

Après installation, allumer le robot puis allumer simplement la manette avec le bouton Xbox. En cas de clés déjà incohérentes, un dernier appairage rapide peut être nécessaire. Ne pas utiliser `xbox-reset` à chaque démarrage. Si la reconnexion échoue encore, relever les lignes `liens BLE memorises`, `adresse/type sauvegardes`, et `tentative de reconnexion directe` après un redémarrage ; elles distinguent une perte des clés d’un échec radio.

Un silence de notifications ne supprime plus le réarmement et n’interrompt plus un mouvement maintenu. Après A, le neutre déjà validé autorise le premier mouvement du stick gauche, même s’il arrive juste avant que la tâche d’équilibrage ait activé les moteurs. L’âge affiché du rapport n’est pas modifié artificiellement.

Ces corrections concernent la gestion des entrées ; elles ne suppriment pas le PI de vitesse, nécessaire au freinage, ni la boucle d’équilibrage.

## Persistance des commandes et perte de liaison

Une entrée HID est un état conservé jusqu’au changement suivant. Les lectures ATT de secours et les reconnexions fondées uniquement sur l’absence de notifications sont retirées : elles pouvaient interrompre une commande stable. Une seule notification de mouvement suffit désormais pour maintenir l’ordre ; une notification de relâchement l’annule.

Le callback NimBLE `onDisconnect` invalide directement la session, y compris pendant une opération GATT bloquante. La boucle principale remet alors translation, rotation et commandes des bras à zéro. La reconnexion conserve les clés et exige de recentrer le stick avant toute reprise. Une consigne maintenue ne redémarre jamais automatiquement après reconnexion.

Le programme demande une supervision radio de **500 ms**, indépendante de la fréquence des rapports HID. Le délai réellement négocié est affiché sur le port série avec `supervision ... ms` ; la remise à zéro intervient après détection de la déconnexion, pas nécessairement 350 ms après le dernier changement de commande. Ce mécanisme détecte la perte de liaison radio ; il ne détecte pas un blocage applicatif de la manette si sa liaison BLE reste active. Le freinage physique dépend ensuite de la régulation du robot.

Dans **Réglages et diagnostic**, la ligne Xbox distingue notifications, rapports rejetés, LT/RT bruts, consigne appliquée et raison d’un éventuel refus. Maintenir le stick dix secondes doit conserver la consigne même si le compteur de notifications reste fixe. Recentrer doit remettre la consigne à zéro. Éteindre la manette pendant une commande doit annuler les consignes à la déconnexion détectée. Ces vérifications matérielles restent à réaliser.

L’état de retour au neutre est aussi mémorisé dans la boîte de réception : un bref relâchement remplacé par l’appui suivant avant le passage de loop() peut désormais réarmer la commande. Un relâchement antérieur à un STOP, une perte de liaison ou un changement de source ne permet jamais une reprise automatique.

La télémétrie web passe à 5 Hz et sa présence à 2 Hz en mode Xbox pour réduire le trafic Wi-Fi, contre 10 Hz en mode Web. Le pilotage web reste à environ 25 Hz. La réception effective du rapport HID sur le firmware de la manette reste à vérifier sur le matériel.

Référence : [profil HID over GATT du Bluetooth SIG](https://www.bluetooth.com/specifications/specs/hid-over-gatt-profile-1-0/).

## Vérifications

- `node check_xbox.cjs` : décodeur et décisions extraits du programme (stick gauche, A/B, bras indépendants, maintien 60 s sans nouvelle notification, relâchement, déconnexion, reconnexion, arbitrage et balancier Y).
- `node check_control.cjs` et `node check_remote_protocol.cjs` : régulation, danse et protocole web.

Les tests de décisions utilisent une traduction JavaScript des fonctions C++ et ne valident pas la liaison radio. La compilation ne remplace pas l’essai avec la manette et le robot réels.

Compilation vérifiée avec Arduino-ESP32 **3.3.11**, NimBLE-Arduino **2.5.1** et la carte `esp32:esp32:esp32s3`, avec la partition par défaut de 1 310 720 octets pour le programme. Les bibliothèques installées sont Adafruit MPU6050 2.2.9, Adafruit BusIO 1.17.4, Adafruit Unified Sensor 1.1.15 et WebSockets 2.7.2.

Références techniques : [API client NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino/tree/master/examples/NimBLE_Client), [format des rapports Xbox](https://github.com/asukiaaa/arduino-XboxControllerNotificationParser).
